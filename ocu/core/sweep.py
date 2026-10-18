"""Laying out the route clearance sweep.

An operator designates an area and the vehicle sweeps it a lane at a time.
The lanes themselves are not arbitrary: a sweep laid across a rock field is
a sweep the vehicle spends its whole mission detouring around, and ground
that gets detoured around never gets looked at. So the axis of the sweep is
chosen for the ground it covers.

The choice is made from what the rover can see. Both the world and the
rover's own map answer "what is at this cell", so the planner takes either;
the rover plans from its map, after its first scan at the deploy point, and
nothing about ground it has not seen influences the decision. Ground it
cannot see counts as half clear: neither assumed drivable nor assumed
blocked, which is what a sensor that cannot see through a rock has the
right to say about the ground behind it. What the vehicle does once it is
driving, discovering obstacles the first scan did not show, routing round
them, and reporting the lane points it cannot reach, is navigation, and that
happens in navigation.py against the map the rover keeps building.

Lanes carry points along their length rather than only at their ends. The
planner holds the leg it is given (navigation.Corridor), so short legs are
what keep the vehicle on the lane across ground it has not yet seen.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.lidar import cast_scan
from ocu.core.terrain import CELL_SIZE_M, IMPASSABLE, PerceptionMap, Terrain, TerrainWorld

#: Four lanes is enough to show lane discipline and a serpentine turn
#: without the sweep outrunning the mission.
LANE_COUNT = 4

#: Lane separation. Wider than any sensor swath on the vehicle, so the
#: coverage this sweep achieves is lane coverage and the gaps between lanes
#: are visible on the map for what they are.
LANE_SPACING_M = 4.0

#: How far out each lane runs, and where it starts. The sweep stays inside
#: the 50 m tether so the aircraft can hold formation over it throughout.
LANE_START_M = 3.0
LANE_LENGTH_M = 24.0

#: Spacing of the points along a lane.
LANE_POINT_SPACING_M = 3.0

#: Candidate axes, every fifteen degrees of the compass.
BEARING_STEP_DEG = 15.0

#: Resolution at which a candidate footprint is scored. Finer than this
#: buys nothing: the question is whether a lane is drivable, not where
#: every individual rock sits.
SCORE_STEP_M = 1.0


def _frame(bearing_deg: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """Forward and right unit vectors for a compass bearing.

    Bearing is measured clockwise from north, and the map's +y is north, so
    forward is (sin, cos) and right is (cos, -sin).
    """
    radians = math.radians(bearing_deg)
    return (math.sin(radians), math.cos(radians)), (math.cos(radians), -math.sin(radians))


@dataclass(frozen=True)
class LaneSweep:
    """A serpentine lane sweep in world coordinates."""

    origin: tuple[float, float]
    bearing_deg: float
    clear_fraction: float
    points: list[tuple[float, float, str]]

    def at(self, lane_index: int, along_m: float, offset_m: float = 0.0) -> tuple[float, float]:
        """A world point on a lane: `along_m` down it, `offset_m` across it.

        This is how anything placed relative to the sweep gets positioned,
        so that what the vehicle drives over does not depend on which way
        the sweep happened to be laid out.
        """
        forward, right = _frame(self.bearing_deg)
        across = lane_index * LANE_SPACING_M + offset_m
        return (
            self.origin[0] + forward[0] * along_m + right[0] * across,
            self.origin[1] + forward[1] * along_m + right[1] * across,
        )


def clear_fraction(
    terrain,
    origin: tuple[float, float],
    bearing_deg: float,
    cell_size_m: float = CELL_SIZE_M,
) -> float:
    """What proportion of the sweep footprint on this axis is drivable.

    `terrain` is anything with `.at(cell)`: the world, or the rover's own
    map. Unknown ground scores half: the rover cannot say it is clear and it
    cannot say it is blocked.
    """
    forward, right = _frame(bearing_deg)
    total = 0.0
    clear = 0.0
    along = LANE_START_M
    while along <= LANE_LENGTH_M:
        across = 0.0
        while across <= (LANE_COUNT - 1) * LANE_SPACING_M:
            x = origin[0] + forward[0] * along + right[0] * across
            y = origin[1] + forward[1] * along + right[1] * across
            cell = (int(math.floor(x / cell_size_m)), int(math.floor(y / cell_size_m)))
            kind = terrain.at(cell)
            total += 1.0
            if kind == Terrain.UNKNOWN:
                clear += 0.5
            elif kind not in IMPASSABLE:
                clear += 1.0
            across += SCORE_STEP_M
        along += SCORE_STEP_M
    return clear / total if total else 0.0


def plan_lane_sweep(
    terrain,
    origin: tuple[float, float] = (0.0, 0.0),
    cell_size_m: float = CELL_SIZE_M,
) -> LaneSweep:
    """Lay the sweep out on the clearest axis available from the origin.

    Every bearing is scored on how much of its footprint is drivable and
    the best is taken. The vehicle still has to find its own way along each
    lane, and will still meet obstacles the first scan did not show; what
    this avoids is committing the whole mission to an axis that was never
    sweepable in the first place.
    """
    candidates = [i * BEARING_STEP_DEG for i in range(int(360.0 / BEARING_STEP_DEG))]
    scored = [(clear_fraction(terrain, origin, b, cell_size_m), -b, b) for b in candidates]
    best_fraction, _, bearing = max(scored)

    forward, right = _frame(bearing)
    points: list[tuple[float, float, str]] = []
    for lane_index in range(LANE_COUNT):
        along_values = _lane_stations(outbound=lane_index % 2 == 0)
        label = f"LANE {lane_index + 1}"
        across = lane_index * LANE_SPACING_M
        for along in along_values:
            points.append(
                (
                    origin[0] + forward[0] * along + right[0] * across,
                    origin[1] + forward[1] * along + right[1] * across,
                    label,
                )
            )
    return LaneSweep(
        origin=origin, bearing_deg=bearing, clear_fraction=best_fraction, points=points
    )


def _lane_stations(outbound: bool) -> list[float]:
    """Distances along a lane at which the vehicle is given a point.

    Alternate lanes run the other way, which is what makes the sweep a
    serpentine rather than a set of runs with a drive back between each.
    """
    stations = []
    along = LANE_START_M
    while along < LANE_LENGTH_M:
        stations.append(along)
        along += LANE_POINT_SPACING_M
    stations.append(LANE_LENGTH_M)
    return stations if outbound else list(reversed(stations))


def sweep_from_first_scan(
    world: TerrainWorld, origin: tuple[float, float] = (0.0, 0.0)
) -> LaneSweep:
    """The sweep the rover will choose once it is deployed at `origin`.

    This is what the rover does at the deploy point -- one scan, then plan
    from what that scan produced -- run ahead of time so that a scenario can
    be set up consistently with it. It consults the world only through the
    scanner, exactly as the rover does, and a scan from the same place marks
    the same cells, so the decision made here and the one made on the
    vehicle cannot differ.
    """
    seen = PerceptionMap()
    cast_scan(world, seen, origin[0], origin[1])
    return plan_lane_sweep(seen, origin)


#: Washouts cut onto the lanes themselves, as (lane, metres along, metres
#: across, radius, depth). Scenario content, like the buried objects in
#: targets.py: the sweep is laid on the clearest ground the first scan can
#: see, so whether the generated washouts happen to fall on it is luck of the
#: seed. These are placed past the range at which a negative obstacle can be
#: resolved, so the rover cannot know about them when it plans and has to
#: find them and route around them as it drives. That is the behaviour worth
#: watching, and it is the one thing the terrain cannot be relied on to stage.
LANE_WASHOUTS = (
    (0, 15.5, 0.10, 0.95, 0.50),
    (2, 11.0, -0.30, 0.80, 0.45),
)


def cut_washouts_on_lanes(world: TerrainWorld, sweep: LaneSweep) -> None:
    """Put the scenario's washouts on the lanes this sweep will drive."""
    for lane, along, across, radius, depth in LANE_WASHOUTS:
        x, y = sweep.at(lane, along, across)
        world.cut_washout(x, y, radius, depth)
