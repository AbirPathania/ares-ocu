"""Laying out the route clearance sweep.

An operator designates an area and the vehicle sweeps it a lane at a time.
The lanes themselves are not arbitrary: a sweep laid across a rock field is
a sweep the vehicle spends its whole mission detouring around, and ground
that gets detoured around never gets looked at. So the axis of the sweep is
chosen for the ground it covers.

The choice is made against the terrain rather than against what the rover
has seen, because this is mission planning, not navigation: a clearance
task is planned on prior imagery of the area before anything drives into
it. What the vehicle does once it is driving -- discovering obstacles the
imagery did not show, routing around them, and reporting the lane points it
cannot reach -- is navigation, and that happens in navigation.py against
the map the rover builds for itself.

Lanes carry points along their length rather than only at their ends. The
planner holds the leg it is given (navigation.Corridor), so short legs are
what keep the vehicle on the lane across ground it has not yet seen.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.terrain import IMPASSABLE, CELL_SIZE_M, Terrain, TerrainWorld

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
    world: TerrainWorld,
    origin: tuple[float, float],
    bearing_deg: float,
    cell_size_m: float = CELL_SIZE_M,
) -> float:
    """What proportion of the sweep footprint on this axis is drivable."""
    forward, right = _frame(bearing_deg)
    total = 0
    clear = 0
    along = LANE_START_M
    while along <= LANE_LENGTH_M:
        across = 0.0
        while across <= (LANE_COUNT - 1) * LANE_SPACING_M:
            x = origin[0] + forward[0] * along + right[0] * across
            y = origin[1] + forward[1] * along + right[1] * across
            cell = (int(math.floor(x / cell_size_m)), int(math.floor(y / cell_size_m)))
            total += 1
            if world.at(cell) not in IMPASSABLE:
                clear += 1
            across += SCORE_STEP_M
        along += SCORE_STEP_M
    return clear / total if total else 0.0


def plan_lane_sweep(
    world: TerrainWorld,
    origin: tuple[float, float] = (0.0, 0.0),
    cell_size_m: float = CELL_SIZE_M,
) -> LaneSweep:
    """Lay the sweep out on the clearest axis available from the origin.

    Every bearing is scored on how much of its footprint is drivable and
    the best is taken. The vehicle still has to find its own way along each
    lane, and will still meet obstacles the planning imagery did not show;
    what this avoids is committing the whole mission to an axis that was
    never sweepable in the first place.
    """
    candidates = [i * BEARING_STEP_DEG for i in range(int(360.0 / BEARING_STEP_DEG))]
    scored = [(clear_fraction(world, origin, b, cell_size_m), -b, b) for b in candidates]
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
