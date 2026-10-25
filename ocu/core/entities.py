"""People, vehicles and animals on the ground: things a camera can see that no buried-object sensor can.

The buried objects in targets.py are found by the ground suite and, if shallow, by a scout's camera. These are
the other half of what ARES is expected to perceive, threats and non-threats that stand or walk on the
surface: a person (the one the spec's threat_type enum names), a vehicle (also named), and an animal (not
named; see DECISIONS.md, the eighth class is still open).

Like the buried objects they are placed on purpose and positioned relative to the sweep, so that what the
rover and the scouts can see does not depend on which way the sweep happened to be laid out. Nothing here
appears at random. Nothing in this file decides whether anything is seen: that is vision.py.

Sizes, speeds and positions are this build's choices, not the baseline's; DECISIONS.md lists them.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from ocu.core.sweep import LaneSweep, sweep_from_first_scan
from ocu.core.terrain import IMPASSABLE, WORLD_EDGE_MARGIN_M, TerrainWorld

KINDS = ("person", "vehicle", "animal")

#: How much room a thing takes on the ground, as a circle, for the planner. A walking person is
#: about 0.5 m across at the shoulders, a deer about 1.6 m long; each is given a little more than
#: that, because the rover treats the circle's edge as the obstacle. The margin for movement is added
#: separately (dynamic_obstacles.py). Chosen, not measured.
OBSTACLE_RADIUS_M = {"person": 0.4, "animal": 0.8, "vehicle": 2.8}

#: The outlines a scanner's beam cuts: a person at chest height (jacket and pack), a deer's barrel and haunch.
#: From the drawn figures (render/figures.py).
LIDAR_SHAPES = {"person": [(-0.045, 0.23, 0.17)], "animal": [(-0.02, 0.17, 0.62)]}

#: Distance covered in one full stride (two steps), so that the legs move as fast as the ground goes by.
STRIDE_M = {"person": 1.45, "animal": 1.35}

#: Nothing is placed nearer than this to the deploy point, where the rover starts and the ground is cleared.
MIN_FROM_DEPLOY_M = 7.0

#: A parked vehicle needs ground this level under it, and every cell it covers (and a metre round it)
#: clear of rock, trees, bushes and holes.
MAX_FOOTPRINT_RELIEF_M = 0.30
CLEAR_MARGIN_M = 1.0


@dataclass(frozen=True)
class SurfaceEntity:
    kind: str
    name: str
    height_m: float  # standing height; the detector's curves are indexed by how tall it looks
    width_m: float  # across the direction it faces
    length_m: float  # along the direction it faces
    path: tuple[tuple[float, float], ...]  # world points; one point means it stays put
    speed_ms: float = 0.0
    heading_deg: float = 0.0  # only for something that stays put; a walker faces the way it walks
    box_offset_m: float = 0.0  # how far ahead of its position the middle of its outline lies (a deer's head is out front)
    style: str = ""  # which drawing of its kind: "buck" is a deer with antlers, "armed" a person carrying a long gun
    carrying: str = ""  # "long_gun", "handgun" or nothing: what a person is carrying in the open

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f"unknown entity kind {self.kind!r}")
        if not self.path:
            raise ValueError("an entity needs at least one position")
        if self.carrying and self.carrying not in self.WEAPON_SIZE_M:
            raise ValueError(f"unknown weapon {self.carrying!r}")
        if self.carrying and self.kind != "person":
            raise ValueError("only a person carries a weapon")

    #: The weapons a person can carry, as (length, height) in metres: a service rifle and a pistol. Used for the share
    #: of the person's crop the weapon fills, which is what the measured weapon curve is indexed by.
    WEAPON_SIZE_M = {"long_gun": (1.00, 0.16), "handgun": (0.21, 0.14)}

    @property
    def weapon_size_m(self) -> tuple[float, float]:
        return self.WEAPON_SIZE_M.get(self.carrying, (0.0, 0.0))

    @property
    def static(self) -> bool:
        return len(self.path) == 1 or self.speed_ms <= 0.0

    def gait_phase(self, t_s: float) -> float:
        """Where in its stride (0 to 1) a walker is after t_s seconds of walking: one stride per STRIDE_M walked."""
        if self.static:
            return 0.0
        return (self.speed_ms * max(t_s, 0.0) / STRIDE_M.get(self.kind, 1.4)) % 1.0

    @property
    def lidar_shapes(self) -> list[tuple[float, float, float]]:
        """The outline a horizontal beam cuts through this thing, as (forward offset, half-width, half-depth) ellipses.
        A parked vehicle has none: it is part of the ground (stamp_object) and the scanner maps it as such."""
        return LIDAR_SHAPES.get(self.kind, [])

    @property
    def obstacle_radius_m(self) -> float:
        return OBSTACLE_RADIUS_M[self.kind]

    def pose_at(self, t_s: float) -> tuple[float, float, float]:
        """(x, y, heading in degrees clockwise from +y) after t_s seconds, walking its path back and forth."""
        if self.static:
            return self.path[0][0], self.path[0][1], self.heading_deg % 360.0
        legs = [math.dist(a, b) for a, b in zip(self.path, self.path[1:], strict=False)]
        total = sum(legs)
        if total <= 0.0:
            return self.path[0][0], self.path[0][1], 0.0
        travelled = (self.speed_ms * max(t_s, 0.0)) % (2.0 * total)
        backwards = travelled > total
        if backwards:
            travelled = 2.0 * total - travelled
        for i, leg in enumerate(legs):
            if travelled <= leg or i == len(legs) - 1:
                f = 0.0 if leg == 0.0 else min(travelled / leg, 1.0)
                (x0, y0), (x1, y1) = self.path[i], self.path[i + 1]
                x, y = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
                dx, dy = (x0 - x1, y0 - y1) if backwards else (x1 - x0, y1 - y0)
                return x, y, math.degrees(math.atan2(dx, dy)) % 360.0
            travelled -= leg
        return self.path[-1][0], self.path[-1][1], 0.0

    def _heading(self, i: int) -> float:
        if len(self.path) < 2:
            return 0.0
        (x0, y0), (x1, y1) = self.path[i], self.path[i + 1]
        return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0


def _passable(world: TerrainWorld, x: float, y: float, radius: float) -> bool:
    """Every cell within `radius` of the point is something a vehicle could drive on."""
    cx, cy = world.to_cell(x, y)
    reach = int(math.ceil(radius / world.cell_size_m))
    for ix in range(cx - reach, cx + reach + 1):
        for iy in range(cy - reach, cy + reach + 1):
            wx, wy = (ix + 0.5) * world.cell_size_m, (iy + 0.5) * world.cell_size_m
            if math.hypot(wx - x, wy - y) <= radius and world.at((ix, iy)) in IMPASSABLE:
                return False
    return True


def _level(world: TerrainWorld, x: float, y: float, radius: float) -> bool:
    heights = [
        world.elevation_at((ix, iy))
        for ix in range(world.to_cell(x, y)[0] - int(radius / world.cell_size_m), world.to_cell(x, y)[0] + int(radius / world.cell_size_m) + 1)
        for iy in range(world.to_cell(x, y)[1] - int(radius / world.cell_size_m), world.to_cell(x, y)[1] + int(radius / world.cell_size_m) + 1)
    ]
    return max(heights) - min(heights) <= MAX_FOOTPRINT_RELIEF_M


def _offsets(max_m: float = 24.0):
    """Candidate shifts, nearest first: the place asked for, then rings round it."""
    yield 0.0, 0.0
    r = 1.0
    while r <= max_m:
        n = max(8, int(2 * math.pi * r / 1.0))
        for k in range(n):
            a = 2 * math.pi * k / n
            yield r * math.sin(a), r * math.cos(a)
        r += 1.0


def settle_entities(world: TerrainWorld, entities: list[SurfaceEntity]) -> list[SurfaceEntity]:
    """Move each entity to the nearest ground it can honestly stand or walk on.

    A vehicle parked inside a stand of trees, a person walking through a boulder: placed on purpose does
    not mean placed blind. A parked vehicle needs level, clear ground under its whole footprint (and a
    metre round it); a walker or animal needs every point it will pass over clear, with room either side.
    The first place that works, nearest to the one asked for, is used. Raises if there is none within 24 m,
    which would mean the scenario is not possible on this ground.
    """
    out = []
    for e in entities:
        radius = max(e.length_m, e.width_m) / 2.0 if e.static else e.obstacle_radius_m
        found = None
        for dx, dy in _offsets():
            for dh in ((0.0, 20.0, -20.0, 40.0, -40.0, 90.0) if e.static else (0.0,)):
                path = tuple((x + dx, y + dy) for x, y in e.path)
                if any(math.hypot(x, y) < MIN_FROM_DEPLOY_M for x, y in path):
                    continue
                if any(max(abs(x), abs(y)) > world.half_extent_m - WORLD_EDGE_MARGIN_M for x, y in path):
                    continue  # off the edge of the map is not ground
                if e.static:
                    ok = _passable(world, *path[0], radius + CLEAR_MARGIN_M) and _level(world, *path[0], radius)
                else:
                    pts = [
                        (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
                        for a, b in zip(path, path[1:], strict=False)
                        for f in np.linspace(0.0, 1.0, max(2, int(math.dist(a, b) / 0.5)))
                    ]
                    ok = all(_passable(world, x, y, radius + 0.4) for x, y in pts or [path[0]])
                if ok:
                    found = replace(e, path=path, heading_deg=(e.heading_deg + dh) % 360.0)
                    break
            if found:
                break
        if found is None:
            raise ValueError(f"no clear ground within 24 m of the requested place for {e.name!r}")
        out.append(found)
    return out


def stamp_static_entities(world: TerrainWorld, entities: list[SurfaceEntity]) -> list[tuple[int, int]]:
    """Put every parked thing into the world as an obstacle, so the LiDAR hits it and the rover plans round it."""
    cells = []
    for e in entities:
        if e.static:
            x, y, heading = e.pose_at(0.0)
            cells += world.stamp_object(x, y, e.length_m, e.width_m, heading)
    return cells


def default_entities(seed: int = 20261016, sweep: LaneSweep | None = None) -> list[SurfaceEntity]:
    """Three entities, each a different lesson, positioned from the sweep.

      - a person walking across the ground ahead of the lanes. Large enough to be seen from the ground
        at tens of metres, and a case where the camera's view from the air is the harder one
      - a parked vehicle off to one side, far enough that the rover's camera barely resolves it, and away from the points
        the operator is likely to send the rover to
      - a buck moving slowly near the first lane: not a threat, and the thing a detector most often
        invents on rocks, so seeing a real one is the contrast
    """
    sweep = sweep if sweep is not None else sweep_from_first_scan(TerrainWorld(seed=seed))
    # Sizes are those of the figures drawn (render/figures.py): the whole outline, head and pack and roof rack included,
    # which is what a person drawing a box round it would draw.
    return [
        SurfaceEntity("person", "walker", 1.78, 0.54, 0.63, (sweep.at(1, 22.0, 12.0), sweep.at(1, 22.0, -4.0)), speed_ms=1.2, box_offset_m=-0.095),
        # someone carrying a rifle in the open, walking across the lanes ahead: the case the Arms class exists for
        SurfaceEntity("person", "armed walker", 1.79, 1.00, 0.68, (sweep.at(2, 17.0, -10.0), sweep.at(2, 24.0, 6.0)), speed_ms=1.1, box_offset_m=-0.098,
                      style="armed", carrying="long_gun"),
        SurfaceEntity("vehicle", "parked truck", 2.20, 2.12, 5.70, (sweep.at(3, 21.0, 14.0),), heading_deg=65.0),
        SurfaceEntity("animal", "buck", 1.91, 0.72, 2.14, (sweep.at(0, 10.0, 9.0), sweep.at(0, 16.0, 11.0), sweep.at(0, 12.0, 7.0)), speed_ms=0.5, box_offset_m=0.27, style="buck"),
        # an ibex, the animal of this country, on the ground the other side of the lanes: shorter, stockier and with a scimitar horn
        SurfaceEntity("animal", "ibex", 1.61, 0.52, 1.64, (sweep.at(1, 13.0, -9.0), sweep.at(1, 18.0, -11.0), sweep.at(1, 15.0, -7.5)), speed_ms=0.35, box_offset_m=0.215, style="ibex"),
    ]
