"""Where the hazards are: four sites spread across the operating area, not one block of lanes.

Everything used to be laid out against the lane sweep, a block sixteen metres by twenty-four beside the deploy
point, so a demonstration of finding hazards across a search area found them all within a few steps of each
other. A real task is a place to be cleared with hazards in several parts of it, some of which a scout sees
from the air long before the ground vehicle can reach them. So the scenario has sites:

  * A, the lanes: the original block, where the ground suite earns its keep (a strong shallow object, a hard
    deep one, scrap to reject, one the swath misses, one lying on the surface ahead);
  * B, a crossing roughly 45 m out: a cluster beside the worn track, with someone watching it and a parked truck;
  * C, a pattern roughly 65 m out the other way: a regular row of shallow objects, the way a laid pattern
    looks, with one set off to the side;
  * D, scattered ground roughly 80 m out behind the deploy point: a deep heavy object, scrap, and animals and a
    civilian on the move, so that not everything out there is a threat.

Sites are found from the ground, not from fixed coordinates: each is placed on open ground nearest to the place
asked for, because the world is generated and a site inside a rock field would be one nobody could reach.
Positions inside a site are relative to its centre and to the sweep's axis, so the scenario is the same
whichever way the sweep happened to be laid. Nothing here is random; the numbers are this build's choices.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.sweep import LaneSweep
from ocu.core.terrain import IMPASSABLE, WORLD_EDGE_MARGIN_M, TerrainWorld

#: Where each site is asked for: bearing from the sweep's axis (clockwise, degrees) and distance from the deploy
#: point, metres. The nearest open ground to this is used.
SITE_PLACES = {
    "B": (62.0, 46.0),
    "C": (-78.0, 64.0),
    "D": (172.0, 80.0),
}

#: Open ground a site needs: this radius round its centre with no rock, tree, bush or hole in it.
SITE_RADIUS_M = 7.0
SEARCH_REACH_M = 60.0


@dataclass(frozen=True)
class Site:
    name: str
    centre: tuple[float, float]
    axis_deg: float  # the sweep's bearing: local "forward"

    def at(self, forward_m: float, right_m: float) -> tuple[float, float]:
        """A point in the site's own frame: along the sweep's axis, and across it to the right."""
        a = math.radians(self.axis_deg)
        fx, fy = math.sin(a), math.cos(a)
        rx, ry = math.cos(a), -math.sin(a)
        return (self.centre[0] + fx * forward_m + rx * right_m, self.centre[1] + fy * forward_m + ry * right_m)


def _open(world: TerrainWorld, x: float, y: float, radius: float) -> bool:
    cell = world.cell_size_m
    reach = int(math.ceil(radius / cell))
    cx, cy = world.to_cell(x, y)
    for ix in range(cx - reach, cx + reach + 1):
        for iy in range(cy - reach, cy + reach + 1):
            if math.hypot((ix + 0.5) * cell - x, (iy + 0.5) * cell - y) <= radius and world.at((ix, iy)) in IMPASSABLE:
                return False
    return True


#: A site is gentle ground: no more than this much height across it. Hazards on a hillside are a different
#: scenario, and a parked truck needs the ground level (entities.py).
SITE_MAX_RELIEF_M = 0.9
SITE_RELIEF_RADIUS_M = 4.0


def _relief(world: TerrainWorld, x: float, y: float, radius: float) -> float:
    cell = world.cell_size_m
    cx, cy = world.to_cell(x, y)
    reach = int(radius / cell)
    heights = [world.elevation_at((ix, iy)) for ix in range(cx - reach, cx + reach + 1, 2) for iy in range(cy - reach, cy + reach + 1, 2)]
    return max(heights) - min(heights)


def find_open_ground(world: TerrainWorld, x: float, y: float, radius: float = SITE_RADIUS_M) -> tuple[float, float]:
    """The nearest point to (x, y) with `radius` of open, gentle ground round it, searched in rings."""
    limit = world.half_extent_m - WORLD_EDGE_MARGIN_M - radius
    ring = 0.0
    while ring <= SEARCH_REACH_M:
        count = 1 if ring == 0.0 else max(8, int(2.0 * math.pi * ring / 2.0))
        for k in range(count):
            a = 2.0 * math.pi * k / count
            px, py = x + ring * math.sin(a), y + ring * math.cos(a)
            if abs(px) <= limit and abs(py) <= limit and math.hypot(px, py) > 20.0 and _open(world, px, py, radius) and _relief(world, px, py, SITE_RELIEF_RADIUS_M) <= SITE_MAX_RELIEF_M:
                return px, py
        ring += 2.0
    raise ValueError(f"no open ground within {SEARCH_REACH_M:.0f} m of ({x:.0f}, {y:.0f})")


def place_sites(world: TerrainWorld, sweep: LaneSweep) -> dict[str, Site]:
    """The sites B, C and D, on open ground. (A is the lanes themselves, and is placed from the sweep.)"""
    sites = {}
    for name, (bearing_off, distance) in SITE_PLACES.items():
        a = math.radians(sweep.bearing_deg + bearing_off)
        wanted = (sweep.origin[0] + distance * math.sin(a), sweep.origin[1] + distance * math.cos(a))
        sites[name] = Site(name, find_open_ground(world, *wanted), sweep.bearing_deg)
    return sites


def nudge_to_open_ground(world: TerrainWorld, x: float, y: float, clearance_m: float = 1.6) -> tuple[float, float]:
    """The nearest point to (x, y) that is passable with `clearance_m` round it, so that whatever is placed there is
    something the rover can actually drive to and a rock is not standing on it."""
    ring = 0.0
    while ring <= 12.0:
        count = 1 if ring == 0.0 else max(8, int(2.0 * math.pi * ring / 0.8))
        for k in range(count):
            a = 2.0 * math.pi * k / count
            px, py = x + ring * math.sin(a), y + ring * math.cos(a)
            if _open(world, px, py, clearance_m):
                return px, py
        ring += 0.5
    raise ValueError(f"no open ground near ({x:.1f}, {y:.1f})")


#: What lies at each site, in the site's frame: (metres forward, metres right, depth, metal in grams, ferrous,
#: vapour 0..1, and what it truly is). These are the same kind of objects targets.py has always described, with the
#: same properties meaning the same things to the sensor models.
SITE_OBJECTS = {
    "B": (
        # a pair of shallow ferrous objects either side of the track's edge
        (0.0, 0.0, 0.06, 190.0, True, 0.55, "crossing_shallow_a"),
        (4.5, 2.5, 0.07, 160.0, True, 0.50, "crossing_shallow_b"),
        # the hard case again, off to one side: deep, barely metallic, non-ferrous
        (-3.5, 4.0, 0.24, 12.0, False, 0.30, "crossing_deep"),
        # laid on the surface, where a scout can see it from the air
        (-5.5, -2.5, 0.02, 100.0, True, 0.30, "crossing_surface"),
    ),
    # a regular row, three metres apart, with the gap a real pattern has
    "C": (
        (-4.5, 0.10, 0.06, 95.0, True, 0.60, "pattern_1"),
        (-1.5, -0.12, 0.08, 120.0, False, 0.55, "pattern_2"),
        (4.5, 0.08, 0.05, 105.0, True, 0.62, "pattern_4"),
        (7.5, 0.00, 0.07, 88.0, False, 0.58, "pattern_5"),
    ),
    "D": (
        # big, heavy and deep: the gradiometer reaches it where induction barely does
        (0.0, 0.0, 0.46, 900.0, True, 0.10, "heavy_deep"),
        # scrap on the same ground, to be rejected
        (-6.0, 5.5, 0.10, 240.0, True, 0.0, "scrap_d"),
    ),
}


def site_targets(world: TerrainWorld, sites: dict[str, Site]):
    """The buried and surface objects at sites B, C and D, as rows ready to become `BuriedTarget`s."""
    rows = []
    for name, objects in SITE_OBJECTS.items():
        site = sites[name]
        for forward, right, depth, mass, ferrous, voc, truth in objects:
            x, y = nudge_to_open_ground(world, *site.at(forward, right))
            rows.append((x, y, depth, mass, ferrous, voc, truth))
    return rows
