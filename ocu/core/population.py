"""What is out in the country beyond the track: the old sim's sites, its vehicle and people, and mines scattered across the whole scene.

The first clearance scenario (core/scenario.py, targets.py, entities.py) laid its content out from one straight sweep: five objects on the lanes, three
more sites of objects further out, and five figures (two walkers, a buck, an ibex, a parked truck). A chain of reconstructed trails has no such sweep, and
it had been given only a couple of objects per leg and people and animals kept off the track, with no vehicle and none of the sites. This puts the rest back,
placed from the chain: the three sites at a third of the way along it, half way and four fifths of the way, to alternate sides and well off the track; a
parked truck at one, an armed walker at the same, a civilian crossing another, an ibex at the third; a truck that drives a route across open ground; and a scatter of
buried and surface objects (mines, deep objects, scrap) over the whole world, some within reach of the track and the rest well out.

Deterministic from the seed. No Qt and no GL import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ocu.core.entities import SurfaceEntity, _passable, _truck_spot
from ocu.core.scenario import SITE_OBJECTS, Site, find_open_ground, nudge_to_open_ground
from ocu.core.targets import BuriedTarget
from ocu.core.terrain import WORLD_EDGE_MARGIN_M, TerrainWorld

#: Where along the chain each site goes (fraction of its length), which side of it (+1 right), and how far out (metres): well off the track, in reach of a rover that leaves it.
SITE_PLACES = {"B": (0.18, +1.0, 24.0), "C": (0.50, -1.0, 22.0), "D": (0.82, +1.0, 26.0)}
#: No site, vehicle or scattered object nearer the track than this (the track, its verge and the rover's picture of the real place; see trail_scenario.CLEAR_ZONE_M).
MIN_FROM_TRACK_M = 9.0
#: A truck that drives, and the route it drives: this long, at this speed (about 8 km/h), on ground a vehicle can cross, this far from the track.
TRUCK_ROUTE_M = 36.0
TRUCK_SPEED_MS = 2.2
TRUCK_ROUTE_FROM_TRACK_M = 14.0
TRUCK_CLEARANCE_M = 3.4  # open ground this far either side of the whole route (the truck is 2.2 m wide; the planner treats it as a circle of 2.8 m)
#: Scattered objects: how many, what share are within reach of the track (the rest are anywhere in the world), the range from the track of those, and the spacing.
SCATTER_COUNT = 28
SCATTER_NEAR_SHARE = 0.4
SCATTER_NEAR_RANGE_M = (7.0, 28.0)
SCATTER_SPACING_M = 6.0
WORLD_MARGIN_M = 12.0

#: The kinds of object scattered, each as the same description the sensor models read (core/targets.py): (weight, depth m, metal g, ferrous, vapour 0..1, what it is). Ranges are those of the
#: lane objects the first scenario used, so the same sensors meet the same cases: a shallow ferrous mine, a deep one the gradiometer cannot see, scrap to be rejected, one lying on
#: the surface, and a heavy deep one.
ARCHETYPES = (
    (0.36, (0.03, 0.10), (130.0, 200.0), True, (0.45, 0.65), "high_metal_shallow"),
    (0.20, (0.20, 0.30), (8.0, 14.0), False, (0.30, 0.40), "low_metal_deep"),
    (0.24, (0.08, 0.15), (200.0, 300.0), True, (0.0, 0.0), "inert_scrap"),
    (0.14, (0.015, 0.025), (90.0, 130.0), True, (0.25, 0.35), "surface_ahead"),
    (0.06, (0.40, 0.50), (700.0, 950.0), True, (0.08, 0.12), "heavy_deep"),
)


@dataclass
class Population:
    targets: list[BuriedTarget] = field(default_factory=list)
    entities: list[SurfaceEntity] = field(default_factory=list)
    sites: dict[str, Site] = field(default_factory=dict)


def _bearing_at(trail, index: int) -> float:
    tx, ty = trail.tangent[index]
    return math.degrees(math.atan2(tx, ty)) % 360.0


def place_sites(world: TerrainWorld, trail) -> dict[str, Site]:
    """The three sites, on open gentle ground well off the track, spread along the chain."""
    sites: dict[str, Site] = {}
    taken: list[tuple[float, float]] = []
    for name, (fraction, side, lateral) in SITE_PLACES.items():
        index = int(np.clip(np.searchsorted(trail.arc, fraction * trail.length_m), 0, len(trail.arc) - 1))
        for attempt_side, attempt_lateral in ((side, lateral), (-side, lateral), (side, lateral + 12.0), (-side, lateral + 12.0)):
            x, y = trail.point(float(trail.arc[index]), attempt_side * attempt_lateral)
            try:
                cx, cy = find_open_ground(world, x, y)
            except ValueError:
                continue
            if trail.distance_to_line(cx, cy) >= MIN_FROM_TRACK_M + 6.0 and all(math.hypot(cx - a, cy - b) > 25.0 for a, b in taken):
                sites[name] = Site(name, (cx, cy), _bearing_at(trail, index))
                taken.append((cx, cy))
                break
    # a site that none of its places would take goes anywhere in the world that has the room: well off the track, apart from the others
    rng = np.random.default_rng(world.seed ^ 0x517E5)
    limit = world.half_extent_m - WORLD_MARGIN_M - 8.0
    for name in SITE_PLACES:
        if name in sites:
            continue
        for _ in range(120):
            x, y = rng.uniform(-limit, limit, 2)
            if trail.distance_to_line(x, y) < MIN_FROM_TRACK_M + 6.0 or any(math.hypot(x - a, y - b) < 25.0 for a, b in taken):
                continue
            try:
                cx, cy = find_open_ground(world, float(x), float(y))
            except ValueError:
                continue
            if trail.distance_to_line(cx, cy) >= MIN_FROM_TRACK_M + 6.0 and all(math.hypot(cx - a, cy - b) > 25.0 for a, b in taken):
                index = int(trail.tree_nearest_index(cx, cy))
                sites[name] = Site(name, (cx, cy), _bearing_at(trail, index))
                taken.append((cx, cy))
                break
    return sites


def site_entities(world: TerrainWorld, sites: dict[str, Site]) -> list[SurfaceEntity]:
    """The people, the animal and the parked truck that stand at the sites (the first scenario's, with the names of the sites)."""
    out: list[SurfaceEntity] = []
    if "B" in sites:
        b = sites["B"]
        out.append(SurfaceEntity("person", "site B armed walker", 1.79, 1.00, 0.68, (b.at(9.0, -9.0), b.at(11.0, 8.0)), speed_ms=0.9, box_offset_m=-0.098, style="armed", carrying="long_gun"))
        out.append(SurfaceEntity("vehicle", "site B truck", 2.20, 2.12, 5.70, (_truck_spot(world, b),), heading_deg=65.0))
    if "C" in sites:
        c = sites["C"]
        out.append(SurfaceEntity("person", "site C walker", 1.78, 0.54, 0.63, (c.at(-8.0, 6.0), c.at(8.0, -6.0), c.at(2.0, 9.0)), speed_ms=1.2, box_offset_m=-0.095))
        out.append(SurfaceEntity("person", "site C walker 2", 1.78, 0.54, 0.63, (c.at(6.0, 10.0), c.at(-5.0, -9.0)), speed_ms=1.0, box_offset_m=-0.095))
    if "D" in sites:
        d = sites["D"]
        out.append(SurfaceEntity("animal", "site D ibex", 1.61, 0.52, 1.64, (d.at(8.0, -9.0), d.at(13.0, -11.0), d.at(10.0, -6.0)), speed_ms=0.35, box_offset_m=0.215, style="ibex"))
        out.append(SurfaceEntity("animal", "site D buck", 1.91, 0.72, 2.14, (d.at(-9.0, 8.0), d.at(-4.0, 12.0)), speed_ms=0.5, box_offset_m=0.27, style="buck"))
    return out


ROAD_OFFSET_M = 18.0
ROAD_HALF_WIDTH_M = 2.2
ROAD_CLEAR_M = 4.5


def service_road(world: TerrainWorld, trail, rng: np.random.Generator) -> tuple[tuple[float, float], ...] | None:
    """A dirt road parallel to the track, a stretch of it about `ROAD_OFFSET_M` off, for the truck to drive: the obstacles along it are cleared and its ground is track, so that it looks like a road
    in the scouts' pictures and a rover can follow it. None if nowhere along the chain has room for one that stays clear of the rest of the track."""
    limit = world.half_extent_m - WORLD_EDGE_MARGIN_M - 6.0
    for fraction in (0.34, 0.66, 0.14, 0.9, 0.5, 0.78):
        for side in (1.0, -1.0):
            start = fraction * trail.length_m
            points = [trail.point(float(a), side * ROAD_OFFSET_M) for a in np.arange(start, start + TRUCK_ROUTE_M, 3.0)]
            if any(abs(x) > limit or abs(y) > limit for x, y in points):
                continue
            if min(trail.distance_to_line(x, y) for x, y in points) < ROAD_OFFSET_M - 5.0:
                continue  # it would come near another stretch of the chain
            _carve_road(world, points)
            return tuple(points)
    return None


def _carve_road(world: TerrainWorld, points: list[tuple[float, float]]) -> None:
    """Clear the world along a polyline and make its ground track."""
    from ocu.core.terrain import Terrain

    cell = world.cell_size_m
    axis = (np.arange(world._span) - world._extent + 0.5) * cell
    gx, gy = np.meshgrid(axis, axis, indexing="ij")
    dense = []
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        n = max(2, int(math.dist((x0, y0), (x1, y1)) / 1.0))
        dense += [(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f) for f in np.linspace(0.0, 1.0, n, endpoint=False)]
    dense.append(points[-1])
    from scipy.spatial import cKDTree

    tree = cKDTree(np.array(dense))
    d, _ = tree.query(np.stack([gx.ravel(), gy.ravel()], axis=1))
    d = d.reshape(gx.shape)
    zone = d <= ROAD_CLEAR_M
    world._grid[zone & np.isin(world._grid, [int(Terrain.ROCK), int(Terrain.TREE), int(Terrain.BUSH), int(Terrain.POTHOLE), int(Terrain.SCRUB)])] = int(Terrain.GROUND)
    world._grid[d <= ROAD_HALF_WIDTH_M] = int(Terrain.TRACK)
    world._rock_height[zone] = 0.0
    world._pothole_depth[zone] = 0.0
    if world.gravel is not None:
        world.gravel[zone] = 0.0
    for name, radius_col in (("boulders", 2), ("trees", 3), ("bushes", 2)):
        rows = getattr(world, name)
        if len(rows):
            dist, _ = tree.query(rows[:, :2])
            setattr(world, name, rows[dist > ROAD_CLEAR_M + rows[:, radius_col]])
    if len(world.pebbles):
        dist, _ = tree.query(world.pebbles[:, :2])
        world.pebbles = world.pebbles[dist > ROAD_HALF_WIDTH_M]


def moving_truck(world: TerrainWorld, trail, sites: dict[str, Site], rng: np.random.Generator) -> SurfaceEntity | None:
    """A truck that drives back and forth along a straight route of open ground, well off the track: found by trying routes round the sites and the chain."""
    road = service_road(world, trail, rng)
    if road is not None:
        return SurfaceEntity("vehicle", "patrol truck", 2.20, 2.12, 5.70, road, speed_ms=TRUCK_SPEED_MS)
    anchors = [s.centre for s in sites.values()]
    for fraction in (0.35, 0.65, 0.12, 0.9):
        index = int(np.clip(np.searchsorted(trail.arc, fraction * trail.length_m), 0, len(trail.arc) - 1))
        anchors.append(trail.point(float(trail.arc[index]), (TRUCK_ROUTE_FROM_TRACK_M + 10.0) * (1 if fraction > 0.5 else -1)))
    limit_world = world.half_extent_m - WORLD_MARGIN_M - TRUCK_ROUTE_M / 2
    anchors += [tuple(rng.uniform(-limit_world, limit_world, 2)) for _ in range(400)]  # then anywhere in the world: the ground round the sites and the track is not always clear enough
    best = None
    for anchor in anchors:
        for angle in np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False) + rng.uniform(0.0, 0.5):
            dx, dy = math.sin(angle), math.cos(angle)
            start = (anchor[0] - dx * TRUCK_ROUTE_M / 2, anchor[1] - dy * TRUCK_ROUTE_M / 2)
            end = (anchor[0] + dx * TRUCK_ROUTE_M / 2, anchor[1] + dy * TRUCK_ROUTE_M / 2)
            points = [(start[0] + (end[0] - start[0]) * f, start[1] + (end[1] - start[1]) * f) for f in np.linspace(0.0, 1.0, int(TRUCK_ROUTE_M / 2.0) + 1)]
            limit = world.half_extent_m - WORLD_EDGE_MARGIN_M - 4.0
            if any(abs(x) > limit or abs(y) > limit for x, y in points):
                continue
            if any(trail.distance_to_line(x, y) < TRUCK_ROUTE_FROM_TRACK_M for x, y in points):
                continue
            if all(_passable(world, x, y, TRUCK_CLEARANCE_M) for x, y in points):
                # a route that can be driven; the nearest the track is the best (the rover and the scouts see it), so keep looking among the rest
                nearness = float(np.mean([trail.distance_to_line(x, y) for x, y in points]))
                if best is None or nearness < best[0]:
                    best = (nearness, start, end)
    return None if best is None else SurfaceEntity("vehicle", "patrol truck", 2.20, 2.12, 5.70, (best[1], best[2]), speed_ms=TRUCK_SPEED_MS)


def scattered_targets(world: TerrainWorld, trail, rng: np.random.Generator, existing: list[BuriedTarget]) -> list[BuriedTarget]:
    """Buried and surface objects over the whole world: some within reach of the track, the rest anywhere, none on it or on top of each other."""
    limit = world.half_extent_m - WORLD_MARGIN_M
    weights = np.array([a[0] for a in ARCHETYPES])
    weights = weights / weights.sum()
    placed: list[BuriedTarget] = []
    occupied = [(t.x, t.y) for t in existing]
    attempts = 0
    while len(placed) < SCATTER_COUNT and attempts < SCATTER_COUNT * 60:
        attempts += 1
        if rng.random() < SCATTER_NEAR_SHARE:
            index = int(rng.integers(0, len(trail.arc)))
            lateral = rng.uniform(*SCATTER_NEAR_RANGE_M) * rng.choice([-1.0, 1.0])
            x, y = trail.point(float(trail.arc[index]), lateral)
        else:
            x, y = rng.uniform(-limit, limit, 2)
        if abs(x) > limit or abs(y) > limit or trail.distance_to_line(x, y) < MIN_FROM_TRACK_M - 2.0:
            continue
        try:
            x, y = nudge_to_open_ground(world, x, y)
        except ValueError:
            continue
        if trail.distance_to_line(x, y) < MIN_FROM_TRACK_M - 2.0 or any(math.hypot(x - a, y - b) < SCATTER_SPACING_M for a, b in occupied):
            continue
        _, depth, mass, ferrous, voc, truth = ARCHETYPES[int(rng.choice(len(ARCHETYPES), p=weights))]
        placed.append(BuriedTarget(float(x), float(y), float(rng.uniform(*depth)), float(rng.uniform(*mass)), bool(ferrous), float(rng.uniform(*voc)) if voc[1] > 0 else 0.0, truth))
        occupied.append((x, y))
    return placed


def populate(world: TerrainWorld, trail, existing_targets: list[BuriedTarget] | None = None, seed: int | None = None) -> Population:
    """Everything beyond the track: the sites with their objects, people, animal and truck, the truck that drives, and the scatter."""
    rng = np.random.default_rng((world.seed if seed is None else seed) ^ 0xA11CE)
    population = Population()
    population.sites = place_sites(world, trail)
    population.entities = site_entities(world, population.sites)
    truck = moving_truck(world, trail, population.sites, rng)
    if truck is not None:
        population.entities.append(truck)
    # the objects of each site that could be placed (a site with no open ground in reach leaves its objects out, rather than somewhere they would not belong)
    for name, site in population.sites.items():
        for forward, right, depth, mass, ferrous, voc, truth in SITE_OBJECTS[name]:
            try:
                x, y = nudge_to_open_ground(world, *site.at(forward, right))
            except ValueError:
                continue
            population.targets.append(BuriedTarget(x, y, depth, mass, ferrous, voc, truth))
    population.targets += scattered_targets(world, trail, rng, list(existing_targets or []) + population.targets)
    return population
