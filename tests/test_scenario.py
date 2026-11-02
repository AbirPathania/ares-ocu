"""The hazards are spread across the operating area, on ground a vehicle can reach."""

import math

import numpy as np
import pytest

from ocu.core.entities import default_entities, settle_entities
from ocu.core.scenario import SITE_OBJECTS, SITE_PLACES, find_open_ground, place_sites
from ocu.core.sweep import cut_washouts_on_lanes, sweep_from_first_scan
from ocu.core.targets import default_field
from ocu.core.terrain import IMPASSABLE, TerrainWorld


@pytest.fixture(scope="module")
def scene():
    world = TerrainWorld()
    sweep = sweep_from_first_scan(world)
    cut_washouts_on_lanes(world, sweep)
    return world, sweep, default_field(sweep=sweep, world=world), settle_entities(world, default_entities(sweep=sweep, world=world))


def test_the_targets_are_in_several_places_not_one(scene):
    _world, _sweep, field, _entities = scene
    points = np.array([(t.x, t.y) for t in field.targets])
    centre = points.mean(axis=0)
    assert np.max(np.hypot(*(points - centre).T)) > 40.0, "spread over the area"
    # Found as groups: single linkage at 15 m gives at least four separate sites.
    groups = []
    for p in points:
        for g in groups:
            if min(math.dist(p, q) for q in g) < 15.0:
                g.append(tuple(p))
                break
        else:
            groups.append([tuple(p)])
    assert len(groups) >= 4, f"{len(groups)} sites"


def test_every_site_is_a_different_distance_and_direction_from_the_deploy_point(scene):
    world, sweep, _field, _entities = scene
    sites = place_sites(world, sweep)
    bearings = sorted(math.degrees(math.atan2(*s.centre)) % 360.0 for s in sites.values())
    gaps = [(b - a) for a, b in zip(bearings, bearings[1:])] + [bearings[0] + 360.0 - bearings[-1]]
    assert min(gaps) > 40.0, "no two sites on the same side"
    assert all(30.0 < math.hypot(*s.centre) < 95.0 for s in sites.values())


def test_every_target_can_be_reached(scene):
    world, _sweep, field, _entities = scene
    for t in field.targets:
        assert world.classify(t.x, t.y) not in IMPASSABLE, t.truth_class
        assert abs(t.x) < world.half_extent_m and abs(t.y) < world.half_extent_m


def test_a_site_is_open_ground(scene):
    world, sweep, _field, _entities = scene
    for site in place_sites(world, sweep).values():
        for angle in range(0, 360, 30):
            x = site.centre[0] + 5.0 * math.sin(math.radians(angle))
            y = site.centre[1] + 5.0 * math.cos(math.radians(angle))
            assert world.classify(x, y) not in IMPASSABLE


def test_the_sites_teach_different_things(scene):
    _world, _sweep, field, _entities = scene
    classes = {t.truth_class for t in field.targets}
    assert {"crossing_deep", "pattern_1", "heavy_deep", "scrap_d", "crossing_surface"} <= classes
    depths = [t.depth_m for t in field.targets]
    assert max(depths) > 0.4 and min(depths) < 0.03
    assert sum(1 for t in field.targets if t.depth_m <= 0.10) >= 3, "a scout can see some of them from the air"


def test_the_pattern_is_a_row(scene):
    _world, _sweep, field, _entities = scene
    row = np.array([(t.x, t.y) for t in field.targets if t.truth_class.startswith("pattern")])
    assert len(row) == len(SITE_OBJECTS["C"]) == 4
    centred = row - row.mean(axis=0)
    # all within half a metre of one straight line
    _u, _s, vt = np.linalg.svd(centred)
    off_line = np.abs(centred @ vt[1])
    assert off_line.max() < 0.8


def test_people_vehicles_and_animals_are_in_more_than_one_place(scene):
    _world, _sweep, _field, entities = scene
    positions = np.array([e.path[0] for e in entities])
    assert np.max(np.hypot(*(positions - positions.mean(axis=0)).T)) > 25.0


def test_the_scenario_is_the_same_every_time():
    a = default_field(world=TerrainWorld())
    b = default_field(world=TerrainWorld())
    assert [(t.x, t.y, t.truth_class) for t in a.targets] == [(t.x, t.y, t.truth_class) for t in b.targets]


def test_find_open_ground_moves_off_a_blocked_place():
    world = TerrainWorld()
    # A place in the middle of a boulder: the answer is somewhere else, and open.
    x, y, *_ = world.boulders[0]
    ox, oy = find_open_ground(world, x, y)
    assert math.hypot(ox - x, oy - y) > 1.0
