"""Parked things are obstacles the LiDAR maps; walking things are obstacles the rover is told about; and
neither is ever driven through."""

import math

import numpy as np
import pytest

from ocu.core import dynamic_obstacles as dyn
from ocu.core.entities import SurfaceEntity, default_entities, settle_entities, stamp_static_entities
from ocu.core.lidar import cast_scan
from ocu.core.local_planner import FOOTPRINT_CLEARANCE_M, LocalPlanner
from ocu.core.terrain import IMPASSABLE, PerceptionMap, Terrain, TerrainWorld


def truck(x=10.0, y=0.0, heading=0.0):
    return SurfaceEntity("vehicle", "truck", 1.85, 1.9, 5.3, ((x, y),), heading_deg=heading)


def walker(path=((12.0, 0.0), (12.0, 10.0)), speed=1.2, kind="person"):
    return SurfaceEntity(kind, "w", 1.75, 0.5, 0.3, path, speed_ms=speed)


# ---- a parked vehicle is part of the ground truth

def test_a_stamped_footprint_covers_the_whole_vehicle_and_a_little_more():
    w = TerrainWorld.blank()
    x, y, length, width = 10.0, 0.0, 5.3, 1.9
    cells = w.stamp_object(x, y, length, width, 0.0)
    assert cells and all(w.at(c) == Terrain.ROCK for c in cells)
    for dx in np.linspace(-width / 2, width / 2, 5):
        for dy in np.linspace(-length / 2, length / 2, 9):
            assert w.at(w.to_cell(x + dx, y + dy)) in IMPASSABLE, f"({dx:.2f}, {dy:.2f}) of the truck is not blocked"
    assert w.at(w.to_cell(x + width / 2 + 1.5, y)) == Terrain.GROUND, "the footprint is not grown without limit"


def test_the_footprint_follows_the_heading():
    w = TerrainWorld.blank()
    w.stamp_object(10.0, 0.0, 5.3, 1.9, 90.0)  # lying east to west
    assert w.at(w.to_cell(12.4, 0.0)) == Terrain.ROCK and w.at(w.to_cell(7.6, 0.0)) == Terrain.ROCK
    assert w.at(w.to_cell(10.0, 2.4)) == Terrain.GROUND


def test_a_lidar_scan_returns_off_a_parked_truck_and_the_map_marks_it_as_an_obstacle():
    w = TerrainWorld.blank()
    ents = [truck(10.0, 0.0, 0.0)]
    stamp_static_entities(w, ents)
    perception = PerceptionMap()
    cast_scan(w, perception, 0.0, 0.0, seed=1)
    assert perception.at(w.to_cell(8.8, 0.0)) == Terrain.ROCK, "the near face is seen, and mapped as something to drive round"
    assert perception.at(w.to_cell(14.0, 0.0)) == Terrain.UNKNOWN, "the ground directly behind it, on the same bearing, is never measured"


def test_only_parked_entities_are_stamped():
    w = TerrainWorld.blank()
    assert stamp_static_entities(w, [walker(), truck(30.0, 5.0)])
    assert w.at(w.to_cell(12.0, 5.0)) == Terrain.GROUND, "a walker's path is not stamped"


# ---- placement on ground that can hold them

def test_a_vehicle_asked_to_park_in_a_stand_of_trees_is_moved_to_clear_level_ground():
    w = TerrainWorld.blank()
    for ix in range(14, 30):
        for iy in range(-6, 6):
            w.set_cell((ix, iy), Terrain.TREE)
    out = settle_entities(w, [truck(10.0, 0.0)])[0]
    x, y, _ = out.pose_at(0.0)
    assert (x, y) != (10.0, 0.0)
    for dx in np.linspace(-1, 1, 5):
        for dy in np.linspace(-2.7, 2.7, 9):
            assert w.at(w.to_cell(x + dx, y + dy)) not in IMPASSABLE


def test_a_walkers_whole_path_is_cleared_of_rock_and_trees_and_moved_if_it_is_not():
    w = TerrainWorld.blank()
    for iy in range(8, 14):
        for ix in range(22, 28):
            w.set_cell((ix, iy), Terrain.ROCK)
    out = settle_entities(w, [walker(((12.0, 0.0), (12.0, 10.0)))])[0]
    for f in np.linspace(0, 1, 40):
        x = out.path[0][0] + (out.path[1][0] - out.path[0][0]) * f
        y = out.path[0][1] + (out.path[1][1] - out.path[0][1]) * f
        assert w.at(w.to_cell(x, y)) not in IMPASSABLE


def test_nothing_is_placed_on_the_deploy_point_and_an_impossible_scenario_says_so():
    w = TerrainWorld.blank()
    assert math.hypot(*settle_entities(w, [truck(0.0, 0.0)])[0].path[0]) >= 7.0
    full = TerrainWorld.blank(half_extent_m=30.0)
    for ix in range(-60, 61):
        for iy in range(-60, 61):
            full.set_cell((ix, iy), Terrain.ROCK)
    with pytest.raises(ValueError):
        settle_entities(full, [truck(12.0, 0.0)])


def test_the_default_scenario_is_valid_on_the_real_terrain():
    w = TerrainWorld()
    for e in settle_entities(w, default_entities()):
        x, y, _ = e.pose_at(0.0)
        assert math.hypot(x, y) >= 7.0
        if e.static:
            assert w.at(w.to_cell(x, y)) not in IMPASSABLE


# ---- walking things

def test_the_rover_notices_only_what_its_lidar_could_see():
    w = TerrainWorld.blank()
    d = dyn.DynamicObstacles()
    d.update(0.0, (0.0, 0.0), [("near", 10.0, 0.0, 0.4), ("far", 80.0, 0.0, 0.4)], w)
    assert [c.name for c in d.contacts] == ["near"], "beyond the LiDAR's reach nothing is seen"
    for iy in range(-6, 7):
        w.set_cell((10, iy), Terrain.ROCK)  # a wall between the rover and (20, 0)
    d2 = dyn.DynamicObstacles()
    d2.update(0.0, (0.0, 0.0), [("hidden", 20.0, 0.0, 0.4)], w)
    assert d2.contacts == [], "a person behind a rock is not seen"


def test_a_contact_is_followed_forgotten_and_given_a_velocity():
    w = TerrainWorld.blank()
    d = dyn.DynamicObstacles()
    for k in range(6):
        d.update(k * 0.5, (0.0, 0.0), [("p", 10.0, k * 0.6, 0.4)], w)  # 1.2 m/s northward
    c = d.contacts[0]
    assert c.vy == pytest.approx(1.2, abs=0.05) and abs(c.vx) < 0.05
    x, y = c.predicted(2.5 + 1.0, 0.0)
    assert y == pytest.approx(3.0 + 1.2, abs=0.2), "held for a second, it is carried along at its velocity"
    d.update(2.5 + dyn.MEMORY_S + 0.1, (0.0, 0.0), [], w)
    assert d.contacts == [], "and forgotten once it has not been seen for the memory"


def test_the_planner_is_given_the_contact_now_and_where_it_is_going():
    w = TerrainWorld.blank()
    d = dyn.DynamicObstacles()
    for k in range(4):
        d.update(k * 0.5, (0.0, 0.0), [("p", 10.0, k * 0.6, 0.4)], w)
    pts = d.points(1.5)
    assert pts.shape[1] == 2 and len(pts) > 50
    assert pts[:, 1].max() > 1.8 + 2.0 * 1.2 - 0.1 and pts[:, 1].min() < 1.8 + 0.4, "from where it is to where it will be in two seconds"
    assert d.nearest_gap_m(10.0, 5.0, 1.5) == pytest.approx(5.0 - 1.8 - 0.4, abs=0.4)
    assert dyn.DynamicObstacles().nearest("x", 0.0, 0.0) if False else dyn.DynamicObstacles().nearest(0.0, 0.0, 0.0) is None


def test_a_disc_is_filled_and_has_a_rim():
    pts = dyn.disc_points(5.0, 5.0, 1.0)
    r = np.hypot(pts[:, 0] - 5.0, pts[:, 1] - 5.0)
    assert r.max() <= 1.0 + 1e-9 and r.min() < 0.1
    assert (r > 0.98).sum() >= 12


@pytest.mark.parametrize("seed", range(12))
def test_the_reactive_planner_never_aims_through_a_person_standing_in_the_way(seed):
    rng = np.random.default_rng(seed)
    d = rng.uniform(4.2, 6.0)
    lateral = rng.uniform(-0.3, 0.3)
    pts = dyn.disc_points(lateral, d, 0.4 + dyn.MOVING_MARGIN_M)
    cmd = LocalPlanner().command(pts, 0.0, 0.0, 0.0, 0.0, 2.5, 1.0, 2.5)  # at 2.5 m/s it looks about 4.6 m ahead
    rad = math.radians(cmd.heading_deg)
    for step in np.arange(0.25, min(cmd.free_m, 6.0), 0.25):
        px, py = math.sin(rad) * step, math.cos(rad) * step
        assert np.min(np.hypot(pts[:, 0] - px, pts[:, 1] - py)) >= FOOTPRINT_CLEARANCE_M - 0.02
    assert cmd.deviating, "a person in the way makes it turn aside"


def test_obstacle_radii_are_in_the_planners_hands_and_sensible():
    from ocu.core.entities import OBSTACLE_RADIUS_M

    assert OBSTACLE_RADIUS_M["person"] < OBSTACLE_RADIUS_M["animal"] < OBSTACLE_RADIUS_M["vehicle"]
