"""Perception has to behave like a sensor, not like a drawing.

The tests that matter here are the ones about what the rover does *not*
know: nothing before it scans, nothing beyond its range, and nothing
behind an obstacle until it has moved far enough to see round it.
"""

import math

from ocu.core.lidar import LIDAR_RANGE_M, cast_scan, has_line_of_sight
from ocu.core.terrain import (
    CELL_SIZE_M,
    IMPASSABLE,
    OCCLUDING,
    PerceptionMap,
    Terrain,
    TerrainWorld,
)


def blank_world() -> TerrainWorld:
    """Open ground, nothing in it, for controlled occlusion tests."""
    world = TerrainWorld(_cells={"placeholder": Terrain.GROUND})
    world._cells = {}
    return world


# --- the map starts empty -------------------------------------------------


def test_nothing_is_known_before_a_scan():
    perception = PerceptionMap()
    assert len(perception) == 0
    assert perception.at((0, 0)) is Terrain.UNKNOWN
    assert not perception.is_known((0, 0))


def test_a_scan_reveals_ground_around_the_vehicle():
    world, perception = blank_world(), PerceptionMap()
    result = cast_scan(world, perception, 0.0, 0.0)
    assert result.new_cells > 0
    assert perception.is_known((1, 0))
    assert perception.at((1, 0)) is Terrain.GROUND


def test_nothing_beyond_sensor_range_is_known():
    world, perception = blank_world(), PerceptionMap()
    cast_scan(world, perception, 0.0, 0.0)
    far = int((LIDAR_RANGE_M + 15.0) / CELL_SIZE_M)
    assert not perception.is_known((far, 0))


def test_rescanning_the_same_spot_adds_nothing_new():
    world, perception = blank_world(), PerceptionMap()
    first = cast_scan(world, perception, 0.0, 0.0)
    second = cast_scan(world, perception, 0.0, 0.0)
    assert first.new_cells > 0
    assert second.new_cells == 0


# --- occlusion: the part that makes it perception -------------------------


def test_a_rock_casts_a_shadow_the_rover_cannot_see_into():
    world, perception = blank_world(), PerceptionMap()
    # A wall of rock due north at 5 m.
    for dx in range(-6, 7):
        world._cells[(dx, 10)] = Terrain.ROCK

    cast_scan(world, perception, 0.0, 0.0)

    assert perception.at((0, 10)) is Terrain.ROCK, "the obstacle itself is seen"
    assert not perception.is_known((0, 16)), "ground directly behind it is not"
    assert not perception.is_known((0, 24))


def test_driving_around_an_obstacle_fills_in_its_shadow():
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-6, 7):
        world._cells[(dx, 10)] = Terrain.ROCK

    cast_scan(world, perception, 0.0, 0.0)
    assert not perception.is_known((0, 16))

    # Drive well past the end of the wall and look back.
    cast_scan(world, perception, 12.0, 10.0)
    assert perception.is_known((0, 16)), "the shadow closes once it can be seen into"


def test_trees_occlude_and_rocks_occlude():
    for kind in (Terrain.TREE, Terrain.ROCK):
        world, perception = blank_world(), PerceptionMap()
        for dx in range(-8, 9):
            world._cells[(dx, 8)] = kind
        cast_scan(world, perception, 0.0, 0.0)
        assert perception.at((0, 8)) is kind
        assert not perception.is_known((0, 20)), f"{kind.name} must block the beam"


def test_scrub_does_not_occlude():
    """Low vegetation is driven over and seen past; it is not a wall."""
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-8, 9):
        world._cells[(dx, 8)] = Terrain.SCRUB
    cast_scan(world, perception, 0.0, 0.0)
    assert perception.at((0, 8)) is Terrain.SCRUB
    assert perception.is_known((0, 20)), "scrub must not cast a shadow"


def test_line_of_sight_agrees_with_the_scan():
    world = blank_world()
    for dx in range(-6, 7):
        world._cells[(dx, 10)] = Terrain.ROCK
    assert has_line_of_sight(world, 0.0, 0.0, 0.0, 4.0)
    assert not has_line_of_sight(world, 0.0, 0.0, 0.0, 9.0)


# --- the generated world --------------------------------------------------


def test_the_deploy_point_is_clear_ground():
    """A rover does not start inside a tree."""
    world = TerrainWorld()
    assert world.classify(0.0, 0.0) not in IMPASSABLE


def test_the_world_contains_every_class():
    world = TerrainWorld()
    present = set(world._cells.values())
    for kind in (Terrain.TRACK, Terrain.SCRUB, Terrain.TREE, Terrain.ROCK):
        assert kind in present, f"generated world has no {kind.name}"


def test_the_world_is_deterministic_for_a_given_seed():
    assert TerrainWorld(seed=7)._cells == TerrainWorld(seed=7)._cells


def test_different_seeds_give_different_ground():
    assert TerrainWorld(seed=1)._cells != TerrainWorld(seed=2)._cells


def test_occluding_and_impassable_agree():
    """Anything that stops the beam is also something you cannot drive through."""
    assert OCCLUDING == IMPASSABLE


def test_coverage_grows_as_the_vehicle_drives():
    world, perception = TerrainWorld(), PerceptionMap()
    cast_scan(world, perception, 0.0, 0.0)
    first = perception.coverage_m2()
    cast_scan(world, perception, 20.0, 20.0)
    assert perception.coverage_m2() > first


def test_coverage_is_reported_in_square_metres():
    perception = PerceptionMap()
    perception.observe((0, 0), Terrain.GROUND)
    assert math.isclose(perception.coverage_m2(), CELL_SIZE_M**2)


# --- returns behave like measurements, not fill ---------------------------


def test_a_scan_produces_discrete_returns():
    world, perception = blank_world(), PerceptionMap()
    result = cast_scan(world, perception, 0.0, 0.0, seed=1)
    assert result.point_count > 100
    assert all(0.0 <= r.intensity <= 1.0 for r in result.returns)
    assert all(r.range_m <= LIDAR_RANGE_M + 1.0 for r in result.returns)


def test_obstacles_return_harder_than_open_ground():
    """A rock face is near normal to the beam; ground is grazed."""
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-3, 4):
        world._cells[(dx, 8)] = Terrain.ROCK
    result = cast_scan(world, perception, 0.0, 0.0, seed=2)

    rock = [r.intensity for r in result.returns if r.kind is Terrain.ROCK]
    ground = [r.intensity for r in result.returns if r.kind is Terrain.GROUND and r.range_m < 6.0]
    assert rock and ground
    assert sum(rock) / len(rock) > sum(ground) / len(ground)


def test_intensity_falls_off_with_range():
    world, perception = blank_world(), PerceptionMap()
    result = cast_scan(world, perception, 0.0, 0.0, seed=3)
    near = [r.intensity for r in result.returns if r.range_m < 6.0]
    far = [r.intensity for r in result.returns if r.range_m > 24.0]
    assert near and far
    assert sum(near) / len(near) > sum(far) / len(far)


def test_no_returns_come_from_behind_an_obstacle():
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-10, 11):
        world._cells[(dx, 10)] = Terrain.ROCK
    result = cast_scan(world, perception, 0.0, 0.0, seed=4)
    # Nothing measured beyond the wall on the bearings it covers.
    behind = [r for r in result.returns if r.y > 6.0 and abs(r.x) < 4.0 and r.kind is not Terrain.ROCK]
    assert not behind


def test_every_ray_that_hits_reports_a_hit():
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-40, 41):
        for dy in range(6, 9):
            world._cells[(dx, dy)] = Terrain.TREE
    result = cast_scan(world, perception, 0.0, 0.0, seed=5)
    assert result.hits > 0
    tree_returns = [r for r in result.returns if r.kind is Terrain.TREE]
    assert len(tree_returns) == result.hits
