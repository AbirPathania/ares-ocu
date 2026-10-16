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
