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
    return TerrainWorld.blank(half_extent_m=60.0)


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
        world.set_cell((dx, 10), Terrain.ROCK)

    cast_scan(world, perception, 0.0, 0.0)

    assert perception.at((0, 10)) is Terrain.ROCK, "the obstacle itself is seen"
    assert not perception.is_known((0, 16)), "ground directly behind it is not"
    assert not perception.is_known((0, 24))


def test_driving_around_an_obstacle_fills_in_its_shadow():
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-6, 7):
        world.set_cell((dx, 10), Terrain.ROCK)

    cast_scan(world, perception, 0.0, 0.0)
    assert not perception.is_known((0, 16))

    # Drive well past the end of the wall and look back.
    cast_scan(world, perception, 12.0, 10.0)
    assert perception.is_known((0, 16)), "the shadow closes once it can be seen into"


def test_trees_occlude_and_rocks_occlude():
    for kind in (Terrain.TREE, Terrain.ROCK):
        world, perception = blank_world(), PerceptionMap()
        for dx in range(-8, 9):
            world.set_cell((dx, 8), kind)
        cast_scan(world, perception, 0.0, 0.0)
        assert perception.at((0, 8)) is kind
        assert not perception.is_known((0, 20)), f"{kind.name} must block the beam"


def test_scrub_does_not_occlude():
    """Low vegetation is driven over and seen past; it is not a wall."""
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-8, 9):
        world.set_cell((dx, 8), Terrain.SCRUB)
    cast_scan(world, perception, 0.0, 0.0)
    assert perception.at((0, 8)) is Terrain.SCRUB
    assert perception.is_known((0, 20)), "scrub must not cast a shadow"


def test_line_of_sight_agrees_with_the_scan():
    world = blank_world()
    for dx in range(-6, 7):
        world.set_cell((dx, 10), Terrain.ROCK)
    assert has_line_of_sight(world, 0.0, 0.0, 0.0, 4.0)
    assert not has_line_of_sight(world, 0.0, 0.0, 0.0, 9.0)


# --- the generated world --------------------------------------------------


_SHARED: list = []


def shared_world() -> TerrainWorld:
    """Generating a full landscape is not cheap, so the tests that only
    inspect the default world share one."""
    if not _SHARED:
        _SHARED.append(TerrainWorld())
    return _SHARED[0]


def test_the_deploy_point_is_clear_ground():
    """A rover does not start inside a tree."""
    assert shared_world().classify(0.0, 0.0) not in IMPASSABLE


def test_the_world_contains_every_class():
    import numpy as np

    present = set(np.unique(shared_world()._grid).tolist())
    for kind in (Terrain.TRACK, Terrain.SCRUB, Terrain.TREE, Terrain.ROCK):
        assert int(kind) in present, f"generated world has no {kind.name}"


def test_the_world_is_deterministic_for_a_given_seed():
    import numpy as np

    a = TerrainWorld(seed=7, half_extent_m=20.0)
    b = TerrainWorld(seed=7, half_extent_m=20.0)
    assert np.array_equal(a._grid, b._grid)


def test_different_seeds_give_different_ground():
    import numpy as np

    a = TerrainWorld(seed=1, half_extent_m=20.0)
    b = TerrainWorld(seed=2, half_extent_m=20.0)
    assert not np.array_equal(a._grid, b._grid)


def test_anything_that_stops_the_beam_also_stops_the_wheels():
    """But not the other way round: a hole stops the wheels and lets the beam
    straight over, which is why negative obstacles need their own sensor."""
    assert OCCLUDING < IMPASSABLE
    assert Terrain.POTHOLE in IMPASSABLE and Terrain.POTHOLE not in OCCLUDING


def test_coverage_grows_as_the_vehicle_drives():
    world, perception = shared_world(), PerceptionMap()
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
        world.set_cell((dx, 8), Terrain.ROCK)
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
        world.set_cell((dx, 10), Terrain.ROCK)
    result = cast_scan(world, perception, 0.0, 0.0, seed=4)
    # Nothing measured beyond the wall on the bearings it covers.
    behind = [r for r in result.returns if r.y > 6.0 and abs(r.x) < 4.0 and r.kind is not Terrain.ROCK]
    assert not behind


def test_every_ray_that_hits_reports_a_hit():
    world, perception = blank_world(), PerceptionMap()
    for dx in range(-40, 41):
        for dy in range(6, 9):
            world.set_cell((dx, dy), Terrain.TREE)
    result = cast_scan(world, perception, 0.0, 0.0, seed=5)
    assert result.hits > 0
    tree_returns = [r for r in result.returns if r.kind is Terrain.TREE]
    assert len(tree_returns) == result.hits


# --- holes in the ground --------------------------------------------------


def pothole_world(distance_m: float) -> TerrainWorld:
    """Open ground with a hole in it, due north at `distance_m`."""
    world = blank_world()
    centre = int(distance_m / CELL_SIZE_M)
    for dx in range(-4, 5):
        for dy in range(centre - 2, centre + 3):
            world.set_cell((dx, dy), Terrain.POTHOLE)
    return world


def test_a_hole_does_not_cast_a_shadow():
    """You can see straight over a hole. Treating it like a wall would hide
    ground the rover can see perfectly well."""
    world, perception = pothole_world(5.0), PerceptionMap()
    cast_scan(world, perception, 0.0, 0.0)
    assert perception.is_known((0, 24)), "the ground beyond a hole is still visible"


def test_a_hole_further_off_than_the_stereo_pair_reads_as_ground():
    """A scanning LiDAR sweeping a horizontal plane passes over a hole and
    returns off what is behind it. That is what makes negative obstacles
    dangerous, and this model says so rather than quietly seeing everything."""
    from ocu.core.lidar import NEGATIVE_OBSTACLE_RANGE_M

    far = NEGATIVE_OBSTACLE_RANGE_M + 6.0
    world, perception = pothole_world(far), PerceptionMap()
    cast_scan(world, perception, 0.0, 0.0)
    assert perception.at((0, int(far / CELL_SIZE_M))) is Terrain.GROUND


def test_driving_closer_resolves_the_hole():
    from ocu.core.lidar import NEGATIVE_OBSTACLE_RANGE_M

    far = NEGATIVE_OBSTACLE_RANGE_M + 6.0
    cell = (0, int(far / CELL_SIZE_M))
    world, perception = pothole_world(far), PerceptionMap()

    cast_scan(world, perception, 0.0, 0.0)
    assert perception.at(cell) is Terrain.GROUND

    cast_scan(world, perception, 0.0, far - 3.0)  # close enough to look into it
    assert perception.at(cell) is Terrain.POTHOLE


def test_the_generated_world_has_holes_in_it_and_they_are_real_dips():
    world = shared_world()
    import numpy as np

    deep = world._pothole_depth >= 0.25
    assert int(deep.sum()) > 40, "the ground should have washouts in it"
    # Where a hole is, the ground is actually lower, so a camera sees a dip.
    index = np.argwhere(deep)[0]
    cell = (int(index[0]) - world._extent, int(index[1]) - world._extent)
    assert world.at(cell) is Terrain.POTHOLE
    assert world.pothole_depth_at(cell) >= 0.25


def test_painting_a_cell_keeps_the_hazard_fields_honest():
    world = blank_world()
    world.set_cell((3, 3), Terrain.POTHOLE)
    assert world.pothole_depth_at((3, 3)) > 0.0
    assert world.rock_height_at((3, 3)) == 0.0

    world.set_cell((3, 3), Terrain.ROCK)
    assert world.pothole_depth_at((3, 3)) == 0.0
    assert world.rock_height_at((3, 3)) > 0.0

    world.set_cell((3, 3), Terrain.GROUND)
    assert world.pothole_depth_at((3, 3)) == 0.0 and world.rock_height_at((3, 3)) == 0.0


def test_a_mapped_hole_is_not_unlearned_by_driving_away_from_it():
    """The far-field reading of a hole is honest open ground: the beam passes
    over it. It is not evidence the hole is gone, and a map that accepts it
    forgets every hazard the moment the rover drives on."""
    from ocu.core.lidar import NEGATIVE_OBSTACLE_RANGE_M

    far = NEGATIVE_OBSTACLE_RANGE_M + 6.0
    cell = (0, int(far / CELL_SIZE_M))
    world, perception = pothole_world(far), PerceptionMap()

    cast_scan(world, perception, 0.0, far - 3.0)  # close: resolved
    assert perception.at(cell) is Terrain.POTHOLE

    cast_scan(world, perception, 0.0, 0.0)  # driven away again
    assert perception.at(cell) is Terrain.POTHOLE, "the map must not forget a hazard"


def test_a_fresh_reading_still_replaces_everything_else():
    """The exception is narrow: anything else the sensor reports is the
    ground as it last saw it."""
    perception = PerceptionMap()
    perception.observe((0, 0), Terrain.SCRUB)
    assert perception.observe((0, 0), Terrain.GROUND)
    assert perception.at((0, 0)) is Terrain.GROUND

    perception.observe((1, 1), Terrain.POTHOLE)
    assert perception.observe((1, 1), Terrain.ROCK), "a hole filled by a rock is still news"
    assert perception.at((1, 1)) is Terrain.ROCK
