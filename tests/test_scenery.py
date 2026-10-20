"""What is drawn is what is there.

The cameras, the LiDAR map and the route planner all work from the same
objects: the boulders, trees and bushes the world generates. These tests hold
the three together, because a rover that avoids what is on its map while the
camera shows something else is a rover that looks as though it drives through
rocks.
"""

import math

import numpy as np
import pytest

from ocu.core.lidar import ROCK_DETECTION_RANGE_M, cast_scan
from ocu.core.terrain import (
    BOULDER_OBSTACLE_MIN_TOP_M,
    FOOTPRINT_GROWTH_M,
    IMPASSABLE,
    OCCLUDING,
    PerceptionMap,
    Terrain,
    TerrainWorld,
)


@pytest.fixture(scope="module")
def world():
    return TerrainWorld()


def test_the_world_has_individual_objects_in_it(world):
    assert len(world.boulders) > 100
    assert len(world.trees) > 100
    assert len(world.bushes) > 100


def test_every_boulder_is_an_obstacle_across_its_whole_footprint(world):
    """Not only where its dome is high: a rover that drove up the low flank of
    a boulder looked, to anyone watching, like a rover driving through a rock."""
    for x, y, a, b, theta, top in world.boulders[:200]:
        assert top >= BOULDER_OBSTACLE_MIN_TOP_M
        c, s = math.cos(theta), math.sin(theta)
        for t in np.linspace(0, math.tau, 24, endpoint=False):
            # A point just inside the ellipse's edge.
            px = x + 0.92 * (a * math.cos(t) * c - b * math.sin(t) * s)
            py = y + 0.92 * (a * math.cos(t) * s + b * math.sin(t) * c)
            assert world.classify(px, py) is Terrain.ROCK, (x, y, a, b)


def test_a_boulder_is_mapped_a_little_bigger_than_it_is(world):
    """So that a vehicle keeping its margin from the mapped edge is clear of the real one."""
    x, y, a, b, theta, top = world.boulders[0]
    just_outside = (x + (a + FOOTPRINT_GROWTH_M * 0.5) * math.cos(theta), y + (a + FOOTPRINT_GROWTH_M * 0.5) * math.sin(theta))
    assert world.classify(*just_outside) in (Terrain.ROCK, Terrain.GROUND, Terrain.SCRUB, Terrain.TRACK, Terrain.BUSH, Terrain.TREE)
    assert (world._grid == int(Terrain.ROCK)).sum() > 0


def test_every_tree_stands_inside_ground_the_map_calls_impassable(world):
    for x, y, height, crown, species in world.trees:
        assert world.classify(x, y) is Terrain.TREE
        # And its crown stays inside the stand with room to spare.
        for t in np.linspace(0, math.tau, 12, endpoint=False):
            assert world.classify(x + crown * 0.8 * math.cos(t), y + crown * 0.8 * math.sin(t)) is Terrain.TREE


def test_every_bush_is_an_impassable_patch(world):
    for x, y, r, seed in world.bushes:
        assert world.classify(x, y) is Terrain.BUSH
        assert world.classify(x + r * 0.7, y) in (Terrain.BUSH, Terrain.GROUND, Terrain.SCRUB)


def test_nothing_stands_on_the_deploy_point(world):
    for group in (world.boulders, world.trees, world.bushes):
        assert not np.any(np.hypot(group[:, 0], group[:, 1]) < 6.0)


def test_a_bush_is_impassable_but_does_not_block_the_beam():
    assert Terrain.BUSH in IMPASSABLE
    assert Terrain.BUSH not in OCCLUDING


def test_a_bush_is_resolved_only_at_close_range():
    world = TerrainWorld.blank(half_extent_m=60.0)
    far = ROCK_DETECTION_RANGE_M + 6.0
    cell = (0, int(far / world.cell_size_m))
    world.set_cell(cell, Terrain.BUSH)
    perception = PerceptionMap()
    cast_scan(world, perception, 0.0, 0.0)
    assert perception.at(cell) is Terrain.GROUND
    cast_scan(world, perception, 0.0, far - 8.0)
    assert perception.at(cell) is Terrain.BUSH


def test_a_mapped_bush_is_not_forgotten_when_the_rover_turns_away():
    perception = PerceptionMap()
    perception.observe((3, 3), Terrain.BUSH)
    perception.observe((3, 3), Terrain.GROUND)
    assert perception.at((3, 3)) is Terrain.BUSH


def test_the_world_is_the_same_every_time():
    a, b = TerrainWorld(), TerrainWorld()
    assert np.array_equal(a.boulders, b.boulders)
    assert np.array_equal(a.trees, b.trees)
    assert np.array_equal(a.bushes, b.bushes)
    assert np.array_equal(a._grid, b._grid)
