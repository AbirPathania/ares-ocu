"""What a scout can see from the air.

A scout finds what is on or just under the surface, inside its camera's view,
in range, with nothing in the way. These tests hold that to the geometry, and
hold optical evidence to being weak: it says something is out of place, never
what it is, so on its own it must not reach the escalation threshold.
"""

import math

import pytest

from ocu.core.aerial import (
    MAX_OPTICAL_CONFIDENCE,
    MAX_SLANT_RANGE_M,
    MAX_YAW_RATE_FOR_SIGHTING_DEG_S,
    MAX_VISIBLE_BURIAL_M,
    scout_sighting,
)
from ocu.core.camera import CameraRig
from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD
from ocu.core.targets import BuriedTarget
from ocu.core.terrain import CELL_SIZE_M, Terrain, TerrainWorld


def open_rig() -> CameraRig:
    return CameraRig(TerrainWorld.blank(half_extent_m=110.0))


def surface_object(x: float, y: float, depth: float = 0.03) -> BuriedTarget:
    return BuriedTarget(x, y, depth, 120.0, True, 0.2, "test")


def sees(rig, target, x=0.0, y=0.0, agl=20.0, heading=0.0, yaw_rate=0.0) -> float:
    return scout_sighting(rig, target, x, y, agl, heading, yaw_rate)


def test_a_surface_object_ahead_in_view_is_seen():
    assert sees(open_rig(), surface_object(0.0, 35.0)) > 0.0


def test_something_behind_the_scout_is_not():
    assert sees(open_rig(), surface_object(0.0, -35.0)) == 0.0


def test_something_far_off_to_the_side_is_outside_the_field_of_view():
    assert sees(open_rig(), surface_object(60.0, 10.0)) == 0.0


def test_the_view_follows_the_heading():
    rig = open_rig()
    target = surface_object(35.0, 0.0)
    assert sees(rig, target, heading=0.0) == 0.0
    assert sees(rig, target, heading=90.0) > 0.0


def test_directly_beneath_is_below_the_camera_view():
    """A camera tipped forward cannot see what is almost under it."""
    assert sees(open_rig(), surface_object(0.0, 4.0), agl=20.0) == 0.0


def test_too_far_to_resolve_is_not_seen():
    rig = open_rig()
    reach = math.sqrt(MAX_SLANT_RANGE_M**2 - 20.0**2)  # across the ground, from 20 m up
    assert sees(rig, surface_object(0.0, reach - 3.0), agl=20.0) > 0.0
    assert sees(rig, surface_object(0.0, reach + 3.0), agl=20.0) == 0.0


def test_something_buried_is_invisible_to_a_camera():
    rig = open_rig()
    assert sees(rig, surface_object(0.0, 35.0, depth=MAX_VISIBLE_BURIAL_M + 0.05)) == 0.0
    assert sees(rig, surface_object(0.0, 35.0, depth=MAX_VISIBLE_BURIAL_M - 0.02)) > 0.0


def test_nearer_is_surer():
    rig = open_rig()
    near = sees(rig, surface_object(0.0, 32.0))
    far = sees(rig, surface_object(0.0, 46.0))
    assert near > far > 0.0


def test_a_camera_alone_never_escalates():
    """It says something is out of place. It cannot say what, so it must not
    reach the threshold at which the operator is asked to decide."""
    rig = open_rig()
    best = max(sees(rig, surface_object(0.0, d), agl=h) for d in range(8, 80, 4) for h in (8.0, 15.0, 25.0, 40.0))
    assert 0.0 < best <= MAX_OPTICAL_CONFIDENCE < ALERT_CONFIDENCE_THRESHOLD


def test_a_wall_of_trees_in_the_way_hides_it():
    clear = open_rig()
    screened_world = TerrainWorld.blank(half_extent_m=110.0)
    for cx in range(-40, 41):
        for cy in range(int(12.0 / CELL_SIZE_M), int(16.0 / CELL_SIZE_M)):
            screened_world.set_cell((cx, cy), Terrain.TREE)
    screened = CameraRig(screened_world)

    # Low and far, so the line to the target passes through the trees.
    target = surface_object(0.0, 45.0)
    assert sees(clear, target, agl=8.0) > 0.0
    assert sees(screened, target, agl=8.0) == 0.0


def test_a_scout_high_enough_looks_over_the_same_trees():
    world = TerrainWorld.blank(half_extent_m=110.0)
    for cx in range(-40, 41):
        for cy in range(int(12.0 / CELL_SIZE_M), int(16.0 / CELL_SIZE_M)):
            world.set_cell((cx, cy), Terrain.TREE)
    rig = CameraRig(world)
    assert sees(rig, surface_object(0.0, 40.0), agl=30.0) > 0.0


def test_height_is_held_over_the_ground_not_over_a_datum():
    flat = TerrainWorld.blank(half_extent_m=110.0)
    raised = TerrainWorld.blank(half_extent_m=110.0)
    raised._elevation[:] = 12.0
    target = surface_object(0.0, 35.0)
    assert math.isclose(sees(CameraRig(flat), target), sees(CameraRig(raised), target))


def test_the_line_of_sight_test_is_the_one_the_picture_uses():
    rig = open_rig()
    assert rig.line_of_sight(0.0, 0.0, 10.0, 0.0, 40.0, 0.5)
    ridge = TerrainWorld.blank(half_extent_m=110.0)
    for cx in range(-60, 61):
        for cy in range(int(18.0 / CELL_SIZE_M), int(20.0 / CELL_SIZE_M)):
            ridge._elevation[ridge._index((cx, cy))] = 12.0
    assert not CameraRig(ridge).line_of_sight(0.0, 0.0, 10.0, 0.0, 40.0, 0.5)


def test_the_range_is_what_the_sensor_can_resolve_not_a_round_number():
    """Four pixels on a quarter-metre object, through an 84 degree lens onto a
    1280 pixel sensor: about 55 m of slant range."""
    from ocu.core.camera import SCOUT_HFOV_DEG

    pixel = math.radians(SCOUT_HFOV_DEG) / 1280
    assert MAX_SLANT_RANGE_M == pytest.approx(0.25 / (4 * pixel))
    assert 50.0 < MAX_SLANT_RANGE_M < 60.0


def test_a_camera_being_swung_fast_sees_nothing_it_sweeps_over():
    rig = open_rig()
    target = surface_object(0.0, 35.0)
    assert sees(rig, target, yaw_rate=0.0) > 0.0
    assert sees(rig, target, yaw_rate=MAX_YAW_RATE_FOR_SIGHTING_DEG_S - 1.0) > 0.0
    assert sees(rig, target, yaw_rate=MAX_YAW_RATE_FOR_SIGHTING_DEG_S + 5.0) == 0.0
    assert sees(rig, target, yaw_rate=-(MAX_YAW_RATE_FOR_SIGHTING_DEG_S + 5.0)) == 0.0
