"""Laying the clearance sweep out on ground that can actually be swept.

The property that matters is that the axis is chosen rather than assumed.
A sweep laid across a rock field is a sweep the vehicle spends its whole
mission detouring around, and ground that is detoured around is ground
nobody looked at.
"""

import math

from ocu.core.sweep import (
    LANE_COUNT,
    LANE_LENGTH_M,
    LANE_SPACING_M,
    LaneSweep,
    clear_fraction,
    plan_lane_sweep,
)
from ocu.core.terrain import CELL_SIZE_M, Terrain, TerrainWorld


def world_with_rock_except_towards(bearing_deg: float) -> TerrainWorld:
    """Everything impassable but a wedge on one bearing."""
    world = TerrainWorld.blank()
    half = int(40 / CELL_SIZE_M)
    for cx in range(-half, half + 1):
        for cy in range(-half, half + 1):
            x, y = cx * CELL_SIZE_M, cy * CELL_SIZE_M
            if math.hypot(x, y) < 1.0:
                world.set_cell((cx, cy), Terrain.GROUND)
                continue
            to_point = math.degrees(math.atan2(x, y)) % 360.0
            offset = abs((to_point - bearing_deg + 180.0) % 360.0 - 180.0)
            world.set_cell((cx, cy), Terrain.GROUND if offset < 40.0 else Terrain.ROCK)
    return world


def test_the_sweep_is_laid_on_the_clearest_axis_available():
    for bearing in (0.0, 90.0, 210.0, 315.0):
        world = world_with_rock_except_towards(bearing)
        sweep = plan_lane_sweep(world)
        error = abs((sweep.bearing_deg - bearing + 180.0) % 360.0 - 180.0)
        assert error <= 30.0, f"wanted about {bearing}, got {sweep.bearing_deg}"


def test_a_clear_axis_scores_better_than_a_blocked_one():
    world = world_with_rock_except_towards(90.0)
    assert clear_fraction(world, (0.0, 0.0), 90.0) > clear_fraction(world, (0.0, 0.0), 270.0)


def test_open_ground_is_reported_as_entirely_clear():
    world = TerrainWorld.blank()
    for cx in range(-200, 201):
        for cy in range(-200, 201):
            world.set_cell((cx, cy), Terrain.GROUND)
    assert clear_fraction(world, (0.0, 0.0), 0.0) == 1.0


def open_world() -> TerrainWorld:
    world = TerrainWorld.blank()
    for cx in range(-200, 201):
        for cy in range(-200, 201):
            world.set_cell((cx, cy), Terrain.GROUND)
    return world


def test_every_lane_is_represented_and_alternates_direction():
    """A serpentine, not a set of runs with a drive back between each."""
    sweep = plan_lane_sweep(open_world())
    labels = [label for _, _, label in sweep.points]
    assert sorted(set(labels)) == [f"LANE {i + 1}" for i in range(LANE_COUNT)]

    forward = (math.sin(math.radians(sweep.bearing_deg)), math.cos(math.radians(sweep.bearing_deg)))
    along = lambda p: p[0] * forward[0] + p[1] * forward[1]
    runs = []
    for index in range(LANE_COUNT):
        lane = [p for p in sweep.points if p[2] == f"LANE {index + 1}"]
        runs.append(along(lane[-1]) - along(lane[0]))
    for first, second in zip(runs, runs[1:]):
        assert first * second < 0, "consecutive lanes must run opposite ways"


def test_lane_points_are_close_enough_together_to_constrain_the_route():
    """The planner holds the leg it is given. Long legs let it bow off the
    lane across ground it has not seen yet."""
    sweep = plan_lane_sweep(open_world())
    for index in range(LANE_COUNT):
        lane = [p for p in sweep.points if p[2] == f"LANE {index + 1}"]
        gaps = [
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(lane, lane[1:])
        ]
        assert max(gaps) <= 3.0 + 1e-9


def test_lanes_are_spaced_as_specified_and_stay_inside_the_tether():
    sweep = plan_lane_sweep(open_world())
    right = (math.cos(math.radians(sweep.bearing_deg)), -math.sin(math.radians(sweep.bearing_deg)))
    across = lambda p: p[0] * right[0] + p[1] * right[1]
    for index in range(LANE_COUNT):
        lane = [p for p in sweep.points if p[2] == f"LANE {index + 1}"]
        for point in lane:
            assert abs(across(point) - index * LANE_SPACING_M) < 1e-6

    # The aircraft hold formation on the rover within a 50 m tether, so the
    # sweep has to stay inside it for the whole mission.
    assert all(math.hypot(x, y) < 50.0 for x, y, _ in sweep.points)


def test_a_point_placed_on_a_lane_lands_on_that_lane():
    """Anything positioned relative to the sweep has to move with it, or a
    change of axis quietly leaves it on ground nobody drives over."""
    sweep = plan_lane_sweep(open_world())
    forward = (math.sin(math.radians(sweep.bearing_deg)), math.cos(math.radians(sweep.bearing_deg)))
    right = (math.cos(math.radians(sweep.bearing_deg)), -math.sin(math.radians(sweep.bearing_deg)))

    x, y = sweep.at(lane_index=1, along_m=12.0, offset_m=0.25)
    assert abs((x * forward[0] + y * forward[1]) - 12.0) < 1e-6
    assert abs((x * right[0] + y * right[1]) - (LANE_SPACING_M + 0.25)) < 1e-6


def test_the_sweep_moves_with_its_axis():
    """The same lane description on different terrain gives different world
    coordinates. That is the point of describing targets this way."""
    a = plan_lane_sweep(world_with_rock_except_towards(0.0))
    b = plan_lane_sweep(world_with_rock_except_towards(180.0))
    assert a.bearing_deg != b.bearing_deg
    assert a.at(0, 10.0) != b.at(0, 10.0)
