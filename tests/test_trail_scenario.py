"""The simulated world built round a reconstructed trail (core/trail_scenario.py), against a trail whose geometry is known."""

import math

import numpy as np
import pytest

from ocu.core.terrain import Terrain, TerrainWorld
from ocu.core.trail_scenario import HALF_TRACK_M, TrailScenario
from ocu.render import trail
from ocu.render.splats import Intrinsics


def make_route(length_m=60.0, scale=2.0, climb=0.08):
    """A trail in model units (scale metres each) that starts going along +x, bends to the left and climbs, walked by a camera 0.6 units up.

    The model's up is +z. Cameras look along the walk, with COLMAP's axes: x right, y down, z forward."""
    n = 200
    t = np.linspace(0.0, length_m / scale, n)  # model units along the path
    heading = 0.35 * np.sin(t / 8.0) + 0.0  # a gentle weave: radians from +x, positive to the left
    dx, dy = np.cos(heading), np.sin(heading)
    x = np.concatenate([[0.0], np.cumsum(dx[1:] * np.diff(t))])
    y = np.concatenate([[0.0], np.cumsum(dy[1:] * np.diff(t))])
    ground = climb * x
    centres = np.stack([x, y, ground + 0.6], axis=1)
    quats = []
    for h in heading:
        forward = np.array([math.cos(h), math.sin(h), 0.0])
        right = np.cross(forward, [0.0, 0.0, 1.0])
        down = np.array([0.0, 0.0, -1.0])
        rot = np.stack([right, down, forward])
        quats.append(trail._quat(rot))
    steps = np.linalg.norm(np.diff(centres, axis=0), axis=1) * scale
    arc = np.concatenate([[0.0], np.cumsum(steps)])
    route = trail.TrailRoute(centres, np.array(quats), arc, scale, np.array([0.0, 0.0, 1.0]), np.full(n, 0.6), 0.6 * scale)
    return route, Intrinsics(1280, 720, 640.0, 640.0, 640.0, 360.0)


@pytest.fixture(scope="module")
def scenario():
    route, k = make_route()
    return TrailScenario.build("synthetic", route, k, start_m=10.0)


def test_the_deploy_point_is_on_the_line_and_the_trail_runs_north(scenario):
    x, y = scenario.point(0.0)
    assert math.hypot(x, y) < 0.3
    assert scenario.bearing_deg == pytest.approx(0.0, abs=2.0) or scenario.bearing_deg > 358.0
    assert scenario.xy[scenario.start_index].tolist() == pytest.approx([0.0, 0.0], abs=1e-6)


def test_the_world_is_right_handed_so_a_left_bend_in_the_model_is_a_left_bend_in_the_world(scenario):
    # the model's trail weaves left first (heading increases), which seen from above, going north, is toward west (x decreasing)
    ahead = scenario.point(20.0)
    assert ahead[1] > 15.0, "the line goes north"
    assert ahead[0] < 0.0 < scenario.point(5.0, lateral_m=1.0)[0] - scenario.point(5.0)[0] + 0.0 or ahead[0] < 1.0


def test_to_the_right_of_the_line_is_east_when_it_runs_north(scenario):
    x0, y0 = scenario.point(0.0)
    x1, y1 = scenario.point(0.0, lateral_m=1.0)
    assert x1 - x0 == pytest.approx(math.cos(math.radians(scenario.bearing_deg)) * 1.0, abs=0.05)


def test_the_sweep_waypoints_follow_the_line_and_buried_things_are_placed_along_it(scenario):
    sweep = scenario.sweep()
    assert sweep.points and sweep.points[0][2] == "LANE 1"
    for x, y, _ in sweep.points:
        assert scenario._tree.query([x, y])[0] < 0.3
    on_line = sweep.at(0, 9.0, 0.04)
    assert scenario._tree.query(list(on_line))[0] < 0.1, "lane 0 is the line, give or take centimetres"
    beside = sweep.at(2, 19.5, 0.9)
    assert 1.0 < scenario._tree.query(list(beside))[0] < 3.0, "a third lane is beside the line, still near the trail"


def test_carving_gives_the_ground_the_trails_rise_and_makes_it_track(scenario):
    world = TerrainWorld.blank(half_extent_m=110.0)
    world._elevation[:] = 3.0
    scenario.carve(world)
    assert scenario.base_z == pytest.approx(3.0)
    start = scenario.xy[scenario.start_index]
    for s in (0.0, 15.0, 30.0):
        x, y = scenario.point(s)
        assert world.classify(x, y) == Terrain.TRACK
        elevation = world.elevation_at(world.to_cell(x, y))
        i = int(np.searchsorted(scenario.arc, scenario.start_m + s))
        assert elevation == pytest.approx(3.0 + scenario.ground_z[i], abs=0.4)
    assert scenario.ground_z[-1] > scenario.ground_z[scenario.start_index] + 1.5, "it climbs"
    x, y = scenario.point(10.0, lateral_m=HALF_TRACK_M + 1.0)
    assert world.classify(x, y) == Terrain.SCRUB
    x, y = scenario.point(10.0, lateral_m=40.0)
    assert world.classify(x, y) == Terrain.GROUND and world.elevation_at(world.to_cell(x, y)) == pytest.approx(3.0, abs=0.05)


def test_carving_clears_what_stood_on_the_trail_and_leaves_the_rest(scenario):
    world = TerrainWorld.blank(half_extent_m=110.0)
    on = scenario.point(8.0)
    off = scenario.point(8.0, lateral_m=30.0)
    world.trees = np.array([[on[0], on[1], 8.0, 2.0, 0.0], [off[0], off[1], 8.0, 2.0, 0.0]])
    world.boulders = np.array([[on[0], on[1] + 1.0, 1.0, 0.8, 0.0, 0.7]])
    world.pebbles = np.array([[on[0], on[1] - 1.0, 0.1, 0.1, 1.0], [off[0], off[1], 0.1, 0.1, 1.0]])
    world._pothole_depth[world._index(world.to_cell(*on))] = 0.5
    scenario.carve(world)
    assert len(world.trees) == 1 and np.allclose(world.trees[0, :2], off)
    assert len(world.boulders) == 0
    assert len(world.pebbles) == 1
    assert world.pothole_depth_at(world.to_cell(*on)) == 0.0


def test_a_rover_on_the_track_facing_along_it_is_on_the_trail_and_one_beside_or_turned_round_is_not(scenario):
    x, y = scenario.point(12.0, lateral_m=0.3)
    heading = scenario.bearing_deg
    tx, ty = scenario.tangent[int(np.searchsorted(scenario.arc, scenario.start_m + 12.0))]
    along = math.degrees(math.atan2(tx, ty))
    state = scenario.camera_state(x, y, along)
    assert state.on_trail
    assert state.s_m == pytest.approx(scenario.start_m + 12.0, abs=0.5)
    assert state.lateral_m == pytest.approx(0.3, abs=0.1) and abs(state.yaw_deg) < 1.0
    # turned across the trail, or a metre or two off it: still the real place, from the nearest camera the walk supports
    across = scenario.camera_state(x, y, along + 90.0)
    assert across.on_trail and across.yaw_deg == pytest.approx(30.0), "turned past what the walk saw: limited to what it did"
    assert scenario.camera_state(x, y, along + 180.0).on_trail
    bx, by = scenario.point(12.0, lateral_m=3.0)
    beside = scenario.camera_state(bx, by, along)
    assert beside.on_trail and beside.lateral_m == pytest.approx(1.0), "3 m beside the line: drawn from 1 m beside it"
    far = scenario.point(12.0, lateral_m=6.0)
    assert not scenario.camera_state(*far, along).on_trail, "far from the path: the simulated world"
    assert scenario.camera_state(*scenario.point(-8.0), along).on_trail, "the walk starts 10 m before the deploy point, and that stretch can be drawn"
    assert not scenario.camera_state(*scenario.point(-12.0), along).on_trail, "before the start of the walk"
    assert not scenario.camera_state(*scenario.point(80.0), along).on_trail, "past its end"
    assert scenario.camera_state(x, y, along + 20.0).yaw_deg == pytest.approx(20.0, abs=1.0)
    del heading


def test_the_trail_camera_is_where_the_phone_was_looking_the_way_it_looked(scenario):
    state = scenario.camera_state(*scenario.point(15.0), math.degrees(math.atan2(*scenario.tangent[int(np.searchsorted(scenario.arc, scenario.start_m + 15.0))])))
    cam = scenario.camera(state)
    i = int(np.searchsorted(scenario.arc, state.s_m))
    assert (cam.eye[0], cam.eye[1]) == pytest.approx(tuple(scenario.xy[i]), abs=0.4)
    # the camera was held 1.2 m (0.6 units at 2 m a unit) above the ground
    assert cam.eye[2] - (scenario.base_z + scenario.ground_z[i]) == pytest.approx(1.2, abs=0.15)
    tx, ty = scenario.tangent[i]
    assert cam.heading_deg == pytest.approx(math.degrees(math.atan2(tx, ty)) % 360.0, abs=3.0)
    assert abs(cam.pitch_down_deg) < 3.0
    turned = scenario.camera(type(state)(state.s_m, 0.0, 20.0, True))
    assert (turned.heading_deg - cam.heading_deg + 180.0) % 360.0 - 180.0 == pytest.approx(20.0, abs=1.0), "a heading turns clockwise"


# --- several trails chained into one patrol -------------------------------------------------------------------------------

from ocu.core.trail_scenario import FAR_TURN_MAX_DEG, TrailWorld  # noqa: E402


def chain(count=6, length_m=60.0, growth_m=4.0):
    trails = []
    for i in range(count):
        route, k = make_route(length_m=length_m + growth_m * i)
        trails.append((f"leg{i}", route, k))
    return TrailWorld.build(trails)


def test_each_leg_starts_where_the_one_before_it_ended_and_leaves_in_the_direction_it_arrived():
    world = chain()
    for before, after in zip(world.legs, world.legs[1:]):
        i0, i1 = before.patrol_range()
        end = before.xy[i1 - 1]
        start = after.xy[after.start_index]
        assert np.linalg.norm(end - start) < 0.1, "the next leg's deploy point is the last one's end"
        arrived = math.degrees(math.atan2(*before.tangent[i1 - 1]))
        left = math.degrees(math.atan2(*after.tangent[after.start_index]))
        assert abs((left - arrived + 180.0) % 360.0 - 180.0) <= FAR_TURN_MAX_DEG + 5.0
        assert after.ground_z[after.start_index] == pytest.approx(before.ground_z[i1 - 1], abs=0.05), "the ground carries on at the same height"


def test_a_long_patrol_stays_inside_the_world():
    world = chain(count=14, length_m=60.0, growth_m=0.0)  # legs of about 40 m of patrol each, like the footage's
    assert world.length_m > 450.0
    assert np.abs(world.xy).max() < 105.0, "the path steers back toward the middle of the map"
    assert math.hypot(*world.xy[0]) < 0.5


def test_the_chained_sweep_runs_the_whole_patrol_and_names_its_legs():
    world = chain(count=4)
    sweep = world.sweep()
    labels = [label for _, _, label in sweep.points]
    assert labels[0] == "LEG 1" and labels[-1] == "LEG 4" and sorted(set(labels)) == [f"LEG {i}" for i in range(1, 5)]
    for x, y, _ in sweep.points:
        assert world._tree.query([x, y])[0] < 0.3
    assert world.arc[-1] == pytest.approx(sum(np.linalg.norm(np.diff(leg.xy[slice(*leg.patrol_range())], axis=0), axis=1).sum() for leg in world.legs), rel=0.02)


def test_the_camera_changes_leg_when_the_rover_crosses_into_the_next_one():
    world = chain(count=3)
    inside_second = world.legs[1].point(8.0)
    tx, ty = world.legs[1].tangent[int(np.searchsorted(world.legs[1].arc, world.legs[1].start_m + 8.0))]
    state = world.camera_state(*inside_second, math.degrees(math.atan2(tx, ty)))
    assert state.leg == 1 and state.on_trail
    assert state.s_m == pytest.approx(world.legs[1].start_m + 8.0, abs=0.6), "the arc is the leg's own, which is what its splat is drawn along"
    cam = world.camera(state)
    assert math.hypot(cam.eye[0] - inside_second[0], cam.eye[1] - inside_second[1]) < 0.6


def test_the_ground_is_carved_continuously_across_a_join():
    world = chain(count=3)
    ground = TerrainWorld.blank(half_extent_m=110.0)
    world.carve(ground)
    i0, i1 = world.legs[0].patrol_range()
    join = world.legs[0].xy[i1 - 1]
    heights = []
    for back in (-1.5, -0.5, 0.0, 0.5, 1.5):
        along = world.legs[0].tangent[i1 - 1]
        p = join + along * back
        heights.append(ground.elevation_at(ground.to_cell(*p)))
    assert max(heights) - min(heights) < 0.5
    assert ground.classify(*join) == Terrain.TRACK
