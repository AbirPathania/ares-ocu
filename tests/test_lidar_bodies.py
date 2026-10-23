"""The scanner sees people and animals: returns off their outlines, occluded like anything else, drawn for a moment
and never written into the map."""

import math
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.core.entities import LIDAR_SHAPES, SurfaceEntity  # noqa: E402
from ocu.core.lidar import LIDAR_RANGE_M, entity_returns  # noqa: E402
from ocu.core.terrain import Terrain, TerrainWorld  # noqa: E402
from ocu.ui import map_view as mv  # noqa: E402

PERSON = ("person", LIDAR_SHAPES["person"])
DEER = ("animal", LIDAR_SHAPES["animal"])


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


def body(spec, x, y, heading=0.0):
    kind, shapes = spec
    return (kind, x, y, heading, shapes)


def on_outline(r, x, y, heading, shape, tol=0.08):
    off, rx, ry = shape
    h = math.radians(heading)
    fwd, side = (math.sin(h), math.cos(h)), (math.cos(h), -math.sin(h))
    out = []
    for px, py in zip(r.xs, r.ys, strict=True):
        dx, dy = px - (x + fwd[0] * off), py - (y + fwd[1] * off)
        u, v = (dx * side[0] + dy * side[1]) / rx, (dx * fwd[0] + dy * fwd[1]) / ry
        out.append(abs(math.hypot(u, v) - 1.0))
    return max(out) < tol * 4  # the range noise is centimetres on a body decimetres across


def test_a_person_ten_metres_ahead_returns_a_few_points_on_the_near_side_of_their_outline():
    w = TerrainWorld.blank()
    r = entity_returns(w, 0.0, 0.0, [body(PERSON, 0.0, 10.0)], seed=1)
    assert 2 <= r.point_count <= 5 and set(r.kinds) == {"person"}
    assert np.all(r.ranges > 9.5) and np.all(r.ranges < 10.0), "off the face toward the scanner, not through the body"
    assert on_outline(r, 0.0, 10.0, 0.0, LIDAR_SHAPES["person"][0])


def test_the_further_away_the_fewer_rays_strike_a_body_because_the_scanner_has_one_degree_between_rays():
    w = TerrainWorld.blank()
    near = entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 6.0, 90.0)], seed=1).point_count
    far = entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 24.0, 90.0)], seed=1).point_count
    assert near > far >= 1


def test_a_deer_side_on_is_struck_by_more_rays_than_one_facing_the_scanner():
    w = TerrainWorld.blank()
    side = entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 12.0, 90.0)], seed=1).point_count
    head_on = entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 12.0, 0.0)], seed=1).point_count
    assert side > head_on


def test_nothing_beyond_the_scanners_range_returns():
    w = TerrainWorld.blank()
    assert entity_returns(w, 0.0, 0.0, [body(PERSON, 0.0, LIDAR_RANGE_M + 3.0)], seed=1).point_count == 0


def test_a_person_behind_a_boulder_returns_nothing_and_beside_it_returns_something():
    w = TerrainWorld.blank()
    for iy in range(-6, 7):
        for ix in range(10, 13):
            w.set_cell((ix, iy), Terrain.ROCK)  # a wall 3 m wide, 5 to 6.5 m out
    behind = entity_returns(w, 14.0, 0.0, [body(PERSON, 0.0, 0.0, 0.0)], seed=1)  # scanner east of the wall, person west of it
    assert behind.point_count == 0, "a body behind something that blocks a beam is not seen"
    clear = entity_returns(w, 0.0, 0.0, [body(PERSON, 0.0, 14.0, 0.0)], seed=1)
    assert clear.point_count > 0, "the wall is to the east, so the person straight ahead is in the open"


def test_returns_are_deterministic_for_a_seed_and_the_intensity_falls_with_range():
    w = TerrainWorld.blank()
    a = entity_returns(w, 0.0, 0.0, [body(PERSON, 0.0, 8.0)], seed=5)
    b = entity_returns(w, 0.0, 0.0, [body(PERSON, 0.0, 8.0)], seed=5)
    assert np.array_equal(a.xs, b.xs) and np.array_equal(a.intensities, b.intensities)
    far = entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 24.0, 90.0)], seed=5)
    near = entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 5.0, 90.0)], seed=5)
    assert near.intensities.mean() > far.intensities.mean()


def test_a_parked_vehicle_has_no_moving_outline_because_the_ground_already_holds_it():
    assert SurfaceEntity("vehicle", "t", 2.2, 2.1, 5.7, ((0.0, 0.0),)).lidar_shapes == []
    assert SurfaceEntity("person", "p", 1.78, 0.54, 0.63, ((0.0, 0.0), (0.0, 5.0)), speed_ms=1.0).lidar_shapes


# ---- on the map

def test_returns_off_bodies_are_held_briefly_then_released_and_never_touch_the_cloud(app):
    view = mv.MapView()
    before = view._cloud_pixels().copy()
    w = TerrainWorld.blank()
    view.add_entity_returns(entity_returns(w, 0.0, 0.0, [body(PERSON, 0.0, 9.0)], seed=1))
    assert len(view._entity_returns) >= 2
    assert np.array_equal(before, view._cloud_pixels()), "a body that moves is not written into the permanent cloud"
    for _ in range(int(mv.ENTITY_RETURN_LIFE_S / (mv.RENDER_TICK_MS / 1000.0)) + 3):
        view._on_radar_tick()
    assert view._entity_returns == [], "and it is let go"


def test_a_walker_leaves_a_trail_while_the_returns_last_and_clearing_the_map_clears_it(app):
    view = mv.MapView()
    w = TerrainWorld.blank()
    for k in range(4):
        view.add_entity_returns(entity_returns(w, 0.0, 0.0, [body(PERSON, 0.3 * k, 9.0)], seed=k))
    ys = {round(p[0], 1) for p in view._entity_returns}
    assert len(ys) >= 4, "four positions, four clusters"
    view.clear_perception()
    assert view._entity_returns == []


def test_the_held_returns_are_capped(app):
    view = mv.MapView()
    w = TerrainWorld.blank()
    for k in range(400):
        view.add_entity_returns(entity_returns(w, 0.0, 0.0, [body(DEER, 0.0, 6.0, 90.0)], seed=k))
    assert len(view._entity_returns) <= mv.ENTITY_RETURN_CAP


# ---- in the window

def test_a_person_walking_in_view_shows_in_the_lidar_picture_and_the_rovers_map_is_left_alone(app):
    from PySide6.QtCore import QTimer

    from ocu.ui.main_window import MainWindow

    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    pose = w.units["rover"].pose
    h = math.radians(pose.heading)
    ahead = lambda m, off=0.0: (pose.x + m * math.sin(h) + off * math.cos(h), pose.y + m * math.cos(h) - off * math.sin(h))  # noqa: E731
    walker = SurfaceEntity("person", "w", 1.78, 0.54, 0.63, (ahead(9.0, -3.0), ahead(9.0, 3.0)), speed_ms=0.8, box_offset_m=-0.095)
    w.surface_entities = [walker]
    seen_kinds = set()
    for k in range(60):
        w.mission_clock._clock_s = k * 0.25 + 0.01
        w._last_scan_s = -1e9
        w._tick_lidar()
        seen_kinds |= {pt[2] for pt in w.map_view._entity_returns}
    assert "person" in seen_kinds, "the walker is struck by the scanner and drawn"
    x, y, _ = walker.pose_at(w.mission_clock.clock_s)
    for dx in (-1.0, 0.0, 1.0):
        assert w.perception.at(w.terrain.to_cell(x + dx, y)) != Terrain.ROCK, "a moving body never becomes a rock on the map"
    w.close()
