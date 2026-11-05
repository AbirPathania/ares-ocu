"""The rover tile's TRAIL chip and the request it makes of the renderer."""

import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


def press(widget, point):
    widget.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, QPointF(point), QPointF(point), Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))


def test_the_trail_chip_is_offered_only_when_a_trail_is_loaded_and_switches_the_rover_tile(app):
    window = MainWindow()
    feed = window.camera_feeds["rover"]
    feed.set_active(True) if hasattr(feed, "set_active") else setattr(feed, "_active", True)
    feed.resize(320, 180)
    window._gpu_worker = types.SimpleNamespace(trail_length_m=51.5, resolved=True, available=True, submit=lambda *a: None, stop=lambda: None)
    feed.set_trail_available(False)
    assert not feed._trail_available
    press(feed, feed._trail_chip_rect().center())
    assert not feed.trail_mode, "no chip, no click"

    feed.set_trail_available(True)
    toggled = []
    feed.trailToggled.connect(lambda uid, on: toggled.append((uid, on)))
    press(feed, feed._trail_chip_rect().center())
    assert feed.trail_mode and toggled == [("rover", True)]
    request = window._camera_request("rover")
    assert request["trail_s"] == pytest.approx(window.TRAIL_START_M, abs=1e-6) and request["entities"] is None

    window.mission_clock._clock_s = 10.0
    assert window._trail_progress_m() == pytest.approx(window.TRAIL_START_M + 10.0 * window.TRAIL_PACE_MS)
    window.mission_clock._clock_s = 100.0
    assert window.TRAIL_START_M <= window._trail_progress_m() < 51.5, "the replay goes round again at the end"

    press(feed, feed._chip_rect().center())  # switching to infrared gives the trail up
    assert not feed.trail_mode
    window.close()


def test_without_the_chip_the_rover_asks_for_the_simulated_world(app):
    window = MainWindow()
    assert "trail_s" not in window._camera_request("rover")
    window.close()


# --- a world built on a trail --------------------------------------------------------------------------------------------

@pytest.fixture
def synthetic_trail(tmp_path, monkeypatch):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent))
    from test_trail_scenario import make_route

    from ocu.render import trail

    route, k = make_route(length_m=80.0)
    route.save(tmp_path / "synthetic.npz", k)
    monkeypatch.setattr(trail, "TRAIL_DIR", tmp_path)
    return "synthetic"


def test_a_world_built_on_a_trail_has_its_ground_sweep_and_content_along_it(app, synthetic_trail):
    from ocu.core.terrain import Terrain

    window = MainWindow(trail=synthetic_trail)
    trail = window.trail
    assert trail is not None and window.sweep.points and window.sweep.points[0][2] == "LEG 1"
    x, y = trail.point(0.0)
    assert window.terrain.classify(x, y) == Terrain.TRACK, "the deploy point is on the carved track"
    for t in window.target_field.targets[:5]:
        assert trail._tree.query([t.x, t.y])[0] < 3.5, "buried objects are laid along the trail, not across four straight lanes"
    assert not any(abs(e.path[0][0]) > 150 for e in window.surface_entities)
    window._plan_sweep_from_first_scan()
    assert window.sweep.points[0][2] == "LEG 1" and window._mission_waypoints is window.sweep.points, "the first scan does not replace the trail's sweep"
    window.close()


def test_the_rover_tile_on_a_trail_draws_from_the_real_place_where_it_was_walked_and_the_world_elsewhere(app, synthetic_trail):
    window = MainWindow(trail=synthetic_trail)
    feed = window.camera_feeds["rover"]
    feed.resize(320, 180)
    feed.set_trail_available(True)
    feed.set_trail(True)
    rover = window.units["rover"].pose
    rover.x, rover.y = window.trail.point(5.0)
    rover.heading = window.trail.bearing_deg
    request = window._camera_request("rover")
    assert request.get("trail_world_view") is True and request["entities"] is not None
    assert request["trail_s"] == pytest.approx(window.trail.legs[0].start_m + 5.0, abs=1.0) and request["trail_name"] == "synthetic"
    assert abs(request["eye"][0] - rover.x) < 1.0 and abs(request["eye"][1] - rover.y) < 1.0, "the splat camera is where the rover is"
    rover.heading = (window.trail.bearing_deg + 120.0) % 360.0  # turned away from the walk
    turned = window._camera_request("rover")
    assert turned.get("trail_world_view") is True and abs(turned["trail_yaw_deg"]) <= 30.0 + 1e-6, "facing a way the walk never looked: the real place, from the nearest camera it supports"
    rover.x, rover.y = window.trail.point(5.0, lateral_m=9.0)
    assert "trail_s" not in window._camera_request("rover"), "far from the path: the simulated world"
    window.close()


def test_the_command_line_asks_for_a_trail():
    from ocu.main import trail_argument

    assert trail_argument(["ocu"]) is None
    assert trail_argument(["ocu", "--trail"]) is True
    assert trail_argument(["ocu", "--trail", "IMG_5129"]) == "IMG_5129"


def test_a_world_built_on_several_trails_has_content_on_every_leg_and_asks_for_the_leg_it_is_on(app, tmp_path, monkeypatch):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent))
    from test_trail_scenario import make_route

    from ocu.render import trail

    for name, length in (("a", 70.0), ("b", 76.0), ("c", 82.0)):
        route, k = make_route(length_m=length)
        route.save(tmp_path / f"{name}.npz", k)
    monkeypatch.setattr(trail, "TRAIL_DIR", tmp_path)
    window = MainWindow(trail=True)
    assert window.trail.names == ["a", "b", "c"]
    labels = {label for _, _, label in window.sweep.points}
    assert labels == {"LEG 1", "LEG 2", "LEG 3"}
    names = [e.name for e in window.surface_entities]
    assert len(names) == len(set(names)), "the later legs' people and animals are named apart"
    assert len(window.surface_entities) > 4 and len(window.target_field.targets) > 5
    feed = window.camera_feeds["rover"]
    feed.resize(320, 180)
    feed.set_trail_available(True)
    feed.set_trail(True)
    leg = window.trail.legs[2]
    x, y = leg.point(6.0)
    i = int(__import__("numpy").searchsorted(leg.arc, leg.start_m + 6.0))
    import math

    rover = window.units["rover"].pose
    rover.x, rover.y, rover.heading = x, y, math.degrees(math.atan2(*leg.tangent[i])) % 360.0
    request = window._camera_request("rover")
    assert request["trail_name"] == "c" and request["trail_s"] == pytest.approx(leg.start_m + 6.0, abs=1.0)
    window.close()
