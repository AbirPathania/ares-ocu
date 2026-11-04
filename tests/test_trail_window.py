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
    assert request["trail_s"] == pytest.approx(0.0, abs=1e-6) and request["entities"] is None

    window.mission_clock._clock_s = 10.0
    assert window._trail_progress_m() == pytest.approx(10.0 * window.TRAIL_PACE_MS % 51.5)
    window.mission_clock._clock_s = 100.0
    assert 0.0 <= window._trail_progress_m() < 51.5, "the replay goes round again at the end"

    press(feed, feed._chip_rect().center())  # switching to infrared gives the trail up
    assert not feed.trail_mode
    window.close()


def test_without_the_chip_the_rover_asks_for_the_simulated_world(app):
    window = MainWindow()
    assert "trail_s" not in window._camera_request("rover")
    window.close()
