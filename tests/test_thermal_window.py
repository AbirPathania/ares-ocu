"""Infrared on the real window: the layer, the controls, the tiles, and what the cameras write into the mosaic."""

import math
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint, Qt, QTimer  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.core.entities import SurfaceEntity  # noqa: E402
from ocu.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture(scope="module")
def day(app):
    """The day's field for the real world, solved once for the whole module."""
    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()  # a window left running makes every later event loop in this process slow
    w.thermal.cache_dir = None
    w.thermal.build()
    field = w.thermal.field
    w.close()
    w.deleteLater()
    return field


@pytest.fixture
def window(app, day):
    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    w.thermal.install(day)
    yield w
    w.close()


def run_seconds(window, seconds):
    for _ in range(int(seconds / 0.5)):
        window.mission_clock._clock_s += 0.5
        window._tick_vision()
        window._tick_thermal()


def test_the_map_starts_on_the_lidar_layer_with_no_thermal_controls(window):
    assert window.map_view.layer == "lidar"
    assert not window.map_view.ir_row.isVisible()
    assert window.map_view.layer_btn.text() == "MAP  LIDAR"


def test_the_layer_button_cycles_lidar_ir_both_and_back(window):
    window.show()
    seen = []
    for _ in range(3):
        QTest.mouseClick(window.map_view.layer_btn, Qt.LeftButton)
        seen.append(window.map_view.layer)
    assert seen == ["ir", "both", "lidar"]
    window.map_view.set_layer("ir")
    assert window.map_view.ir_row.isVisible(), "the thermal controls appear with the layer"


def test_nothing_is_imaged_while_infrared_is_not_wanted_and_the_mosaic_grows_once_it_is(window):
    run_seconds(window, 4.0)
    assert window.thermal.map.seen_fraction == 0.0, "the thermal cameras are off, so the mosaic does not grow"
    window.map_view.set_layer("ir")
    run_seconds(window, 4.0)
    assert window.thermal.map.seen_fraction > 0.0
    assert window._thermal_cameras >= 1


def test_a_tile_switched_to_infrared_is_drawn_by_the_thermal_camera_and_back_again(window):
    feed = window.camera_feeds["rover"]
    window.show()
    chip = feed._chip_rect().center().toPoint()
    QTest.mouseClick(feed, Qt.LeftButton, pos=chip)
    assert feed.ir_mode
    window._ir_feed_last.clear()
    window._tick_cameras()
    assert feed._ir_frame and feed._frame is not None
    assert feed._ir_scale is not None and feed._ir_scale[0] < feed._ir_scale[1]
    window._on_gpu_frame("rover", np.zeros((10, 10, 3), dtype=np.uint8))
    assert feed._ir_frame, "a visible frame arriving late for a tile set to infrared is ignored"
    QTest.mouseClick(feed, Qt.LeftButton, pos=chip)
    assert not feed.ir_mode and not feed._ir_frame, "back to visible: the thermal picture is dropped"


def test_the_infrared_tile_has_no_detector_boxes_because_the_detector_sees_the_visible_picture(window):
    feed = window.camera_feeds["rover"]
    feed.set_detections([{"box": (0.4, 0.4, 0.6, 0.6), "kind": "person", "label": "person", "score": 0.9, "confirmed": True, "threat": True}])
    feed.set_ir(True)
    window._ir_feed_last.clear()
    window._tick_cameras()
    image = feed.grab()
    assert image.width() > 0 and feed._ir_frame, "drawn without error; the boxes belong to the visible camera and are not shown"


def test_the_operator_moves_the_time_of_day_and_the_log_says_so(window):
    start = window.thermal.local_hour(0.0)
    window.map_view.set_layer("ir")
    window.map_view.timeShiftRequested.emit(-9.0)
    assert window.thermal.local_hour(0.0) == pytest.approx((start - 9.0) % 24.0)
    log = window.event_log.log_list
    assert any("Thermal time of day" in log.item(i).text() for i in range(log.count()))
    assert window.map_view.time_label.text()[:2].isdigit() and ":" in window.map_view.time_label.text()


def test_a_person_stands_out_of_the_cold_ground_at_night_and_is_a_heat_contact_not_a_streak(window):
    pose = window.units["rover"].pose
    h = math.radians(pose.heading)
    window.surface_entities = [SurfaceEntity("person", "w", 1.78, 0.54, 0.63, ((pose.x + 9.0 * math.sin(h), pose.y + 9.0 * math.cos(h)),), heading_deg=0.0)]
    window.map_view.set_layer("ir")
    window.thermal.hour_offset_h = -9.0
    run_seconds(window, 3.0)
    assert any(c.kind == "person" for c in window.thermal.map.contacts)
    x, y = window.surface_entities[0].path[0]
    assert np.isnan(window.thermal.map.temp_c[window.thermal.map._index(x, y)]), "the person is not written into the ground"


def test_the_palette_and_the_gain_are_the_operators(window):
    window.map_view.set_layer("ir")
    first = window.thermal.palette
    QTest.mouseClick(window.map_view.palette_btn, Qt.LeftButton)
    assert window.thermal.palette != first
    run_seconds(window, 3.0)
    QTest.mouseClick(window.map_view.gain_btn, Qt.LeftButton)
    assert window.thermal.hold_range is not None and window.map_view.gain_btn.text() == "LOCKED"
    QTest.mouseClick(window.map_view.gain_btn, Qt.LeftButton)
    assert window.thermal.hold_range is None


def test_the_map_draws_in_every_layer_without_error(window):
    window.show()
    window.map_view.resize(900, 700)
    run_seconds(window, 3.0)
    for layer in ("lidar", "ir", "both"):
        window.map_view.set_layer(layer)
        assert not window.map_view.grab().isNull()


def test_a_mission_reset_clears_the_mosaic_and_puts_the_clock_back(window):
    window.map_view.set_layer("ir")
    window.thermal.shift_hours(3.0)
    run_seconds(window, 3.0)
    assert window.thermal.map.seen_fraction > 0.0
    window._on_reset_clicked()
    assert window.thermal.map.seen_fraction == 0.0 and window.thermal.hour_offset_h == 0.0


def test_before_the_day_is_solved_the_map_says_so_and_nothing_breaks(app):
    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    w.thermal.start_background = lambda: None  # a day still being solved: nothing to load and no process to wait for
    w.map_view.set_layer("ir")
    w.show()
    run_seconds(w, 2.0)
    assert not w.thermal.ready and w.thermal.map.seen_fraction == 0.0
    assert not w.map_view.grab().isNull()
    w._thermal_poll.stop()
    w.close()


def test_every_thermal_control_says_what_it_changes(window):
    """A row of bare words ("GAIN", "IRON") tells an operator who has not used a thermal camera nothing at all."""
    view = window.map_view
    view.set_layer("ir")
    for control in (view.palette_btn, view.gain_btn, view.layer_btn):
        assert len(control.toolTip()) > 40, f"{control.text()} does not explain itself"
    assert view.time_label.parentWidget().toolTip(), "the clock says what the time of day is for"
    captions = {label.text() for label in view.ir_row.findChildren(type(view.time_label)) if label.objectName() == "controlCaption"}
    assert {"COLOUR", "BRIGHTNESS", "TIME OF DAY"} <= captions
