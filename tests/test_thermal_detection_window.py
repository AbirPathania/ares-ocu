"""The thermal detector, wired into the real window: confirmed, reported once, and folded in with the visible camera."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import ocu.ui.main_window as main_window  # noqa: E402
from ocu.core.thermal_detect import ThermalContact  # noqa: E402
from ocu.core.thermal_system import ThermalSystem  # noqa: E402
from ocu.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture
def window(app, monkeypatch):
    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    monkeypatch.setattr(ThermalSystem, "ready", property(lambda self: True))
    monkeypatch.setattr(ThermalSystem, "look", lambda self, *a, **k: None)  # the picture is not what is under test
    monkeypatch.setattr(MainWindow, "THERMAL_STEP_WALL_S", 0.0)
    yield w
    w.close()


def contact(kind, x, y, confidence=0.8):
    return ThermalContact(kind, x, y, 20.0, 0.6, 1.8, 6.0, 300, confidence, (0.4, 0.3, 0.5, 0.6), -1)


def steps(window, found, count):
    poses = window._entity_poses()
    for _ in range(count):
        window._tick_thermal_detection(poses)


def test_a_person_the_imager_shows_for_several_steps_is_reported_once_as_thermal(window, monkeypatch):
    monkeypatch.setattr(main_window.thermal_detect, "detect", lambda frame, kind: [contact("person", 30.0, 40.0)])
    steps(window, None, 30)
    threats = list(window.threat_panel._threats.values())
    people = [t for t in threats if t["type"] == "person"]
    assert len(people) == 1, "one person is one threat however many looks confirm them"
    assert "thermal" in people[0]["sensors"]


def test_one_look_is_not_a_report(window, monkeypatch):
    monkeypatch.setattr(main_window.thermal_detect, "detect", lambda frame, kind: [contact("person", 30.0, 40.0)])
    steps(window, None, 1)
    assert not window.threat_panel._threats


def test_an_anomaly_or_an_unclassed_object_is_not_reported(window, monkeypatch):
    monkeypatch.setattr(main_window.thermal_detect, "detect",
                        lambda frame, kind: [contact("anomaly", 30.0, 40.0), contact("object", 10.0, 10.0)])
    steps(window, None, 30)
    assert not window.threat_panel._threats


def test_a_second_sensors_plain_person_does_not_undo_armed_person(window):
    panel = window.threat_panel
    panel.add_detection({"x": 5.0, "y": 5.0, "type": "weapon", "label": "armed person", "confidence": 0.7, "sensors": ["camera"]})
    panel.add_detection({"x": 5.0, "y": 5.0, "type": "person", "label": "person", "confidence": 0.9, "sensors": ["thermal"]})
    (threat,) = panel._threats.values()
    assert threat["type"] == "weapon" and threat["label"] == "armed person"
    assert threat["confidence"] == pytest.approx(0.9)
    assert threat["sensors"] == ["camera", "thermal"]


def test_the_thermal_boxes_are_what_an_ir_tile_draws(window, monkeypatch):
    monkeypatch.setattr(main_window.thermal_detect, "detect", lambda frame, kind: [contact("person", 30.0, 40.0)])
    steps(window, None, 6)
    boxes = window._thermal_boxes["rover"]
    assert boxes and boxes[0]["box"] == (0.4, 0.3, 0.5, 0.6)


def test_a_thermal_vehicle_is_not_raised(window, monkeypatch):
    monkeypatch.setattr(main_window.thermal_detect, "detect", lambda frame, kind: [contact("vehicle", 30.0, 40.0)])
    steps(window, None, 30)
    assert not window.threat_panel._threats
