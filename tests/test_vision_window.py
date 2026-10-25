"""The camera detector, wired into the real window: what an operator would see."""

import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ocu.core.entities import SurfaceEntity  # noqa: E402
from ocu.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    yield QApplication.instance() or QApplication([])


@pytest.fixture
def window(app):
    w = MainWindow()
    for timer in w.findChildren(QTimer):
        timer.stop()
    yield w
    w.close()


def ahead_of_rover(window, metres, kind="person", name="p", height=1.75):
    pose = window.units["rover"].pose
    h = math.radians(pose.heading)
    x, y = pose.x + metres * math.sin(h), pose.y + metres * math.cos(h)
    return SurfaceEntity(kind, name, height, 0.5, 0.4, ((x, y),))


def run_frames(window, frames):
    for f in range(frames):
        window.mission_clock._clock_s = f / 2.0 + 0.01  # one detector frame per half second
        window._tick_vision()


def test_the_window_starts_with_one_of_each_kind_and_a_camera_detector(window):
    assert sorted(e.kind for e in window.surface_entities) == ["animal", "animal", "person", "person", "vehicle"]
    assert [e.name for e in window.surface_entities if e.carrying] == ["armed walker"], "one of the two people is carrying a rifle"
    run_frames(window, 3)  # runs without error at the start of a mission


def test_a_person_in_front_of_the_rover_is_boxed_confirmed_and_reported_without_flooding_the_panel(window):
    window.surface_entities = [ahead_of_rover(window, 9.0)]
    boxes_seen = 0
    for f in range(200):
        window.mission_clock._clock_s = f / 2.0 + 0.01
        window._tick_vision()
        boxes_seen += bool(window.camera_feeds["rover"]._detections)
    assert boxes_seen > 20, "the detector's boxes are drawn on the rover's feed"
    types = [t["type"] for t in window.threat_panel._threats.values()]
    assert "person" in types, "a persistent person becomes a report"
    assert len(window.threat_panel._threats) <= 4, "a person that stays in view is one threat, not a stack of them"
    assert all(t.get("label") == "person" for t in window.threat_panel._threats.values())


def test_a_vehicle_is_reported_as_a_vehicle_and_an_animal_as_unknown_never_as_a_new_class(window):
    window.surface_entities = [ahead_of_rover(window, 9.0, "vehicle", "v", 1.9), ahead_of_rover(window, 14.0, "animal", "a", 1.0)]
    run_frames(window, 240)
    types = {t["type"]: t["label"] for t in window.threat_panel._threats.values()}
    assert types.get("vehicle") == "vehicle"
    assert types.get("unknown") == "animal", "the spec's enum has no animal class; the label keeps the word"
    assert all(t in ("person", "vehicle", "unknown", "mine", "ied", "disturbed_soil", "safe_object") for t in types)


def test_an_animal_raises_no_alert_by_default_but_a_person_does(window):
    window._on_detection({"x": 5.0, "y": 5.0, "confidence": 0.95, "type": "unknown", "label": "animal", "sensors": ["camera"], "source": "rover", "source_id": "rover"})
    assert not window._escalated, "animals are off in the operator's threat classes: a contact is logged and drawn, and demands no decision"
    window._on_detection({"x": 7.0, "y": 7.0, "confidence": 0.95, "type": "person", "label": "person", "sensors": ["camera"], "source": "rover", "source_id": "rover"})
    assert (7.0, 7.0) in window._escalated


def test_the_gate_keeps_the_empty_ground_quiet_while_the_raw_detector_does_not(window):
    window.surface_entities = []
    window.camera_detector.false_per_frame = 0.05  # turned up so a short run is a fair test
    raw = 0
    for f in range(3000):
        window.mission_clock._clock_s = f / 2.0 + 0.01
        window._tick_vision()
        raw += len([1 for t in window.camera_gate.tracks if t.hits and t.hits[-1] == f])
    assert raw > 40, "the detector does invent things when asked to"
    assert len(window.threat_panel._threats) <= 3, "almost none of them persist long enough to be reported"


def test_reset_clears_boxes_tracks_and_what_was_reported(window):
    window.surface_entities = [ahead_of_rover(window, 9.0)]
    run_frames(window, 120)
    window._on_reset_clicked()
    assert window.camera_gate.tracks == [] and not window.camera_contacts.contacts
    assert window.camera_feeds["rover"]._detections == []


# ---- the weapon stage, on the real window

def armed_ahead(window, metres):
    pose = window.units["rover"].pose
    h = math.radians(pose.heading)
    x, y = pose.x + metres * math.sin(h), pose.y + metres * math.cos(h)
    return SurfaceEntity("person", "armed", 1.79, 1.00, 0.68, ((x, y),), box_offset_m=-0.098, style="armed", carrying="long_gun")


def test_a_person_carrying_a_rifle_is_reported_as_armed_and_lands_in_the_arms_class(window):
    from ocu.core.threat_filter import group_of

    window.surface_entities = [armed_ahead(window, 10.0)]
    run_frames(window, 120)
    threats = list(window.threat_panel._threats.values())
    assert threats, "the camera found them"
    armed_threats = [t for t in threats if t["type"] == "weapon"]
    assert armed_threats, f"reported as {[t['type'] for t in threats]}"
    assert all(group_of(t) == "arms" for t in armed_threats)
    assert "armed person" in armed_threats[0]["label"]
    assert len(threats) <= 3, "one person is one contact, not a person and a weapon side by side"


def test_an_unarmed_person_in_the_same_place_is_not_reported_as_armed(window):
    window.surface_entities = [ahead_of_rover(window, 10.0)]
    run_frames(window, 120)
    assert [t["type"] for t in window.threat_panel._threats.values()].count("weapon") == 0


def test_the_armed_contact_raises_an_alert_while_arms_is_on_and_falls_quiet_when_it_is_off(window):
    window.threat_filter.set_enabled("arms", False)
    window.surface_entities = [armed_ahead(window, 10.0)]
    run_frames(window, 120)
    assert not window._escalated, "arms switched off: a contact, no decision demanded"
    listed = [t for t in window.threat_panel._threats.values() if window.threat_panel._shown(t)]
    assert not any(t["type"] == "weapon" for t in listed), "and not in the threat list"
    window.threat_filter.set_enabled("arms", True)
    listed = [t for t in window.threat_panel._threats.values() if window.threat_panel._shown(t)]
    assert any(t["type"] == "weapon" for t in listed), "switched back on, it is listed"
