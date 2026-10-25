"""The operator's choice of threat classes, on the real window: what a contact is and is not."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint, Qt, QTimer  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import ocu.ui.threat_filter_bar as bar_module  # noqa: E402
from ocu.core.constants import IRREVERSIBLE_COMMAND_HOLD_S  # noqa: E402
from ocu.core.entities import SurfaceEntity  # noqa: E402
from ocu.ui.main_window import MainWindow  # noqa: E402

VEHICLE = {"x": 10.0, "y": 10.0, "confidence": 0.95, "type": "vehicle", "label": "vehicle", "sensors": ["camera"], "sensor": "camera", "source": "rover", "source_id": "rover"}
ANIMAL = {"x": 20.0, "y": 5.0, "confidence": 0.95, "type": "unknown", "label": "animal", "sensors": ["camera"], "sensor": "camera", "source": "rover", "source_id": "rover"}
PERSON = {"x": 7.0, "y": 7.0, "confidence": 0.95, "type": "person", "label": "person", "sensors": ["camera"], "sensor": "camera", "source": "rover", "source_id": "rover"}
MINE = {"x": 3.0, "y": 9.0, "confidence": 0.95, "type": "mine", "label": "mine", "sensors": ["gpr"], "sensor": "gpr", "source": "rover", "source_id": "rover"}


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


def log_lines(window):
    log = window.event_log.log_list
    return [log.item(i).text() for i in range(log.count())]


def listed(window):
    return [t for t in window.threat_panel._threats.values() if window.threat_panel._shown(t)]


def test_a_vehicle_is_a_contact_by_default_not_a_threat(window):
    window._on_detection(dict(VEHICLE))
    assert not window._escalated, "no alert, no decision demanded"
    assert listed(window) == [], "not in the threat list"
    assert any(d["type"] == "vehicle" for d in window.map_view._detections), "but on the map"
    assert not window.map_view._is_threat(window.map_view._detections[0])
    assert window.threat_filter_bar.contacts.text().startswith("CONTACTS") and "1 vehicle" in window.threat_filter_bar.contacts.text()


def test_a_contact_is_logged_quietly_as_a_contact(window):
    window._on_detection(dict(VEHICLE))
    assert any("Contact" in e and "VEHICLE" in e for e in log_lines(window))


def test_a_contact_is_still_shared_with_the_fleet(window):
    seen = []
    window.mesh_link.observe = lambda side, observation: seen.append((side, observation.threat_type))
    assert window._comms.mesh_traffic, "the default link carries mesh traffic"
    window._on_detection(dict(VEHICLE))
    assert seen == [("rover", "vehicle")], "the filter is the operator's view, not the fleet's record"


def test_a_person_and_a_buried_mine_are_threats_by_default_and_raise_alerts(window):
    window._on_detection(dict(PERSON))
    window._on_detection(dict(MINE))
    assert {(7.0, 7.0), (3.0, 9.0)} <= window._escalated
    assert {t["type"] for t in listed(window)} == {"person", "mine"}


def test_turning_vehicles_on_lists_the_contact_and_lets_the_next_one_alert(window):
    window._on_detection(dict(VEHICLE))
    assert listed(window) == []
    window.threat_filter.set_enabled("vehicles", True)
    assert [t["type"] for t in listed(window)] == ["vehicle"], "the contact becomes a pending threat"
    assert window.map_view._is_threat(window.map_view._detections[0])
    assert not window._escalated, "no retroactive alert"
    window._on_detection(dict(VEHICLE, x=30.0, y=30.0))
    assert (30.0, 30.0) in window._escalated, "a detection that arrives after the change does alert"
    assert any("Treating as threats" in e and "VEHICLES" in e for e in log_lines(window)), "the choice is logged"


def test_turning_a_class_off_removes_its_cards_but_a_confirmed_one_stays(window):
    window._on_detection(dict(PERSON))
    window._on_detection(dict(PERSON, x=40.0, y=40.0))
    window.threat_panel.set_status(40.0, 40.0, "CONFIRMED")
    window.threat_filter.set_enabled("people", False)
    shown = listed(window)
    assert [t["status"] for t in shown] == ["CONFIRMED"], "a decision the operator already made stands"
    assert "1 CONFIRMED" in window.threat_panel.header.text()


def test_an_animal_alerts_only_when_the_operator_turns_animals_on(window):
    window._on_detection(dict(ANIMAL))
    assert not window._escalated
    window.threat_filter.set_enabled("animals", True)
    window._on_detection(dict(ANIMAL, x=25.0, y=25.0))
    assert (25.0, 25.0) in window._escalated


def test_a_camera_box_is_muted_for_a_contact_and_normal_for_a_threat(window):
    window.surface_entities = [SurfaceEntity("vehicle", "v", 1.9, 2.0, 5.0, ((window.units["rover"].pose.x, window.units["rover"].pose.y + 9.0),))]
    for f in range(40):
        window.mission_clock._clock_s = f / 2.0 + 0.01
        window._tick_vision()
    boxes = window.camera_feeds["rover"]._detections
    assert boxes and all(b["threat"] is False for b in boxes), "vehicles are off by default"
    window.threat_filter.set_enabled("vehicles", True)
    window.mission_clock._clock_s += 0.5
    window._vision_frame = -1
    window._tick_vision()
    assert all(b["threat"] is True for b in window.camera_feeds["rover"]._detections)


def test_the_chips_show_the_defaults_and_every_one_of_them_says_what_drives_it(window):
    chips = window.threat_filter_bar.chips
    assert {g for g, c in chips.items() if c.on} == {"explosives", "arms", "people"}
    for group, chip in chips.items():
        assert "Found by" in chip.toolTip(), f"{group} does not say where its detections come from"
    QTest.mouseClick(chips["arms"], Qt.LeftButton)
    assert not chips["arms"].on and not window.threat_filter.is_enabled("arms"), "arms is a working control now"
    QTest.mouseClick(chips["arms"], Qt.LeftButton)
    assert chips["arms"].on


def test_a_plain_click_switches_people_off_but_mines_need_the_hold(window, monkeypatch):
    chips = window.threat_filter_bar.chips
    QTest.mouseClick(chips["people"], Qt.LeftButton)
    assert not window.threat_filter.is_enabled("people"), "one click"
    monkeypatch.setattr(bar_module, "IRREVERSIBLE_COMMAND_HOLD_S", 0.3)
    QTest.mouseClick(chips["explosives"], Qt.LeftButton)
    assert window.threat_filter.is_enabled("explosives"), "a click and release is not enough"
    QTest.mousePress(chips["explosives"], Qt.LeftButton, pos=QPoint(5, 5))
    assert chips["explosives"].holding
    QTest.qWait(500)
    QTest.mouseRelease(chips["explosives"], Qt.LeftButton, pos=QPoint(5, 5))
    assert not window.threat_filter.is_enabled("explosives"), "held long enough"
    QTest.mouseClick(chips["explosives"], Qt.LeftButton)
    assert window.threat_filter.is_enabled("explosives"), "switching them back on is one click"


def test_releasing_early_cancels_the_hold(window, monkeypatch):
    monkeypatch.setattr(bar_module, "IRREVERSIBLE_COMMAND_HOLD_S", 0.6)
    chip = window.threat_filter_bar.chips["explosives"]
    QTest.mousePress(chip, Qt.LeftButton, pos=QPoint(5, 5))
    QTest.qWait(150)
    QTest.mouseRelease(chip, Qt.LeftButton, pos=QPoint(5, 5))
    QTest.qWait(700)
    assert window.threat_filter.is_enabled("explosives")


def test_the_hold_is_the_same_two_seconds_as_any_irreversible_command():
    assert bar_module.IRREVERSIBLE_COMMAND_HOLD_S == IRREVERSIBLE_COMMAND_HOLD_S == 2.0


def test_a_buried_device_first_read_as_unknown_is_listed_once_it_is_known_to_be_a_mine(window):
    window.show()
    QTest.qWait(50)
    window._on_detection(dict(MINE, type="unknown", label="unknown", confidence=0.3))
    assert listed(window) == [], "an unidentified hit is an 'other' contact, which is off"
    window._on_detection(dict(MINE))
    assert [t["type"] for t in listed(window)] == ["mine"]
    card = next(iter(window.threat_panel._cards.values()))
    QTest.qWait(400)
    assert card.isVisible(), "and its card actually appears, not only its count"
