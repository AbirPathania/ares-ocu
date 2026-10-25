"""The weapon stage: a second pass on a person the camera has already found (decisions 47 and 48)."""

import json
import math

import pytest

import ocu.core.vision as pc
from ocu.core.entities import SurfaceEntity
from ocu.core.threat_filter import ThreatFilter, group_of


def armed(kind="long_gun", x=0.0, y=12.0):
    width = 1.00 if kind == "long_gun" else 0.54
    return SurfaceEntity("person", "armed", 1.79, width, 0.68, ((x, y),), carrying=kind, style="armed")


def unarmed(x=0.0, y=12.0):
    return SurfaceEntity("person", "walker", 1.78, 0.54, 0.63, ((x, y),))


# --- what the curves say ------------------------------------------------------------------------------------

def test_the_curves_are_the_measured_ones_and_say_where_they_came_from():
    c = pc.load_weapon_curves()
    assert c["decisions"] == [47, 48]
    assert set(c["curves"]) == {"handgun", "long_gun"}
    assert "crop around a detected person" in c["size_is"]
    assert c["false_alarms"]["rover_frames_with_a_false_weapon"] == 0
    assert any("upper bound" in s for s in c["caveats"]), "the photographs caveat is carried into the console"
    for name, curve in c["curves"].items():
        assert curve["bins"], name
        assert all(b["n"] >= 8 for b in curve["bins"]), "no bin is built from a handful of boxes"


def test_a_carried_rifle_fills_the_share_of_a_person_crop_the_curve_was_measured_at():
    share = pc.weapon_share(armed().weapon_size_m, armed())
    assert 0.06 < share < 0.12, f"{share:.3f}"
    assert 0.3 < pc.weapon_probability("long_gun", share) < 0.8, "found about half the frames it is in view"


def test_a_handgun_on_a_person_is_far_below_anything_that_was_measured_so_it_is_not_claimed_to_be_found():
    person = armed("handgun")
    share = pc.weapon_share(person.weapon_size_m, person)
    smallest = min(b["share_lo"] for b in pc.load_weapon_curves()["curves"]["handgun"]["bins"])
    assert share < smallest, "nothing that small was measured for a handgun"
    assert pc.weapon_probability("handgun", share) < 0.05, "so the console does not pretend to find one"


def test_an_unknown_weapon_class_is_never_found_rather_than_guessed():
    assert pc.weapon_probability("rocket", 0.5) == 0.0


# --- the sensor ---------------------------------------------------------------------------------------------

class Rig:
    def elevation_at(self, x, y):
        return 0.0

    def line_of_sight(self, *a):
        return True


def rover_cam(width=640, height=360):
    return pc.CameraSpec(0.0, 0.0, 1.5, 0.0, 3.0, 112.0, width, height, aerial=False)


def call_rate(entity, frames=400, distance=12.0):
    """How often the weapon stage calls something, over many frames."""
    sensor, rig, cam = pc.PerceptionSensor(), Rig(), rover_cam()
    moved = SurfaceEntity(entity.kind, entity.name, entity.height_m, entity.width_m, entity.length_m,
                          ((0.0, distance),), carrying=entity.carrying, style=entity.style)
    calls = 0
    for frame in range(frames):
        for d in sensor.observe("rover", cam, [moved], [(0.0, distance, 180.0)], rig, frame):
            calls += bool(d.weapon)
    return calls / frames


def test_a_rifle_carried_in_the_open_is_called_often_and_nothing_else_is_ever_called_on_it():
    sensor, rig, cam = pc.PerceptionSensor(), Rig(), rover_cam()
    seen = {d.weapon for frame in range(300) for d in sensor.observe("rover", cam, [armed()], [(0.0, 12.0, 180.0)], rig, frame) if d.weapon}
    assert seen == {"long_gun"}
    assert call_rate(armed()) > 0.2


def test_someone_carrying_nothing_is_called_armed_at_the_measured_rate_on_people_holding_ordinary_things():
    measured = pc.load_weapon_curves()["false_alarms"]["per_image"]
    rate = call_rate(unarmed(), frames=4000)
    assert rate == pytest.approx(measured, abs=0.02), f"{rate:.3f} against the measured {measured:.3f}"
    assert 0.0 < measured < 0.05


def test_a_weapon_is_not_called_on_an_animal_or_a_vehicle():
    sensor, rig, cam = pc.PerceptionSensor(), Rig(), rover_cam()
    for kind, size in (("animal", (1.9, 0.7, 2.1)), ("vehicle", (2.2, 2.1, 5.7))):
        e = SurfaceEntity(kind, kind, *size, ((0.0, 14.0),))
        assert not any(d.weapon for frame in range(200) for d in sensor.observe("rover", cam, [e], [(0.0, 14.0, 0.0)], rig, frame))


def test_too_far_away_there_is_nothing_in_the_crop_to_enlarge_and_the_stage_does_not_run():
    near, far = call_rate(armed(), distance=10.0), call_rate(armed(), distance=60.0)
    assert near > 0.2 and far == 0.0
    # the gate is a weapon of so many pixels across, so it has a definite range
    cam = rover_cam()
    reach = 1.00 / (pc.MIN_WEAPON_PIXELS * math.radians(cam.hfov_deg) / cam.width_px)
    assert 20.0 < reach < 45.0, f"{reach:.0f} m"
    assert call_rate(armed(), distance=reach * 0.8) > 0.0 and call_rate(armed(), distance=reach * 1.2) == 0.0


def test_the_same_seed_gives_the_same_calls():
    assert call_rate(armed()) == call_rate(armed())


# --- the persistence gate -----------------------------------------------------------------------------------

def test_one_frame_of_a_classifier_is_not_a_report():
    track = pc.Track(1, "person", "rover", 0.0, 0.0)
    track.weapons = ["", "long_gun", "", "", ""]
    assert track.weapon == "", "a single call is not enough"
    track.weapons = ["", "long_gun", "", "long_gun", ""]
    assert track.weapon == "long_gun"


def test_a_weapon_call_ages_out_when_it_stops_being_made():
    track = pc.Track(1, "person", "rover", 0.0, 0.0)
    track.weapons = ["long_gun", "long_gun"] + [""] * pc.CONFIRM_WINDOW
    assert track.weapon == "", "an old call does not stay on a track for ever"


def test_the_gate_records_a_weapon_call_against_the_track_it_belongs_to():
    gate = pc.ConfirmationGate()
    for frame in range(pc.CONFIRM_HITS):
        gate.update("rover", frame, [pc.Detection("person", 0.8, 4.0, 9.0, (0.4, 0.3, 0.5, 0.8), False, "armed", "long_gun")])
    track = gate.tracks[0]
    assert track.confirmed and track.weapon == "long_gun"


# --- how it is reported -------------------------------------------------------------------------------------

def test_an_armed_person_is_one_contact_in_the_arms_class_not_a_person_and_a_weapon():
    assert pc.WEAPON_THREAT_TYPE == "weapon"
    reported = {"type": pc.WEAPON_THREAT_TYPE, "label": pc.WEAPON_LABELS["long_gun"]}
    assert group_of(reported) == "arms"
    assert ThreatFilter().is_threat(reported), "arms is on by default"
    assert "armed person" in reported["label"] and "long gun" in reported["label"]


def test_the_console_curves_match_the_perception_repositorys_own_manifest():
    """The file the console reads is a copy; if the two drift apart the console is quoting a number nobody measured."""
    from pathlib import Path

    mirror = Path.home() / "Downloads" / "ares-perception" / "perception" / "data" / "manifests" / "ocu_weapon_curves.json"
    if not mirror.exists():
        pytest.skip("the perception repository is not beside this one")
    assert json.loads(mirror.read_text()) == pc.load_weapon_curves()
