"""The vision model: geometry, measured curves, persistence, and the rules the spec's enum imposes."""

import json
import math

import numpy as np
import pytest

from ocu.core import vision as pc
from ocu.core.constants import THREAT_CLASSES
from ocu.core.entities import KINDS, SurfaceEntity, default_entities


class FlatRig:
    """The two things perception asks of the camera rig: ground height and whether a line is clear."""

    def __init__(self, clear=True):
        self.clear = clear

    def elevation_at(self, x, y):
        return 0.0

    def line_of_sight(self, x, y, z, tx, ty, tz):
        return self.clear


def rover_cam(heading=0.0, w=640, h=360):
    return pc.CameraSpec(0.0, 0.0, 1.5, heading, 3.0, 112.0, w, h, aerial=False)


def scout_cam(heading=0.0):
    return pc.CameraSpec(0.0, 0.0, 12.0, heading, 18.0, 84.0, 1280, 486, aerial=True)


# ---- entities

def test_a_standing_entity_stays_and_a_walking_one_goes_back_and_forth_at_its_speed():
    still = SurfaceEntity("vehicle", "v", 1.9, 1.9, 5.0, ((3.0, 4.0),))
    assert still.pose_at(0.0)[:2] == still.pose_at(500.0)[:2] == (3.0, 4.0)
    walker = SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 0.0), (0.0, 10.0)), speed_ms=2.0)
    assert walker.pose_at(2.0)[:2] == pytest.approx((0.0, 4.0))
    assert walker.pose_at(2.0)[2] == pytest.approx(0.0), "walking north faces heading 0"
    x, y, heading = walker.pose_at(7.0)  # 14 m: 4 m back from the far end
    assert (x, y) == pytest.approx((0.0, 6.0)) and heading == pytest.approx(180.0)
    assert walker.pose_at(10.0)[:2] == pytest.approx((0.0, 0.0)), "a full out-and-back is 20 m"


def test_an_entity_must_have_a_known_kind_and_a_position():
    with pytest.raises(ValueError):
        SurfaceEntity("dragon", "d", 1, 1, 1, ((0, 0),))
    with pytest.raises(ValueError):
        SurfaceEntity("person", "p", 1, 1, 1, ())


def test_the_default_entities_cover_every_kind_and_are_inside_the_tether():
    ents = default_entities()
    assert set(e.kind for e in ents) == set(KINDS), "every kind is there"
    assert sorted(e.kind for e in ents) == sorted([*KINDS, "animal"]), "and there are two animals: a buck and an ibex"
    for e in ents:
        for x, y in e.path:
            assert math.hypot(x, y) < 50.0, f"{e.name} is beyond the 50 m tether"
    assert len({e.name for e in ents}) == len(ents)


# ---- geometry

def test_a_person_straight_ahead_is_centred_and_as_tall_on_screen_as_optics_says():
    cam = rover_cam()
    person = SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 20.0),))
    ap = pc.appearance(cam, person, (0.0, 20.0, 0.0), FlatRig())
    assert ap is not None
    assert (ap.box[0] + ap.box[2]) / 2 == pytest.approx(0.5, abs=0.01)
    # a rectilinear lens: height on the sensor is (h / range) over twice the tangent of half the vertical field
    expected_share = (1.75 / 20.0) / (2.0 * math.tan(math.radians(cam.vfov_deg) / 2.0))
    assert ap.share == pytest.approx(expected_share, rel=0.08)
    assert ap.ground_range_m == pytest.approx(20.0)


def test_things_behind_to_the_side_or_hidden_are_not_in_view():
    person = SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 20.0),))
    cam = rover_cam()
    assert pc.appearance(cam, person, (0.0, -20.0, 0.0), FlatRig()) is None, "behind"
    assert pc.appearance(cam, person, (80.0, 5.0, 0.0), FlatRig()) is None, "far off to the side"
    assert pc.appearance(cam, person, (0.0, 20.0, 0.0), FlatRig(clear=False)) is None, "something in the way"


def test_a_pixel_row_recovers_the_ground_position_to_within_a_pixel_of_noise():
    cam = scout_cam()
    person = SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 25.0),))
    ap = pc.appearance(cam, person, (3.0, 25.0, 0.0), FlatRig())
    assert ap is not None
    errors = []
    for seed in range(200):
        rng = np.random.default_rng(seed)
        x, y = pc.ground_position(cam, ap.box[3], (ap.box[0] + ap.box[2]) / 2, 0.0, rng)
        errors.append(math.hypot(x - 3.0, y - 25.0))
    assert np.median(errors) < 1.5, "a two-pixel row error at 25 m from 12 m up is about a metre"
    assert max(errors) < 6.0


# ---- the measured curves

def test_the_curves_file_carries_its_provenance_and_has_sane_numbers():
    c = pc.load_curves()
    assert c["model_checkpoint_sha256"] and len(c["model_checkpoint_sha256"]) == 64 and c["operating_threshold"] > 0
    for kind, curve in c["curves"].items():
        recalls = [b["recall"] for b in curve["bins"]]
        assert recalls and all(0.0 <= r <= 1.0 for r in recalls), kind
        assert all(b["n"] >= 8 and b["share_lo"] < b["share_hi"] for b in curve["bins"]), kind
    assert c["false_alarms"]["vehicle"] is None, "no vehicle false-alarm rate is known and none is invented"
    assert 0.0 < c["false_alarms"]["person_or_animal_per_frame"] < 0.1
    assert c["caveats"], "the limits travel with the numbers"


@pytest.mark.parametrize("kind", ["animal", "person", "vehicle"])
def test_detection_probability_rises_with_apparent_size_and_never_leaves_zero_to_one(kind):
    shares = [0.005, 0.02, 0.05, 0.1, 0.2, 0.4, 0.7]
    p = [pc.detection_probability(kind, s, aerial=False) for s in shares]
    assert all(0.0 <= x <= 1.0 for x in p)
    assert p[-1] >= p[0]
    assert p[0] < 0.3, "a speck is mostly missed"
    assert pc.detection_probability(kind, 0.0, False) == 0.0


def test_from_the_air_a_person_at_scout_distances_is_not_found_but_a_near_one_is_treated_like_a_ground_one():
    assert pc.detection_probability("person", 0.04, aerial=True) == 0.0, "0 of 505 drone-frame people were found at that size"
    assert pc.detection_probability("person", 0.04, aerial=False) > 0.4, "the ground curve is far more optimistic: that is the difference"
    assert pc.detection_probability("person", 0.3, aerial=True) == pc.detection_probability("person", 0.3, aerial=False)


# ---- a sensor, deterministic and calibrated

def test_the_same_mission_seed_gives_the_same_detections_and_a_different_seed_does_not():
    cam, person = rover_cam(), SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 12.0),))
    def run(seed):
        s = pc.PerceptionSensor(seed)
        return [(len(s.observe("rover", cam, [person], [(0.0, 12.0, 0.0)], FlatRig(), f))) for f in range(300)]
    assert run(1) == run(1)
    assert run(1) != run(2)


def test_over_many_frames_a_sensor_finds_things_at_the_measured_rate():
    cam, person = rover_cam(), SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 12.0),))
    ap = pc.appearance(cam, person, (0.0, 12.0, 0.0), FlatRig())
    expected = pc.detection_probability("person", ap.share, False)
    s = pc.PerceptionSensor(7)
    hits = sum(any(not d.false_alarm for d in s.observe("rover", cam, [person], [(0.0, 12.0, 0.0)], FlatRig(), f)) for f in range(6000))
    assert hits / 6000 == pytest.approx(expected, abs=0.03)


def test_invented_detections_happen_at_the_measured_rate_and_are_marked_as_such():
    cam = rover_cam()
    s = pc.PerceptionSensor(3)
    n = 20000
    fa = [d for f in range(n) for d in s.observe("rover", cam, [], [], FlatRig(), f)]
    assert all(d.false_alarm and d.truth == "false alarm" for d in fa)
    assert len(fa) / n == pytest.approx(s.false_per_frame, abs=0.006)
    assert all(d.score >= s.floor for d in fa)


def test_scores_are_at_least_the_operating_threshold_and_never_certain():
    cam, person = rover_cam(), SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 8.0),))
    s = pc.PerceptionSensor(11)
    scores = [d.score for f in range(2000) for d in s.observe("rover", cam, [person], [(0.0, 8.0, 0.0)], FlatRig(), f) if not d.false_alarm]
    assert scores and min(scores) >= s.floor and max(scores) <= pc.MAX_CONFIDENCE


# ---- persistence

def det(kind="person", x=10.0, y=10.0, score=0.7, truth="p"):
    return pc.Detection(kind, score, x, y, (0.4, 0.4, 0.5, 0.7), truth == "false alarm", truth)


def test_a_track_is_confirmed_on_its_third_hit_in_five_frames_and_only_once():
    g = pc.ConfirmationGate()
    assert g.update("rover", 0, [det()]) == [] and g.update("rover", 1, [det(x=10.4)]) == []
    confirmed = g.update("rover", 2, [det(x=10.8)])
    assert len(confirmed) == 1 and confirmed[0].confirmed
    assert g.update("rover", 3, [det(x=11.0)]) == [], "already confirmed: not announced again"


def test_a_lone_flicker_never_confirms_and_the_gate_forgets_it():
    g = pc.ConfirmationGate()
    assert g.update("rover", 0, [det(truth="false alarm")]) == []
    for f in range(1, pc.TRACK_MEMORY_FRAMES + 2):
        assert g.update("rover", f, []) == []
    assert g.tracks == [], "forgotten once it has gone unseen for the track memory"


def test_scattered_hits_outside_the_window_do_not_add_up_and_far_apart_detections_are_different_objects():
    g = pc.ConfirmationGate()
    g.update("rover", 0, [det()])
    g.update("rover", 4, [det()])  # a track survives 4 empty frames? it expires when 5 pass
    assert g.update("rover", 9, [det()]) == []
    h = pc.ConfirmationGate()
    h.update("rover", 0, [det(x=0.0)])
    h.update("rover", 1, [det(x=30.0)])
    assert h.update("rover", 2, [det(x=60.0)]) == [], "three detections in three places are three tracks"


def test_tracks_do_not_mix_sensors_or_kinds():
    g = pc.ConfirmationGate()
    g.update("rover", 0, [det()])
    g.update("aircraft", 1, [det()])
    assert g.update("rover", 2, [det(kind="animal")]) == []


def test_a_track_that_goes_unseen_for_a_few_frames_is_the_same_track_when_it_returns():
    g = pc.ConfirmationGate()
    g.update("rover", 0, [det()])
    g.update("rover", 1, [det()])
    ids = {t.id for t in g.tracks}
    g.update("rover", 8, [det(x=10.5)])  # seven frames unseen, still inside the memory
    assert {t.id for t in g.tracks} == ids


def test_re_confirmations_of_one_thing_are_one_contact_and_far_apart_things_are_two():
    r = pc.ContactRegistry()
    a = r.resolve("vehicle", 20.0, -13.0)
    b = r.resolve("vehicle", 22.5, -15.0)  # the same truck, placed a couple of metres differently
    c = r.resolve("vehicle", 60.0, 5.0)
    assert a.id == b.id and a.reports == 2 and c.id != a.id
    assert (b.x, b.y) == (20.0, -13.0), "the marker stays where it was first reported"
    assert r.resolve("person", 20.0, -13.0).id != a.id, "different kinds are different contacts"
    assert len(r.contacts) == 3


def test_a_gate_needs_sensible_parameters():
    with pytest.raises(ValueError):
        pc.ConfirmationGate(hits=6, window=5)
    with pytest.raises(ValueError):
        pc.ConfirmationGate(hits=0, window=5)
    with pytest.raises(ValueError):
        pc.ConfirmationGate(hits=3, window=5, memory=4)


def test_confirmation_removes_almost_all_invented_reports_and_keeps_real_ones():
    """The gate's job, measured on this model: 20000 frames of nothing, and 2000 frames of a person in view."""
    cam, person = rover_cam(), SurfaceEntity("person", "p", 1.75, 0.5, 0.3, ((0.0, 14.0),))
    s = pc.PerceptionSensor(5)
    empty = pc.ConfirmationGate()
    false_reports = sum(len(empty.update("rover", f, s.observe("rover", cam, [], [], FlatRig(), f))) for f in range(20000))
    raw = sum(len(s.observe("rover", cam, [], [], FlatRig(), f)) for f in range(20000))
    assert raw > 100 and false_reports <= raw * 0.05, "single-frame invented detections almost never persist"
    seen = pc.ConfirmationGate()
    s2 = pc.PerceptionSensor(6)
    real = sum(len(seen.update("rover", f, s2.observe("rover", cam, [person], [(0.0, 14.0, 0.0)], FlatRig(), f))) for f in range(2000))
    assert real >= 1


# ---- what the spec's enum allows

def test_an_animal_is_reported_as_unknown_because_the_eighth_class_is_an_open_decision():
    assert pc.THREAT_TYPE["animal"] == "unknown"
    assert all(t in THREAT_CLASSES for t in pc.THREAT_TYPE.values()), "no type outside the spec's enum is ever invented"
    assert "animal" not in THREAT_CLASSES
