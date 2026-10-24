import pytest

from ocu.core.threat_filter import DEFAULT_ENABLED, GROUPS, GUARDED, LABELS, UNAVAILABLE, ThreatFilter, group_of


@pytest.mark.parametrize(
    "detection,group",
    [
        ({"type": "mine"}, "explosives"),
        ({"type": "ied"}, "explosives"),
        ({"type": "disturbed_soil"}, "explosives"),
        ({"type": "person", "label": "person"}, "people"),
        ({"type": "vehicle", "label": "vehicle"}, "vehicles"),
        ({"type": "unknown", "label": "animal"}, "animals"),
        ({"type": "unknown"}, "other"),
        ({"type": "safe_object"}, "other"),
        ({"type": "weapon"}, "arms"),
        ({"type": "something the spec never named"}, "other"),
        ({}, "other"),
    ],
)
def test_every_detection_falls_in_exactly_one_class(detection, group):
    assert group_of(detection) == group


def test_the_defaults_are_a_clearance_and_border_role():
    f = ThreatFilter()
    assert f.enabled == {"explosives", "people"}, "arms is on by default in intent but has no source, so it is not enabled"
    assert {"explosives", "arms", "people"} == DEFAULT_ENABLED
    assert not f.is_threat({"type": "vehicle"}) and not f.is_threat({"type": "unknown", "label": "animal"})
    assert f.is_threat({"type": "person"}) and f.is_threat({"type": "mine"})


def test_arms_cannot_be_switched_on_because_nothing_produces_it():
    f = ThreatFilter()
    assert "arms" in UNAVAILABLE and not f.available("arms")
    assert f.set_enabled("arms", True) is False
    assert not f.is_enabled("arms")
    assert f.is_threat({"type": "weapon"}) is False


def test_switching_a_class_tells_listeners_once_and_a_repeat_changes_nothing():
    f = ThreatFilter()
    heard = []
    f.subscribe(lambda flt: heard.append(flt.enabled))
    assert f.set_enabled("vehicles", True) is True
    assert f.set_enabled("vehicles", True) is False, "already on"
    assert len(heard) == 1 and "vehicles" in heard[0]
    assert f.set_enabled("vehicles", False) and "vehicles" not in f.enabled


def test_an_unknown_class_is_an_error_not_a_silent_no_op():
    with pytest.raises(ValueError):
        ThreatFilter().set_enabled("aliens", True)


def test_only_mines_and_ieds_are_guarded_and_every_group_has_a_label():
    assert GUARDED == {"explosives"}
    assert set(LABELS) == set(GROUPS)


def test_the_summary_names_what_is_on():
    assert ThreatFilter().summary() == "MINES / IEDs, PEOPLE"
    assert ThreatFilter(enabled=()).summary() == "nothing"
