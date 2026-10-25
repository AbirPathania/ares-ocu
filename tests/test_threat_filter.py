import pytest

from ocu.core.threat_filter import DEFAULT_ENABLED, GROUPS, GUARDED, LABELS, SOURCES, UNAVAILABLE, ThreatFilter, group_of, replaces


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
    assert f.enabled == {"explosives", "arms", "people"} == DEFAULT_ENABLED
    assert not f.is_threat({"type": "vehicle"}) and not f.is_threat({"type": "unknown", "label": "animal"})
    assert f.is_threat({"type": "person"}) and f.is_threat({"type": "mine"}) and f.is_threat({"type": "weapon"})


def test_every_class_has_a_source_and_can_be_switched():
    f = ThreatFilter()
    assert UNAVAILABLE == {}, "nothing on this strip is a dead control any more"
    for group in GROUPS:
        assert f.available(group) and SOURCES.get(group), f"{group} has no stated source"
    assert f.set_enabled("arms", False) and not f.is_threat({"type": "weapon"})
    assert f.set_enabled("arms", True) and f.is_threat({"type": "weapon"})


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
    assert ThreatFilter().summary() == "MINES / IEDs, ARMS, PEOPLE"
    assert ThreatFilter(enabled=()).summary() == "nothing"


def test_an_armed_call_replaces_a_plain_person_on_the_same_contact():
    """The weapon stage runs on a person the camera already found, so it describes that contact better rather
    than competing with it. It must win whatever the confidences are."""
    assert replaces("weapon", "person") and replaces("weapon", "unknown")
    assert not replaces("person", "weapon"), "and it never goes back the other way"
    assert not replaces("vehicle", "person")
