from ocu.core.control_authority import ControlAuthority
from ocu.core.unit import Unit


def make_fleet():
    return {
        "rover": Unit(id="rover", kind="rover", control_mode="AUTO"),
        "tricopter-1": Unit(id="tricopter-1", kind="tricopter", control_mode="GUIDED"),
        "tricopter-2": Unit(id="tricopter-2", kind="tricopter", control_mode="AUTO"),
    }


def test_only_one_manual_holder_at_a_time():
    fleet = make_fleet()
    authority = ControlAuthority(fleet)
    authority.take_manual("tricopter-1")
    assert authority.manual_holder == "tricopter-1"
    assert fleet["tricopter-1"].control_mode == "MANUAL"

    authority.take_manual("tricopter-2")
    assert authority.manual_holder == "tricopter-2"
    assert fleet["tricopter-2"].control_mode == "MANUAL"
    # tricopter-1 drops back to whatever it held before MANUAL (GUIDED)
    assert fleet["tricopter-1"].control_mode == "GUIDED"


def test_release_manual_reverts_to_prior_mode():
    fleet = make_fleet()
    authority = ControlAuthority(fleet)
    authority.take_manual("tricopter-2")  # was AUTO
    authority.release_manual()
    assert fleet["tricopter-2"].control_mode == "AUTO"
    assert authority.manual_holder is None


def test_set_mode_refuses_to_bypass_manual_handoff():
    fleet = make_fleet()
    authority = ControlAuthority(fleet)
    try:
        authority.set_mode("rover", "MANUAL")
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_set_mode_refuses_on_current_manual_holder():
    fleet = make_fleet()
    authority = ControlAuthority(fleet)
    authority.take_manual("rover")
    try:
        authority.set_mode("rover", "HOLD")
        assert False, "expected ValueError"
    except ValueError:
        pass
