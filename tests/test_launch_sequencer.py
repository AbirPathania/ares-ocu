"""The launch record: its order, and that it takes its stage from flight.

The sequencer no longer moves anything. These tests are about the state
machine and about what it is allowed to be told. The flying itself, and the
properties that make a launch correct (slots held on a moving rover, limits
respected, separation kept), are tested against the flight director in
test_launch_flight.py.
"""

import pytest

from ocu.core.formation import Target
from ocu.core.launch_sequencer import LaunchSequencer, LaunchStage
from ocu.core.constants import LAUNCH_STAGGER_S


def make_staged_sequencer(n=2, origin=Target(0, 0, 0)):
    seq = LaunchSequencer()
    seq.arm()
    tricopter_ids = [f"tricopter-{i + 1}" for i in range(n)]
    targets = [Target(origin.x + 10, origin.y + i * 5, 20) for i in range(n)]
    seq.stage_formation(tricopter_ids, targets, origin)
    return seq, tricopter_ids


def test_state_machine_order():
    seq = LaunchSequencer()
    assert seq.stage == LaunchStage.IDLE
    seq.arm()
    assert seq.stage == LaunchStage.ARMED
    seq, _ = make_staged_sequencer()
    assert seq.stage == LaunchStage.STAGED
    seq.launch()
    assert seq.stage == LaunchStage.LAUNCHING


def test_cannot_skip_states():
    with pytest.raises(RuntimeError):
        LaunchSequencer().launch()
    seq = LaunchSequencer()
    with pytest.raises(RuntimeError):
        seq.stage_formation(["tricopter-1"], [Target(1, 1, 1)], Target(0, 0, 0))


def test_liftoffs_are_staggered_in_the_plan():
    seq, ids = make_staged_sequencer(n=3)
    assert [p.liftoff_offset_s for p in seq.plans] == [i * LAUNCH_STAGGER_S for i in range(3)]


def test_a_slot_is_an_offset_from_the_rover_not_a_place_on_the_map():
    """The same formation staged at two different rover positions has the
    same slots. A world position goes stale as the rover drives; an offset
    does not."""
    here, _ = make_staged_sequencer(n=3, origin=Target(0, 0, 0))
    there, _ = make_staged_sequencer(n=3, origin=Target(137.5, -42.0, 0))
    assert [p.slot_offset for p in here.plans] == [p.slot_offset for p in there.plans]
    assert here.plans[1].slot_offset == (10.0, 5.0)


def test_the_stage_is_reported_by_the_aircraft_and_only_moves_forward():
    seq, _ = make_staged_sequencer()
    seq.launch()
    assert seq.report("CLIMBING") is True
    assert seq.stage == LaunchStage.CLIMBING
    assert seq.report("LAUNCHING") is False, "a late report cannot wind a launch back"
    assert seq.report("CLIMBING") is False, "nor can a repeat advance it"
    assert seq.report("TRANSLATING") and seq.report("FORMED")
    assert seq.stage == LaunchStage.FORMED


def test_a_stage_can_skip_ahead_if_the_aircraft_say_so():
    """Flight can be quicker than any stage list: two aircraft already on
    station by the time the first report arrives."""
    seq, _ = make_staged_sequencer()
    seq.launch()
    assert seq.report("FORMED") is True


def test_nothing_is_reported_before_launch():
    seq, _ = make_staged_sequencer()
    assert seq.report("CLIMBING") is False
    assert seq.stage == LaunchStage.STAGED
    idle = LaunchSequencer()
    assert idle.report("FORMED") is False


def test_nonsense_reports_are_ignored():
    seq, _ = make_staged_sequencer()
    seq.launch()
    assert seq.report("BANANAS") is False
    assert seq.report("IDLE") is False
    assert seq.stage == LaunchStage.LAUNCHING


def test_reset_returns_to_idle():
    seq, _ = make_staged_sequencer()
    seq.launch()
    seq.reset()
    assert seq.stage == LaunchStage.IDLE
    assert seq.plans == []
