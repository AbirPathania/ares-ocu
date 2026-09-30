from ocu.core.formation import Target
from ocu.core.launch_sequencer import (
    CLIMB_DURATION_S,
    TRANSLATE_DURATION_S,
    LaunchSequencer,
    LaunchStage,
)
from ocu.core.constants import LAUNCH_STAGGER_S


def make_staged_sequencer(n=2):
    seq = LaunchSequencer()
    seq.arm()
    tricopter_ids = [f"tricopter-{i + 1}" for i in range(n)]
    origin = Target(0, 0, 0)
    targets = [Target(10, i * 5, 20) for i in range(n)]
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
    seq = LaunchSequencer()
    try:
        seq.launch()
        assert False, "expected RuntimeError"
    except RuntimeError:
        pass


def test_staggered_liftoff_timing():
    seq, tricopter_ids = make_staged_sequencer(n=2)
    seq.launch()
    # just after launch, tricopter-1 has lifted (offset 0), tricopter-2 has not (offset 1.5)
    seq.tick(0.1)
    p1 = seq.position_for("tricopter-1")
    p2 = seq.position_for("tricopter-2")
    assert p1.z > 0  # tricopter-1 already climbing
    assert p2.z == 0  # tricopter-2 still on the ground, waiting its 1.5s turn


def test_full_sequence_reaches_formed():
    seq, tricopter_ids = make_staged_sequencer(n=2)
    seq.launch()
    total_time = (len(tricopter_ids) - 1) * LAUNCH_STAGGER_S + CLIMB_DURATION_S + TRANSLATE_DURATION_S
    steps = int(total_time / 0.1) + 5
    for _ in range(steps):
        seq.tick(0.1)
    assert seq.stage == LaunchStage.FORMED
    for i, tricopter_id in enumerate(tricopter_ids):
        pos = seq.position_for(tricopter_id)
        assert abs(pos.x - 10) < 1e-6
        assert abs(pos.y - i * 5) < 1e-6
        assert abs(pos.z - 20) < 1e-6


def test_heading_rotates_onto_the_formation_bearing():
    seq = LaunchSequencer()
    seq.arm()
    seq.stage_formation(["tricopter-1"], [Target(10, 0, 20, heading=90.0)], Target(0, 0, 0))
    seq.launch()

    seq.tick(CLIMB_DURATION_S * 0.5)
    assert seq.heading_for("tricopter-1") == 0.0  # still climbing straight up

    for _ in range(int((CLIMB_DURATION_S + TRANSLATE_DURATION_S) / 0.1) + 4):
        seq.tick(0.1)
    assert abs(seq.heading_for("tricopter-1") - 90.0) < 1e-6


def test_heading_takes_the_short_way_around():
    seq = LaunchSequencer()
    seq.arm()
    seq.stage_formation(["tricopter-1"], [Target(1, 0, 20, heading=350.0)], Target(0, 0, 0))
    seq.plans[0].start_heading = 10.0
    seq.launch()
    seq.tick(CLIMB_DURATION_S + TRANSLATE_DURATION_S * 0.5)
    heading = seq.heading_for("tricopter-1")
    # Turning 10 -> 350 must pass through 0, never sweep down through 180.
    assert heading > 340.0 or heading < 20.0


def test_reactive_flag_recorded():
    seq = LaunchSequencer()
    seq.arm()
    seq.stage_formation(["tricopter-1"], [Target(1, 1, 1)], Target(0, 0, 0), is_reactive=True)
    assert seq.is_reactive is True


def test_reset_returns_to_idle():
    seq, _ = make_staged_sequencer()
    seq.launch()
    seq.tick(1.0)
    seq.reset()
    assert seq.stage == LaunchStage.IDLE
    assert seq.plans == []
    assert seq.elapsed_s == 0.0
