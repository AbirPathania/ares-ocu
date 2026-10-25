"""The guarantee under test: any two replicas that have seen the same set
of updates, in any order, converge to the exact same map. Section 8.2.
"""

import itertools
import json
import random

from ocu.core.mesh_state import MeshLink, MeshState, VoxelObservation, merge, reconcile


def make_obs(key=(1, 2, 3), t=1000, confidence=0.5, threat="mine", source="rover"):
    return VoxelObservation(
        voxel_key=key, timestamp_us=t, confidence=confidence, threat_type=threat, source_id=source
    )


def test_merge_higher_confidence_wins():
    a = make_obs(confidence=0.3, t=100)
    b = make_obs(confidence=0.9, t=50)
    assert merge(a, b) == b
    assert merge(b, a) == b


def test_merge_tie_breaks_on_newer_timestamp():
    a = make_obs(confidence=0.5, t=100)
    b = make_obs(confidence=0.5, t=200)
    assert merge(a, b) == b
    assert merge(b, a) == b


def test_merge_rejects_different_voxels():
    a = make_obs(key=(1, 1, 1))
    b = make_obs(key=(2, 2, 2))
    try:
        merge(a, b)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_merge_is_idempotent():
    a = make_obs(confidence=0.7, t=100)
    assert merge(a, a) == a


def test_mesh_state_apply_all_order_independent():
    observations = [
        make_obs(key=(0, 0, 0), t=10, confidence=0.2, source="rover"),
        make_obs(key=(0, 0, 0), t=20, confidence=0.9, source="tricopter-1"),
        make_obs(key=(0, 0, 0), t=15, confidence=0.9, source="tricopter-2"),  # tie on confidence
        make_obs(key=(1, 0, 0), t=5, confidence=0.5, source="rover"),
        make_obs(key=(1, 0, 0), t=6, confidence=0.5, source="tricopter-3"),
        make_obs(key=(2, 2, 2), t=1, confidence=1.0, source="rover"),
    ]

    reference = MeshState("reference")
    reference.apply_all(observations)
    reference_snapshot = reference.snapshot()

    rng = random.Random(42)
    for _ in range(20):
        shuffled = observations[:]
        rng.shuffle(shuffled)
        replica = MeshState("replica")
        replica.apply_all(shuffled)
        assert replica.snapshot() == reference_snapshot


def test_mesh_state_apply_all_permutations_small_set():
    observations = [
        make_obs(key=(0, 0, 0), t=1, confidence=0.4, source="a"),
        make_obs(key=(0, 0, 0), t=2, confidence=0.4, source="b"),
        make_obs(key=(0, 0, 0), t=1, confidence=0.9, source="c"),
    ]

    snapshots = set()
    for perm in itertools.permutations(observations):
        replica = MeshState("replica")
        replica.apply_all(list(perm))
        snapshots.add(tuple(sorted(replica.snapshot().items())))

    assert len(snapshots) == 1


def test_apply_returns_whether_state_changed():
    state = MeshState("x")
    assert state.apply(make_obs(t=10, confidence=0.5)) is True
    assert state.apply(make_obs(t=5, confidence=0.1)) is False  # loses to existing
    assert state.apply(make_obs(t=20, confidence=0.9)) is True  # wins


def test_reconcile_converges_two_diverged_replicas():
    a = MeshState("rover")
    b = MeshState("aircraft")

    a.apply(make_obs(key=(0, 0, 0), t=1, confidence=0.5, source="rover"))
    b.apply(make_obs(key=(0, 0, 0), t=2, confidence=0.5, source="tricopter-1"))
    a.apply(make_obs(key=(5, 5, 5), t=1, confidence=0.9, source="rover"))
    b.apply(make_obs(key=(9, 9, 9), t=1, confidence=0.9, source="tricopter-1"))

    delta = reconcile(a, b)

    assert a.snapshot() == b.snapshot()
    assert delta == 3  # (0,0,0) reconciled, (5,5,5) copied to b, (9,9,9) copied to a


def test_reconcile_is_idempotent_second_call_finds_no_delta():
    a = MeshState("rover")
    b = MeshState("aircraft")
    a.apply(make_obs(key=(0, 0, 0), t=1, confidence=0.5))
    reconcile(a, b)
    assert reconcile(a, b) == 0


def test_mesh_link_diverges_while_cut_and_reconverges_on_restore():
    link = MeshLink()

    link.observe("rover", make_obs(key=(1, 1, 1), t=1, confidence=0.6, source="rover"))
    assert link.rover_map.snapshot() == link.aircraft_map.snapshot()

    link.cut()
    link.observe("rover", make_obs(key=(2, 2, 2), t=2, confidence=0.7, source="rover"))
    link.observe("aircraft", make_obs(key=(3, 3, 3), t=3, confidence=0.8, source="tricopter-1"))

    assert link.rover_map.get((2, 2, 2)) is not None
    assert link.aircraft_map.get((2, 2, 2)) is None
    assert link.aircraft_map.get((3, 3, 3)) is not None
    assert link.rover_map.get((3, 3, 3)) is None

    delta = link.restore()

    assert delta == 2
    assert link.rover_map.snapshot() == link.aircraft_map.snapshot()
    assert link.connected is True


def test_holdings_marks_which_replica_has_each_voxel():
    link = MeshLink()
    link.observe("rover", make_obs(key=(1, 1, 1)))
    link.cut()
    link.observe("rover", make_obs(key=(2, 2, 2), source="rover"))
    link.observe("aircraft", make_obs(key=(3, 3, 3), source="tricopter-1"))

    held = link.holdings()
    assert held[(1, 1, 1)] == {"rover", "aircraft"}  # crossed before the cut
    assert held[(2, 2, 2)] == {"rover"}
    assert held[(3, 3, 3)] == {"aircraft"}


def test_divergence_equals_the_delta_a_restore_reports():
    link = MeshLink()
    link.cut()
    link.observe("rover", make_obs(key=(1, 0, 0), t=1, confidence=0.4))
    link.observe("aircraft", make_obs(key=(2, 0, 0), t=2, confidence=0.5))
    link.observe("aircraft", make_obs(key=(1, 0, 0), t=3, confidence=0.9))

    predicted = link.divergence()
    assert predicted == 2
    assert link.restore() == predicted


def test_divergence_is_zero_while_connected():
    link = MeshLink()
    for i in range(5):
        link.observe("rover", make_obs(key=(i, 0, 0)))
    assert link.divergence() == 0
    assert link.rover_voxels == link.aircraft_voxels == 5


def test_holdings_converge_after_restore():
    link = MeshLink()
    link.cut()
    link.observe("rover", make_obs(key=(9, 9, 9)))
    assert link.holdings()[(9, 9, 9)] == {"rover"}
    link.restore()
    assert link.holdings()[(9, 9, 9)] == {"rover", "aircraft"}
    assert link.divergence() == 0


# --- the full-tie defect: merge was not commutative ---------------------------
#
# Before the total order, two observations that tied on confidence and
# timestamp but differed in threat_type or source_id made merge(a, b) return a
# and merge(b, a) return b.


def canonical(state):
    """The map as bytes: rows sorted, no whitespace, so equal maps give equal bytes."""
    rows = sorted(
        [*o.voxel_key, o.timestamp_us, o.confidence, o.threat_type, o.source_id]
        for o in state.snapshot().values()
    )
    return json.dumps(rows, separators=(",", ":")).encode("ascii")


def test_full_tie_on_confidence_and_timestamp_is_commutative():
    a = VoxelObservation((1, 2, 0), 1000, 0.9, "mine", "rover")
    b = VoxelObservation((1, 2, 0), 1000, 0.9, "ied", "aircraft_1")
    assert merge(a, b) == merge(b, a)
    # Highest source_id wins the tie; here "rover" > "aircraft_1".
    assert merge(a, b) == a


def test_tie_on_everything_but_threat_type_is_commutative():
    a = make_obs(t=5, confidence=0.5, threat="mine", source="rover")
    b = make_obs(t=5, confidence=0.5, threat="ied", source="rover")
    assert merge(a, b) == merge(b, a) == a  # "mine" > "ied"


def _random_obs(rng):
    # A small value pool, so that ties on every field are common.
    return VoxelObservation(
        voxel_key=(rng.choice([0, 1]), 0, 0),
        timestamp_us=rng.choice([10, 20]),
        confidence=rng.choice([0.0, -0.0, 0.5, 0.9]),
        threat_type=rng.choice(["mine", "ied", "person"]),
        source_id=rng.choice(["rover", "aircraft_1", "aircraft_2"]),
    )


def _same_voxel(rng):
    o = _random_obs(rng)
    return VoxelObservation((0, 0, 0), o.timestamp_us, o.confidence, o.threat_type, o.source_id)


def test_merge_is_commutative_associative_and_idempotent_over_many_ties():
    rng = random.Random(20261025)
    for _ in range(5000):
        a, b, c = _same_voxel(rng), _same_voxel(rng), _same_voxel(rng)
        assert merge(a, b) == merge(b, a)
        assert merge(merge(a, b), c) == merge(a, merge(b, c))
        assert merge(a, a) == a
        # Equal observations must also serialize identically, not merely compare equal.
        assert repr(merge(a, b)) == repr(merge(b, a))


def test_replicas_converge_to_byte_identical_maps_under_shuffles_and_duplicates():
    rng = random.Random(7)
    updates = [_random_obs(rng) for _ in range(200)]

    reference = MeshState("reference")
    reference.apply_all(updates)
    expected = canonical(reference)

    for replica_index in range(40):
        order = updates[:]
        order += [rng.choice(updates) for _ in range(100)]  # duplicates
        rng.shuffle(order)
        replica = MeshState(f"replica-{replica_index}")
        replica.apply_all(order)
        assert canonical(replica) == expected


def test_pairwise_reconcile_of_tied_replicas_gives_identical_bytes():
    a, b = MeshState("a"), MeshState("b")
    a.apply(VoxelObservation((1, 2, 0), 1000, 0.9, "mine", "rover"))
    b.apply(VoxelObservation((1, 2, 0), 1000, 0.9, "ied", "aircraft_1"))
    assert reconcile(a, b) == 1
    assert canonical(a) == canonical(b)


def test_confidence_outside_unit_interval_or_nan_is_rejected():
    for bad in (float("nan"), -0.001, 1.001, float("inf"), float("-inf")):
        try:
            make_obs(confidence=bad)
            assert False, f"accepted confidence {bad!r}"
        except ValueError:
            pass
    assert make_obs(confidence=0.0).confidence == 0.0
    assert make_obs(confidence=1.0).confidence == 1.0


def test_negative_zero_confidence_is_stored_as_positive_zero():
    assert repr(make_obs(confidence=-0.0).confidence) == "0.0"
