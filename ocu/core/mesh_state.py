"""The CRDT merge logic, section 8.2 and 8.5.

This is the part of the demo that proves the patent claim: the shared,
conflict-free map is the strongest claim in the ARES portfolio, and it is
the one part of the system that can be demonstrated completely and
correctly with zero hardware (section 8). Build and rehearse this module
first, section 15.

Merge rule: observations of one voxel are totally ordered by
(confidence, timestamp_us, source_id, threat_type), compared left to right,
and merge() returns the maximum. A total order makes merge commutative,
associative, and idempotent by construction, so any two replicas that have
seen the same set of updates, in any order, converge to the exact same map.
That guarantee is worth a real unit test that feeds the same updates in
different orders and asserts a byte-identical result, not a smoke test; see
tests/test_mesh_state.py.

The earlier rule stopped at confidence and timestamp. Two observations that
tied on both but differed in source_id or threat_type made merge(a, b) return
a and merge(b, a) return b, so replicas could disagree permanently. See
DECISIONS.md.

No PySide import belongs anywhere in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.constants import VOXEL_RESOLUTION_CM


def voxel_key(x: float, y: float, z: float = 0.0) -> tuple[int, int, int]:
    """World meters -> the 5 cm grid index the shared map keys on, section 8.2."""
    resolution_m = VOXEL_RESOLUTION_CM / 100.0
    return (round(x / resolution_m), round(y / resolution_m), round(z / resolution_m))


@dataclass(frozen=True)
class VoxelObservation:
    voxel_key: tuple[int, int, int]  # (x, y, z) at 5 cm grid resolution, section 8.2
    timestamp_us: int
    confidence: float  # 0.0 to 1.0
    threat_type: str  # see constants.THREAT_CLASSES; eighth class open, section 14
    source_id: str  # which unit made this observation

    def __post_init__(self) -> None:
        # NaN compares false against everything, which would break the total
        # order merge() relies on, so it is rejected here, not tolerated.
        if math.isnan(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError(f"confidence must be a number from 0.0 to 1.0, got {self.confidence!r}")
        # Store a plain float with +0.0 for -0.0: they compare equal but serialize
        # differently, so two replicas could otherwise hold unequal bytes for equal observations.
        object.__setattr__(self, "confidence", float(self.confidence) + 0.0)

    def order_key(self) -> tuple[float, int, str, str]:
        """The total order merge() maximizes: confidence, then timestamp,
        then source_id, then threat_type."""
        return (self.confidence, self.timestamp_us, self.source_id, self.threat_type)


def merge(a: VoxelObservation, b: VoxelObservation) -> VoxelObservation:
    """The maximum of a and b under VoxelObservation.order_key. a and b must
    observe the same voxel. Commutative, associative, and idempotent."""
    if a.voxel_key != b.voxel_key:
        raise ValueError(
            f"cannot merge observations of different voxels: {a.voxel_key} != {b.voxel_key}"
        )
    return a if a.order_key() >= b.order_key() else b


class MeshState:
    """One replica's view of the shared voxel map."""

    def __init__(self, replica_id: str) -> None:
        self.replica_id = replica_id
        self._voxels: dict[tuple[int, int, int], VoxelObservation] = {}

    def apply(self, observation: VoxelObservation) -> bool:
        """Merge one observation into this replica. Returns True if it changed state."""
        existing = self._voxels.get(observation.voxel_key)
        if existing is None:
            self._voxels[observation.voxel_key] = observation
            return True
        merged = merge(existing, observation)
        if merged == existing:
            return False
        self._voxels[observation.voxel_key] = merged
        return True

    def apply_all(self, observations: list[VoxelObservation]) -> None:
        for obs in observations:
            self.apply(obs)

    def get(self, voxel_key: tuple[int, int, int]) -> VoxelObservation | None:
        return self._voxels.get(voxel_key)

    def snapshot(self) -> dict[tuple[int, int, int], VoxelObservation]:
        """A plain-dict copy, for equality checks and the CRDT convergence test."""
        return dict(self._voxels)

    def __len__(self) -> int:
        return len(self._voxels)


def reconcile(a: MeshState, b: MeshState) -> int:
    """Two-way merge on reconnect, section 8.5's Restore Link.

    Every voxel either side has ever seen ends up identical on both sides
    afterward. Returns the delta count: the number of voxels that actually
    needed to cross the link to reach convergence, not the size of the
    whole map, only the difference, per section 8.5.
    """
    delta = 0
    all_keys = set(a._voxels) | set(b._voxels)
    for key in all_keys:
        obs_a = a._voxels.get(key)
        obs_b = b._voxels.get(key)

        if obs_a is None:
            a._voxels[key] = obs_b
            delta += 1
            continue
        if obs_b is None:
            b._voxels[key] = obs_a
            delta += 1
            continue

        merged = merge(obs_a, obs_b)
        changed = False
        if merged != obs_a:
            a._voxels[key] = merged
            changed = True
        if merged != obs_b:
            b._voxels[key] = merged
            changed = True
        if changed:
            delta += 1

    return delta


class MeshLink:
    """Simulates the rover <-> aircraft mesh connection for the link-cut
    demonstration, section 8.5, described as "the single most important
    interaction in the whole application."

    While connected, an observation on either side propagates immediately.
    While cut, both sides keep logging independently and visibly diverge.
    Restore Link runs reconcile() and reports the delta.
    """

    def __init__(self) -> None:
        self.rover_map = MeshState("rover")
        self.aircraft_map = MeshState("aircraft")
        self.connected = True
        self.last_restore_delta: int | None = None

    def cut(self) -> None:
        self.connected = False
        self.last_restore_delta = None

    def observe(self, side: str, observation: VoxelObservation) -> None:
        """A detection arrives at one side ("rover" or "aircraft")."""
        if side not in ("rover", "aircraft"):
            raise ValueError(f"unknown side: {side!r}")

        local = self.rover_map if side == "rover" else self.aircraft_map
        local.apply(observation)

        if self.connected:
            remote = self.aircraft_map if side == "rover" else self.rover_map
            remote.apply(observation)

    def restore(self) -> int:
        """Reconnect and merge. Returns the delta count for the UI's counter."""
        self.connected = True
        delta = reconcile(self.rover_map, self.aircraft_map)
        self.last_restore_delta = delta
        return delta

    def holdings(self) -> dict[tuple[int, int, int], set[str]]:
        """Which replicas currently hold each voxel.

        This is what makes a cut link visible rather than merely logged: a
        voxel held by one side only is an observation that has not crossed,
        and the map can draw it as such instead of pretending the fleet
        shares one view while the link is down.
        """
        held: dict[tuple[int, int, int], set[str]] = {}
        for key in self.rover_map.snapshot():
            held.setdefault(key, set()).add("rover")
        for key in self.aircraft_map.snapshot():
            held.setdefault(key, set()).add("aircraft")
        return held

    def divergence(self) -> int:
        """How many voxels would have to cross the link to reconcile right now.

        Counts a voxel once if the two replicas disagree about it at all,
        whether because only one side has seen it or because they hold
        different observations of it. Equals the delta a restore reports.
        """
        rover = self.rover_map.snapshot()
        aircraft = self.aircraft_map.snapshot()
        return sum(1 for key in set(rover) | set(aircraft) if rover.get(key) != aircraft.get(key))

    @property
    def rover_voxels(self) -> int:
        return len(self.rover_map)

    @property
    def aircraft_voxels(self) -> int:
        return len(self.aircraft_map)
