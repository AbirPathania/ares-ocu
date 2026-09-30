"""Launch state machine, section 6.3.

IDLE -> ARMED -> STAGED -> LAUNCHING -> CLIMBING -> TRANSLATING -> FORMED

The same sequencer drives both launch paths described in section 6: guided
formation launch (section 6.2), where an operator sets bearing/pattern/
spacing/rally altitude live and this class runs on a real wall-clock tick,
and reactive launch (section 6.1), where the scripted mission timeline
triggers this same class with a bearing computed toward the scripted
occlusion, no operator click. Section 6.1 is explicit that reactive launch
"trigger[s] the same launch sequencer described in 6.3" rather than a
separate code path, so there is only one implementation here.

Aircraft lift off one at a time, about 1.5s apart (LAUNCH_STAGGER_S), each
climbs straight up to rally altitude before any lateral motion, then
translates to its formation slot. CLIMB_DURATION_S, TRANSLATE_DURATION_S
and POSITION_TOLERANCE_M are not figures from the technical baseline, the
spec gives no aircraft climb/cruise rate; they are this build's own
animation timing so the sequencer has something concrete to tick against.
See DECISIONS.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from ocu.core.constants import LAUNCH_STAGGER_S
from ocu.core.formation import Target

CLIMB_DURATION_S = 3.0
TRANSLATE_DURATION_S = 4.0
POSITION_TOLERANCE_M = 0.5


class LaunchStage(str, Enum):
    IDLE = "IDLE"
    ARMED = "ARMED"
    STAGED = "STAGED"
    LAUNCHING = "LAUNCHING"
    CLIMBING = "CLIMBING"
    TRANSLATING = "TRANSLATING"
    FORMED = "FORMED"


@dataclass
class AircraftPlan:
    tricopter_id: str
    origin: Target
    target: Target
    liftoff_offset_s: float
    start_heading: float = 0.0


@dataclass
class LaunchSequencer:
    stage: LaunchStage = LaunchStage.IDLE
    is_reactive: bool = False
    plans: list[AircraftPlan] = field(default_factory=list)
    elapsed_s: float = 0.0

    def arm(self) -> None:
        if self.stage != LaunchStage.IDLE:
            raise RuntimeError(f"cannot arm from {self.stage}")
        self.stage = LaunchStage.ARMED

    def stage_formation(
        self,
        tricopter_ids: list[str],
        targets: list[Target],
        origin: Target,
        is_reactive: bool = False,
    ) -> None:
        """Verify step: bearing, pattern, spacing set; targets computed; nothing moved yet."""
        if self.stage != LaunchStage.ARMED:
            raise RuntimeError(f"cannot stage from {self.stage}")
        if len(tricopter_ids) != len(targets):
            raise ValueError("tricopter_ids and targets must be the same length")

        self.plans = [
            AircraftPlan(
                tricopter_id=tricopter_id,
                origin=origin,
                target=target,
                liftoff_offset_s=i * LAUNCH_STAGGER_S,
            )
            for i, (tricopter_id, target) in enumerate(zip(tricopter_ids, targets))
        ]
        self.is_reactive = is_reactive
        self.stage = LaunchStage.STAGED

    def launch(self) -> None:
        if self.stage != LaunchStage.STAGED:
            raise RuntimeError(f"cannot launch from {self.stage}")
        self.elapsed_s = 0.0
        self.stage = LaunchStage.LAUNCHING

    def tick(self, dt_s: float) -> None:
        """Advance the wall clock and step the overall stage forward when appropriate."""
        if self.stage not in (
            LaunchStage.LAUNCHING,
            LaunchStage.CLIMBING,
            LaunchStage.TRANSLATING,
        ):
            return

        self.elapsed_s += dt_s

        all_lifted = all(self.elapsed_s >= p.liftoff_offset_s for p in self.plans)
        all_climbed = all(
            self.elapsed_s >= p.liftoff_offset_s + CLIMB_DURATION_S for p in self.plans
        )
        all_translated = all(
            self.elapsed_s
            >= p.liftoff_offset_s + CLIMB_DURATION_S + TRANSLATE_DURATION_S
            for p in self.plans
        )

        if self.stage == LaunchStage.LAUNCHING and all_lifted:
            self.stage = LaunchStage.CLIMBING
        if self.stage == LaunchStage.CLIMBING and all_climbed:
            self.stage = LaunchStage.TRANSLATING
        if self.stage == LaunchStage.TRANSLATING and all_translated:
            self.stage = LaunchStage.FORMED

    def position_for(self, tricopter_id: str) -> Target:
        """Interpolated position of one aircraft in this launch at the current tick."""
        plan = next((p for p in self.plans if p.tricopter_id == tricopter_id), None)
        if plan is None:
            raise KeyError(f"{tricopter_id} is not part of this launch")

        t = self.elapsed_s - plan.liftoff_offset_s
        if t <= 0.0:
            return Target(plan.origin.x, plan.origin.y, plan.origin.z)

        if t < CLIMB_DURATION_S:
            frac = t / CLIMB_DURATION_S
            z = plan.origin.z + frac * (plan.target.z - plan.origin.z)
            return Target(plan.origin.x, plan.origin.y, z)

        t -= CLIMB_DURATION_S
        if t < TRANSLATE_DURATION_S:
            frac = t / TRANSLATE_DURATION_S
            x = plan.origin.x + frac * (plan.target.x - plan.origin.x)
            y = plan.origin.y + frac * (plan.target.y - plan.origin.y)
            return Target(x, y, plan.target.z)

        return Target(plan.target.x, plan.target.y, plan.target.z)

    def heading_for(self, tricopter_id: str) -> float:
        """Heading through the sequence: hold the launch heading while climbing
        straight up, then rotate onto the formation bearing across the
        translation, so each aircraft arrives at its slot already pointing the
        way the formation faces."""
        plan = next((p for p in self.plans if p.tricopter_id == tricopter_id), None)
        if plan is None:
            raise KeyError(f"{tricopter_id} is not part of this launch")

        target_heading = plan.target.heading
        t = self.elapsed_s - plan.liftoff_offset_s
        if t <= CLIMB_DURATION_S:
            return plan.start_heading

        t -= CLIMB_DURATION_S
        if t >= TRANSLATE_DURATION_S:
            return target_heading % 360.0

        fraction = t / TRANSLATE_DURATION_S
        difference = (target_heading - plan.start_heading + 180.0) % 360.0 - 180.0
        return (plan.start_heading + difference * fraction) % 360.0

    def reset(self) -> None:
        self.stage = LaunchStage.IDLE
        self.is_reactive = False
        self.plans = []
        self.elapsed_s = 0.0
