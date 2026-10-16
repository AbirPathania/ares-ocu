"""Launch state machine, section 6.3.

IDLE -> ARMED -> STAGED -> LAUNCHING -> CLIMBING -> TRANSLATING -> FORMED

The same record serves both launch paths described in section 6: guided
formation launch (section 6.2), where an operator sets bearing, pattern,
spacing and rally altitude, and reactive launch (section 6.1), where the
rover's own planner decides. Section 6.1 is explicit that reactive launch
"trigger[s] the same launch sequencer described in 6.3" rather than a
separate code path, so there is only one implementation here.

What this class does *not* do is move anything. It used to: it interpolated
each aircraft along a fixed line over a fixed number of seconds, which put
an aircraft from rest to 7.5 m/s in one tick (peak acceleration around 75
m/s^2, about 7 g) and, worse, aimed it at a world position worked out when
the launch was staged, which is wrong as soon as the rover has driven on.
The aircraft are now flown by the flight director under the same
acceleration limits as everything else, holding a slot that is an offset
from the rover, and the stage here is *reported from the aircraft*:
FORMED means every one of them is actually on its slot, not that a timer ran
out. The staggered lift-off interval (LAUNCH_STAGGER_S) is still the plan;
the physics decides how long the rest takes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from ocu.core.constants import LAUNCH_STAGGER_S
from ocu.core.formation import Target

#: Order of the in-flight stages, so a report can only move a launch forward.
_FLIGHT_ORDER = ("LAUNCHING", "CLIMBING", "TRANSLATING", "FORMED")


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

    @property
    def slot_offset(self) -> tuple[float, float]:
        """Where the aircraft is to hold, as an offset from the rover.

        A formation is a shape held on a moving vehicle, so the slot is
        relative to it. The target position is a world point at the moment
        of staging and goes stale as the rover drives.
        """
        return (self.target.x - self.origin.x, self.target.y - self.origin.y)


@dataclass
class LaunchSequencer:
    stage: LaunchStage = LaunchStage.IDLE
    is_reactive: bool = False
    plans: list[AircraftPlan] = field(default_factory=list)

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
        self.stage = LaunchStage.LAUNCHING

    def report(self, stage: str) -> bool:
        """Take the stage the aircraft report. Returns True if it advanced.

        Only forward, and only once launched: a late or repeated report
        cannot wind a launch back, and nothing can skip staging.
        """
        if self.stage.value not in _FLIGHT_ORDER or stage not in _FLIGHT_ORDER:
            return False
        if _FLIGHT_ORDER.index(stage) <= _FLIGHT_ORDER.index(self.stage.value):
            return False
        self.stage = LaunchStage(stage)
        return True

    def reset(self) -> None:
        self.stage = LaunchStage.IDLE
        self.is_reactive = False
        self.plans = []
