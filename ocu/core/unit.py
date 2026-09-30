"""Unit state, section 12.1.

Two separate layers of state live on this object and must not be conflated:
mission_mode (section 4.1, documented ARES doctrine: STANDARD, AUTONOMOUS,
GHOST, LCA) and control_mode (section 4.2, an interface convention invented
for this build: MANUAL, GUIDED, AUTO, HOLD). A Unit can be AUTONOMOUS mission
mode and AUTO control mode at once; the two axes answer different questions.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ocu.core.constants import (
    CONTROL_MODES,
    LINK_STATES,
    LINK_LOSS_EMERGENCY_RETURN_S,
    MISSION_MODES,
)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0  # degrees, 0 = rover forward / compass north in sim


@dataclass
class Unit:
    id: str  # "rover", "tricopter-1", "tricopter-2", "tricopter-3"
    kind: str  # "rover" or "tricopter"
    pose: Pose = field(default_factory=Pose)
    battery_pct: float = 100.0
    control_mode: str = "AUTO"
    mission_mode: str = "STANDARD"
    link_state: str = "CONNECTED"
    emergency_return: bool = False
    current_task: str = "idle"

    def __post_init__(self) -> None:
        if self.kind not in ("rover", "tricopter"):
            raise ValueError(f"unknown unit kind: {self.kind!r}")
        if self.control_mode not in CONTROL_MODES:
            raise ValueError(f"unknown control_mode: {self.control_mode!r}")
        if self.mission_mode not in MISSION_MODES:
            raise ValueError(f"unknown mission_mode: {self.mission_mode!r}")
        if self.link_state not in LINK_STATES:
            raise ValueError(f"unknown link_state: {self.link_state!r}")

    @property
    def is_tricopter(self) -> bool:
        return self.kind == "tricopter"


def evaluate_emergency_return(unit: Unit, link_loss_duration_s: float) -> bool:
    """Per-unit Emergency Return, section 4.1: a distinct per-unit flag, not a
    mission mode, so it can fire on one tricopter while the rest of the fleet
    continues normally. Any unit that loses mesh comms for more than 15
    seconds returns to its last safe waypoint, then to the rover if it is a
    tricopter, or to the start point if it is the rover itself.
    """
    unit.emergency_return = link_loss_duration_s > LINK_LOSS_EMERGENCY_RETURN_S
    return unit.emergency_return
