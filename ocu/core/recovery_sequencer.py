"""Aircraft recovery, section 7.

Fully autonomous, fixed four-layer sequence. There is no operator decision
here beyond issuing a recall; the interface's only job is to visualize
progress honestly, as the aircraft icon's targeting ring shrinking through
the four stages rather than a jump cut from airborne to docked.

Stage precision figures (50 cm, 10 cm, 3 cm) are sourced from section 7.
The rate at which an aircraft closes that distance is not given anywhere in
the technical baseline; APPROACH_DURATION_S below is this build's own
animation timing, not a baseline figure. See DECISIONS.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ocu.core.constants import (
    CHARGE_PAD_POWER_W,
    CHARGE_TIME_FROM_EMPTY_MIN_HIGH,
    RECOVERY_STAGE_1_PRECISION_CM,
    RECOVERY_STAGE_2_PRECISION_CM,
    RECOVERY_STAGE_3_PRECISION_CM,
)

APPROACH_DURATION_S = 6.0  # not baseline, this build's animation timing only


class RecoveryStage(str, Enum):
    APPROACHING = "APPROACHING"
    COARSE = "COARSE"
    FINE = "FINE"
    MECHANICAL = "MECHANICAL"
    DOCKED = "DOCKED"


STAGE_GUIDANCE = {
    RecoveryStage.COARSE: "ultra wideband ranging",
    RecoveryStage.FINE: "ArUco fiducial, downward camera",
    RecoveryStage.MECHANICAL: "funnel pad",
    RecoveryStage.DOCKED: "cuts power, gravity completes alignment",
}

STAGE_PRECISION_CM = {
    RecoveryStage.APPROACHING: None,
    RecoveryStage.COARSE: RECOVERY_STAGE_1_PRECISION_CM,
    RecoveryStage.FINE: RECOVERY_STAGE_2_PRECISION_CM,
    RecoveryStage.MECHANICAL: RECOVERY_STAGE_3_PRECISION_CM,
    RecoveryStage.DOCKED: 0.0,
}

# Stage boundaries as a fraction of APPROACH_DURATION_S elapsed, evenly split
# across the three closing stages (coarse -> fine -> mechanical -> docked).
_STAGE_ORDER = [
    RecoveryStage.APPROACHING,
    RecoveryStage.COARSE,
    RecoveryStage.FINE,
    RecoveryStage.MECHANICAL,
    RecoveryStage.DOCKED,
]


@dataclass
class RecoverySequencer:
    stage: RecoveryStage = RecoveryStage.APPROACHING
    elapsed_s: float = 0.0
    charge_pct: float = 0.0
    docked: bool = False

    def recall(self) -> None:
        self.stage = RecoveryStage.APPROACHING
        self.elapsed_s = 0.0
        self.docked = False

    def tick(self, dt_s: float) -> None:
        if self.docked:
            self._tick_charging(dt_s)
            return

        self.elapsed_s += dt_s
        closing_stages = _STAGE_ORDER[1:]  # coarse, fine, mechanical, docked
        step = APPROACH_DURATION_S / (len(closing_stages) - 1)
        index = min(int(self.elapsed_s // step), len(closing_stages) - 1)
        self.stage = closing_stages[index]
        if self.stage == RecoveryStage.DOCKED:
            self.docked = True

    def _tick_charging(self, dt_s: float) -> None:
        full_charge_s = CHARGE_TIME_FROM_EMPTY_MIN_HIGH * 60.0
        self.charge_pct = min(100.0, self.charge_pct + (dt_s / full_charge_s) * 100.0)

    def targeting_ring_cm(self) -> float | None:
        """Current radius of the visualized targeting ring, for the map to shrink."""
        return STAGE_PRECISION_CM[self.stage]

    def charge_power_w(self) -> float:
        return CHARGE_PAD_POWER_W if self.docked and self.charge_pct < 100.0 else 0.0
