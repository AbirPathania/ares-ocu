"""MANUAL lock and handoff logic, section 4.2.

Only one unit can hold MANUAL at a time, because there is one operator with
one pair of hands. Taking MANUAL on a different unit always drops the
previous holder back to whatever it held immediately before MANUAL (GUIDED
or AUTO), never silently and never as a side effect of anything else. The
caller is expected to be a single explicit "take control" action.
"""

from __future__ import annotations

from ocu.core.unit import Unit


class ControlAuthority:
    """Tracks which single unit currently holds MANUAL across a fleet."""

    def __init__(self, units: dict[str, Unit]) -> None:
        self._units = units
        self._manual_holder: str | None = None
        self._pre_manual_mode: dict[str, str] = {}

    @property
    def manual_holder(self) -> str | None:
        return self._manual_holder

    def take_manual(self, unit_id: str) -> None:
        """Give MANUAL to unit_id, releasing whoever held it before.

        A single explicit call per the section 4.2 rule: this is never
        invoked automatically by mode changes, link events, or timers.
        """
        if unit_id not in self._units:
            raise KeyError(f"no such unit: {unit_id!r}")

        if self._manual_holder == unit_id:
            return

        if self._manual_holder is not None:
            self.release_manual()

        unit = self._units[unit_id]
        self._pre_manual_mode[unit_id] = unit.control_mode
        unit.control_mode = "MANUAL"
        self._manual_holder = unit_id

    def release_manual(self) -> None:
        """Drop the current MANUAL holder back to its prior mode.

        Reverts to GUIDED or AUTO, whichever the unit last held, per the
        section 4.2 rule. Defaults to AUTO if no prior mode was recorded.
        """
        if self._manual_holder is None:
            return

        unit = self._units[self._manual_holder]
        unit.control_mode = self._pre_manual_mode.pop(self._manual_holder, "AUTO")
        self._manual_holder = None

    def set_mode(self, unit_id: str, mode: str) -> None:
        """Set GUIDED, AUTO, or HOLD on a unit that does not hold MANUAL.

        Use take_manual to give a unit MANUAL; this method refuses to set
        MANUAL directly so the handoff bookkeeping above cannot be bypassed.
        """
        if mode == "MANUAL":
            raise ValueError("use take_manual() to grant MANUAL")
        if unit_id == self._manual_holder:
            raise ValueError(f"{unit_id} holds MANUAL, release it first")
        self._units[unit_id].control_mode = mode
