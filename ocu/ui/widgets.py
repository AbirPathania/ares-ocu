"""Shared small widgets. Not part of the section 11.2 file list verbatim,
but every irreversible-command control in this UI needs the same safety
behavior from section 10, so it lives in one place rather than three.
"""

from __future__ import annotations

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import QPushButton

from ocu.core.constants import IRREVERSIBLE_COMMAND_HOLD_S


class HoldToConfirmButton(QPushButton):
    """A launch, a full fleet return, a geofence override: any irreversible
    command requires a deliberate second confirmation, a two second hold,
    never a dialog box people learn to click through out of habit, section 10.

    Emits confirmed() only after the button is held down continuously for
    IRREVERSIBLE_COMMAND_HOLD_S. Releasing early cancels silently.
    """

    confirmed = Signal()

    def __init__(self, label: str, parent=None) -> None:
        super().__init__(label, parent)
        self._label = label
        self._hold_ms = int(IRREVERSIBLE_COMMAND_HOLD_S * 1000)
        self._elapsed_ms = 0
        self._tick_ms = 50

        self._timer = QTimer(self)
        self._timer.setInterval(self._tick_ms)
        self._timer.timeout.connect(self._on_tick)

        self.pressed.connect(self._start_hold)
        self.released.connect(self._cancel_hold)

    def _start_hold(self) -> None:
        if not self.isEnabled():
            return
        self._elapsed_ms = 0
        self._timer.start()

    def _cancel_hold(self) -> None:
        self._timer.stop()
        self._elapsed_ms = 0
        self.setText(self._label)

    def _on_tick(self) -> None:
        self._elapsed_ms += self._tick_ms
        remaining = max(0, self._hold_ms - self._elapsed_ms)
        self.setText(f"{self._label}… hold ({remaining / 1000:.1f}s)")
        if self._elapsed_ms >= self._hold_ms:
            self._timer.stop()
            self.setText(self._label)
            self.confirmed.emit()
