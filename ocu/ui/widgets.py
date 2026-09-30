"""Shared small widgets. Not part of the section 11.2 file list verbatim,
but every irreversible-command control in this UI needs the same safety
behavior from section 10, so it lives in one place rather than three.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QComboBox, QListView, QPushButton

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


def make_styled_combo(entries: list[tuple[str, str]]) -> QComboBox:
    """A combo whose popup obeys the stylesheet and fits its longest entry.

    A combo's popup is a separate top-level widget: it inherits none of the
    styling applied to the closed control, and its width follows the closed
    combo rather than its own contents, so the longest option elides. Both
    have to be handled explicitly, in one place rather than per call site.

    entries are (label, value) pairs; the value is stored as item data so the
    displayed text never has to double as the data.
    """
    combo = QComboBox()
    combo.setView(QListView())
    combo.setSizeAdjustPolicy(QComboBox.AdjustToContents)
    for label, value in entries:
        combo.addItem(label, value)

    view = combo.view()
    metrics = view.fontMetrics()
    widest = max((metrics.horizontalAdvance(label) for label, _ in entries), default=0)
    view.setTextElideMode(Qt.ElideNone)
    view.setMinimumWidth(widest + 72)
    return combo
