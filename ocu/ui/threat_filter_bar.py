"""The strip of chips that lets the operator choose which classes count as a threat.

A chip is on or off. A class with no source cannot be turned on and says why. Mines and IEDs are guarded: switching
them off takes the same two second hold as an irreversible command (section 10), because an accidental click that
silenced them would be the most dangerous mistake this strip can make; switching them on is a plain click.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ocu.core.constants import IRREVERSIBLE_COMMAND_HOLD_S
from ocu.core.threat_filter import GROUPS, GUARDED, LABELS, UNAVAILABLE, ThreatFilter
from ocu.ui.theme import HAIRLINE, TEXT, TEXT_DIM, TEXT_FAINT, THREAT

CHIP_HEIGHT = 24
HOLD_TICK_MS = 40


class ThreatChip(QWidget):
    """One class: on (lit), off (outline), unavailable (struck through), or being held off."""

    toggled = Signal(str, bool)

    def __init__(self, group: str, on: bool, parent=None) -> None:
        super().__init__(parent)
        self.group = group
        self._on = on
        self._held_ms = 0
        self._timer = QTimer(self)
        self._timer.setInterval(HOLD_TICK_MS)
        self._timer.timeout.connect(self._advance_hold)
        self.setFixedHeight(CHIP_HEIGHT)
        self.setMinimumWidth(len(LABELS[group]) * 8 + 18)
        self.setCursor(Qt.PointingHandCursor if group not in UNAVAILABLE else Qt.ForbiddenCursor)
        if group in UNAVAILABLE:
            self.setToolTip(f"{LABELS[group]}: {UNAVAILABLE[group]}")
        elif group in GUARDED:
            self.setToolTip(f"{LABELS[group]}. Switching this off takes a {IRREVERSIBLE_COMMAND_HOLD_S:.0f} second hold.")
        else:
            self.setToolTip(f"Treat {LABELS[group].lower()} as threats")

    @property
    def on(self) -> bool:
        return self._on

    def set_on(self, on: bool) -> None:
        self._on = on
        self._cancel_hold()
        self.update()

    @property
    def holding(self) -> bool:
        return self._timer.isActive()

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or self.group in UNAVAILABLE:
            return
        if self._on and self.group in GUARDED:
            self._held_ms = 0
            self._timer.start()
            self.update()
        else:
            self.toggled.emit(self.group, not self._on)

    def mouseReleaseEvent(self, event) -> None:
        self._cancel_hold()

    def leaveEvent(self, event) -> None:
        self._cancel_hold()

    def _cancel_hold(self) -> None:
        if self._timer.isActive():
            self._timer.stop()
        self._held_ms = 0
        self.update()

    def _advance_hold(self) -> None:
        self._held_ms += HOLD_TICK_MS
        if self._held_ms >= IRREVERSIBLE_COMMAND_HOLD_S * 1000:
            self._timer.stop()
            self._held_ms = 0
            self.toggled.emit(self.group, False)
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        unavailable = self.group in UNAVAILABLE
        if unavailable:
            border, fill, text = QColor(HAIRLINE), QColor(0, 0, 0, 0), QColor(TEXT_FAINT)
        elif self._on:
            border, fill, text = QColor(THREAT), QColor(THREAT.red(), THREAT.green(), THREAT.blue(), 46), QColor(TEXT)
        else:
            border, fill, text = QColor(HAIRLINE), QColor(0, 0, 0, 0), QColor(TEXT_DIM)
        painter.setPen(QPen(border, 1))
        painter.setBrush(fill)
        painter.drawRoundedRect(rect, 3, 3)
        if self._held_ms:  # the hold, as a bar filling along the foot of the chip
            share = min(self._held_ms / (IRREVERSIBLE_COMMAND_HOLD_S * 1000), 1.0)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(THREAT))
            painter.drawRect(QRectF(rect.left() + 1, rect.bottom() - 2.5, (rect.width() - 2) * share, 2.5))
        painter.setFont(QFont("DIN Condensed", 8 + 3, QFont.Bold))
        painter.setPen(QPen(text))
        painter.drawText(rect, Qt.AlignCenter, LABELS[self.group])
        if unavailable:
            painter.setPen(QPen(QColor(TEXT_FAINT), 1))
            painter.drawLine(rect.left() + 6, rect.center().y(), rect.right() - 6, rect.center().y())
        painter.end()


class ThreatFilterBar(QWidget):
    """Chips for every class, a caption, and a count of what is being shown as a quiet contact."""

    changed = Signal()

    def __init__(self, threat_filter: ThreatFilter, parent=None) -> None:
        super().__init__(parent)
        self.filter = threat_filter
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 4)
        outer.setSpacing(4)
        self.caption = QLabel("TREAT AS A THREAT")
        self.caption.setObjectName("sectionLabel")
        outer.addWidget(self.caption)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        self.chips: dict[str, ThreatChip] = {}
        for group in GROUPS:
            chip = ThreatChip(group, threat_filter.is_enabled(group))
            chip.toggled.connect(self._on_chip)
            self.chips[group] = chip
            row.addWidget(chip)
        row.addStretch(1)
        outer.addLayout(row)

        self.contacts = QLabel("")
        self.contacts.setObjectName("mutedNote")
        outer.addWidget(self.contacts)
        threat_filter.subscribe(self._sync)

    def _on_chip(self, group: str, on: bool) -> None:
        self.filter.set_enabled(group, on)

    def _sync(self, _filter) -> None:
        for group, chip in self.chips.items():
            chip.set_on(self.filter.is_enabled(group))
        self.changed.emit()

    def set_contacts(self, counts: dict[str, int]) -> None:
        """What is being held back as a contact, by class, e.g. '2 vehicles, 1 animal'."""
        names = {"vehicles": "vehicle", "animals": "animal", "people": "person", "explosives": "buried device", "other": "other", "arms": "arm"}
        parts = [f"{n} {names[g]}{'s' if n != 1 and g != 'people' else ''}" if g != "people" else f"{n} {'people' if n != 1 else 'person'}" for g, n in counts.items() if n]
        self.contacts.setText("CONTACTS  " + ", ".join(parts) if parts else "")
