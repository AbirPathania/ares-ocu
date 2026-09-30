"""Mission phase rail, the eight phases of section 5 as one glanceable row.

The phase name alone tells an operator what the machine is doing; the rail
tells them how far through the mission that is and what comes next, which
is the question a log line cannot answer without being read.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

from ocu.core.constants import MISSION_PHASES

SEGMENT_W = 26
SEGMENT_H = 6
GAP = 3

DONE_COLOR = QColor("#2c5f57")
ACTIVE_COLOR = QColor("#5ec9c2")
PENDING_COLOR = QColor("#222c33")


class PhaseRail(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._index = -1
        self.setFixedHeight(30)
        self.setMinimumWidth(len(MISSION_PHASES) * (SEGMENT_W + GAP) + 250)
        self.setToolTip(self._build_tooltip())

    def _build_tooltip(self) -> str:
        lines = ["Mission phase, section 5 of the build spec:"]
        lines += [
            f"  {i + 1}. {name.replace('_', ' ').title()}" for i, name in enumerate(MISSION_PHASES)
        ]
        return "\n".join(lines)

    def set_phase(self, phase: str) -> None:
        try:
            self._index = MISSION_PHASES.index(phase)
        except ValueError:
            self._index = -1
        self.update()

    def reset(self) -> None:
        self._index = -1
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)

        painter.setPen(QPen(QColor("#6c7a80")))
        painter.setFont(QFont("Menlo", 7))
        painter.drawText(QRectF(0, 0, 200, 10), Qt.AlignLeft | Qt.AlignVCenter, "MISSION PHASE")
        painter.setPen(Qt.NoPen)

        y = self.height() / 2 + 1
        for i in range(len(MISSION_PHASES)):
            x = i * (SEGMENT_W + GAP)
            if i < self._index:
                color = DONE_COLOR
            elif i == self._index:
                color = ACTIVE_COLOR
            else:
                color = PENDING_COLOR
            painter.fillRect(QRectF(x, y, SEGMENT_W, SEGMENT_H), color)

        label_x = len(MISSION_PHASES) * (SEGMENT_W + GAP) + 10
        if self._index >= 0:
            name = MISSION_PHASES[self._index].replace("_", " ").upper()
            painter.setPen(QPen(ACTIVE_COLOR))
            painter.setFont(QFont("Menlo", 10, QFont.Bold))
            painter.drawText(
                QRectF(label_x, 0, 180, self.height()), Qt.AlignLeft | Qt.AlignVCenter, name
            )
            painter.setPen(QPen(QColor("#6c7a80")))
            painter.setFont(QFont("Menlo", 9))
            painter.drawText(
                QRectF(label_x + 152, 0, 130, self.height()),
                Qt.AlignLeft | Qt.AlignVCenter,
                f"STEP {self._index + 1} OF {len(MISSION_PHASES)}",
            )
        else:
            painter.setPen(QPen(QColor("#6c7a80")))
            painter.setFont(QFont("Menlo", 10))
            painter.drawText(
                QRectF(label_x, 0, 180, self.height()), Qt.AlignLeft | Qt.AlignVCenter, "STANDBY"
            )
        painter.end()
