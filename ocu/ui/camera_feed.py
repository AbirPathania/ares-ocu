"""Simulated camera feed thumbnails, one per unit, left side of the app.

Section 13 of the build spec leaves camera feeds out of version one for a
specific reason: "no hardware exists yet," and a rendered feed could be
mistaken for real footage. The spec's own escape hatch is section 9.4's
note: "add it later only with SIMULATED FEED clearly labeled if a future
version needs one." This widget is that: a synthetic HUD-style panel, not
a photograph of anything, watermarked so it can never read as live video.
Nothing here is generated from or claims to be sensor data.
"""

from __future__ import annotations

import random

from PySide6.QtCore import QPointF, QRectF, QTimer, Qt
from PySide6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QWidget

TICK_MS = 220

LINK_COLORS = {
    "CONNECTED": QColor("#5ec9c2"),
    "DEGRADED": QColor("#e8b23c"),
    "LOST": QColor("#e05c4b"),
}


class SimulatedCameraFeed(QWidget):
    def __init__(self, unit_id: str, label: str, parent=None) -> None:
        super().__init__(parent)
        self.unit_id = unit_id
        self._label = label
        self._link_state = "CONNECTED"
        self._telemetry = ""
        self._rng = random.Random(hash(unit_id) & 0xFFFF)
        self._noise_offset = 0.0
        self._active = False

        self.setMinimumHeight(104)
        self.setMaximumHeight(124)

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()

    def set_active(self, active: bool) -> None:
        self._active = active
        self.update()

    def set_link_state(self, state: str) -> None:
        self._link_state = state
        self.update()

    def set_telemetry(self, text: str) -> None:
        self._telemetry = text
        self.update()

    def _on_tick(self) -> None:
        self._noise_offset = self._rng.uniform(0, 1000)
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())

        if not self._active:
            painter.fillRect(rect, QColor("#0b0e11"))
            painter.setPen(QPen(QColor("#3a4149")))
            painter.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
            painter.setPen(QPen(QColor("#4a5359")))
            painter.setFont(QFont("Menlo", 7, QFont.Bold))
            painter.drawText(QRectF(2, 2, rect.width() - 4, 12), Qt.AlignLeft, self._label)
            painter.setFont(QFont("Menlo", 7))
            painter.drawText(rect, Qt.AlignCenter, "IN BAY")
            painter.end()
            return

        gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0.0, QColor("#141b1f"))
        gradient.setColorAt(1.0, QColor("#0a0d10"))
        painter.fillRect(rect, gradient)

        # cheap synthetic "static": a handful of translucent bars, reseeded each tick
        rng = random.Random(int(self._noise_offset * 997) ^ hash(self.unit_id))
        for _ in range(20):
            x = rng.uniform(0, rect.width())
            y = rng.uniform(0, rect.height())
            w = rng.uniform(4, 22)
            h = rng.uniform(1, 3)
            alpha = rng.randint(6, 22)
            painter.fillRect(QRectF(x, y, w, h), QColor(255, 255, 255, alpha))

        link_color = LINK_COLORS.get(self._link_state, QColor("#666666"))
        pen = QPen(link_color)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
        self._draw_corner_ticks(painter, rect, link_color)

        painter.setPen(QPen(QColor(255, 255, 255, 90)))
        cx, cy = rect.center().x(), rect.center().y()
        painter.drawLine(cx - 6, cy, cx + 6, cy)
        painter.drawLine(cx, cy - 6, cx, cy + 6)

        painter.setFont(QFont("Menlo", 7, QFont.Bold))
        painter.setPen(QPen(QColor("#cfe0dd")))
        painter.drawText(QRectF(4, 2, rect.width() - 8, 11), Qt.AlignLeft, self._label)

        painter.setPen(QPen(QColor("#7c8891")))
        painter.setFont(QFont("Menlo", 7))
        painter.drawText(QRectF(4, rect.height() - 13, rect.width() - 8, 11), Qt.AlignLeft, self._telemetry)

        painter.setPen(QPen(QColor("#e8b23c")))
        painter.setFont(QFont("Menlo", 6, QFont.Bold))
        painter.drawText(QRectF(0, 2, rect.width() - 4, 11), Qt.AlignRight, "SIMULATED")

        painter.end()

    def _draw_corner_ticks(self, painter: QPainter, rect: QRectF, color: QColor) -> None:
        pen = QPen(color)
        pen.setWidth(2)
        painter.setPen(pen)
        length = 8
        corners = [
            (rect.topLeft(), (1, 0), (0, 1)),
            (rect.topRight(), (-1, 0), (0, 1)),
            (rect.bottomLeft(), (1, 0), (0, -1)),
            (rect.bottomRight(), (-1, 0), (0, -1)),
        ]
        for corner, dx, dy in corners:
            painter.drawLine(corner, corner + QPointF(dx[0] * length, dx[1] * length))
            painter.drawLine(corner, corner + QPointF(dy[0] * length, dy[1] * length))
