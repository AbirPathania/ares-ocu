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
    "CONNECTED": QColor("#4A9FD4"),
    "DEGRADED": QColor("#C9952F"),
    "LOST": QColor("#C04438"),
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
        self._emcon = False
        self._downlink_reason = ""

        self.setMinimumHeight(104)
        self.setMaximumHeight(124)

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()

    def set_active(self, active: bool) -> None:
        self._active = active
        self.update()

    def set_downlink(self, video: bool, mode: str) -> None:
        """Video is a function of the link, not of the camera.

        In Autonomous the aircraft is flying and its camera is running; the
        link simply is not carrying video. In Ghost nothing is radiated at
        all. Saying which is which beats a blank panel that looks broken.
        """
        self._emcon = not video
        self._downlink_reason = {
            "AUTONOMOUS": "NO VIDEO \u00b7 STATUS LINK ONLY",
            "GHOST": "EMISSIONS SILENT \u00b7 NO DOWNLINK",
            "LCA": "CONTACT LOST",
        }.get(mode, "")
        self.update()

    def set_emcon(self, emcon: bool) -> None:
        """Ghost Mode, section 4.1: zero intentional emissions, so there is no
        video downlink to show. The panel says why it is dark rather than
        going blank as though the feed had failed."""
        self._emcon = emcon
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

        if self._emcon:
            painter.fillRect(rect, QColor("#0C1116"))
            painter.setPen(QPen(QColor("#4A4128")))
            painter.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
            painter.setPen(QPen(QColor("#8A7A4A")))
            painter.setFont(QFont("DIN Condensed", 7 + 3, QFont.Bold))
            painter.drawText(QRectF(2, 2, rect.width() - 4, 12), Qt.AlignLeft, self._label)
            painter.setPen(QPen(QColor("#C9952F")))
            painter.drawText(rect, Qt.AlignCenter, self._downlink_reason or "NO DOWNLINK")
            painter.end()
            return

        if not self._active:
            painter.fillRect(rect, QColor("#0C1116"))
            painter.setPen(QPen(QColor("#3A4756")))
            painter.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
            painter.setPen(QPen(QColor("#4F5B68")))
            painter.setFont(QFont("DIN Condensed", 7 + 3, QFont.Bold))
            painter.drawText(QRectF(2, 2, rect.width() - 4, 12), Qt.AlignLeft, self._label)
            painter.setFont(QFont("Menlo", 7))
            painter.drawText(rect, Qt.AlignCenter, "IN BAY")
            painter.end()
            return

        gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0.0, QColor("#141C24"))
        gradient.setColorAt(1.0, QColor("#0C1116"))
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

        painter.setFont(QFont("DIN Condensed", 7 + 3, QFont.Bold))
        painter.setPen(QPen(QColor("#C6D0DA")))
        painter.drawText(QRectF(4, 2, rect.width() - 8, 11), Qt.AlignLeft, self._label)

        painter.setPen(QPen(QColor("#7B8794")))
        painter.setFont(QFont("Menlo", 7))
        painter.drawText(QRectF(4, rect.height() - 13, rect.width() - 8, 11), Qt.AlignLeft, self._telemetry)


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
