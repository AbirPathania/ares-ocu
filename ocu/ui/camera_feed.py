"""Camera feed tiles, one per unit, left side of the app.

Each tile shows a view rendered from the same simulated world the sensors
are simulated against (core/camera.py): the rover's forward camera, and the
tricopters' straight-down cameras. A camera sees the world rather than the
map, so what is on a tile is what is actually out there, including ground
the LiDAR has not reached yet.

The widget itself only presents frames and the status around them. Whether
a frame is available is the link's decision, not the camera's: in
Autonomous the aircraft is flying and its camera is running, the link just
is not carrying video, and in Ghost nothing is radiated at all. The tile
says which, instead of going blank as though the feed had failed.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

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
        self._active = False
        self._emcon = False
        self._downlink_reason = ""
        self._frame: QImage | None = None

        # A 16:9 feed wants height, and the left column has it: the roster
        # below is a fixed stack of four cards, so whatever is left belongs
        # to the feeds rather than to empty background.
        # Low floor, generous ceiling. Four tiles with a 104 px floor each
        # demanded 723 px of the left column on their own, which is more
        # than a small screen has for the whole window. They give way before
        # anything else does, because a feed is still legible at a third of
        # its preferred size and a roster card is not.
        self.setMinimumHeight(44)
        self.setMaximumHeight(220)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

    def set_active(self, active: bool) -> None:
        self._active = active
        if not active:
            self._frame = None  # a unit back in its bay has no picture to show
        self.update()

    def render_size(self) -> tuple[int, int]:
        """The resolution frames should be rendered at for this tile.

        Rendered at the tile's own pixels rather than below them and scaled
        up: a soft, upscaled picture is the one thing on this display that
        cannot be defended as what the sensor really saw.
        """
        ratio = self.devicePixelRatioF() or 1.0
        return (
            int(min(max(self.width() * ratio, 96), 960)),
            int(min(max(self.height() * ratio, 40), 440)),
        )

    def set_frame(self, rgb: np.ndarray) -> None:
        """Show a rendered frame. `rgb` is (height, width, 3) uint8."""
        frame = np.ascontiguousarray(rgb)
        height, width, _ = frame.shape
        # copy(): QImage borrows the array's memory, which will not outlive it.
        self._frame = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888).copy()
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

        link_color = LINK_COLORS.get(self._link_state, QColor("#666666"))

        if self._frame is not None:
            painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
            # Fill the tile and crop, rather than letterbox: a thin black bar
            # above and below a 2.7:1 tile is a lot of its height.
            source = QRectF(self._frame.rect())
            scale = max(rect.width() / source.width(), rect.height() / source.height())
            shown_w, shown_h = rect.width() / scale, rect.height() / scale
            crop = QRectF(
                (source.width() - shown_w) / 2, (source.height() - shown_h) / 2, shown_w, shown_h
            )
            painter.drawImage(rect, self._frame, crop)
        else:
            gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
            gradient.setColorAt(0.0, QColor("#141C24"))
            gradient.setColorAt(1.0, QColor("#0C1116"))
            painter.fillRect(rect, gradient)

        # Scrims behind the text, so the label and telemetry stay legible
        # over whatever the camera happens to be looking at, bright sky
        # included.
        for edge_top, edge_bottom, a_top, a_bottom in (
            (0.0, 18.0, 150, 0),
            (rect.height() - 18.0, rect.height(), 0, 150),
        ):
            scrim = QLinearGradient(0, edge_top, 0, edge_bottom)
            scrim.setColorAt(0.0, QColor(6, 9, 12, a_top))
            scrim.setColorAt(1.0, QColor(6, 9, 12, a_bottom))
            painter.fillRect(QRectF(0, edge_top, rect.width(), edge_bottom - edge_top), scrim)

        pen = QPen(link_color)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
        self._draw_corner_ticks(painter, rect, link_color)

        painter.setPen(QPen(QColor(255, 255, 255, 90)))
        cx, cy = rect.center().x(), rect.center().y()
        painter.drawLine(cx - 6, cy, cx + 6, cy)
        painter.drawLine(cx, cy - 6, cx, cy + 6)

        label_rect = QRectF(5, 2, rect.width() - 10, 12)
        painter.setFont(QFont("DIN Condensed", 7 + 3, QFont.Bold))
        painter.setPen(QPen(QColor(0, 0, 0, 170)))
        painter.drawText(label_rect.translated(1, 1), Qt.AlignLeft, self._label)
        painter.setPen(QPen(QColor("#E4EBF1")))
        painter.drawText(label_rect, Qt.AlignLeft, self._label)

        telemetry_rect = QRectF(5, rect.height() - 14, rect.width() - 10, 12)
        painter.setFont(QFont("Menlo", 7))
        painter.setPen(QPen(QColor(0, 0, 0, 170)))
        painter.drawText(telemetry_rect.translated(1, 1), Qt.AlignLeft, self._telemetry)
        painter.setPen(QPen(QColor("#C3CED8")))
        painter.drawText(telemetry_rect, Qt.AlignLeft, self._telemetry)

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
