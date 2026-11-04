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
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

LINK_COLORS = {
    "CONNECTED": QColor("#4A9FD4"),
    "DEGRADED": QColor("#C9952F"),
    "LOST": QColor("#C04438"),
}


IR_CHIP = QRectF(0, 0, 26, 14)  # size of the visible / infrared toggle in a tile's top right corner
IR_COLOR = QColor("#F0A23C")
TRAIL_CHIP = QRectF(0, 0, 40, 14)  # the toggle that shows the reconstructed trail, left of the IR chip
TRAIL_COLOR = QColor("#5CC9A7")
GROUND_BIAS = 0.8  # how far toward the bottom of a thermal frame the crop sits when the tile is wider than the frame


class SimulatedCameraFeed(QWidget):
    irToggled = Signal(str, bool)  # unit id, whether the tile now shows infrared
    trailToggled = Signal(str, bool)  # unit id, whether the tile now shows the reconstructed trail

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
        self._starting = False  # the picture is being built; show that, not a stand-in
        self._clearing = False  # an aircraft still beside the rover's deck
        self._clearing_threshold = 0.0
        self._detections: list[dict] = []  # boxes from the detector, in the frame's own 0..1 coordinates
        self._ir = False  # the tile is set to show the thermal camera
        self._ir_frame = False  # and the frame held is a thermal one
        self._ir_scale: tuple[float, float, str] | None = None  # black and white levels (degrees C) and the palette
        self._ir_note = ""
        self._trail_available = False  # a reconstructed trail is loaded, so the chip is offered
        self._trail = False  # the tile is showing it

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

    def set_starting(self, starting: bool) -> None:
        """The camera is still starting up. Say so, rather than showing a
        different, cruder picture for a few seconds and then swapping it."""
        self._starting = starting
        self.update()

    def set_clearing(self, clearing: bool, threshold_m: float = 0.0) -> None:
        """An aircraft that has lifted off but has not yet cleared the rover. Its
        camera would show the vehicle's own body, so it stays off until it has."""
        self._clearing = clearing
        self._clearing_threshold = threshold_m
        if clearing:
            self._frame = None
        self.update()

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

    @property
    def ir_mode(self) -> bool:
        return self._ir

    def set_ir(self, on: bool) -> None:
        """Switch the tile between the visible camera and the thermal one. The frame of the other kind is dropped, so a
        picture from the wrong camera is never held on screen while the right one is being drawn."""
        if on != self._ir:
            self._ir = on
            self._ir_frame = False
            self._frame = None
            self._ir_scale = None
        self.update()

    @property
    def trail_mode(self) -> bool:
        return self._trail

    def set_trail_available(self, available: bool) -> None:
        self._trail_available = available
        if not available:
            self._trail = False
        self.update()

    def set_trail(self, on: bool) -> None:
        if on != self._trail:
            self._trail = on
            self._frame = None  # the other kind of picture is not held while the right one is being drawn
            self.update()

    def _trail_chip_rect(self) -> QRectF:
        ir = self._chip_rect()
        return QRectF(ir.left() - TRAIL_CHIP.width() - 4, ir.top(), TRAIL_CHIP.width(), TRAIL_CHIP.height())

    def set_ir_scale(self, lo: float, hi: float, palette: str, note: str = "") -> None:
        self._ir_scale = (lo, hi, palette)
        self._ir_note = note

    def _chip_rect(self) -> QRectF:
        return QRectF(self.width() - IR_CHIP.width() - 5, 3, IR_CHIP.width(), IR_CHIP.height())

    def mousePressEvent(self, event) -> None:
        if self._active and not self._emcon and self._trail_available and self._trail_chip_rect().contains(event.position()):
            if self._ir:  # one picture at a time: infrared gives way
                self._ir = False
                self._ir_frame = False
                self._ir_scale = None
                self.irToggled.emit(self.unit_id, False)
            self.set_trail(not self._trail)
            self.trailToggled.emit(self.unit_id, self._trail)
            return
        if self._active and not self._emcon and self._chip_rect().contains(event.position()):
            if self._trail:
                self.set_trail(False)
                self.trailToggled.emit(self.unit_id, False)
            self._ir = not self._ir
            self._ir_frame = False
            self._frame = None
            self._ir_scale = None
            self.update()
            self.irToggled.emit(self.unit_id, self._ir)
            return
        super().mousePressEvent(event)

    def set_frame(self, rgb: np.ndarray, ir: bool = False) -> None:
        """Show a rendered frame. `rgb` is (height, width, 3) uint8. A frame from the camera the tile is not set to
        show (a visible one arriving late for a tile that has been switched to infrared) is ignored."""
        if ir != self._ir:
            return
        self._ir_frame = ir
        frame = np.ascontiguousarray(rgb)
        height, width, _ = frame.shape
        # copy(): QImage borrows the array's memory, which will not outlive it.
        self._frame = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888).copy()
        self.update()

    def set_detections(self, detections: list[dict]) -> None:
        """Boxes to draw over the picture: {box: (u0, v0, u1, v1), label, score, confirmed}, as fractions of the
        rendered frame. A box is the detector's claim; a solid one has persisted, a dashed one has not yet."""
        self._detections = list(detections)
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

        if self._frame is not None and self._ir_frame:
            # Thermal: fill the tile's width and show the part of the sensor's frame that matters. The top of a forward-looking
            # thermal frame is sky, so the crop is weighted toward the ground at the bottom rather than centred.
            painter.fillRect(rect, QColor("#05070A"))
            painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
            source = QRectF(self._frame.rect())
            scale = max(rect.width() / source.width(), rect.height() / source.height())
            shown_w, shown_h = rect.width() / scale, rect.height() / scale
            crop = QRectF((source.width() - shown_w) / 2, (source.height() - shown_h) * GROUND_BIAS, shown_w, shown_h)
            painter.drawImage(rect, self._frame, crop)
            self._draw_ir_scale(painter, rect)
        elif self._frame is not None:
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
            self._draw_detections(painter, rect, source, crop)
        else:
            gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
            gradient.setColorAt(0.0, QColor("#141C24"))
            gradient.setColorAt(1.0, QColor("#0C1116"))
            painter.fillRect(rect, gradient)
            caption = (
                "CAMERA STARTING" if self._starting
                else f"CLEARING ROVER \u00b7 CAMERA ON AT {self._clearing_threshold:.0f} m" if self._clearing
                else ""
            )
            if caption:
                painter.setPen(QPen(QColor("#6F8396")))
                painter.setFont(QFont("Menlo", 8))
                # Below the centre, where the frame's crosshair is.
                painter.drawText(rect.adjusted(0, rect.height() * 0.3, 0, 0), Qt.AlignCenter, caption)

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
        text = f"{self._telemetry}  {self._ir_note}" if self._ir and self._ir_note else self._telemetry
        painter.drawText(telemetry_rect.translated(1, 1), Qt.AlignLeft, text)
        painter.setPen(QPen(QColor("#C3CED8")))
        painter.drawText(telemetry_rect, Qt.AlignLeft, text)

        self._draw_ir_chip(painter)
        self._draw_trail_chip(painter)
        painter.end()

    def _draw_ir_chip(self, painter: QPainter) -> None:
        """The small IR toggle in the corner: lit when the tile shows the thermal camera."""
        chip = self._chip_rect()
        painter.setBrush(QColor(240, 162, 60, 70) if self._ir else QColor(8, 12, 16, 150))
        painter.setPen(QPen(IR_COLOR if self._ir else QColor("#4F5B68"), 1))
        painter.drawRoundedRect(chip, 3, 3)
        painter.setFont(QFont("DIN Condensed", 7 + 2, QFont.Bold))
        painter.setPen(QPen(IR_COLOR if self._ir else QColor("#8796A4")))
        painter.drawText(chip, Qt.AlignCenter, "IR")

    def _draw_trail_chip(self, painter: QPainter) -> None:
        """The toggle for the reconstructed trail: offered only when one is loaded, lit when the tile shows it."""
        if not self._trail_available:
            return
        chip = self._trail_chip_rect()
        painter.setBrush(QColor(92, 201, 167, 70) if self._trail else QColor(8, 12, 16, 150))
        painter.setPen(QPen(TRAIL_COLOR if self._trail else QColor("#4F5B68"), 1))
        painter.drawRoundedRect(chip, 3, 3)
        painter.setFont(QFont("DIN Condensed", 7 + 2, QFont.Bold))
        painter.setPen(QPen(TRAIL_COLOR if self._trail else QColor("#8796A4")))
        painter.drawText(chip, Qt.AlignCenter, "TRAIL")

    def _draw_ir_scale(self, painter: QPainter, shown: QRectF) -> None:
        """A thin colour scale down the right of the thermal frame, with its two levels in degrees C."""
        if self._ir_scale is None:
            return
        from ocu.core.thermal_display import LUTS

        lo, hi, palette = self._ir_scale
        bar = QRectF(shown.right() - 7, shown.top() + 20, 5, max(shown.height() - 40, 10))
        lut = LUTS[palette]
        steps = max(int(bar.height()), 2)
        for i in range(steps):
            r, g, b = lut[int((1.0 - i / (steps - 1)) * 255)]
            painter.fillRect(QRectF(bar.left(), bar.top() + i * bar.height() / steps, bar.width(), bar.height() / steps + 0.6), QColor(int(r), int(g), int(b)))
        painter.setFont(QFont("Menlo", 6))
        painter.setPen(QPen(QColor("#E4EBF1")))
        painter.drawText(QRectF(bar.left() - 32, bar.top() - 11, 40, 10), Qt.AlignRight, f"{hi:.0f}\u00b0")
        painter.drawText(QRectF(bar.left() - 32, bar.bottom() + 1, 40, 10), Qt.AlignRight, f"{lo:.0f}\u00b0")

    DETECTION_COLORS = {"person": "#E5484D", "vehicle": "#4DA3FF", "animal": "#B58CFF"}
    #: A box round something the operator has not chosen to treat as a threat: thin and muted.
    CONTACT_BOX_COLOR = "#7F93A3"

    def _draw_detections(self, painter: QPainter, rect: QRectF, source: QRectF, crop: QRectF) -> None:
        """The detector's boxes, mapped through the same crop the picture is drawn with."""
        if not self._detections:
            return
        sx, sy = rect.width() / crop.width(), rect.height() / crop.height()
        painter.setFont(QFont("Menlo", 7))
        for d in self._detections:
            u0, v0, u1, v1 = d["box"]
            r = QRectF(
                rect.left() + (u0 * source.width() - crop.left()) * sx,
                rect.top() + (v0 * source.height() - crop.top()) * sy,
                (u1 - u0) * source.width() * sx,
                (v1 - v0) * source.height() * sy,
            )
            threat = d.get("threat", True)
            color = QColor(self.DETECTION_COLORS.get(d.get("kind", ""), "#E4EBF1") if threat else self.CONTACT_BOX_COLOR)
            pen = QPen(color)
            pen.setWidthF((1.6 if d.get("confirmed") else 1.0) if threat else 0.8)
            if not d.get("confirmed"):
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(r)
            text = f'{str(d.get("label", "")).upper()} {d.get("score", 0.0):.2f}'
            painter.setPen(QPen(QColor(0, 0, 0, 190)))
            painter.drawText(r.topLeft() + QPointF(2.0, -2.0 if r.top() > 11 else 10.0 + r.height()), text)
            painter.setPen(QPen(color))
            painter.drawText(r.topLeft() + QPointF(1.0, -3.0 if r.top() > 11 else 9.0 + r.height()), text)

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
