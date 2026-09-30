"""QPainter canvas: units, trails, detections, formation preview, section 9.3.

This is where the CRDT map from section 8 lives visually, where the launch
console's live formation preview (section 6.4) draws itself as the operator
turns the bearing dial, and where the Cut Link / Restore Link controls from
section 8.5 sit, since this is the panel where their effect is actually
visible.

The rover carries all three aircraft "folded in a single drawer bay on the
top deck" (section 2) until they launch — so a docked aircraft is not a
separate icon wandering near the rover, it is a slot in that bay. Terrain
features are cosmetic, procedurally generated once from a fixed seed; there
is no real terrain data anywhere in this build (see DECISIONS.md).
"""

from __future__ import annotations

import math
import random

from PySide6.QtCore import QPointF, QRectF, QTimer, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD, TETHER_LIMIT_M
from ocu.core.mesh_state import voxel_key
from ocu.ui.icons import draw_threat_glyph
from ocu.ui.launch_console import short_label

SCALE_PX_PER_M = 9.0
TRAIL_MAX_POINTS = 240
COMPASS_RADIUS_PX = 60.0
RADAR_TICK_MS = 60
TERRAIN_SEED = 20260930
BAY_SLOT_ORDER = ["tricopter-1", "tricopter-2", "tricopter-3"]

UNIT_COLORS = {
    "rover": QColor("#5ec9c2"),
    "tricopter-1": QColor("#e8b23c"),
    "tricopter-2": QColor("#c97ee8"),
    "tricopter-3": QColor("#7ea8e8"),
}
SELECTED_RING_COLOR = QColor("#f4f7f6")
TRAIL_COLOR = QColor(255, 255, 255, 36)
DETECTION_COLOR = QColor("#e8b23c")
ALERT_COLOR = QColor("#e05c4b")
CONFIRMED_COLOR = QColor("#ff6a55")
GEOFENCE_COLOR = QColor("#e8b23c")
LINK_CUT_TINT = QColor("#e05c4b")
TERRAIN_VEGETATION = QColor(94, 201, 130, 14)
TERRAIN_ROCK = QColor(150, 130, 105, 18)
TERRAIN_CONTOUR = QColor(255, 255, 255, 13)
TETHER_COLOR = QColor(94, 201, 194, 46)
TETHER_LABEL_COLOR = QColor(94, 201, 194, 110)


class MapView(QWidget):
    cutLinkRequested = Signal()
    restoreLinkRequested = Signal()
    bearingDragged = Signal(float)
    unitClicked = Signal(str)
    bayLaunchRequested = Signal()  # operator clicked a docked slot in the rover's bay

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 420)
        self.setMouseTracking(True)

        self._units: dict[str, dict] = {}
        self._docked: dict[str, bool] = {uid: True for uid in BAY_SLOT_ORDER}
        self._trails: dict[str, list[tuple[float, float]]] = {}
        self._detections: list[dict] = []
        self._detection_by_key: dict[tuple, dict] = {}
        self._geofence_vertices: list[tuple[float, float]] = []
        self._formation_preview: list[tuple[float, float, float]] | None = None
        self._formation_origin: tuple[float, float] | None = None
        self._selected_unit: str | None = None
        self._link_cut = False
        self._link_cut_units: list[str] = []
        self._compass_active = False
        self._bearing_deg = 0.0
        self._dragging_compass = False
        self._cursor_world: tuple[float, float] | None = None

        self._pan = QPointF(0.0, 0.0)
        self._panning = False
        self._pan_start = QPointF()
        self._pan_origin = QPointF()

        self._terrain = self._generate_terrain()
        self._radar_angle = 0.0
        self._radar_timer = QTimer(self)
        self._radar_timer.setInterval(RADAR_TICK_MS)
        self._radar_timer.timeout.connect(self._on_radar_tick)
        self._radar_timer.start()

        self._build_link_controls()

    def _generate_terrain(self) -> list[dict]:
        rng = random.Random(TERRAIN_SEED)
        features = []
        for _ in range(26):
            features.append(
                {
                    "kind": "vegetation",
                    "x": rng.uniform(-15, 45),
                    "y": rng.uniform(-25, 25),
                    "r": rng.uniform(1.5, 5.0),
                }
            )
        for _ in range(14):
            features.append(
                {
                    "kind": "rock",
                    "x": rng.uniform(-15, 45),
                    "y": rng.uniform(-25, 25),
                    "r": rng.uniform(0.8, 2.2),
                }
            )
        for _ in range(5):
            features.append(
                {
                    "kind": "contour",
                    "x": rng.uniform(-10, 40),
                    "y": rng.uniform(-20, 20),
                    "r": rng.uniform(8.0, 18.0),
                }
            )
        return features

    def _build_link_controls(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)

        self.launch_aircraft_btn = QPushButton("▲ LAUNCH  3 READY")
        self.launch_aircraft_btn.setObjectName("launchAircraftButton")
        self.launch_aircraft_btn.clicked.connect(self.bayLaunchRequested.emit)
        toolbar.addWidget(self.launch_aircraft_btn)

        self.cut_link_btn = QPushButton("CUT LINK")
        self.cut_link_btn.setObjectName("cutLinkButton")
        self.cut_link_btn.clicked.connect(self._on_cut_clicked)
        toolbar.addWidget(self.cut_link_btn)

        layout.addLayout(toolbar)

    def _on_cut_clicked(self) -> None:
        if self._link_cut:
            self.restoreLinkRequested.emit()
        else:
            self.cutLinkRequested.emit()

    def _on_radar_tick(self) -> None:
        self._radar_angle = (self._radar_angle + 2.4) % 360.0
        self.update()

    # --- public API, called by main_window in response to engine signals ---

    def set_unit_pose(self, unit_id: str, x: float, y: float, z: float, heading: float) -> None:
        self._units[unit_id] = {"x": x, "y": y, "z": z, "heading": heading}
        if not self._docked.get(unit_id, False):
            trail = self._trails.setdefault(unit_id, [])
            if not trail or trail[-1] != (x, y):
                trail.append((x, y))
                if len(trail) > TRAIL_MAX_POINTS:
                    trail.pop(0)
        self.update()

    def set_unit_docked(self, unit_id: str, docked: bool) -> None:
        if unit_id in self._docked:
            self._docked[unit_id] = docked
            if not docked:
                self._trails[unit_id] = []
        self._refresh_launch_button()
        self.update()

    def _refresh_launch_button(self) -> None:
        ready = sum(1 for docked in self._docked.values() if docked)
        self.launch_aircraft_btn.setEnabled(ready > 0)
        self.launch_aircraft_btn.setText(
            f"▲ LAUNCH  {ready} READY" if ready else "▲ BAY EMPTY"
        )

    def focus_on(self, x: float, y: float) -> None:
        """Center the view on a world coordinate, used when a threat is picked."""
        self._pan = QPointF(-x * SCALE_PX_PER_M, y * SCALE_PX_PER_M)
        self.update()

    def clear(self) -> None:
        self._units.clear()
        self._trails.clear()
        self._detections.clear()
        self._detection_by_key.clear()
        self._formation_preview = None
        self._link_cut = False
        self._link_cut_units = []
        self._docked = {uid: True for uid in BAY_SLOT_ORDER}
        self._pan = QPointF(0.0, 0.0)
        self.cut_link_btn.setText("CUT LINK")
        self._refresh_launch_button()
        self.update()

    def set_selected(self, unit_id: str | None) -> None:
        self._selected_unit = unit_id
        self.update()

    def add_detection(self, detection: dict) -> None:
        """One marker per voxel, not per observation. Four sensors hitting the
        same buried object is one thing on the ground, and drawing it four
        times stacked four labels on top of each other. Highest confidence
        wins, matching the merge rule in core/mesh_state.py."""
        key = voxel_key(detection["x"], detection["y"], 0.0)
        existing = self._detection_by_key.get(key)
        if existing is None:
            entry = dict(detection, confirmed=False)
            self._detection_by_key[key] = entry
            self._detections.append(entry)
        elif detection.get("confidence", 0.0) > existing.get("confidence", 0.0):
            existing["confidence"] = detection["confidence"]
            existing["type"] = detection.get("type", existing.get("type", "unknown"))
        self.update()

    def confirm_detection(self, x: float, y: float) -> None:
        entry = self._detection_by_key.get(voxel_key(x, y, 0.0))
        if entry is not None:
            entry["confirmed"] = True
        self.update()

    def set_geofence(self, vertices: list[tuple[float, float]]) -> None:
        self._geofence_vertices = vertices
        self.update()

    def set_link_cut_visual(self, cut: bool, units: list[str] | None = None) -> None:
        self._link_cut = cut
        self._link_cut_units = units or []
        self.cut_link_btn.setText("RESTORE LINK" if cut else "CUT LINK")
        self.update()

    def set_formation_preview(
        self,
        targets: list[tuple[float, float, float]] | None,
        origin: tuple[float, float] | None = None,
    ) -> None:
        self._formation_preview = targets
        self._formation_origin = origin
        self.update()

    def set_compass_active(self, active: bool, origin: tuple[float, float] | None = None) -> None:
        self._compass_active = active
        if origin is not None:
            self._formation_origin = origin
        self.update()

    def set_bearing(self, bearing_deg: float) -> None:
        self._bearing_deg = bearing_deg % 360.0
        self.update()

    # --- coordinate transform ---

    def _center(self) -> QPointF:
        return QPointF(self.width() / 2.0 + self._pan.x(), self.height() / 2.0 + self._pan.y())

    def _to_screen(self, x: float, y: float) -> QPointF:
        c = self._center()
        return QPointF(c.x() + x * SCALE_PX_PER_M, c.y() - y * SCALE_PX_PER_M)

    def _to_world(self, px: QPointF) -> tuple[float, float]:
        c = self._center()
        return ((px.x() - c.x()) / SCALE_PX_PER_M, -(px.y() - c.y()) / SCALE_PX_PER_M)

    def _rover_pose(self) -> dict:
        return self._units.get("rover", {"x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0})

    # --- painting ---

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0e1215"))

        self._draw_grid(painter)
        self._draw_terrain(painter)
        self._draw_geofence(painter)
        self._draw_tether_ring(painter)
        self._draw_radar_sweep(painter)
        self._draw_trails(painter)
        self._draw_formation_preview(painter)
        self._draw_detections(painter)
        self._draw_units(painter)
        self._draw_rover_bay(painter)
        if self._compass_active:
            self._draw_bearing_compass(painter)
        self._draw_north_indicator(painter)
        self._draw_scale_bar(painter)
        if self._link_cut:
            self._draw_link_cut_banner(painter)
        self._draw_cursor_readout(painter)

        painter.end()

    def _draw_grid(self, painter: QPainter) -> None:
        pen = QPen(QColor(255, 255, 255, 14))
        pen.setWidth(1)
        painter.setPen(pen)
        step = SCALE_PX_PER_M * 5  # every 5 m
        c = self._center()
        x = c.x() % step
        while x < self.width():
            painter.drawLine(QPointF(x, 0), QPointF(x, self.height()))
            x += step
        y = c.y() % step
        while y < self.height():
            painter.drawLine(QPointF(0, y), QPointF(self.width(), y))
            y += step

    def _draw_terrain(self, painter: QPainter) -> None:
        painter.setPen(Qt.NoPen)
        for feature in self._terrain:
            center = self._to_screen(feature["x"], feature["y"])
            radius_px = feature["r"] * SCALE_PX_PER_M
            if feature["kind"] == "vegetation":
                painter.setBrush(QBrush(TERRAIN_VEGETATION))
                painter.drawEllipse(center, radius_px, radius_px)
            elif feature["kind"] == "rock":
                painter.setBrush(QBrush(TERRAIN_ROCK))
                painter.drawEllipse(center, radius_px * 0.6, radius_px * 0.6)
            else:  # contour
                pen = QPen(TERRAIN_CONTOUR)
                pen.setWidth(1)
                painter.setPen(pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(center, radius_px, radius_px)
                painter.setPen(Qt.NoPen)

    def _draw_tether_ring(self, painter: QPainter) -> None:
        """The UWB tether, section 2. Aircraft never work outside this ring, so
        it is drawn as a standing boundary rather than only when something
        approaches it."""
        rover = self._rover_pose()
        center = self._to_screen(rover["x"], rover["y"])
        radius_px = TETHER_LIMIT_M * SCALE_PX_PER_M

        pen = QPen(TETHER_COLOR)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, radius_px, radius_px)

        painter.setPen(QPen(TETHER_LABEL_COLOR))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(
            QRectF(center.x() - 60, center.y() - radius_px - 16, 120, 13),
            Qt.AlignHCenter,
            f"UWB TETHER {TETHER_LIMIT_M:.0f} m",
        )

    def _draw_radar_sweep(self, painter: QPainter) -> None:
        rover = self._rover_pose()
        center = self._to_screen(rover["x"], rover["y"])
        sweep_radius = 200.0
        path = QPainterPath()
        path.moveTo(center)
        start_angle = -self._radar_angle
        path.arcTo(
            QRectF(center.x() - sweep_radius, center.y() - sweep_radius, sweep_radius * 2, sweep_radius * 2),
            start_angle,
            -18,
        )
        path.closeSubpath()
        gradient_color = QColor("#5ec9c2")
        gradient_color.setAlpha(14)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(gradient_color))
        painter.drawPath(path)

    def _draw_geofence(self, painter: QPainter) -> None:
        if len(self._geofence_vertices) < 3:
            return
        pen = QPen(GEOFENCE_COLOR)
        pen.setWidth(2)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        polygon = QPolygonF([self._to_screen(x, y) for x, y in self._geofence_vertices])
        painter.drawPolygon(polygon)

    def _draw_trails(self, painter: QPainter) -> None:
        """Older track fades out, so the eye reads direction of travel without
        a legend. Drawn in three alpha bands rather than per-segment so the
        pen changes stay cheap at the radar tick rate."""
        pen = QPen()
        pen.setWidth(1)
        for unit_id, trail in self._trails.items():
            if len(trail) < 2:
                continue
            base = QColor(UNIT_COLORS.get(unit_id, QColor("#cccccc")))
            bands = 3
            band_len = max(len(trail) // bands, 1)
            for band in range(bands):
                start = band * band_len
                end = len(trail) if band == bands - 1 else (band + 1) * band_len + 1
                segment = trail[start:end]
                if len(segment) < 2:
                    continue
                color = QColor(base)
                color.setAlpha(18 + band * 34)
                pen.setColor(color)
                painter.setPen(pen)
                for a, b in zip(segment, segment[1:]):
                    painter.drawLine(self._to_screen(*a), self._to_screen(*b))

    def _draw_detections(self, painter: QPainter) -> None:
        pulse = (math.sin(math.radians(self._radar_angle * 4)) + 1.0) / 2.0  # 0..1
        for d in self._detections:
            center = self._to_screen(d["x"], d["y"])
            confidence = d.get("confidence", 0.0)
            threat_type = d.get("type", "unknown")

            if d.get("confirmed"):
                color = CONFIRMED_COLOR
                radius = 13.0
            elif confidence >= ALERT_CONFIDENCE_THRESHOLD:
                color = ALERT_COLOR
                radius = 10.0 + pulse * 3.0
            else:
                color = DETECTION_COLOR
                radius = 6.0 + confidence * 5.0

            if d.get("confirmed") or confidence >= ALERT_CONFIDENCE_THRESHOLD:
                ring_color = QColor(color)
                ring_color.setAlpha(60)
                painter.setPen(QPen(ring_color, 1))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(center, radius + 8 + pulse * 4, radius + 8 + pulse * 4)

            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 50)))
            painter.drawEllipse(center, radius, radius)
            draw_threat_glyph(painter, center, radius, threat_type)

            # Below the marker, centered: unit labels sit to the right of their
            # icons, and a scout hovering over its own detection collided with them.
            painter.setPen(QPen(QColor("#cfd8d6")))
            painter.setFont(QFont("Menlo", 8))
            label = f"{threat_type.upper()} {confidence:.2f}"
            painter.drawText(
                QRectF(center.x() - 60, center.y() + radius + 3, 120, 13), Qt.AlignHCenter, label
            )

    def _draw_formation_preview(self, painter: QPainter) -> None:
        """Show the formation before it is committed, section 6.2.

        The operator is choosing a shape, so the preview draws the shape:
        the bearing axis they are pointing down, the slots joined in order,
        which aircraft takes which slot, and each slot's altitude. Three
        loose circles would not tell them a vee from a line.
        """
        if not self._formation_preview or not self._formation_origin:
            return

        origin = self._to_screen(*self._formation_origin)
        points = [self._to_screen(tx, ty) for tx, ty, _ in self._formation_preview]

        # Bearing axis: the direction the whole formation faces.
        bearing_rad = math.radians(self._bearing_deg)
        axis_end = origin + QPointF(
            220 * math.sin(bearing_rad), -220 * math.cos(bearing_rad)
        )
        axis_pen = QPen(QColor(94, 201, 194, 70))
        axis_pen.setStyle(Qt.DashLine)
        axis_pen.setWidth(1)
        painter.setPen(axis_pen)
        painter.drawLine(origin, axis_end)

        # The formation's own outline, slot to slot in order.
        shape_pen = QPen(QColor(244, 247, 246, 130))
        shape_pen.setWidth(1)
        painter.setPen(shape_pen)
        painter.setBrush(Qt.NoBrush)
        for a, b in zip(points, points[1:]):
            painter.drawLine(a, b)

        # Tie each slot back to the rover so the standoff is legible.
        tie_pen = QPen(QColor(255, 255, 255, 45))
        tie_pen.setStyle(Qt.DotLine)
        painter.setPen(tie_pen)
        for point in points:
            painter.drawLine(origin, point)

        for i, point in enumerate(points):
            altitude = self._formation_preview[i][2]
            painter.setPen(QPen(QColor("#f4f7f6"), 1))
            painter.setBrush(QBrush(QColor(94, 201, 194, 70)))
            painter.drawEllipse(point, 9.0, 9.0)

            painter.setFont(QFont("Menlo", 8, QFont.Bold))
            painter.setPen(QPen(QColor("#0e1215")))
            painter.drawText(
                QRectF(point.x() - 9, point.y() - 8, 18, 16), Qt.AlignCenter, str(i + 1)
            )

            painter.setPen(QPen(QColor("#cfe0dd")))
            painter.setFont(QFont("Menlo", 8))
            painter.drawText(
                QRectF(point.x() - 40, point.y() + 11, 80, 13),
                Qt.AlignHCenter,
                f"TRI-{i + 1}  {altitude:.0f}m",
            )

    def _draw_units(self, painter: QPainter) -> None:
        for unit_id, pose in self._units.items():
            if self._docked.get(unit_id, False):
                continue  # rendered as a bay slot on the rover instead

            center = self._to_screen(pose["x"], pose["y"])
            color = UNIT_COLORS.get(unit_id, QColor("#cccccc"))
            if self._link_cut and unit_id in self._link_cut_units:
                color = LINK_CUT_TINT

            if unit_id == self._selected_unit:
                glow = QColor(color)
                glow.setAlpha(40)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(glow))
                painter.drawEllipse(center, 20.0, 20.0)
                ring = QPen(SELECTED_RING_COLOR)
                ring.setWidth(2)
                painter.setPen(ring)
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(center, 14.0, 14.0)

            painter.setPen(QPen(color.darker(140), 1))
            painter.setBrush(QBrush(color))
            if unit_id == "rover":
                self._draw_rover_glyph(painter, center, pose.get("heading", 0.0), color)
            else:
                self._draw_aircraft_glyph(painter, center, pose.get("heading", 0.0), color)

            painter.setPen(QPen(QColor("#e6e6e6")))
            painter.setFont(QFont("Menlo", 9))
            # To the right at mid height: the rover's bay occupies the space above it.
            painter.drawText(center + QPointF(15, 4), short_label(unit_id))

    def _draw_rover_glyph(self, painter: QPainter, center: QPointF, heading: float, color: QColor) -> None:
        half = 9.0
        rect = QRectF(center.x() - half, center.y() - half, half * 2, half * 2)
        painter.drawRoundedRect(rect, 2, 2)
        heading_rad = math.radians(heading)
        tip = center + QPointF(half * 1.6 * math.sin(heading_rad), -half * 1.6 * math.cos(heading_rad))
        pen = QPen(color)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(center, tip)

    def _draw_aircraft_glyph(self, painter: QPainter, center: QPointF, heading: float, color: QColor) -> None:
        heading_rad = math.radians(heading)
        tip = center + QPointF(9 * math.sin(heading_rad), -9 * math.cos(heading_rad))
        left = center + QPointF(
            -6 * math.cos(heading_rad) - 3 * math.sin(heading_rad),
            -6 * math.sin(heading_rad) + 3 * math.cos(heading_rad),
        )
        right = center + QPointF(
            6 * math.cos(heading_rad) - 3 * math.sin(heading_rad),
            6 * math.sin(heading_rad) + 3 * math.cos(heading_rad),
        )
        back = center + QPointF(-3 * math.sin(heading_rad), 3 * math.cos(heading_rad))
        painter.drawPolygon(QPolygonF([tip, left, back, right]))

    def _draw_rover_bay(self, painter: QPainter) -> None:
        """The single drawer bay on the rover's top deck, section 2: three
        slots, filled with a color chip per docked aircraft, empty once
        launched. Clicking a filled slot jumps straight to the launch console.
        """
        rover = self._rover_pose()
        center = self._to_screen(rover["x"], rover["y"])
        slot_w, slot_h, gap = 10.0, 7.0, 3.0
        total_w = len(BAY_SLOT_ORDER) * slot_w + (len(BAY_SLOT_ORDER) - 1) * gap
        start_x = center.x() - total_w / 2.0
        bay_y = center.y() - 24.0

        painter.setPen(QPen(QColor("#3a4149")))
        painter.setBrush(QBrush(QColor("#12161a")))
        painter.drawRoundedRect(QRectF(start_x - 3, bay_y - 3, total_w + 6, slot_h + 6), 2, 2)

        for i, tricopter_id in enumerate(BAY_SLOT_ORDER):
            slot_rect = QRectF(start_x + i * (slot_w + gap), bay_y, slot_w, slot_h)
            if self._docked.get(tricopter_id, True):
                color = UNIT_COLORS.get(tricopter_id, QColor("#cccccc"))
                painter.setPen(QPen(color.darker(130), 1))
                painter.setBrush(QBrush(color))
            else:
                painter.setPen(QPen(QColor("#3a4149"), 1, Qt.DashLine))
                painter.setBrush(Qt.NoBrush)
            painter.drawRoundedRect(slot_rect, 1, 1)

        self._bay_rect = QRectF(start_x - 3, bay_y - 3, total_w + 6, slot_h + 6)

    def _draw_bearing_compass(self, painter: QPainter) -> None:
        if self._formation_origin is None:
            return
        center = self._to_screen(*self._formation_origin)
        pen = QPen(QColor(255, 255, 255, 130))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, COMPASS_RADIUS_PX, COMPASS_RADIUS_PX)

        bearing_rad = math.radians(self._bearing_deg)
        handle = center + QPointF(
            COMPASS_RADIUS_PX * math.sin(bearing_rad), -COMPASS_RADIUS_PX * math.cos(bearing_rad)
        )
        painter.drawLine(center, handle)
        painter.setBrush(QBrush(QColor("#f4f7f6")))
        painter.drawEllipse(handle, 6.0, 6.0)

    def _draw_north_indicator(self, painter: QPainter) -> None:
        x, y = self.width() - 34, 34
        painter.setPen(QPen(QColor("#7c8891"), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(x, y), 16, 16)
        painter.setPen(QPen(QColor("#5ec9c2"), 2))
        painter.drawLine(QPointF(x, y + 10), QPointF(x, y - 12))
        painter.drawLine(QPointF(x, y - 12), QPointF(x - 4, y - 5))
        painter.drawLine(QPointF(x, y - 12), QPointF(x + 4, y - 5))
        painter.setFont(QFont("Menlo", 8, QFont.Bold))
        painter.drawText(QRectF(x - 8, y - 30, 16, 12), Qt.AlignCenter, "N")

    def _draw_scale_bar(self, painter: QPainter) -> None:
        bar_m = 10.0
        bar_px = bar_m * SCALE_PX_PER_M
        x0, y0 = 16.0, self.height() - 20.0
        pen = QPen(QColor("#9fb0ac"), 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(x0, y0), QPointF(x0 + bar_px, y0))
        painter.drawLine(QPointF(x0, y0 - 4), QPointF(x0, y0 + 4))
        painter.drawLine(QPointF(x0 + bar_px, y0 - 4), QPointF(x0 + bar_px, y0 + 4))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(QRectF(x0, y0 - 16, bar_px, 12), Qt.AlignCenter, f"{bar_m:.0f} m")

    def _draw_cursor_readout(self, painter: QPainter) -> None:
        if self._cursor_world is None:
            return
        x, y = self._cursor_world
        painter.setPen(QPen(QColor("#7c8891")))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(
            QRectF(self.width() - 120, self.height() - 20, 110, 14), Qt.AlignRight, f"{x:6.1f}, {y:6.1f} m"
        )

    def _draw_link_cut_banner(self, painter: QPainter) -> None:
        painter.setPen(QPen(LINK_CUT_TINT))
        painter.setFont(QFont("Menlo", 11, QFont.Bold))
        painter.drawText(QRectF(0, 8, self.width(), 24), Qt.AlignCenter, "LINK CUT — MAPS DIVERGING")

    # --- mouse interaction: drag the compass rose, pan the map, click units/bay ---

    def mousePressEvent(self, event) -> None:
        if self._compass_active and self._formation_origin is not None:
            center = self._to_screen(*self._formation_origin)
            if _dist(event.position(), center) <= COMPASS_RADIUS_PX + 12:
                self._dragging_compass = True
                self._update_bearing_from_mouse(event.position())
                return

        if hasattr(self, "_bay_rect") and self._bay_rect.contains(event.position()):
            self.bayLaunchRequested.emit()
            return

        for unit_id, pose in self._units.items():
            if self._docked.get(unit_id, False):
                continue
            center = self._to_screen(pose["x"], pose["y"])
            if _dist(event.position(), center) <= 12:
                self.unitClicked.emit(unit_id)
                return

        self._panning = True
        self._pan_start = event.position()
        self._pan_origin = QPointF(self._pan)

    def mouseMoveEvent(self, event) -> None:
        self._cursor_world = self._to_world(event.position())
        if self._dragging_compass:
            self._update_bearing_from_mouse(event.position())
        elif self._panning:
            delta = event.position() - self._pan_start
            self._pan = self._pan_origin + delta
            self.update()
        else:
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        self._dragging_compass = False
        self._panning = False

    def leaveEvent(self, event) -> None:
        self._cursor_world = None
        self.update()

    def _update_bearing_from_mouse(self, pos: QPointF) -> None:
        center = self._to_screen(*self._formation_origin)
        dx, dy = pos.x() - center.x(), pos.y() - center.y()
        bearing = math.degrees(math.atan2(dx, -dy)) % 360.0
        self._bearing_deg = bearing
        self.bearingDragged.emit(bearing)
        self.update()


def _dist(a: QPointF, b: QPointF) -> float:
    return math.hypot(a.x() - b.x(), a.y() - b.y())
