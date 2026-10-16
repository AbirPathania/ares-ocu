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
from PySide6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from ocu.core.constants import (
    ALERT_CONFIDENCE_THRESHOLD,
    PULSE_INDUCTION_SWEEP_WIDTH_CM,
    TETHER_LIMIT_M,
)
from ocu.core.lidar import LIDAR_RANGE_M
from ocu.core.terrain import CELL_SIZE_M, WORLD_HALF_EXTENT_M, Terrain
from ocu.ui.theme import (
    GRID,
    GRID_MAJOR,
    RETURN_COLORS,
    TERRAIN_COLORS,
    TERRAIN_LABELS,
    UNIT_TINTS,
)
from ocu.core.mesh_state import voxel_key
from ocu.ui.icons import (
    draw_friendly_air,
    draw_friendly_ground,
    draw_hostile,
    draw_threat_glyph,
    draw_unknown_contact,
)
from ocu.ui.launch_console import short_label

SCALE_PX_PER_M = 9.0
TRAIL_MAX_POINTS = 240

#: The band the detection array actually covers, drawn at the width it
#: really is. Showing it true to scale against the area the LiDAR maps is
#: the honest picture of clearance work: the vehicle sees a long way and
#: clears a hand's breadth.
SWEPT_WIDTH_M = PULSE_INDUCTION_SWEEP_WIDTH_CM / 100.0
COMPASS_RADIUS_PX = 60.0
RADAR_TICK_MS = 60
TERRAIN_SEED = 20260930
BAY_SLOT_ORDER = ["tricopter-1", "tricopter-2", "tricopter-3"]

UNIT_COLORS = dict(UNIT_TINTS)
SELECTED_RING_COLOR = QColor("#E8EDF2")
TRAIL_COLOR = QColor(255, 255, 255, 36)
DETECTION_COLOR = QColor("#C9952F")
ALERT_COLOR = QColor("#C04438")
CONFIRMED_COLOR = QColor("#C04438")
GEOFENCE_COLOR = QColor("#C9952F")
LINK_CUT_TINT = QColor("#C04438")
UNSYNCED_COLOR = QColor("#C9952F")

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
        # Ground the detection array has actually passed over. Kept as runs
        # so a gap -- the suite off, the link down -- stays a gap instead of
        # being bridged by a line across ground nobody swept.
        self._swept: list[list[tuple[float, float]]] = []
        self._lane_plan: list[tuple[float, float, str]] = []
        self._detections: list[dict] = []
        self._detection_by_key: dict[tuple, dict] = {}
        self._holdings: dict[tuple, set[str]] = {}
        self._mesh_rover = 0
        self._mesh_aircraft = 0
        self._mesh_diverged = 0
        self._reconciled_flash = 0.0  # seconds of 'just reconciled' banner left
        self._stale = False
        self._stale_caption = ""

        # The point cloud is accumulated into a cached image at 12.5 cm per
        # pixel, which is fine enough that individual returns read as points
        # rather than as filled cells. Only new returns are painted into it.
        self._cloud_px_per_m = 8.0
        self._cloud_half_px = int(WORLD_HALF_EXTENT_M * self._cloud_px_per_m)
        span = self._cloud_half_px * 2
        self._perception_image = QImage(span, span, QImage.Format_ARGB32_Premultiplied)
        self._perception_image.fill(QColor(0, 0, 0, 0))
        self._mapped_m2 = 0.0
        self._cloud_points = 0
        self._fresh_returns = []
        self._fresh_age = 0.0
        self._planned_path: list[tuple[float, float]] = []
        self._planned_goal: tuple[float, float] | None = None
        self._lidar_stats: dict[str, float] = {}
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
        self._fresh_age += RADAR_TICK_MS / 1000.0
        if self._reconciled_flash > 0:
            self._reconciled_flash = max(0.0, self._reconciled_flash - RADAR_TICK_MS / 1000.0)
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
        self._swept.clear()
        self._detections.clear()
        self._detection_by_key.clear()
        self._holdings.clear()
        self._mesh_rover = self._mesh_aircraft = self._mesh_diverged = 0
        self._reconciled_flash = 0.0
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

    def set_mesh_state(
        self,
        holdings: dict[tuple, set[str]],
        rover_voxels: int,
        aircraft_voxels: int,
        diverged: int,
    ) -> None:
        """Push the real replica state in, so the map can draw what each side
        actually holds rather than one merged view that hides the divergence."""
        self._holdings = holdings
        self._mesh_rover = rover_voxels
        self._mesh_aircraft = aircraft_voxels
        self._mesh_diverged = diverged
        self.update()

    def set_lane_plan(self, points: list[tuple[float, float, str]]) -> None:
        """The lanes the vehicle has been tasked to sweep."""
        self._lane_plan = list(points)
        self.update()

    def add_swept(self, x: float, y: float, continuing: bool) -> None:
        """Record a point of ground the detection array has covered."""
        if not continuing or not self._swept:
            self._swept.append([])
        run = self._swept[-1]
        if not run or run[-1] != (x, y):
            run.append((x, y))

    def clear_swept(self) -> None:
        self._swept.clear()

    def set_planned_path(
        self, path: list[tuple[float, float]], goal: tuple[float, float] | None
    ) -> None:
        """The route the rover has planned for itself, and where it is going.

        This is the output of the planner, not an operator-drawn line. Seeing
        it bend around something the rover has only just mapped is the whole
        point: it is the difference between a vehicle following a line and a
        vehicle deciding on one.
        """
        self._planned_path = list(path)
        self._planned_goal = goal
        self.update()

    def add_returns(self, returns, mapped_m2: float, stats: dict) -> None:
        """Accumulate one scan's returns into the cached point cloud.

        Each return is a measurement, painted at its own position with a
        brightness taken from its intensity, so hard surfaces build bright
        dense clusters and grazed ground stays sparse and dim. That texture
        is the difference between a point cloud and a filled shape.
        """
        width = self._perception_image.width()
        height = self._perception_image.height()
        painter = QPainter(self._perception_image)
        painter.setRenderHint(QPainter.Antialiasing, False)

        for ret in returns:
            px = int(ret.x * self._cloud_px_per_m) + self._cloud_half_px
            py = self._cloud_half_px - int(ret.y * self._cloud_px_per_m)
            if not (0 <= px < width and 0 <= py < height):
                continue

            name = Terrain(ret.kind).name
            dim = TERRAIN_COLORS.get(name)
            bright = RETURN_COLORS.get(name)
            if dim is None or bright is None:
                continue

            weight = ret.intensity
            color = QColor(
                int(dim.red() + (bright.red() - dim.red()) * weight),
                int(dim.green() + (bright.green() - dim.green()) * weight),
                int(dim.blue() + (bright.blue() - dim.blue()) * weight),
                int(110 + 145 * weight),
            )
            painter.setPen(QPen(color))
            painter.drawPoint(px, py)
            if ret.kind in (Terrain.ROCK, Terrain.TREE):
                # A hard surface fills more of the beam, so it returns a
                # tighter cluster rather than one lonely sample.
                painter.drawPoint(px + 1, py)
                painter.drawPoint(px, py + 1)
                painter.drawPoint(px + 1, py + 1)
        painter.end()

        self._cloud_points += len(returns)
        self._mapped_m2 = mapped_m2
        self._lidar_stats = stats
        self.update()

    def set_fresh_returns(self, returns) -> None:
        """Hold the newest scan brighter for a moment before it settles.

        Real ground control displays do not draw the beam, because a laser
        is not visible and a drawn one is a cartoon. What they show is the
        returns, with the most recent ones held brighter and decayed into
        the accumulated cloud, which is how RViz-style displays convey that
        a sensor is live. Same data, no fiction.
        """
        self._fresh_returns = returns
        self._fresh_age = 0.0
        self.update()


    def clear_perception(self) -> None:
        self._perception_image.fill(QColor(0, 0, 0, 0))
        self._mapped_m2 = 0.0
        self._cloud_points = 0
        self._lidar_stats = {}
        self.update()

    def set_telemetry_stale(self, stale: bool, caption: str) -> None:
        """Mark the picture as last-known rather than live.

        When the downlink is closed the tracks on this map are history. They
        are drawn dimmed and dashed, and captioned with how long ago, so the
        operator is never reading an old position as a current one.
        """
        self._stale = stale
        self._stale_caption = caption
        self.update()

    def flash_reconciled(self, seconds: float = 4.0) -> None:
        self._reconciled_flash = seconds
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
        painter.fillRect(self.rect(), QColor("#0A0E12"))

        self._draw_perception(painter)
        self._draw_fresh_returns(painter)
        self._draw_sensor_range(painter)
        self._draw_grid(painter)
        self._draw_geofence(painter)
        self._draw_tether_ring(painter)
        self._draw_lane_plan(painter)
        self._draw_swept(painter)
        self._draw_planned_path(painter)
        self._draw_trails(painter)
        self._draw_formation_preview(painter)
        self._draw_detections(painter)
        self._draw_units(painter)
        self._draw_rover_bay(painter)
        if self._compass_active:
            self._draw_bearing_compass(painter)
        self._draw_deploy_point(painter)
        self._draw_north_indicator(painter)
        self._draw_legend(painter)
        self._draw_scale_bar(painter)
        self._draw_mesh_banner(painter)
        if self._stale:
            self._draw_stale_overlay(painter)
        self._draw_cursor_readout(painter)

        painter.end()

    def _draw_perception(self, painter: QPainter) -> None:
        """The map the rover has built, blitted in one call.

        Unknown ground is transparent, so the background shows through and
        the operator can see at a glance how much has actually been swept
        and where the sensor shadows still are.
        """
        top_left = self._to_screen(-WORLD_HALF_EXTENT_M, WORLD_HALF_EXTENT_M)
        bottom_right = self._to_screen(WORLD_HALF_EXTENT_M, -WORLD_HALF_EXTENT_M)
        target = QRectF(top_left, bottom_right)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        painter.drawImage(target, self._perception_image)

    def _draw_fresh_returns(self, painter: QPainter) -> None:
        """The most recent returns, bright and decaying."""
        if not self._fresh_returns:
            return
        fade = max(0.0, 1.0 - self._fresh_age * 2.2)
        if fade <= 0.0:
            return

        for ret in self._fresh_returns:
            name = Terrain(ret.kind).name
            base = RETURN_COLORS.get(name)
            if base is None:
                continue
            color = QColor(base)
            color.setAlpha(int(200 * fade * (0.4 + 0.6 * ret.intensity)))
            painter.setPen(QPen(color))
            painter.drawPoint(self._to_screen(ret.x, ret.y))

    def _draw_lane_plan(self, painter: QPainter) -> None:
        """The lanes the vehicle was tasked with, so the ground it has
        covered can be read against the ground it was asked to cover."""
        if not self._lane_plan:
            return
        pen = QPen(QColor(124, 135, 148, 58))
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)

        run: list[tuple[float, float, str]] = []
        for point in list(self._lane_plan) + [None]:
            if point is not None and (not run or point[2] == run[0][2]):
                run.append(point)
                continue
            if len(run) > 1:
                painter.drawPolyline([self._to_screen(x, y) for x, y, _ in run])
                painter.setFont(QFont("Menlo", 7))
                painter.drawText(self._to_screen(run[0][0], run[0][1]) + QPointF(5, -4), run[0][2])
            run = [point] if point is not None else []

    def _draw_swept(self, painter: QPainter) -> None:
        """Ground the detection array has covered, at the width it covers.

        This is the mission's actual product. Drawn true to scale it is a
        narrow ribbon inside a very large mapped area, which is the honest
        shape of clearance work and the thing an operator is really
        tracking: not how far the vehicle has been, but how much of where
        it has been has been cleared.
        """
        if not self._swept:
            return
        pen = QPen(QColor(122, 196, 236, 52))
        pen.setWidthF(max(SWEPT_WIDTH_M * SCALE_PX_PER_M, 1.5))
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        for run in self._swept:
            if len(run) > 1:
                painter.drawPolyline([self._to_screen(x, y) for x, y in run])

    def _draw_planned_path(self, painter: QPainter) -> None:
        """The planned route, and the waypoint it terminates at."""
        if not self._planned_path:
            return

        points = [self._to_screen(x, y) for x, y in self._planned_path]

        # Drawn twice: a wide translucent casing so the line reads over the
        # point cloud, and a thin bright core so its corners stay legible.
        casing = QPen(QColor(79, 157, 105, 110))
        casing.setWidth(5)
        casing.setCapStyle(Qt.RoundCap)
        casing.setJoinStyle(Qt.RoundJoin)
        painter.setPen(casing)
        painter.setBrush(Qt.NoBrush)
        painter.drawPolyline(points)

        core = QPen(QColor(126, 219, 158, 255))
        core.setWidthF(1.6)
        painter.setPen(core)
        painter.drawPolyline(points)

        if self._planned_goal is not None:
            goal = self._to_screen(*self._planned_goal)
            pen = QPen(QColor(110, 198, 140, 200))
            pen.setWidth(1)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(goal, 5.5, 5.5)
            painter.drawLine(goal + QPointF(-9, 0), goal + QPointF(9, 0))
            painter.drawLine(goal + QPointF(0, -9), goal + QPointF(0, 9))

    def _draw_sensor_range(self, painter: QPainter) -> None:
        """Where the scanner stops. A range ring is standard on any sensor
        display and it is the honest version of showing the beam: it marks
        the limit without pretending the beam can be seen."""
        rover = self._rover_pose()
        centre = self._to_screen(rover["x"], rover["y"])
        radius_px = LIDAR_RANGE_M * SCALE_PX_PER_M
        pen = QPen(QColor(122, 196, 236, 34))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(centre, radius_px, radius_px)

    def _draw_deploy_point(self, painter: QPainter) -> None:
        """Point A. Everything on this map is referenced to it, and the rover
        holds that frame on its own compass and inertial solution."""
        centre = self._to_screen(0.0, 0.0)
        pen = QPen(QColor(124, 135, 148, 170))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(centre, 7.0, 7.0)
        painter.drawLine(centre + QPointF(-11, 0), centre + QPointF(11, 0))
        painter.drawLine(centre + QPointF(0, -11), centre + QPointF(0, 11))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(centre + QPointF(14, 4), "DEPLOY")

    def _draw_legend(self, painter: QPainter) -> None:
        """Classification key and the scanner's own numbers.

        An operator judging a map needs to know how much of it is measured
        and how recently, not only what colour means what.
        """
        x = 16.0
        y = self.height() - 164.0

        painter.setFont(QFont("DIN Condensed", 13, QFont.Bold))
        painter.setPen(QPen(QColor("#8FC9E8")))
        painter.drawText(QRectF(x, y - 16, 240, 14), Qt.AlignLeft, "LIDAR \u00b7 SLAM")

        painter.setFont(QFont("Menlo", 8))
        for i, key in enumerate(("TRACK", "GROUND", "SCRUB", "TREE", "ROCK")):
            row = y + i * 13
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(RETURN_COLORS[key]))
            painter.drawRect(QRectF(x, row, 9, 9))
            painter.setPen(QPen(QColor(150, 164, 178, 210)))
            painter.drawText(QRectF(x + 14, row - 2, 92, 13), Qt.AlignLeft, TERRAIN_LABELS[key])

        stats = self._lidar_stats
        lines = [
            f"points    {self._cloud_points:,}",
            f"mapped    {self._mapped_m2:,.0f} m\u00b2",
            f"returns/s {stats.get('returns_per_s', 0):,.0f}",
            f"obstacles {stats.get('obstacles', 0):,.0f}",
            f"range     {stats.get('range_m', 0):.0f} m @ {stats.get('scan_hz', 0):.0f} Hz",
        ]
        painter.setPen(QPen(QColor(150, 164, 178, 210)))
        painter.setFont(QFont("Menlo", 8))
        for i, line in enumerate(lines):
            painter.drawText(QRectF(x, y + 68 + i * 12, 240, 13), Qt.AlignLeft, line)

    def _draw_grid(self, painter: QPainter) -> None:
        """A 10 m grid with coordinates on the majors, so distances on this
        map can be read rather than estimated."""
        self._draw_grid_lines(painter, 10.0, GRID, labelled=False)
        self._draw_grid_lines(painter, 50.0, GRID_MAJOR, labelled=True)

    def _draw_grid_lines(self, painter: QPainter, spacing_m: float, color, labelled: bool) -> None:
        pen = QPen(color)
        pen.setWidth(1)
        painter.setPen(pen)
        step = SCALE_PX_PER_M * spacing_m
        c = self._center()
        painter.setFont(QFont("Menlo", 7))
        x = c.x() % step
        while x < self.width():
            painter.drawLine(QPointF(x, 0), QPointF(x, self.height()))
            if labelled:
                world_x = (x - c.x()) / SCALE_PX_PER_M
                painter.drawText(QRectF(x + 3, 2, 60, 11), Qt.AlignLeft, f"{world_x:+.0f}E")
            x += step
        y = c.y() % step
        while y < self.height():
            painter.drawLine(QPointF(0, y), QPointF(self.width(), y))
            if labelled:
                world_y = -(y - c.y()) / SCALE_PX_PER_M
                painter.drawText(QRectF(3, y + 2, 60, 11), Qt.AlignLeft, f"{world_y:+.0f}N")
            y += step

    def _draw_terrain(self, painter: QPainter) -> None:
        """Contour rings only.

        The soft filled blobs this replaces were decoration: they carried no
        elevation, no trafficability, nothing an operator could act on, and
        coloured ground competes with the status palette. Contours at least
        read as terrain without pretending to be survey data.
        """
        painter.setBrush(Qt.NoBrush)
        pen = QPen(TERRAIN_CONTOUR)
        pen.setWidth(1)
        painter.setPen(pen)
        for feature in self._terrain:
            if feature["kind"] != "contour":
                continue
            center = self._to_screen(feature["x"], feature["y"])
            for ring in range(3):
                radius_px = (feature["r"] - ring * 2.5) * SCALE_PX_PER_M
                if radius_px > 4:
                    painter.drawEllipse(center, radius_px, radius_px)

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
        for index, d in enumerate(self._detections):
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
            if d.get("confirmed"):
                draw_hostile(painter, center, radius + 2)
            else:
                draw_unknown_contact(painter, center, radius)
            painter.setBrush(Qt.NoBrush)
            draw_threat_glyph(painter, center, radius * 0.8, threat_type)

            # An observation only one replica holds has not crossed the link.
            # Drawing it the same as a shared one would hide exactly the thing
            # the cut-link demonstration exists to show.
            holders = self._holdings.get(voxel_key(d["x"], d["y"], 0.0))
            if holders is not None and len(holders) == 1:
                side = next(iter(holders))
                badge = "R" if side == "rover" else "A"
                painter.setPen(QPen(UNSYNCED_COLOR, 1, Qt.DashLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(center, radius + 6, radius + 6)

                corner = center + QPointF(radius + 4, -radius - 4)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(UNSYNCED_COLOR))
                painter.drawEllipse(corner, 7.0, 7.0)
                painter.setPen(QPen(QColor("#0A0E12")))
                painter.setFont(QFont("DIN Condensed", 8 + 3, QFont.Bold))
                painter.drawText(QRectF(corner.x() - 7, corner.y() - 7, 14, 14), Qt.AlignCenter, badge)

            # Below the marker, centered: unit labels sit to the right of their
            # icons, and a scout hovering over its own detection collided with them.
            painter.setPen(QPen(QColor("#C6D0DA")))
            painter.setFont(QFont("Menlo", 8))
            label = f"{threat_type.upper()} {confidence:.2f}"
            # Contacts on a swept lane cluster along it, so labels are
            # stepped to stop them stacking on one another.
            offset = radius + 3 + (index % 3) * 12
            painter.drawText(
                QRectF(center.x() - 60, center.y() + offset, 120, 13), Qt.AlignHCenter, label
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
            painter.setPen(QPen(QColor("#E8EDF2"), 1))
            painter.setBrush(QBrush(QColor(94, 201, 194, 70)))
            painter.drawEllipse(point, 9.0, 9.0)

            painter.setFont(QFont("DIN Condensed", 8 + 3, QFont.Bold))
            painter.setPen(QPen(QColor("#0A0E12")))
            painter.drawText(
                QRectF(point.x() - 9, point.y() - 8, 18, 16), Qt.AlignCenter, str(i + 1)
            )

            painter.setPen(QPen(QColor("#C6D0DA")))
            painter.setFont(QFont("Menlo", 8))
            painter.drawText(
                QRectF(point.x() - 40, point.y() + 11, 80, 13),
                Qt.AlignHCenter,
                f"TRI-{i + 1}  {altitude:.0f}m",
            )

    def _draw_units(self, painter: QPainter) -> None:
        if self._stale:
            painter.setOpacity(0.45)
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

            painter.setPen(QPen(QColor("#C6D0DA")))
            painter.setFont(QFont("Menlo", 9))
            # To the right at mid height: the rover's bay occupies the space above it.
            painter.drawText(center + QPointF(15, 4), short_label(unit_id))
        painter.setOpacity(1.0)

    def _draw_rover_glyph(self, painter: QPainter, center: QPointF, heading: float, color: QColor) -> None:
        """Friendly ground track, MIL-STD-2525: a rectangle, with a leader
        line showing heading."""
        draw_friendly_ground(painter, center, 10.0)
        heading_rad = math.radians(heading)
        pen = QPen(color)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(
            center,
            center + QPointF(17 * math.sin(heading_rad), -17 * math.cos(heading_rad)),
        )

    def _draw_aircraft_glyph(self, painter: QPainter, center: QPointF, heading: float, color: QColor) -> None:
        """Friendly air track, MIL-STD-2525: an arch, with a leader line."""
        draw_friendly_air(painter, center, 9.0)
        heading_rad = math.radians(heading)
        pen = QPen(color)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(
            center,
            center + QPointF(15 * math.sin(heading_rad), -15 * math.cos(heading_rad)),
        )

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

        painter.setPen(QPen(QColor("#3A4756")))
        painter.setBrush(QBrush(QColor("#121820")))
        painter.drawRoundedRect(QRectF(start_x - 3, bay_y - 3, total_w + 6, slot_h + 6), 2, 2)

        for i, tricopter_id in enumerate(BAY_SLOT_ORDER):
            slot_rect = QRectF(start_x + i * (slot_w + gap), bay_y, slot_w, slot_h)
            if self._docked.get(tricopter_id, True):
                color = UNIT_COLORS.get(tricopter_id, QColor("#cccccc"))
                painter.setPen(QPen(color.darker(130), 1))
                painter.setBrush(QBrush(color))
            else:
                painter.setPen(QPen(QColor("#3A4756"), 1, Qt.DashLine))
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
        painter.setBrush(QBrush(QColor("#E8EDF2")))
        painter.drawEllipse(handle, 6.0, 6.0)

    def _draw_north_indicator(self, painter: QPainter) -> None:
        x, y = self.width() - 34, 34
        painter.setPen(QPen(QColor("#7B8794"), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(x, y), 16, 16)
        painter.setPen(QPen(QColor("#4A9FD4"), 2))
        painter.drawLine(QPointF(x, y + 10), QPointF(x, y - 12))
        painter.drawLine(QPointF(x, y - 12), QPointF(x - 4, y - 5))
        painter.drawLine(QPointF(x, y - 12), QPointF(x + 4, y - 5))
        painter.setFont(QFont("DIN Condensed", 8 + 3, QFont.Bold))
        painter.drawText(QRectF(x - 8, y - 30, 16, 12), Qt.AlignCenter, "N")

    def _draw_scale_bar(self, painter: QPainter) -> None:
        bar_m = 10.0
        bar_px = bar_m * SCALE_PX_PER_M
        x0, y0 = 16.0, self.height() - 20.0
        pen = QPen(QColor("#7B8794"), 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(x0, y0), QPointF(x0 + bar_px, y0))
        painter.drawLine(QPointF(x0, y0 - 4), QPointF(x0, y0 + 4))
        painter.drawLine(QPointF(x0 + bar_px, y0 - 4), QPointF(x0 + bar_px, y0 + 4))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(QRectF(x0, y0 - 16, bar_px, 12), Qt.AlignCenter, f"{bar_m:.0f} m")

    def _draw_stale_overlay(self, painter: QPainter) -> None:
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(10, 14, 18, 120)))
        painter.drawRect(self.rect())

        width, height = 430.0, 44.0
        rect = QRectF((self.width() - width) / 2, (self.height() - height) / 2 - 40, width, height)
        painter.setPen(QPen(QColor("#C9952F"), 1))
        painter.setBrush(QBrush(QColor(31, 28, 20, 225)))
        painter.drawRect(rect)
        painter.setPen(QPen(QColor("#C9952F")))
        painter.setFont(QFont("DIN Condensed", 15, QFont.Bold))
        painter.drawText(QRectF(rect.x(), rect.y() + 4, width, 18), Qt.AlignCenter, self._stale_caption)
        painter.setPen(QPen(QColor("#7B8794")))
        painter.setFont(QFont("Menlo", 9))
        painter.drawText(
            QRectF(rect.x(), rect.y() + 24, width, 15),
            Qt.AlignCenter,
            "Tracks shown are last known. Fleet is still executing.",
        )

    def _draw_cursor_readout(self, painter: QPainter) -> None:
        if self._cursor_world is None:
            return
        x, y = self._cursor_world
        painter.setPen(QPen(QColor("#7B8794")))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(
            QRectF(self.width() - 120, self.height() - 20, 110, 14), Qt.AlignRight, f"{x:6.1f}, {y:6.1f} m"
        )

    def _draw_mesh_banner(self, painter: QPainter) -> None:
        """The shared map's state, in numbers, where the operator is looking.

        A cut link that produces only a log line asks the viewer to take the
        divergence on trust. The counts make it checkable: two replicas, how
        many voxels each holds, and how many must cross for them to agree.
        """
        if self._link_cut:
            headline = "LINK CUT — MAPS DIVERGING"
            detail = (
                f"ROVER MAP {self._mesh_rover}   AIRCRAFT MAP {self._mesh_aircraft}"
                f"   UNRECONCILED {self._mesh_diverged}"
            )
            accent, box = LINK_CUT_TINT, QColor(60, 24, 24, 210)
        elif self._reconciled_flash > 0:
            headline = "LINK RESTORED — MAPS RECONCILED"
            detail = (
                f"ROVER MAP {self._mesh_rover}   AIRCRAFT MAP {self._mesh_aircraft}"
                f"   DIVERGENCE {self._mesh_diverged}"
            )
            accent, box = QColor("#4A9FD4"), QColor(20, 54, 50, 210)
        else:
            painter.setPen(QPen(QColor(124, 136, 145, 170)))
            painter.setFont(QFont("Menlo", 8))
            painter.drawText(
                QRectF(0, 52, self.width(), 14),
                Qt.AlignCenter,
                f"MESH SYNCED · {self._mesh_rover} voxels on both replicas",
            )
            return

        # Below the toolbar row, which occupies the top-left corner.
        width, height = 430.0, 44.0
        rect = QRectF((self.width() - width) / 2, 46, width, height)
        painter.setPen(QPen(accent, 1))
        painter.setBrush(QBrush(box))
        painter.drawRoundedRect(rect, 3, 3)

        painter.setPen(QPen(accent))
        painter.setFont(QFont("DIN Condensed", 11 + 3, QFont.Bold))
        painter.drawText(QRectF(rect.x(), rect.y() + 5, width, 16), Qt.AlignCenter, headline)
        painter.setPen(QPen(QColor("#C6D0DA")))
        painter.setFont(QFont("Menlo", 9))
        painter.drawText(QRectF(rect.x(), rect.y() + 23, width, 15), Qt.AlignCenter, detail)

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
