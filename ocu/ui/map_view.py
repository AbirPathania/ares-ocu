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

import numpy as np
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
    QPixmap,
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

#: How long a return off a moving body stays on the map, and the most held at once. Seconds of mission display, not data:
#: the returns are not kept anywhere else.
ENTITY_RETURN_LIFE_S = 2.2
ENTITY_RETURN_CAP = 600
ENTITY_RETURN_RGB = {"person": (255, 176, 60), "animal": (190, 140, 255)}

#: How many pixels a metre is drawn as. The default is half what it was: a
#: rover at speed crossed the old map too quickly to read, and showing twice
#: the ground in the same window is the honest way to slow the picture, since
#: the vehicle's position on screen still is its position. It can be changed
#: with the scroll wheel or a pinch.
MAP_SCALE_DEFAULT = 4.5
MAP_SCALE_MIN = 2.5
MAP_SCALE_MAX = 20.0
MAP_ZOOM_PER_NOTCH = 1.15
TRAIL_MAX_POINTS = 240

#: The band the detection array actually covers, drawn at the width it
#: really is. Showing it true to scale against the area the LiDAR maps is
#: the honest picture of clearance work: the vehicle sees a long way and
#: clears a hand's breadth.
SWEPT_WIDTH_M = PULSE_INDUCTION_SWEEP_WIDTH_CM / 100.0

#: Terrain colours as RGB rows, indexed by Terrain value, so a whole scan's
#: colours are one lookup rather than a dict get per point.
_TERRAIN_RGB = np.zeros((max(int(t) for t in Terrain) + 1, 3), dtype=np.float64)
_RETURN_RGB = np.zeros_like(_TERRAIN_RGB)
for _terrain in Terrain:
    _dim = TERRAIN_COLORS.get(_terrain.name)
    _bright = RETURN_COLORS.get(_terrain.name)
    if _dim is not None and _bright is not None:
        _TERRAIN_RGB[int(_terrain)] = (_dim.red(), _dim.green(), _dim.blue())
        _RETURN_RGB[int(_terrain)] = (_bright.red(), _bright.green(), _bright.blue())
COMPASS_RADIUS_PX = 60.0
#: Display refresh. The simulation advances ten times a second; drawing at
#: that rate makes an aircraft crossing the map jump five pixels at a time.
#: The display runs at sixty and eases each icon toward wherever the
#: simulation last put it, so motion is continuous without the simulation
#: having to be.
#: Height set aside at the bottom of the map for the scale bar, so the
#: legend above it is laid out clear of it.
SCALE_BAR_RESERVED_PX = 48.0

RENDER_TICK_MS = 16

#: Time constant of that easing. Short enough that an icon is effectively on
#: its reported position within a simulation tick, long enough to smooth the
#: step between two of them.
POSE_SMOOTHING_S = 0.055
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


#: How far the pointer may move between press and release and still be a click.
CLICK_SLOP_PX = 4.0

#: The view follows the rover once it is within this fraction of the map's size
#: from an edge, easing over about this long.
FOLLOW_MARGIN = 0.22
FOLLOW_SMOOTHING_S = 0.5
FOLLOW_QUIET_S = 4.0


class MapView(QWidget):
    cutLinkRequested = Signal()
    restoreLinkRequested = Signal()
    bearingDragged = Signal(float)
    unitClicked = Signal(str)
    bayLaunchRequested = Signal()  # operator clicked a docked slot in the rover's bay
    destinationPlaced = Signal(float, float)  # operator clicked a point on the map to go to
    destinationCleared = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 420)
        self.setMouseTracking(True)

        self._units: dict[str, dict] = {}
        # Where each icon is actually drawn, eased toward _units. Separate,
        # because the simulation's position is the one anything else reads.
        self._shown: dict[str, dict] = {}
        # Units whose next frame should jump rather than ease: a reset or a
        # dock teleports, and easing across that draws a unit flying over
        # ground it never crossed.
        self._snap: set[str] = set()
        self._docked: dict[str, bool] = {uid: True for uid in BAY_SLOT_ORDER}
        self._trails: dict[str, list[tuple[float, float]]] = {}
        # Ground the detection array has actually passed over. Kept as runs
        # so a gap -- the suite off, the link down -- stays a gap instead of
        # being bridged by a line across ground nobody swept.
        self._swept: list[list[tuple[float, float]]] = []
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
        # Finer than the half-metre terrain grid, so two returns from
        # neighbouring cells land on different pixels instead of stacking on
        # one and flattening the cloud into fill.
        self._cloud_px_per_m = 14.0
        self._cloud_half_px = int(WORLD_HALF_EXTENT_M * self._cloud_px_per_m)
        span = self._cloud_half_px * 2
        self._perception_image = QImage(span, span, QImage.Format_ARGB32_Premultiplied)
        self._perception_image.fill(QColor(0, 0, 0, 0))
        self._cloud_dirty = True
        self._mapped_m2 = 0.0
        self._cloud_points = 0
        self._fresh_returns = None
        self._fresh_layer: QPixmap | None = None
        # Two more layers that only change when their contents do, not every
        # frame: the grid (pan and size) and the accumulated cloud (a scan).
        # Rebuilding either costs a few milliseconds, and at sixty frames a
        # second that is most of the budget for something that did not move.
        self._grid_layer: QPixmap | None = None
        self._cloud_layer: QPixmap | None = None
        self._cloud_layer_pan = QPointF(0.0, 0.0)
        self._cloud_dirty = True
        self._grid_layer_pan = QPointF(0.0, 0.0)
        self._fresh_layer_pan = QPointF(0.0, 0.0)
        self._fresh_age = 0.0
        #: Returns off people and animals: (x, y, kind, intensity, age_s). Kept apart from the cloud on purpose.
        self._entity_returns: list[list] = []
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
        self._scale = MAP_SCALE_DEFAULT
        # Where the operator has sent the rover, and whether the next click on
        # the map is going to set it.
        self._destination: tuple[float, float] | None = None
        self._placing_destination = False
        self._press_pos: QPointF | None = None
        self._quiet_until_s = 0.0  # seconds the view is left alone after a drag
        #: Things the operator has to know that are not about the map: a vehicle
        #: acting on its own. Drawn over it until they expire.
        self._notices: list[dict] = []

        self._pan = QPointF(0.0, 0.0)
        self._panning = False
        self._pan_start = QPointF()
        self._pan_origin = QPointF()

        self._terrain = self._generate_terrain()
        self._radar_angle = 0.0
        self._radar_timer = QTimer(self)
        self._radar_timer.setInterval(RENDER_TICK_MS)
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

        self.destination_btn = QPushButton()
        self.destination_btn.setObjectName("destinationButton")
        self.destination_btn.clicked.connect(self._on_destination_clicked)
        toolbar.addWidget(self.destination_btn)
        self._refresh_destination_button()

        layout.addLayout(toolbar)

    # --- destination: where the operator sends the rover ---

    def _on_destination_clicked(self) -> None:
        if self._destination is not None:
            self.destinationCleared.emit()
        else:
            self.set_placing_destination(not self._placing_destination)

    def set_placing_destination(self, placing: bool) -> None:
        """Arm or disarm the map: while armed, the next click is a destination."""
        self._placing_destination = placing
        self.setCursor(Qt.CrossCursor if placing else Qt.ArrowCursor)
        self._refresh_destination_button()
        self.update()

    def set_destination(self, destination: tuple[float, float] | None) -> None:
        self._destination = destination
        self._placing_destination = False
        self.setCursor(Qt.ArrowCursor)
        self._refresh_destination_button()
        self.update()

    def _refresh_destination_button(self) -> None:
        if self._destination is not None:
            text, state = "\u2715 CLEAR DESTINATION", "set"
        elif self._placing_destination:
            text, state = "\u25ce CLICK THE MAP", "placing"
        else:
            text, state = "\u25ce GO TO", "idle"
        self.destination_btn.setText(text)
        self.destination_btn.setProperty("state", state)
        self.destination_btn.style().unpolish(self.destination_btn)
        self.destination_btn.style().polish(self.destination_btn)

    def _on_cut_clicked(self) -> None:
        if self._link_cut:
            self.restoreLinkRequested.emit()
        else:
            self.cutLinkRequested.emit()

    def _on_radar_tick(self) -> None:
        dt = RENDER_TICK_MS / 1000.0
        self._fresh_age += dt
        if self._entity_returns:
            for point in self._entity_returns:
                point[4] += dt
            self._entity_returns = [pt for pt in self._entity_returns if pt[4] < ENTITY_RETURN_LIFE_S]
        if self._reconciled_flash > 0:
            self._reconciled_flash = max(0.0, self._reconciled_flash - dt)
        self._radar_angle = (self._radar_angle + 0.64) % 360.0
        self._advance_shown_poses(dt)
        self._follow_rover(dt)
        if self._notices:
            for notice in self._notices:
                notice["left"] -= dt
            self._notices = [n for n in self._notices if n["left"] > 0.0]
        self.update()

    def post_notice(self, headline: str, detail: str = "", severity: str = "caution", seconds: float = 18.0) -> None:
        """Put a notice over the map. A line in the log is easy to miss; a vehicle
        that has just taken itself out of the mission is not."""
        self._notices = [n for n in self._notices if n["headline"] != headline]
        self._notices.append({"headline": headline, "detail": detail, "severity": severity, "left": seconds})
        self._notices = self._notices[-3:]
        self.update()

    def clear_notices(self) -> None:
        self._notices.clear()
        self.update()

    def _draw_notices(self, painter: QPainter) -> None:
        palette = {
            "threat": (QColor("#D4675A"), QColor(60, 24, 24, 225)),
            "caution": (QColor("#E8B04A"), QColor(58, 44, 16, 225)),
            "info": (QColor("#8FC9E8"), QColor(20, 40, 56, 225)),
        }
        y = 112.0
        for notice in self._notices:
            accent, box = palette.get(notice["severity"], palette["caution"])
            width, height = 460.0, 46.0
            rect = QRectF((self.width() - width) / 2, y, width, height)
            painter.setPen(QPen(accent, 1.2))
            painter.setBrush(QBrush(box))
            painter.drawRoundedRect(rect, 3, 3)
            painter.setPen(QPen(accent))
            painter.setFont(QFont("DIN Condensed", 14, QFont.Bold))
            painter.drawText(QRectF(rect.x(), rect.y() + 4, width, 18), Qt.AlignCenter, notice["headline"])
            painter.setPen(QPen(QColor("#C6D0DA")))
            painter.setFont(QFont("Menlo", 9))
            painter.drawText(QRectF(rect.x(), rect.y() + 25, width, 15), Qt.AlignCenter, notice["detail"])
            y += height + 6.0

    def _follow_rover(self, dt_s: float) -> None:
        """Keep the rover on the map.

        A rover sent across the ground drives off the edge of a fixed view,
        and the operator is then watching empty map. The view stays where it
        is while the rover is comfortably inside it and moves only when the
        rover nears the edge, and only as far as it has to, so the picture is
        not forever sliding under the pointer. Left alone for a few seconds
        after the operator has dragged the map, which is them saying where
        they want to look.
        """
        if self._panning or self._quiet_until_s > 0.0:
            self._quiet_until_s = max(0.0, self._quiet_until_s - dt_s)
            return
        if "rover" not in self._units or self.width() < 100:
            return
        point = self._to_screen(self._drawn_pose("rover")["x"], self._drawn_pose("rover")["y"])
        margin_x, margin_y = self.width() * FOLLOW_MARGIN, self.height() * FOLLOW_MARGIN
        shift_x = max(margin_x - point.x(), 0.0) - max(point.x() - (self.width() - margin_x), 0.0)
        shift_y = max(margin_y - point.y(), 0.0) - max(point.y() - (self.height() - margin_y), 0.0)
        if shift_x == 0.0 and shift_y == 0.0:
            return
        alpha = 1.0 - math.exp(-dt_s / FOLLOW_SMOOTHING_S)
        self._pan = QPointF(self._pan.x() + shift_x * alpha, self._pan.y() + shift_y * alpha)

    def _advance_shown_poses(self, dt_s: float) -> None:
        """Ease each drawn icon toward where the simulation last put it.

        The simulation is the truth and is never changed by this; what moves
        is only the position the icon is drawn at. An exponential approach
        rather than a fixed step, so the result does not depend on how often
        this happens to run.
        """
        alpha = 1.0 - math.exp(-dt_s / POSE_SMOOTHING_S)
        for unit_id, pose in self._units.items():
            shown = self._shown.get(unit_id)
            if shown is None or unit_id in self._snap:
                self._snap.discard(unit_id)
                self._shown[unit_id] = dict(pose)
                continue
            for axis in ("x", "y", "z"):
                shown[axis] += (pose[axis] - shown[axis]) * alpha
            turn = (pose["heading"] - shown["heading"] + 180.0) % 360.0 - 180.0
            shown["heading"] = (shown["heading"] + turn * alpha) % 360.0

    def _drawn_pose(self, unit_id: str) -> dict:
        """Where an icon is drawn, which trails the truth by a few hundredths
        of a second so that motion between simulation ticks is continuous."""
        return self._shown.get(unit_id) or self._units.get(
            unit_id, {"x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0}
        )

    # --- public API, called by main_window in response to engine signals ---

    def set_unit_pose(self, unit_id: str, x: float, y: float, z: float, heading: float) -> None:
        self._units[unit_id] = {"x": x, "y": y, "z": z, "heading": heading}
        if unit_id not in self._shown:
            self._shown[unit_id] = {"x": x, "y": y, "z": z, "heading": heading}
        if not self._docked.get(unit_id, False):
            trail = self._trails.setdefault(unit_id, [])
            if not trail or trail[-1] != (x, y):
                trail.append((x, y))
                if len(trail) > TRAIL_MAX_POINTS:
                    trail.pop(0)
        self.update()

    def set_unit_docked(self, unit_id: str, docked: bool) -> None:
        # Docking puts a unit on the rover from wherever it was; easing
        # across that gap draws it flying over ground it never crossed.
        self._snap.add(unit_id)
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

    @property
    def scale(self) -> float:
        """Pixels per metre."""
        return self._scale

    def set_scale(self, scale: float, anchor: QPointF | None = None) -> None:
        """Zoom, keeping the ground under `anchor` (the pointer, by default the
        middle of the view) where it is."""
        scale = max(MAP_SCALE_MIN, min(MAP_SCALE_MAX, float(scale)))
        if scale == self._scale:
            return
        if anchor is None:
            anchor = QPointF(self.width() / 2.0, self.height() / 2.0)
        wx, wy = self._to_world(anchor)
        self._scale = scale
        self._pan = QPointF(
            anchor.x() - wx * scale - self.width() / 2.0,
            anchor.y() + wy * scale - self.height() / 2.0,
        )
        # Everything cached was drawn at the old scale.
        self._cloud_dirty = True
        self._grid_layer = None
        self._fresh_layer = None
        self._quiet_until_s = max(self._quiet_until_s, FOLLOW_QUIET_S / 2.0)
        self.update()

    def wheelEvent(self, event) -> None:
        notches = event.angleDelta().y() / 120.0
        if notches:
            self.set_scale(self._scale * MAP_ZOOM_PER_NOTCH**notches, event.position())
        event.accept()

    def event(self, event) -> bool:
        from PySide6.QtCore import QEvent

        if event.type() == QEvent.NativeGesture and event.gestureType() == Qt.ZoomNativeGesture:
            self.set_scale(self._scale * (1.0 + event.value()), event.position())
            return True
        return super().event(event)

    def focus_on(self, x: float, y: float) -> None:
        """Center the view on a world coordinate, used when a threat is picked."""
        self._pan = QPointF(-x * self._scale, y * self._scale)
        self.update()

    def clear(self) -> None:
        self._notices.clear()
        self._units.clear()
        self._shown.clear()
        self._snap.clear()
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
            existing["label"] = detection.get("label", existing.get("label"))
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

    def add_returns(self, scan, mapped_m2: float, stats: dict) -> None:
        """Accumulate one scan's returns into the cached point cloud.

        Each return is a measurement, painted at its own position with a
        brightness taken from its intensity, so hard surfaces build bright
        dense clusters and grazed ground stays sparse and dim. That texture
        is the difference between a point cloud and a filled shape.

        Written straight into the image's pixels. A scan is several thousand
        points and a QPainter call each was most of a frame on its own.
        """
        width = self._perception_image.width()
        height = self._perception_image.height()

        px = (scan.xs * self._cloud_px_per_m).astype(np.int64) + self._cloud_half_px
        py = self._cloud_half_px - (scan.ys * self._cloud_px_per_m).astype(np.int64)
        inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        px, py = px[inside], py[inside]
        kinds = scan.kinds[inside]
        weight = scan.intensities[inside]

        dim = _TERRAIN_RGB[kinds]
        bright = _RETURN_RGB[kinds]
        rgb = dim + (bright - dim) * weight[:, None]
        alpha = 110.0 + 145.0 * weight

        # Premultiplied ARGB32, which is the image's own format: one packed
        # 32-bit value per pixel, written by index.
        scale = alpha / 255.0
        packed = (
            (alpha.astype(np.uint32) << 24)
            | ((rgb[:, 0] * scale).astype(np.uint32) << 16)
            | ((rgb[:, 1] * scale).astype(np.uint32) << 8)
            | (rgb[:, 2] * scale).astype(np.uint32)
        )

        pixels = self._cloud_pixels()
        pixels[py, px] = packed
        # A hard surface fills more of the beam, so it returns a tighter
        # cluster rather than one lonely sample.
        hard = (kinds == int(Terrain.ROCK)) | (kinds == int(Terrain.TREE))
        if hard.any():
            hx, hy, hp = px[hard], py[hard], packed[hard]
            for dx, dy in ((1, 0), (0, 1), (1, 1)):
                nx, ny = hx + dx, hy + dy
                ok = (nx < width) & (ny < height)
                pixels[ny[ok], nx[ok]] = hp[ok]

        self._cloud_dirty = True
        self._cloud_points += scan.point_count
        self._mapped_m2 = mapped_m2
        self._lidar_stats = stats
        self.update()

    def _cloud_pixels(self) -> np.ndarray:
        """The cloud image's pixels as a 2-D array of packed ARGB values."""
        image = self._perception_image
        view = np.frombuffer(image.bits(), dtype=np.uint32)
        return view.reshape(image.height(), image.bytesPerLine() // 4)

    def add_entity_returns(self, returns) -> None:
        """Returns off people and animals this scan. They are drawn for a couple of seconds and let go: a body that
        moves is not ground, and written into the cloud it would leave a wall along everywhere it walked. Kept for
        a short time they leave the trail a real display shows behind something that moves."""
        for x, y, kind, intensity in zip(returns.xs.tolist(), returns.ys.tolist(), returns.kinds, returns.intensities.tolist(), strict=True):
            self._entity_returns.append([x, y, kind, intensity, 0.0])
        if len(self._entity_returns) > ENTITY_RETURN_CAP:
            self._entity_returns = self._entity_returns[-ENTITY_RETURN_CAP:]
        self.update()

    def set_fresh_returns(self, scan) -> None:
        """Hold the newest scan brighter for a moment before it settles.

        Real ground control displays do not draw the beam, because a laser
        is not visible and a drawn one is a cartoon. What they show is the
        returns, with the most recent ones held brighter and decayed into
        the accumulated cloud, which is how RViz-style displays convey that
        a sensor is live. Same data, no fiction.
        """
        self._fresh_returns = scan
        self._fresh_age = 0.0
        self._build_fresh_layer()
        self.update()


    def clear_perception(self) -> None:
        self._entity_returns = []
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
        return QPointF(c.x() + x * self._scale, c.y() - y * self._scale)

    def _to_world(self, px: QPointF) -> tuple[float, float]:
        c = self._center()
        return ((px.x() - c.x()) / self._scale, -(px.y() - c.y()) / self._scale)

    def _rover_pose(self) -> dict:
        return self._drawn_pose("rover")

    # --- painting ---

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0A0E12"))

        self._draw_perception(painter)
        self._draw_fresh_returns(painter)
        self._draw_entity_returns(painter)
        self._draw_sensor_range(painter)
        self._draw_grid(painter)
        self._draw_geofence(painter)
        self._draw_tether_ring(painter)
        self._draw_swept(painter)
        self._draw_planned_path(painter)
        self._draw_destination(painter)
        self._draw_trails(painter)
        self._draw_formation_preview(painter)
        self._draw_detections(painter)
        self._draw_units(painter)
        self._draw_rover_bay(painter)
        if self._compass_active:
            self._draw_bearing_compass(painter)
        self._draw_deploy_point(painter)
        self._draw_north_indicator(painter)
        self._draw_scale_bar(painter)
        self._draw_mesh_banner(painter)
        self._draw_notices(painter)
        if self._stale:
            self._draw_stale_overlay(painter)
        self._draw_cursor_readout(painter)

        painter.end()

    def _draw_perception(self, painter: QPainter) -> None:
        """The map the rover has built, blitted in one call.

        Unknown ground is transparent, so the background shows through and
        the operator can see at a glance how much has actually been swept
        and where the sensor shadows still are.

        The cloud is held at a much finer resolution than the screen, so
        scaling it down is real work. It only changes when a scan comes in,
        so the scaled result is kept and the frame is a straight blit.
        """
        if (
            self._cloud_layer is None
            or self._cloud_dirty
            or self._cloud_layer.size() != self.size()
            or self._cloud_layer_pan != self._pan
        ):
            self._build_cloud_layer()
        if self._cloud_layer is not None:
            painter.drawPixmap(0, 0, self._cloud_layer)

    def _build_cloud_layer(self) -> None:
        if self.width() <= 0 or self.height() <= 0:
            return
        layer = QPixmap(self.size())
        layer.fill(Qt.transparent)
        into = QPainter(layer)
        into.setRenderHint(QPainter.SmoothPixmapTransform, False)
        top_left = self._to_screen(-WORLD_HALF_EXTENT_M, WORLD_HALF_EXTENT_M)
        bottom_right = self._to_screen(WORLD_HALF_EXTENT_M, -WORLD_HALF_EXTENT_M)
        into.drawImage(QRectF(top_left, bottom_right), self._perception_image)
        into.end()
        self._cloud_layer = layer
        self._cloud_layer_pan = QPointF(self._pan)
        self._cloud_dirty = False

    def _build_fresh_layer(self) -> None:
        """Rasterise the newest scan once, to be faded as a layer.

        A scan is several thousand returns and the overlay decays over about
        half a second, so drawing it point by point costs that work on every
        frame: measured, 23 ms of a 33 ms frame, which on its own held the
        whole display at thirty hertz. The returns do not move once measured,
        so they are drawn once into a layer the size of the viewport and the
        fade is applied to the layer.
        """
        self._fresh_layer = None
        scan = self._fresh_returns
        if scan is None or scan.point_count == 0 or self.width() <= 0:
            return

        layer = QPixmap(self.size())
        layer.fill(Qt.transparent)
        painter = QPainter(layer)
        painter.setRenderHint(QPainter.Antialiasing, False)

        alpha = (255.0 * (0.4 + 0.6 * scan.intensities)).astype(np.int64)
        for name in ("GROUND", "TRACK", "SCRUB", "TREE", "ROCK", "POTHOLE", "BUSH"):
            mask = scan.kinds == int(Terrain[name])
            if not mask.any():
                continue
            base = RETURN_COLORS[name]
            xs, ys, alphas = scan.xs[mask], scan.ys[mask], alpha[mask]
            # Grouped into a few alpha bands, so the pen changes a handful of
            # times per scan rather than once per point.
            bands = np.clip(alphas // 24, 0, 10)
            for band in np.unique(bands):
                in_band = bands == band
                colour = QColor(base)
                colour.setAlpha(int(band * 24 + 12))
                painter.setPen(QPen(colour))
                painter.drawPoints(
                    [self._to_screen(x, y) for x, y in zip(xs[in_band].tolist(), ys[in_band].tolist())]
                )
        painter.end()
        self._fresh_layer = layer
        self._fresh_layer_pan = QPointF(self._pan)

    def _draw_fresh_returns(self, painter: QPainter) -> None:
        """The most recent returns, bright and decaying."""
        fade = max(0.0, 1.0 - self._fresh_age * 2.2)
        if fade <= 0.0 or self._fresh_returns is None:
            return
        if self._fresh_layer is None or self._fresh_layer.size() != self.size():
            self._build_fresh_layer()
        if self._fresh_layer is None:
            return

        # Panned since the layer was drawn: shift it rather than rebuild, so
        # dragging the map stays as cheap as sitting still.
        offset = self._pan - self._fresh_layer_pan
        painter.setOpacity(fade)
        painter.drawPixmap(offset, self._fresh_layer)
        painter.setOpacity(1.0)

    def _draw_entity_returns(self, painter: QPainter) -> None:
        """Returns off people and animals: bright warm points for a person, violet for an animal, fading as they age."""
        if not self._entity_returns:
            return
        painter.setPen(Qt.NoPen)
        for x, y, kind, intensity, age in self._entity_returns:
            fade = max(0.0, 1.0 - age / ENTITY_RETURN_LIFE_S)
            r, g, b = ENTITY_RETURN_RGB.get(kind, (200, 200, 200))
            painter.setBrush(QBrush(QColor(r, g, b, int(255 * fade * (0.55 + 0.45 * intensity)))))
            centre = self._to_screen(x, y)
            radius = 1.6 + 1.6 * fade
            painter.drawEllipse(centre, radius, radius)

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
        pen.setWidthF(max(SWEPT_WIDTH_M * self._scale, 1.5))
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

    def _draw_destination(self, painter: QPainter) -> None:
        """The place the operator sent the rover, and how far off it still is.

        Drawn apart from the planned route: that is where the rover has
        decided to go and changes as it learns the ground, this is where it was
        told to end up and does not.
        """
        colour = QColor("#E8B04A")
        if self._placing_destination and self._cursor_world is not None:
            ghost = self._to_screen(*self._cursor_world)
            pen = QPen(QColor(232, 176, 74, 150))
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(ghost, 9, 9)
            painter.drawLine(ghost + QPointF(-14, 0), ghost + QPointF(14, 0))
            painter.drawLine(ghost + QPointF(0, -14), ghost + QPointF(0, 14))
        if self._destination is None:
            return
        point = self._to_screen(*self._destination)
        rover = self._rover_pose()
        remaining = math.hypot(self._destination[0] - rover["x"], self._destination[1] - rover["y"])
        inset = QRectF(self.rect()).adjusted(22, 22, -22, -22)
        if not inset.contains(point):
            self._draw_destination_edge_marker(painter, point, inset, remaining, colour)
            return
        painter.setBrush(Qt.NoBrush)
        pen = QPen(colour)
        pen.setWidthF(1.6)
        painter.setPen(pen)
        painter.drawEllipse(point, 10, 10)
        painter.drawEllipse(point, 3, 3)
        painter.drawLine(point + QPointF(-16, 0), point + QPointF(-10, 0))
        painter.drawLine(point + QPointF(10, 0), point + QPointF(16, 0))
        painter.drawLine(point + QPointF(0, -16), point + QPointF(0, -10))
        painter.drawLine(point + QPointF(0, 10), point + QPointF(0, 16))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(point + QPointF(16, -10), f"DEST  {remaining:,.0f} m")

    def _draw_destination_edge_marker(
        self, painter: QPainter, point: QPointF, inset: QRectF, remaining: float, colour: QColor
    ) -> None:
        """A destination off the edge of the view: an arrow on the edge it lies
        past, with how far away it is, so it is never simply lost."""
        centre = inset.center()
        dx, dy = point.x() - centre.x(), point.y() - centre.y()
        scale = min(
            (inset.width() / 2.0) / abs(dx) if dx else math.inf,
            (inset.height() / 2.0) / abs(dy) if dy else math.inf,
        )
        edge = QPointF(centre.x() + dx * scale, centre.y() + dy * scale)
        angle = math.atan2(dy, dx)
        tip = edge + QPointF(math.cos(angle), math.sin(angle)) * 9.0
        left = edge + QPointF(math.cos(angle + 2.5), math.sin(angle + 2.5)) * 11.0
        right = edge + QPointF(math.cos(angle - 2.5), math.sin(angle - 2.5)) * 11.0
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(colour))
        painter.drawPolygon([tip, left, right])
        painter.setPen(QPen(colour))
        painter.setFont(QFont("Menlo", 8))
        label = f"DEST  {remaining:,.0f} m"
        anchor_x = edge.x() - 78.0 if edge.x() > centre.x() else edge.x() + 14.0
        anchor_y = edge.y() + 14.0 if edge.y() < centre.y() else edge.y() - 6.0
        painter.drawText(QPointF(max(8.0, min(anchor_x, self.width() - 90.0)), anchor_y), label)

    def _draw_sensor_range(self, painter: QPainter) -> None:
        """Where the scanner stops. A range ring is standard on any sensor
        display and it is the honest version of showing the beam: it marks
        the limit without pretending the beam can be seen."""
        rover = self._rover_pose()
        centre = self._to_screen(rover["x"], rover["y"])
        radius_px = LIDAR_RANGE_M * self._scale
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

    def _draw_grid(self, painter: QPainter) -> None:
        """A 10 m grid with coordinates on the majors, so distances on this
        map can be read rather than estimated.

        Kept as a layer: it is a few hundred lines and labels that only move
        when the view does.
        """
        if self._grid_layer is None or self._grid_layer.size() != self.size():
            self._build_grid_layer()
        if self._grid_layer is not None:
            painter.drawPixmap(self._pan - self._grid_layer_pan, self._grid_layer)

    def _build_grid_layer(self) -> None:
        if self.width() <= 0 or self.height() <= 0:
            return
        layer = QPixmap(self.size())
        layer.fill(Qt.transparent)
        into = QPainter(layer)
        into.setRenderHint(QPainter.Antialiasing)
        self._draw_grid_lines(into, 10.0, GRID, labelled=False)
        self._draw_grid_lines(into, 50.0, GRID_MAJOR, labelled=True)
        into.end()
        self._grid_layer = layer
        self._grid_layer_pan = QPointF(self._pan)

    def _draw_grid_lines(self, painter: QPainter, spacing_m: float, color, labelled: bool) -> None:
        pen = QPen(color)
        pen.setWidth(1)
        painter.setPen(pen)
        step = self._scale * spacing_m
        c = self._center()
        painter.setFont(QFont("Menlo", 7))
        x = c.x() % step
        while x < self.width():
            painter.drawLine(QPointF(x, 0), QPointF(x, self.height()))
            if labelled:
                world_x = (x - c.x()) / self._scale
                painter.drawText(QRectF(x + 3, 2, 60, 11), Qt.AlignLeft, f"{world_x:+.0f}E")
            x += step
        y = c.y() % step
        while y < self.height():
            painter.drawLine(QPointF(0, y), QPointF(self.width(), y))
            if labelled:
                world_y = -(y - c.y()) / self._scale
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
                radius_px = (feature["r"] - ring * 2.5) * self._scale
                if radius_px > 4:
                    painter.drawEllipse(center, radius_px, radius_px)

    def _draw_tether_ring(self, painter: QPainter) -> None:
        """The UWB tether, section 2. Aircraft never work outside this ring, so
        it is drawn as a standing boundary rather than only when something
        approaches it."""
        rover = self._rover_pose()
        center = self._to_screen(rover["x"], rover["y"])
        radius_px = TETHER_LIMIT_M * self._scale

        pen = QPen(TETHER_COLOR)
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, radius_px, radius_px)

        # The label sits on the ring's east-south-east side, inside it. On top
        # of the ring it collides with the mesh banner whenever the rover is
        # near the top of the view, and at the bottom with the legend.
        angle = math.radians(110.0)
        anchor = QPointF(
            center.x() + radius_px * math.sin(angle), center.y() - radius_px * math.cos(angle)
        )
        painter.setPen(QPen(TETHER_LABEL_COLOR))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(
            QRectF(anchor.x() - 126, anchor.y() - 7, 120, 13),
            Qt.AlignRight,
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
            label = f"{str(d.get('label') or threat_type).upper()} {confidence:.2f}"
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
        for unit_id in self._units:
            pose = self._drawn_pose(unit_id)
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
        # The longest round length that still fits a comfortable bar.
        bar_m = next((m for m in (100.0, 50.0, 20.0, 10.0, 5.0, 2.0, 1.0) if m * self._scale <= 150.0), 1.0)
        bar_px = bar_m * self._scale
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

        for unit_id in self._units:
            if self._docked.get(unit_id, False):
                continue
            center = self._to_screen(*(self._drawn_pose(unit_id)[k] for k in ("x", "y")))
            if _dist(event.position(), center) <= 12:
                self.unitClicked.emit(unit_id)
                return

        if event.button() == Qt.RightButton and self._placing_destination:
            self.set_placing_destination(False)
            return
        self._press_pos = QPointF(event.position())
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
        if self._panning and self._press_pos is not None and _dist(self._press_pos, event.position()) >= CLICK_SLOP_PX:
            self._quiet_until_s = FOLLOW_QUIET_S
        self._panning = False
        # A click, not the end of a pan: placing a destination on a map the
        # operator is dragging around would otherwise set one at every pan.
        press, self._press_pos = self._press_pos, None
        if (
            self._placing_destination
            and press is not None
            and event.button() == Qt.LeftButton
            and _dist(press, event.position()) < CLICK_SLOP_PX
        ):
            x, y = self._to_world(event.position())
            self.destinationPlaced.emit(x, y)

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
