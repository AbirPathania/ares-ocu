"""Right panel, context sensitive, section 9.4.

Selection drives this panel, not tabs: select the rover and you get drive
telemetry, select an aircraft and you get its flight state, press Launch
and you get the launch console. The spec describes this panel by selection
("Rover / An aircraft / The formation"), and making the operator first
find the right tab was a step that answered no question.

The detection suite and the perception stack stay two visibly separate
groups, per section 9.4 ("These are different systems doing different jobs
and should never be shown as one list"), but they are shown as live state
rather than a printed inventory: an operator already knows the rover
carries GPR, and wants to know whether it just hit on something.
"""

from __future__ import annotations

import time

from PySide6.QtCore import QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from ocu.core.constants import (
    ARES_A_CEILING_M,
    ROVER_COMPUTE_LOAD_W,
    ROVER_TOP_SPEED_KMH,
    TETHER_LIMIT_M,
)
from ocu.ui.launch_console import LaunchConsole
from ocu.ui.widgets import HoldToConfirmButton

HIT_DECAY_S = 5.0

DETECTION_SUITE = [
    ("gpr", "GPR"),
    ("pulse_induction", "PULSE INDUCTION"),
    ("magnetic_gradiometer", "GRADIOMETER"),
    ("saw_voc", "SAW VOC"),
]
PERCEPTION_STACK = "LiDAR · STEREO · IR"


class Readout(QWidget):
    """One big value with a small dim label. Reads at a glance, unlike a form row."""

    def __init__(self, label: str, value: str = "—", suffix: str = "", parent=None) -> None:
        super().__init__(parent)
        self._label = label
        self._value = value
        self._suffix = suffix
        self._color = QColor("#C6D0DA")
        self.setFixedHeight(40)

    def set_value(self, value: str, color: QColor | None = None) -> None:
        self._value = value
        if color is not None:
            self._color = color
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())

        painter.setPen(QPen(QColor("#7B8794")))
        painter.setFont(QFont("Menlo", 8))
        painter.drawText(QRectF(rect.left(), rect.top(), rect.width(), 12), Qt.AlignLeft, self._label)

        painter.setPen(QPen(self._color))
        painter.setFont(QFont("DIN Condensed", 16 + 3, QFont.Bold))
        painter.drawText(
            QRectF(rect.left(), rect.top() + 13, rect.width(), 24), Qt.AlignLeft, self._value
        )

        if self._suffix:
            painter.setPen(QPen(QColor("#7B8794")))
            painter.setFont(QFont("Menlo", 9))
            painter.drawText(
                QRectF(rect.left(), rect.top() + 18, rect.width(), 18), Qt.AlignRight, self._suffix
            )
        painter.end()


class SensorStrip(QWidget):
    """The detection suite as live state: scanning, or hit within the last few seconds."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._last_hit: dict[str, float] = {}
        self._scanning = False
        self.setFixedHeight(len(DETECTION_SUITE) * 19 + 4)

        self._timer = QTimer(self)
        self._timer.setInterval(400)
        self._timer.timeout.connect(self.update)
        self._timer.start()

    def set_scanning(self, scanning: bool) -> None:
        self._scanning = scanning
        self.update()

    def register_hit(self, sensor: str) -> None:
        self._last_hit[sensor] = time.monotonic()
        self.update()

    def clear(self) -> None:
        self._last_hit.clear()
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        now = time.monotonic()

        for i, (key, label) in enumerate(DETECTION_SUITE):
            y = i * 19 + 2
            hit_age = now - self._last_hit.get(key, -1e9)
            if hit_age < HIT_DECAY_S:
                color, state = QColor("#C9952F"), "HIT"
            elif self._scanning:
                color, state = QColor("#4A9FD4"), "SCANNING"
            else:
                color, state = QColor("#4F5B68"), "IDLE"

            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(QRectF(2, y + 5, 6, 6))

            painter.setPen(QPen(QColor("#C6D0DA") if state != "IDLE" else QColor("#4F5B68")))
            painter.setFont(QFont("Menlo", 9))
            painter.drawText(QRectF(14, y, self.width() - 90, 16), Qt.AlignLeft | Qt.AlignVCenter, label)

            painter.setPen(QPen(color))
            painter.setFont(QFont("DIN Condensed", 8 + 3, QFont.Bold))
            painter.drawText(
                QRectF(self.width() - 80, y, 78, 16), Qt.AlignRight | Qt.AlignVCenter, state
            )
        painter.end()


class RoverPanel(QWidget):
    manualToggled = Signal()
    returnToBaseRequested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(4)

        title = QLabel("ROVER  ·  ARES-G")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        self.speed = Readout("SPEED", "0.0", f"/ {ROVER_TOP_SPEED_KMH:.0f} km/h")
        self.heading = Readout("HEADING", "0°")
        self.power = Readout("POWER DRAW", f"{ROVER_COMPUTE_LOAD_W:.0f}", "W")
        self.endurance = Readout("ENDURANCE", "\u2014")
        for widget in (self.speed, self.heading, self.power, self.endurance):
            layout.addWidget(widget)

        suite_label = QLabel("DETECTION SUITE")
        suite_label.setObjectName("sectionLabel")
        layout.addWidget(suite_label)
        self.sensors = SensorStrip()
        layout.addWidget(self.sensors)

        perception = QLabel(f"PERCEPTION (DRIVE)   {PERCEPTION_STACK}")
        perception.setObjectName("mutedNote")
        layout.addWidget(perception)

        self.rtb_btn = HoldToConfirmButton("⏎ RETURN TO BASE")
        self.rtb_btn.setObjectName("returnButton")
        self.rtb_btn.confirmed.connect(self.returnToBaseRequested.emit)
        layout.addWidget(self.rtb_btn)

        self.rtb_note = QLabel(
            "Recalls any aircraft still out. They travel home with the rover "
            "and dock once it is stationary."
        )
        self.rtb_note.setObjectName("mutedNote")
        self.rtb_note.setWordWrap(True)
        layout.addWidget(self.rtb_note)

        self.manual_btn = QPushButton("TAKE MANUAL CONTROL")
        self.manual_btn.clicked.connect(self.manualToggled.emit)
        layout.addWidget(self.manual_btn)

        self.manual_hint = QLabel("W / S  drive      A / D  steer")
        self.manual_hint.setObjectName("manualHint")
        self.manual_hint.setVisible(False)
        layout.addWidget(self.manual_hint)

        layout.addStretch(1)
        self._last_pos = (0.0, 0.0)

    def update_pose(self, x: float, y: float, heading: float, dt_s: float) -> None:
        dx, dy = x - self._last_pos[0], y - self._last_pos[1]
        distance_m = (dx * dx + dy * dy) ** 0.5
        speed_kmh = (distance_m / dt_s) * 3.6 if dt_s > 0 else 0.0
        self.speed.set_value(f"{min(speed_kmh, ROVER_TOP_SPEED_KMH):.1f}")
        self.heading.set_value(f"{heading:.0f}°")
        self._last_pos = (x, y)

    def update_power(self, watts: float, endurance: str) -> None:
        """Total draw, not just the compute slice.

        Endurance is only checkable against the 430 Wh baseline if the draw
        it was computed from is on screen next to it.
        """
        self.power.set_value(f"{watts:.0f}")
        self.endurance.set_value(endurance)

    def set_manual(self, is_holder: bool) -> None:
        self.manual_btn.setText("RELEASE MANUAL" if is_holder else "TAKE MANUAL CONTROL")
        self.manual_hint.setVisible(is_holder)


class AircraftPanel(QWidget):
    manualToggled = Signal()
    altitudeCommanded = Signal(float)
    recallRequested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(4)

        self.title_label = QLabel("AIRCRAFT")
        self.title_label.setObjectName("panelTitle")
        layout.addWidget(self.title_label)

        self.altitude = Readout("ALTITUDE", "0", f"/ {ARES_A_CEILING_M:.0f} m")
        self.heading = Readout("HEADING", "0°")
        self.battery = Readout("BATTERY", "100%")
        self.stage = Readout("FLIGHT MODE", "—")
        self.endurance = Readout("ENDURANCE", "—")
        for widget in (self.altitude, self.heading, self.battery, self.stage, self.endurance):
            layout.addWidget(widget)

        command_label = QLabel("COMMANDED ALTITUDE")
        command_label.setObjectName("sectionLabel")
        layout.addWidget(command_label)

        altitude_row = QHBoxLayout()
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(1.0, ARES_A_CEILING_M)
        self.altitude_spin.setValue(20.0)
        self.altitude_spin.setSuffix(" m")
        self.altitude_spin.setSingleStep(5.0)
        altitude_row.addWidget(self.altitude_spin, stretch=1)

        self.apply_altitude_btn = QPushButton("SET")
        self.apply_altitude_btn.clicked.connect(
            lambda: self.altitudeCommanded.emit(self.altitude_spin.value())
        )
        altitude_row.addWidget(self.apply_altitude_btn)
        layout.addLayout(altitude_row)

        self.tether_label = QLabel(f"TETHER  — / {TETHER_LIMIT_M:.0f} m")
        self.tether_label.setObjectName("sectionLabel")
        layout.addWidget(self.tether_label)

        self.link_label = QLabel("LINK  CONNECTED")
        self.link_label.setObjectName("mutedNote")
        layout.addWidget(self.link_label)

        self.recall_btn = QPushButton("⏎ RETURN TO ROVER")
        self.recall_btn.setObjectName("returnButton")
        self.recall_btn.clicked.connect(self.recallRequested.emit)
        layout.addWidget(self.recall_btn)

        self.manual_btn = QPushButton("TAKE MANUAL CONTROL")
        self.manual_btn.clicked.connect(self.manualToggled.emit)
        layout.addWidget(self.manual_btn)

        self.manual_hint = QLabel("W / S  fly      A / D  yaw      R / F  altitude")
        self.manual_hint.setObjectName("manualHint")
        self.manual_hint.setVisible(False)
        layout.addWidget(self.manual_hint)

        layout.addStretch(1)

    def show_unit(self, unit_id: str) -> None:
        self.title_label.setText(f"{unit_id.upper()}  ·  ARES-A")

    def set_flight_controls_enabled(self, airborne: bool) -> None:
        """Altitude and recall only mean anything for an aircraft that is
        actually flying; in the bay they are disabled rather than silently
        doing nothing."""
        self.altitude_spin.setEnabled(airborne)
        self.apply_altitude_btn.setEnabled(airborne)
        self.recall_btn.setEnabled(airborne)

    def set_commanded_altitude(self, altitude_m: float) -> None:
        self.altitude_spin.blockSignals(True)
        self.altitude_spin.setValue(altitude_m)
        self.altitude_spin.blockSignals(False)

    def update_tether(self, distance_m: float | None) -> None:
        if distance_m is None:
            self.tether_label.setText(f"TETHER  — / {TETHER_LIMIT_M:.0f} m")
            return
        self.tether_label.setText(f"TETHER  {distance_m:.0f} / {TETHER_LIMIT_M:.0f} m")

    def update_pose(self, altitude_m: float, heading: float) -> None:
        self.altitude.set_value(f"{altitude_m:.0f}")
        self.heading.set_value(f"{heading:.0f}°")

    def update_battery(self, pct: float) -> None:
        color = QColor("#C04438") if pct <= 25 else QColor("#C9952F") if pct <= 50 else QColor("#C6D0DA")
        self.battery.set_value(f"{pct:.0f}%", color)

    def update_link(self, link_state: str) -> None:
        self.link_label.setText(f"LINK  {link_state}")

    def update_stage(self, stage: str) -> None:
        self.stage.set_value(stage)

    def update_endurance(self, endurance: str, regime: str) -> None:
        self.endurance.set_value(f"{endurance}  {regime}")

    def set_manual(self, is_holder: bool) -> None:
        self.manual_btn.setText("RELEASE MANUAL" if is_holder else "TAKE MANUAL CONTROL")
        self.manual_hint.setVisible(is_holder)


class ContextPanel(QStackedWidget):
    takeManualRequested = Signal(str)
    releaseManualRequested = Signal()
    altitudeCommanded = Signal(str, float)  # unit_id, metres
    recallRequested = Signal(str)  # unit_id
    returnToBaseRequested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._selected_unit_id = "rover"
        self._manual_holder: str | None = None

        self.rover_panel = RoverPanel()
        self.aircraft_panel = AircraftPanel()
        self.launch_console = LaunchConsole()

        self.addWidget(self.rover_panel)
        self.addWidget(self.aircraft_panel)
        self.addWidget(self.launch_console)

        self.rover_panel.manualToggled.connect(self._on_manual_toggled)
        self.aircraft_panel.manualToggled.connect(self._on_manual_toggled)
        self.rover_panel.returnToBaseRequested.connect(self.returnToBaseRequested.emit)
        self.aircraft_panel.altitudeCommanded.connect(
            lambda alt: self.altitudeCommanded.emit(self._selected_unit_id, alt)
        )
        self.aircraft_panel.recallRequested.connect(
            lambda: self.recallRequested.emit(self._selected_unit_id)
        )

    def _on_manual_toggled(self) -> None:
        if self._manual_holder == self._selected_unit_id:
            self.releaseManualRequested.emit()
        else:
            self.takeManualRequested.emit(self._selected_unit_id)

    def set_selected_unit(self, unit_id: str) -> None:
        self._selected_unit_id = unit_id

    def set_manual_holder_display(self, manual_holder: str | None) -> None:
        self._manual_holder = manual_holder
        is_holder = manual_holder == self._selected_unit_id
        self.rover_panel.set_manual(is_holder)
        self.aircraft_panel.set_manual(is_holder)

    def set_manual_available(self, available: bool, mode: str) -> None:
        """Ghost and Autonomous take the sticks away, section 4.1. The button
        says why rather than simply refusing to work."""
        reason = {
            "AUTONOMOUS": "Unavailable in AUTONOMOUS: mission is pre-loaded, no real-time intervention",
            "GHOST": "Unavailable in GHOST: emissions silent, no command uplink",
        }.get(mode, "")
        for panel in (self.rover_panel, self.aircraft_panel):
            panel.manual_btn.setEnabled(available)
            panel.manual_btn.setToolTip(reason)
            if not available:
                panel.manual_hint.setVisible(False)
        self.rover_panel.rtb_btn.setEnabled(available)
        self.rover_panel.rtb_btn.setToolTip(reason)
        self.aircraft_panel.recall_btn.setEnabled(available)
        self.aircraft_panel.recall_btn.setToolTip(reason)

    def show_rover(self) -> None:
        self.setCurrentWidget(self.rover_panel)

    def show_aircraft(self, unit_id: str) -> None:
        self.aircraft_panel.show_unit(unit_id)
        self.setCurrentWidget(self.aircraft_panel)

    def show_formation(self) -> None:
        self.setCurrentWidget(self.launch_console)
