"""Top bar plus the four-region layout, section 9. This file owns fleet
state and wires every panel and the playback engine together. No equation
lives here; it only reads core/ and sim/ and reflects their state.

Layout priority is the map, because the map is the product. Panels earn
their width by carrying state that changes: a roster that shows battery
and link, a threat list the operator acts on, a context panel that follows
the selection. Nothing here prints the platform's spec sheet at an
operator who already knows what the rover carries.

One unit's position has exactly one owner at a time, tracked in
_position_owner. The scripted playback owns a unit by default; the guided
launch console takes ownership of the aircraft it launches and keeps it.
Without that rule both write the same tricopter's pose ten times a second and
the map draws the fight as a zigzag fan.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ocu.core.constants import ALERT_CONFIDENCE_THRESHOLD
from ocu.core.control_authority import ControlAuthority
from ocu.core.flight_director import AircraftMode, FlightDirector
from ocu.core.geofence import Geofence
from ocu.core.mesh_state import MeshLink, VoxelObservation, voxel_key
from ocu.core.unit import Unit
from ocu.sim.playback_engine import PlaybackEngine
from ocu.ui.alert_banner import AlertBanner
from ocu.ui.camera_feed import SimulatedCameraFeed
from ocu.ui.context_panel import ContextPanel
from ocu.ui.event_log import EventLog
from ocu.ui.fleet_roster import FleetRoster
from ocu.ui.launch_console import short_label
from ocu.ui.map_view import MapView
from ocu.ui.phase_rail import PhaseRail
from ocu.ui.threat_panel import ThreatPanel

TRICOPTER_IDS = ["tricopter-1", "tricopter-2", "tricopter-3"]
ALL_UNIT_IDS = ["rover"] + TRICOPTER_IDS
LINK_SEVERITY = {state: i for i, state in enumerate(("CONNECTED", "DEGRADED", "LOST"))}
DIRECTOR_TICK_MS = 100

SCANNING_PHASES = {"drive_and_scan", "reactive_launch", "aerial_recon", "fusion", "alert"}


def _default_units() -> dict[str, Unit]:
    units = {"rover": Unit(id="rover", kind="rover", current_task="idle")}
    for tricopter_id in TRICOPTER_IDS:
        units[tricopter_id] = Unit(id=tricopter_id, kind="tricopter", current_task="docked")
    return units


def _chip(label: str, value: str, color: str) -> str:
    return f'<span style="color:#6c7a80">{label}</span> <span style="color:{color}">{value}</span>'


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ARES Operator Control Unit — Patroclus Systems")
        self.resize(1500, 900)

        self.units = _default_units()
        self.control_authority = ControlAuthority(self.units)
        self.mesh_link = MeshLink()
        self.geofence = Geofence()
        self.playback = PlaybackEngine()
        self.selected_unit_id = "rover"
        self._last_clock_s = 0.0
        self._docked_state: dict[str, bool] = {uid: True for uid in TRICOPTER_IDS}
        self._position_owner: dict[str, str] = {uid: "scripted" for uid in ALL_UNIT_IDS}
        self._keys_down: set[int] = set()
        self.setFocusPolicy(Qt.StrongFocus)

        # The flight director owns every unit it has been handed, on its own
        # wall clock, independently of whether the scripted mission is playing.
        self.flight_director = FlightDirector()
        self._director_timer = QTimer(self)
        self._director_timer.setInterval(DIRECTOR_TICK_MS)
        self._director_timer.timeout.connect(self._on_director_tick)
        self._director_timer.start()

        self._build_ui()
        self._wire_signals()
        self._refresh_all_rows()
        self._select_unit("rover")
        self.playback.apply_initial_state()

    # --- layout ---

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_top_bar())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_left_column())

        map_container = QWidget()
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.setSpacing(0)
        self.alert_banner = AlertBanner()
        map_layout.addWidget(self.alert_banner)
        self.map_view = MapView()
        map_layout.addWidget(self.map_view, stretch=1)
        splitter.addWidget(map_container)

        splitter.addWidget(self._build_right_column())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([288, 900, 312])
        root.addWidget(splitter, stretch=1)

        self.event_log = EventLog()
        self.event_log.setFixedHeight(118)
        root.addWidget(self.event_log)

    def _build_left_column(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(6, 4, 4, 4)
        layout.setSpacing(3)

        feeds_label = QLabel("CAMERA FEEDS  ·  SIMULATED, NO HARDWARE")
        feeds_label.setObjectName("sectionLabel")
        layout.addWidget(feeds_label)

        feeds_column = QVBoxLayout()
        feeds_column.setSpacing(3)
        self.camera_feeds: dict[str, SimulatedCameraFeed] = {}
        for unit_id in ALL_UNIT_IDS:
            feed = SimulatedCameraFeed(unit_id, short_label(unit_id))
            self.camera_feeds[unit_id] = feed
            feeds_column.addWidget(feed)
        self.camera_feeds["rover"].set_active(True)
        layout.addLayout(feeds_column)

        roster_label = QLabel("FLEET")
        roster_label.setObjectName("sectionLabel")
        layout.addWidget(roster_label)

        self.fleet_roster = FleetRoster()
        layout.addWidget(self.fleet_roster)
        layout.addStretch(1)
        return container

    def _build_right_column(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(4, 4, 6, 4)
        layout.setSpacing(4)

        self.threat_panel = ThreatPanel()
        layout.addWidget(self.threat_panel, stretch=1)

        self.context_panel = ContextPanel()
        self.context_panel.setMinimumHeight(330)
        layout.addWidget(self.context_panel)
        return container

    def _build_top_bar(self) -> QWidget:
        bar = QWidget()
        bar.setObjectName("topBar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 2, 10, 2)
        layout.setSpacing(12)

        self.recording_label = QLabel("● REC")
        self.recording_label.setObjectName("recordingIndicator")
        layout.addWidget(self.recording_label)

        self.clock_label = QLabel("00:00")
        self.clock_label.setObjectName("missionClock")
        layout.addWidget(self.clock_label)

        self.phase_rail = PhaseRail()
        layout.addWidget(self.phase_rail)

        layout.addStretch(1)

        self.mode_chip = QLabel()
        self.link_chip = QLabel()
        self.battery_chip = QLabel()
        self.manual_chip = QLabel()
        for chip in (self.mode_chip, self.link_chip, self.battery_chip, self.manual_chip):
            chip.setObjectName("statusChip")
            layout.addWidget(chip)
        self.mode_chip.setText(_chip("MODE", "STANDARD", "#cfe0dd"))

        self.play_btn = QPushButton("▶ PLAY")
        self.play_btn.setObjectName("playButton")
        self.play_btn.clicked.connect(self._on_play_clicked)
        layout.addWidget(self.play_btn)

        self.reset_btn = QPushButton("RESET")
        self.reset_btn.clicked.connect(self._on_reset_clicked)
        layout.addWidget(self.reset_btn)

        return bar

    # --- signal wiring ---

    def _wire_signals(self) -> None:
        self.fleet_roster.unitSelected.connect(self._select_unit)
        self.map_view.unitClicked.connect(self._select_unit)
        self.map_view.cutLinkRequested.connect(self._on_cut_link)
        self.map_view.restoreLinkRequested.connect(self._on_restore_link)
        self.map_view.bearingDragged.connect(self.context_panel.launch_console.bearing_from_map)
        self.map_view.bayLaunchRequested.connect(self._on_bay_launch_requested)

        self.threat_panel.threatSelected.connect(self._on_threat_selected)

        self.context_panel.takeManualRequested.connect(self._on_take_manual)
        self.context_panel.releaseManualRequested.connect(self._on_release_manual)
        self.context_panel.altitudeCommanded.connect(self._on_altitude_commanded)
        self.context_panel.recallRequested.connect(self._on_recall_requested)
        self.context_panel.returnToBaseRequested.connect(self._on_return_to_base)

        console = self.context_panel.launch_console
        console.previewChanged.connect(self.map_view.set_formation_preview)
        console.previewCleared.connect(lambda: self.map_view.set_formation_preview(None))
        console.bearingChanged.connect(self.map_view.set_bearing)
        console.aircraftPositionChanged.connect(self._on_guided_aircraft_position)
        console.aircraftUndocked.connect(self._on_aircraft_undocked)
        console.launchComplete.connect(self._on_launch_complete)
        console.stageChanged.connect(self._on_guided_stage_changed)
        console.eventLogged.connect(lambda msg: self.event_log.log(self.playback.clock_s, msg))

        self.alert_banner.confirmed.connect(self._on_alert_confirmed)
        self.alert_banner.rejected.connect(self._on_alert_rejected)
        self.alert_banner.timedOut.connect(self._on_alert_timed_out)

        self.playback.unitPoseChanged.connect(self._on_unit_pose_changed)
        self.playback.unitBatteryChanged.connect(self._on_unit_battery_changed)
        self.playback.phaseChanged.connect(self._on_phase_changed)
        self.playback.detectionOccurred.connect(self._on_detection)
        self.playback.linkEvent.connect(self._on_scripted_link_event)
        self.playback.reactiveLaunchStageChanged.connect(self._on_reactive_stage_changed)
        self.playback.unitDockedChanged.connect(self._on_unit_docked_changed)
        self.playback.clockTick.connect(self._on_clock_tick)
        self.playback.missionFinished.connect(self._on_mission_finished)

    # --- selection ---

    def _select_unit(self, unit_id: str) -> None:
        if unit_id not in self.units:
            return
        self.selected_unit_id = unit_id
        self.fleet_roster.select_unit(unit_id)
        self.map_view.set_selected(unit_id)
        self.context_panel.set_selected_unit(unit_id)

        unit = self.units[unit_id]
        if unit.kind == "rover":
            self.context_panel.show_rover()
        else:
            self.context_panel.show_aircraft(unit_id)
            self.context_panel.aircraft_panel.update_pose(unit.pose.z, unit.pose.heading)
            self.context_panel.aircraft_panel.update_battery(unit.battery_pct)
            self.context_panel.aircraft_panel.update_link(unit.link_state)
            self._refresh_aircraft_flight_state(unit_id)
        self.context_panel.set_manual_holder_display(self.control_authority.manual_holder)

    def _on_threat_selected(self, threat: dict) -> None:
        self.map_view.focus_on(threat["x"], threat["y"])

    # --- launch and dock state ---

    def _on_bay_launch_requested(self) -> None:
        rover = self.units["rover"]
        origin = (rover.pose.x, rover.pose.y, rover.pose.z)
        available = [uid for uid in TRICOPTER_IDS if self._docked_state.get(uid, True)]
        self.context_panel.launch_console.set_available_tricopters(available, origin)
        self.context_panel.show_formation()
        self.map_view.set_compass_active(True, (origin[0], origin[1]))

    def _on_aircraft_undocked(self, tricopter_ids: list[str]) -> None:
        for tricopter_id in tricopter_ids:
            self._position_owner[tricopter_id] = "operator"
            self._set_docked(tricopter_id, False)

    def _on_unit_docked_changed(self, unit_id: str, docked: bool) -> None:
        self._set_docked(unit_id, docked)

    def _set_docked(self, unit_id: str, docked: bool) -> None:
        if unit_id in self._docked_state:
            self._docked_state[unit_id] = docked
        self.map_view.set_unit_docked(unit_id, docked)
        self.fleet_roster.set_docked(unit_id, docked)
        feed = self.camera_feeds.get(unit_id)
        if feed is not None:
            feed.set_active(not docked)
        self._refresh_all_rows()

    # --- MANUAL handoff, section 4.2 ---

    def _on_take_manual(self, unit_id: str) -> None:
        previous = self.control_authority.manual_holder
        if previous is not None and previous != unit_id:
            self.flight_director.release_manual(
                previous, self.units["rover"].pose.x, self.units["rover"].pose.y
            )
        self.control_authority.take_manual(unit_id)
        rover = self.units["rover"]
        if unit_id != "rover":
            self._ensure_director_owns(unit_id)
        if self.flight_director.take_manual(unit_id, rover.pose.x, rover.pose.y, rover.pose.heading):
            self._position_owner[unit_id] = "operator"
            self.event_log.log(
                self.playback.clock_s,
                f"{short_label(unit_id)} on manual sticks: W/S drive, A/D turn"
                + (", R/F altitude" if unit_id != "rover" else ""),
            )
        self.setFocus()
        self.event_log.log(self.playback.clock_s, f"MANUAL control taken: {unit_id}")
        self._on_manual_authority_changed()

    def _on_release_manual(self) -> None:
        holder = self.control_authority.manual_holder
        if holder is not None:
            rover = self.units["rover"]
            self.flight_director.release_manual(holder, rover.pose.x, rover.pose.y)
        self.control_authority.release_manual()
        if holder:
            self.event_log.log(self.playback.clock_s, f"MANUAL control released: {holder}")
        self._on_manual_authority_changed()

    def _on_manual_authority_changed(self) -> None:
        holder = self.control_authority.manual_holder
        self.context_panel.set_manual_holder_display(holder)
        self._refresh_all_rows()

    # --- guided formation launch, section 6.2 ---

    # --- flight director: everything after launch ---

    def _apply_manual_input(self) -> None:
        """Translate held keys into stick demand for whoever holds MANUAL.

        Nothing here moves a unit directly: the demand goes into the same
        acceleration-limited flight model as autonomous motion, so driving by
        hand feels like driving rather than dragging an icon.
        """
        holder = self.control_authority.manual_holder
        if holder is None:
            return
        forward = (Qt.Key_W in self._keys_down) - (Qt.Key_S in self._keys_down)
        turn = (Qt.Key_D in self._keys_down) - (Qt.Key_A in self._keys_down)
        vertical = (Qt.Key_R in self._keys_down) - (Qt.Key_F in self._keys_down)
        self.flight_director.set_manual_input(holder, forward, turn, vertical)

    def _on_director_tick(self) -> None:
        dt = DIRECTOR_TICK_MS / 1000.0
        self._apply_manual_input()
        rover = self.units["rover"]
        poses, rover_pose, events = self.flight_director.tick(
            dt, rover.pose.x, rover.pose.y, rover.pose.heading
        )

        if rover_pose is not None:
            rover.pose.x, rover.pose.y, rover.pose.heading = rover_pose
            self.map_view.set_unit_pose("rover", rover.pose.x, rover.pose.y, 0.0, rover.pose.heading)

        for unit_id, (x, y, z, heading) in poses.items():
            unit = self.units.get(unit_id)
            if unit is None:
                continue
            unit.pose.x, unit.pose.y, unit.pose.z, unit.pose.heading = x, y, z, heading
            self.map_view.set_unit_pose(unit_id, x, y, z, heading)
            if self.selected_unit_id == unit_id:
                self.context_panel.aircraft_panel.update_pose(z, heading)

        for unit_id, event in events:
            self._handle_director_event(unit_id, event)

        if self.selected_unit_id in TRICOPTER_IDS:
            self._refresh_aircraft_flight_state(self.selected_unit_id)

    def _handle_director_event(self, unit_id: str, event: str) -> None:
        if event == "DOCKED":
            self._position_owner[unit_id] = "scripted"
            self.flight_director.release(unit_id)
            self._set_docked(unit_id, True)
            self.event_log.log(self.playback.clock_s, f"{short_label(unit_id)} docked and charging")
        elif event == "HOLDING":
            self.event_log.log(
                self.playback.clock_s, f"{short_label(unit_id)} holding above rover, awaiting dock slot"
            )
        elif event == "DOCKING":
            self.event_log.log(self.playback.clock_s, f"{short_label(unit_id)} cleared to dock")
        elif event == "AT_BASE":
            self.event_log.log(self.playback.clock_s, "Rover at base, stationary. Docking may proceed.")
        elif event.startswith("RECOVERY:"):
            self.event_log.log(
                self.playback.clock_s, f"{short_label(unit_id)} recovery stage {event.split(':')[1]}"
            )

    def _refresh_aircraft_flight_state(self, unit_id: str) -> None:
        panel = self.context_panel.aircraft_panel
        airborne = not self._docked_state.get(unit_id, True)
        panel.set_flight_controls_enabled(airborne)
        panel.update_stage(self.flight_director.mode_of(unit_id) if airborne else "IN BAY")
        rover = self.units["rover"]
        panel.update_tether(self.flight_director.tether_distance(unit_id, rover.pose.x, rover.pose.y))
        commanded = self.flight_director.commanded_altitude(unit_id)
        if commanded is not None:
            panel.set_commanded_altitude(commanded)

    def _on_launch_complete(self, tricopter_ids: list[str]) -> None:
        """Hand the formation from the launch sequencer to the flight director,
        which holds each slot relative to the rover from here on."""
        rover = self.units["rover"]
        for unit_id in tricopter_ids:
            unit = self.units.get(unit_id)
            if unit is None:
                continue
            self.flight_director.adopt(
                unit_id, unit.pose.x, unit.pose.y, unit.pose.z, unit.pose.heading,
                rover.pose.x, rover.pose.y,
            )
        self.event_log.log(
            self.playback.clock_s,
            f"Formation on station: {', '.join(short_label(i) for i in tricopter_ids)}",
        )

    def _ensure_director_owns(self, unit_id: str) -> bool:
        """Operator commands must work on any airborne aircraft, including ones
        the scripted mission launched. Issuing one transfers that aircraft from
        the playback to the flight director, which then owns it for good."""
        if unit_id in self.flight_director.aircraft:
            return True
        if self._docked_state.get(unit_id, True):
            return False
        unit = self.units.get(unit_id)
        if unit is None:
            return False
        rover = self.units["rover"]
        self.flight_director.adopt(
            unit_id, unit.pose.x, unit.pose.y, unit.pose.z, unit.pose.heading,
            rover.pose.x, rover.pose.y,
        )
        self._position_owner[unit_id] = "operator"
        return True

    def _on_altitude_commanded(self, unit_id: str, altitude_m: float) -> None:
        if not self._ensure_director_owns(unit_id):
            return
        self.flight_director.set_commanded_altitude(unit_id, altitude_m)
        self.event_log.log(
            self.playback.clock_s, f"{short_label(unit_id)} commanded to {altitude_m:.0f} m"
        )

    def _on_recall_requested(self, unit_id: str) -> None:
        if not self._ensure_director_owns(unit_id):
            return
        recalled = self.flight_director.recall([unit_id])
        if recalled:
            self.event_log.log(self.playback.clock_s, f"{short_label(unit_id)} returning to rover")

    def _on_return_to_base(self) -> None:
        for unit_id in TRICOPTER_IDS:
            self._ensure_director_owns(unit_id)
        self._position_owner["rover"] = "operator"
        rover = self.units["rover"]
        recalled = self.flight_director.return_rover_to_base(
            0.0, 0.0, rover.pose.x, rover.pose.y, rover.pose.heading
        )
        if recalled:
            self.event_log.log(
                self.playback.clock_s,
                "RETURN TO BASE: recalling "
                + ", ".join(short_label(i) for i in recalled)
                + " to travel home with the rover",
            )
        else:
            self.event_log.log(self.playback.clock_s, "RETURN TO BASE: rover returning to start point")

    def _on_guided_aircraft_position(
        self, unit_id: str, x: float, y: float, z: float, heading: float
    ) -> None:
        unit = self.units.get(unit_id)
        if unit is None:
            return
        unit.pose.x, unit.pose.y, unit.pose.z, unit.pose.heading = x, y, z, heading
        self.map_view.set_unit_pose(unit_id, x, y, z, heading)
        if self.selected_unit_id == unit_id:
            self.context_panel.aircraft_panel.update_pose(z, heading)

    def _on_guided_stage_changed(self, stage: str) -> None:
        if self.selected_unit_id in TRICOPTER_IDS:
            self.context_panel.aircraft_panel.update_stage(stage)

    def _on_reactive_stage_changed(self, stage: str) -> None:
        if stage == "LAUNCHING":
            still_docked = [u for u in TRICOPTER_IDS if self._docked_state.get(u, True)]
            held = ", ".join(short_label(u) for u in still_docked)
            self.event_log.log(
                self.playback.clock_s,
                "Reactive launch: rover planner committed 2 of 3 scouts"
                + (f"; {held} held in bay for operator tasking" if held else ""),
            )
        self.event_log.log(self.playback.clock_s, f"Reactive launch: {stage}")
        if self.selected_unit_id in ("tricopter-1", "tricopter-2"):
            self.context_panel.aircraft_panel.update_stage(stage)

    # --- link cut / restore, section 8.5 ---

    def _on_cut_link(self) -> None:
        self._apply_link_cut(TRICOPTER_IDS)
        self.event_log.log(self.playback.clock_s, "Operator cut link: rover and aircraft mesh disconnected")

    def _on_restore_link(self) -> None:
        delta = self._apply_link_restore(TRICOPTER_IDS)
        self.event_log.log(
            self.playback.clock_s, f"Operator restored link: {delta} observation(s) crossed to reconcile"
        )

    def _on_scripted_link_event(self, event: str, unit_ids: list[str]) -> None:
        if event == "cut":
            self._apply_link_cut(unit_ids)
            self.event_log.log(
                self.playback.clock_s, f"Link cut: {', '.join(unit_ids)} diverging from rover map"
            )
        else:
            delta = self._apply_link_restore(unit_ids)
            self.event_log.log(
                self.playback.clock_s,
                f"Link restored: {', '.join(unit_ids)}, {delta} observation(s) crossed to reconcile",
            )

    def _apply_link_cut(self, unit_ids: list[str]) -> None:
        self.mesh_link.cut()
        self.map_view.set_link_cut_visual(True, unit_ids)
        for unit_id in unit_ids:
            self.units[unit_id].link_state = "LOST"
        self._refresh_all_rows()

    def _apply_link_restore(self, unit_ids: list[str]) -> int:
        delta = self.mesh_link.restore()
        self.map_view.set_link_cut_visual(False)
        for unit_id in unit_ids:
            self.units[unit_id].link_state = "CONNECTED"
        self._refresh_all_rows()
        return delta

    # --- alert / confirm workflow, section 8.3 ---

    def _on_detection(self, detection: dict) -> None:
        x, y, confidence = detection["x"], detection["y"], detection["confidence"]
        source = detection.get("source", "rover")
        source_id = detection.get("source_id", source)
        observation = VoxelObservation(
            voxel_key=voxel_key(x, y, 0.0),
            timestamp_us=int(self.playback.clock_s * 1_000_000),
            confidence=confidence,
            threat_type=detection.get("type", "unknown"),
            source_id=source_id,
        )
        self.mesh_link.observe(source, observation)
        self.map_view.add_detection(detection)
        self.threat_panel.add_detection(detection)
        self.context_panel.rover_panel.sensors.register_hit(detection.get("sensor", ""))
        self.event_log.log(
            self.playback.clock_s,
            f"Detection: {detection.get('sensor')} at ({x:.0f}, {y:.0f}), confidence {confidence:.2f}",
        )
        if confidence >= ALERT_CONFIDENCE_THRESHOLD:
            self.alert_banner.show_alert(detection)
            self.event_log.log(
                self.playback.clock_s, f"ALERT escalated: confidence {confidence:.2f} at ({x:.0f}, {y:.0f})"
            )

    def _on_alert_confirmed(self, detection: dict) -> None:
        self.map_view.confirm_detection(detection["x"], detection["y"])
        self.threat_panel.set_status(detection["x"], detection["y"], "CONFIRMED")
        self.event_log.log(
            self.playback.clock_s,
            f"Threat confirmed at ({detection['x']:.0f}, {detection['y']:.0f}); marked as exclusion zone",
        )

    def _on_alert_rejected(self, detection: dict) -> None:
        self.threat_panel.set_status(detection["x"], detection["y"], "CLEARED")
        self.event_log.log(
            self.playback.clock_s, f"Alert rejected at ({detection['x']:.0f}, {detection['y']:.0f})"
        )

    def _on_alert_timed_out(self, detection: dict) -> None:
        self.event_log.log(
            self.playback.clock_s,
            f"Alert timed out, unresolved, at ({detection['x']:.0f}, {detection['y']:.0f})",
        )

    # --- playback engine reflection ---

    def _on_clock_tick(self, clock_s: float) -> None:
        minutes, seconds = divmod(int(clock_s), 60)
        self.clock_label.setText(f"{minutes:02d}:{seconds:02d}")
        dt = clock_s - self._last_clock_s
        self._last_clock_s = clock_s
        if self.selected_unit_id == "rover":
            rover = self.units["rover"]
            self.context_panel.rover_panel.update_pose(rover.pose.x, rover.pose.y, rover.pose.heading, dt)

    def _on_phase_changed(self, phase: str, t: float) -> None:
        self.phase_rail.set_phase(phase)
        self.context_panel.rover_panel.sensors.set_scanning(phase in SCANNING_PHASES)
        self.event_log.log(t, f"Phase: {phase.replace('_', ' ')}")
        for unit in self.units.values():
            unit.current_task = phase.replace("_", " ")
        self._refresh_all_rows()

    def _on_unit_pose_changed(self, unit_id: str, x: float, y: float, z: float, heading: float) -> None:
        unit = self.units.get(unit_id)
        if unit is None or self._position_owner.get(unit_id) != "scripted":
            return
        unit.pose.x, unit.pose.y, unit.pose.z, unit.pose.heading = x, y, z, heading
        self.map_view.set_unit_pose(unit_id, x, y, z, heading)
        if self.selected_unit_id == unit_id and unit.kind == "tricopter":
            self.context_panel.aircraft_panel.update_pose(z, heading)

        breach_action = self.geofence.check_breach(x, y)
        if breach_action is not None and not unit.emergency_return:
            unit.emergency_return = True
            self.event_log.log(
                self.playback.clock_s, f"Geofence breach: {unit_id}, action = {breach_action}"
            )

    def _on_unit_battery_changed(self, unit_id: str, pct: float) -> None:
        unit = self.units.get(unit_id)
        if unit is None:
            return
        unit.battery_pct = pct
        if self.selected_unit_id == unit_id and unit.kind == "tricopter":
            self.context_panel.aircraft_panel.update_battery(pct)
        self._refresh_all_rows()

    def _on_mission_finished(self) -> None:
        self.event_log.log(
            self.playback.clock_s, "Mission complete: waypoints exhausted, rover returned to start point"
        )
        self.play_btn.setText("▶ PLAY")

    # --- transport ---

    def _on_play_clicked(self) -> None:
        if self.playback.playing:
            self.playback.pause()
            self.play_btn.setText("▶ PLAY")
        else:
            self.playback.play()
            self.play_btn.setText("⏸ PAUSE")

    def _on_reset_clicked(self) -> None:
        self.playback.reset()
        self.map_view.clear()
        self.event_log.clear()
        self.threat_panel.clear()
        self.phase_rail.reset()
        self.mesh_link = MeshLink()
        self.units = _default_units()
        self.control_authority = ControlAuthority(self.units)
        self.context_panel.launch_console.reset()
        self.context_panel.rover_panel.sensors.clear()
        self.context_panel.rover_panel.sensors.set_scanning(False)
        self.flight_director.reset()
        self._keys_down.clear()
        self._position_owner = {uid: "scripted" for uid in ALL_UNIT_IDS}
        for unit_id in TRICOPTER_IDS:
            self._set_docked(unit_id, True)
        self._last_clock_s = 0.0
        self.play_btn.setText("▶ PLAY")
        self.clock_label.setText("00:00")
        self.playback.apply_initial_state()
        self._select_unit(self.selected_unit_id)
        self._refresh_all_rows()

    # --- manual driving ---

    def keyPressEvent(self, event) -> None:
        if not event.isAutoRepeat():
            self._keys_down.add(event.key())
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if not event.isAutoRepeat():
            self._keys_down.discard(event.key())
        super().keyReleaseEvent(event)

    def focusOutEvent(self, event) -> None:
        # Dropping focus with a key held would leave the unit driving itself.
        self._keys_down.clear()
        holder = self.control_authority.manual_holder
        if holder is not None:
            self.flight_director.set_manual_input(holder, 0.0, 0.0, 0.0)
        super().focusOutEvent(event)

    # --- fleet-wide summaries ---

    def _refresh_all_rows(self) -> None:
        for unit_id, unit in self.units.items():
            manual = self.control_authority.manual_holder == unit_id
            self.fleet_roster.update_unit(
                unit_id, unit.control_mode, unit.battery_pct, unit.link_state, unit.current_task, manual
            )
            self.event_log.set_link(unit_id, unit.link_state)
            feed = self.camera_feeds.get(unit_id)
            if feed is not None:
                feed.set_link_state(unit.link_state)
                if unit.kind == "rover":
                    feed.set_telemetry(f"{unit.battery_pct:.0f}%  HDG {unit.pose.heading:.0f}°")
                else:
                    feed.set_telemetry(f"{unit.battery_pct:.0f}%  ALT {unit.pose.z:.0f}m")

        lowest = min(u.battery_pct for u in self.units.values())
        battery_color = "#e05c4b" if lowest <= 25 else "#e8b23c" if lowest <= 50 else "#cfe0dd"
        self.battery_chip.setText(_chip("BATT", f"{lowest:.0f}%", battery_color))

        worst = max(self.units.values(), key=lambda u: LINK_SEVERITY[u.link_state])
        link_color = {"CONNECTED": "#5ec9c2", "DEGRADED": "#e8b23c", "LOST": "#e05c4b"}[worst.link_state]
        self.link_chip.setText(_chip("LINK", worst.link_state, link_color))

        holder = self.control_authority.manual_holder
        self.manual_chip.setText(
            _chip("\U0001F512", holder or "—", "#f4d78a" if holder else "#6c7a80")
        )
