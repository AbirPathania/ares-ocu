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

import math

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
from ocu.core.comms import GHOST, LCA, STANDARD, profile_for
from ocu.core.control_authority import ControlAuthority
from ocu.core.flight_director import AircraftMode, FlightDirector
from ocu.core.geofence import Geofence
from ocu.core.detection import REPORT_FLOOR, classify, fuse, sweep
from ocu.core.lidar import LIDAR_RANGE_M, LIDAR_SCAN_HZ, cast_scan
from ocu.core.mesh_state import MeshLink, VoxelObservation, voxel_key
from ocu.core.power import (
    charge_pct_per_s,
    flight_regime,
    format_endurance,
    rover_drain_pct_per_s,
    rover_endurance_s,
    rover_power_w,
    tricopter_drain_pct_per_s,
    tricopter_endurance_s,
)
from ocu.core.targets import default_field
from ocu.core.terrain import PerceptionMap, TerrainWorld
from ocu.core.unit import Unit
from ocu.sim.playback_engine import PlaybackEngine
from ocu.ui.alert_banner import AlertBanner
from ocu.ui.camera_feed import SimulatedCameraFeed
from ocu.ui.context_panel import ContextPanel
from ocu.ui.event_log import EventLog
from ocu.ui.fleet_roster import FleetRoster
from ocu.ui.launch_console import short_label
from ocu.ui.map_view import MapView
from ocu.ui.widgets import make_styled_combo
from ocu.ui.threat_panel import ThreatPanel

TRICOPTER_IDS = ["tricopter-1", "tricopter-2", "tricopter-3"]
ALL_UNIT_IDS = ["rover"] + TRICOPTER_IDS
LINK_SEVERITY = {state: i for i, state in enumerate(("CONNECTED", "DEGRADED", "LOST"))}
DIRECTOR_TICK_MS = 100

SCANNING_PHASES = {"drive_and_scan", "reactive_launch", "aerial_recon", "fusion", "alert"}


def _default_units() -> dict[str, Unit]:
    units = {"rover": Unit(id="rover", kind="rover", battery_pct=96.0, current_task="idle")}
    for tricopter_id in TRICOPTER_IDS:
        units[tricopter_id] = Unit(id=tricopter_id, kind="tricopter", current_task="docked")
    return units


def _chip(label: str, value: str, color: str) -> str:
    return f'<span style="color:#7B8794">{label}</span> <span style="color:{color}">{value}</span>'


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
        self._comms = profile_for(STANDARD)
        # Ground truth, and separately what the rover has actually seen of it.
        self.terrain = TerrainWorld()
        # What is actually buried out there, and what the suite has
        # accumulated about each object it has swept over.
        self.target_field = default_field()
        self._target_evidence: dict[int, dict[str, float]] = {}
        self._escalated: set = set()
        self.wind_bearing_deg = 215.0
        self._last_sweep_xy: tuple[float, float] | None = None
        self.perception = PerceptionMap()
        self._last_scan_s = -1e9
        self._obstacle_returns = 0
        self._last_downlink_s = 0.0
        self._silence_began_s: float | None = None
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

        feeds_label = QLabel("CAMERA FEEDS")
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

        self.clock_label = QLabel("00:00")
        self.clock_label.setObjectName("missionClock")
        layout.addWidget(self.clock_label)

        self.phase_label = QLabel("STANDBY")
        self.phase_label.setObjectName("phaseLabel")
        layout.addWidget(self.phase_label)

        layout.addStretch(1)

        # Mission mode is real doctrine (section 4.1) with real consequences,
        # so it is a control rather than a caption. LCA is not offered here:
        # it is entered automatically on total loss of contact, never chosen.
        self.mode_combo = make_styled_combo(
            [
                ("STANDARD · full RF, live video", "STANDARD"),
                ("AUTONOMOUS · status pings only", "AUTONOMOUS"),
                ("GHOST · zero emissions", "GHOST"),
            ]
        )
        self.mode_combo.setObjectName("modeCombo")
        self.mode_combo.setToolTip(
            "Mission mode, section 4.1.\n"
            "STANDARD: full RF, live video and telemetry, real-time override.\n"
            "AUTONOMOUS: minimal RF, periodic status only, no real-time intervention.\n"
            "GHOST: zero intentional emissions, no downlink, UWB only.\n"
            "LCA is entered automatically if the rover loses contact entirely."
        )
        self.mode_combo.currentIndexChanged.connect(self._on_mission_mode_changed)
        layout.addWidget(self.mode_combo)

        self.link_chip = QLabel()
        self.battery_chip = QLabel()
        self.manual_chip = QLabel()
        for chip in (self.link_chip, self.battery_chip, self.manual_chip):
            chip.setObjectName("statusChip")
            layout.addWidget(chip)
        self.manual_chip.setVisible(False)  # only shown when somebody holds the sticks

        self.play_btn = QPushButton("▶ DEPLOY")
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
        self.threat_panel.confirmRequested.connect(self._on_alert_confirmed)
        self.threat_panel.rejectRequested.connect(self._on_alert_rejected)

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

    # --- mission mode, section 4.1 ---

    def _on_mission_mode_changed(self) -> None:
        """Apply the selected mission mode, section 4.1.

        A mode is a communications posture, not a permission level. It
        decides what the fleet radiates and therefore what arrives here:
        Autonomous drops video and delivers state on a ping instead of a
        stream, Ghost stops the downlink entirely. The fleet keeps flying
        either way. Freezing this display while the simulation continues
        underneath is what those modes actually look like from an operator's
        seat, and it is the part worth showing.
        """
        self._apply_comms(self.mode_combo.currentData() or STANDARD)

    def _apply_comms(self, mode: str) -> None:
        self._comms = profile_for(mode)
        for unit in self.units.values():
            unit.mission_mode = mode

        for feed in self.camera_feeds.values():
            feed.set_downlink(self._comms.video, mode)

        if not self._comms.uplink and self.control_authority.manual_holder is not None:
            self._on_release_manual()
        self.context_panel.set_manual_available(self._comms.uplink, mode)

        if self._comms.telemetry:
            self._silence_began_s = None
            self.map_view.set_telemetry_stale(False, "")
        else:
            # Mark the picture stale the moment the downlink closes, rather
            # than on the next clock tick: with playback paused there may not
            # be a next tick, and a frozen picture must never look live.
            if self._silence_began_s is None:
                self._silence_began_s = self.playback.clock_s
            self._update_silence_indicator()

        self.event_log.log(self.playback.clock_s, f"Mission mode {mode}: {self._comms.summary}")
        self._refresh_all_rows()

    def _downlink_open(self) -> bool:
        """Whether a fresh picture is allowed through to the operator right now."""
        if self._comms.telemetry and self._comms.continuous:
            return True
        if not self._comms.telemetry:
            return False
        # Autonomous: a snapshot on the ping, nothing between pings.
        if self.playback.clock_s - self._last_downlink_s >= self._comms.ping_interval_s:
            self._last_downlink_s = self.playback.clock_s
            return True
        return False

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

        downlink = self._downlink_open()
        for unit_id, (x, y, z, heading) in poses.items():
            unit = self.units.get(unit_id)
            if unit is None:
                continue
            # The aircraft is there whether or not it is telling us. State
            # always advances; only the operator's copy of it is gated.
            unit.pose.x, unit.pose.y, unit.pose.z, unit.pose.heading = x, y, z, heading
            if not downlink:
                continue
            self.map_view.set_unit_pose(unit_id, x, y, z, heading)
            if self.selected_unit_id == unit_id:
                self.context_panel.aircraft_panel.update_pose(z, heading)

        self._tick_power(dt)
        self._tick_lidar()
        self._tick_detection_suite()

        for unit_id, event in events:
            self._handle_director_event(unit_id, event)

        if self.selected_unit_id in TRICOPTER_IDS:
            self._refresh_aircraft_flight_state(self.selected_unit_id)

    def _tick_lidar(self) -> None:
        """Sweep the rover's scanner and fold the returns into its map.

        The scan runs on the rover's own clock, not the operator's: the
        vehicle keeps building its map whatever the downlink is doing. What
        the comms posture decides is whether the operator gets to see it,
        which is why Ghost freezes the picture while the map underneath
        carries on filling in.
        """
        now = self.playback.clock_s
        if now - self._last_scan_s < 1.0 / LIDAR_SCAN_HZ:
            return
        self._last_scan_s = now

        rover = self.units["rover"]
        scan = cast_scan(self.terrain, self.perception, rover.pose.x, rover.pose.y)
        self.perception.drain_new()
        self._obstacle_returns += scan.hits

        if not self._comms.telemetry:
            # The rover is still scanning and still building its map. The
            # operator simply is not being sent it.
            return

        self.map_view.add_returns(
            scan.returns,
            self.perception.coverage_m2(),
            {
                "returns_per_s": scan.point_count * LIDAR_SCAN_HZ,
                "obstacles": self._obstacle_returns,
                "range_m": LIDAR_RANGE_M,
                "scan_hz": LIDAR_SCAN_HZ,
            },
        )
        self.map_view.set_live_scan(
            (rover.pose.x, rover.pose.y), [(r.x, r.y) for r in scan.returns[::7]]
        )

    def _tick_detection_suite(self) -> None:
        """Sweep the detection suite over whatever the rover is standing on.

        The suite runs continuously while driving, per section 2: the rover
        does not stop to scan. Each sensor responds to the physical
        properties of a buried object rather than to a timestamp, so what
        appears on screen is the output of the geometry. Drive a different
        route and the suite finds different things, or nothing.

        Evidence accumulates per object: the best reading each sensor has
        ever taken of it. A single pass at the edge of a swath is weak, and
        driving closer strengthens it, which is what the operator is
        watching when a confidence climbs.
        """
        rover = self.units["rover"]
        scanning = self._comms.mesh_traffic  # the suite is on the vehicle

        # The sensors sweep continuously along the ground the rover covers,
        # so the suite is sampled along the segment driven since the last
        # tick rather than at a single point. The pulse induction array is
        # only 40 cm wide: point-sampling a moving vehicle steps straight
        # over objects it should have driven across, and the faster the
        # playback the more it misses.
        previous = self._last_sweep_xy or (rover.pose.x, rover.pose.y)
        self._last_sweep_xy = (rover.pose.x, rover.pose.y)
        travelled = math.hypot(rover.pose.x - previous[0], rover.pose.y - previous[1])
        samples = max(1, int(travelled / 0.10) + 1)
        path = [
            (
                previous[0] + (rover.pose.x - previous[0]) * i / samples,
                previous[1] + (rover.pose.y - previous[1]) * i / samples,
            )
            for i in range(samples + 1)
        ]

        for buried in self.target_field.near(rover.pose.x, rover.pose.y, 4.0):
            readings = []
            for sx, sy in path:
                readings.extend(sweep(buried, sx, sy, self.wind_bearing_deg))
            if not readings:
                continue

            evidence = self._target_evidence.setdefault(id(buried), {})
            improved = False
            for reading in readings:
                if reading.confidence > evidence.get(reading.sensor, 0.0):
                    evidence[reading.sensor] = reading.confidence
                    improved = True
            if not improved or not scanning:
                continue

            fused = fuse(evidence)
            if fused < REPORT_FLOOR:
                continue

            self._report_detection(
                buried,
                sensors=sorted(evidence),
                confidence=fused,
                threat_type=classify(evidence),
            )

    def _report_detection(self, buried, sensors: list, confidence: float, threat_type: str) -> None:
        # Every sensor that has contributed, because which ones agree is the
        # evidence: metal with no vapour is scrap, metal with vapour is not.
        self._on_detection(
            {
                "x": buried.x,
                "y": buried.y,
                "confidence": confidence,
                "sensor": sensors[0] if sensors else "",
                "sensors": sensors,
                "source": "rover",
                "source_id": "rover",
                "type": threat_type,
            }
        )

    def _tick_power(self, dt_s: float) -> None:
        """Integrate every battery from what that unit is actually doing.

        Drain follows the power model in core/power.py, which is built to
        reproduce the published baseline: 3.0 h for the rover at its mission
        average, 10 to 12 minutes of cruise for an ARES-A, 8 to 10 minutes to
        charge. Hovering costs more than cruising, climbing more than either,
        and a docked aircraft charges and tapers.
        """
        # Battery has to burn on mission time, not wall-clock time. The
        # director ticks in real seconds, but the mission can be played back
        # faster; integrating against the wall clock would leave a
        # fast-forwarded run showing a pack that barely moved.
        if self.playback.playing:
            dt_s *= self.playback.speed

        rover = self.units["rover"]
        rover_speed = self.flight_director.speed_of("rover") or 0.0
        rover.battery_pct = max(0.0, rover.battery_pct - rover_drain_pct_per_s(rover_speed) * dt_s)

        for unit_id in TRICOPTER_IDS:
            unit = self.units.get(unit_id)
            if unit is None:
                continue
            if self._docked_state.get(unit_id, True):
                unit.battery_pct = min(
                    100.0, unit.battery_pct + charge_pct_per_s(unit.battery_pct) * dt_s
                )
                continue
            state = self.flight_director.aircraft.get(unit_id)
            speed = state.body.speed_ms if state else 0.0
            vertical = state.body.vz if state else 0.0
            unit.battery_pct = max(
                0.0, unit.battery_pct - tricopter_drain_pct_per_s(speed, vertical) * dt_s
            )

        if self.selected_unit_id == "rover" and self._comms.telemetry:
            self.context_panel.rover_panel.update_power(
                rover_power_w(rover_speed), self._endurance_text("rover")
            )
        elif self.selected_unit_id in TRICOPTER_IDS and self._comms.telemetry:
            state = self.flight_director.aircraft.get(self.selected_unit_id)
            speed = state.body.speed_ms if state else 0.0
            vertical = state.body.vz if state else 0.0
            self.context_panel.aircraft_panel.update_endurance(
                self._endurance_text(self.selected_unit_id), flight_regime(speed, vertical)
            )

        self._refresh_all_rows()

    def _endurance_text(self, unit_id: str) -> str:
        """Time remaining at the current draw, which is what an operator reads.

        A three hour platform barely moves its percentage across a ninety
        second mission. That is correct, and on its own it is also useless.
        """
        unit = self.units[unit_id]
        if unit.kind == "rover":
            return format_endurance(rover_endurance_s(unit.battery_pct, self.flight_director.speed_of("rover") or 0.0))
        if self._docked_state.get(unit_id, True):
            return "CHG" if unit.battery_pct < 100.0 else "FULL"
        state = self.flight_director.aircraft.get(unit_id)
        speed = state.body.speed_ms if state else 0.0
        vertical = state.body.vz if state else 0.0
        return format_endurance(tricopter_endurance_s(unit.battery_pct, speed, vertical))

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

    def _check_lost_comms_autonomous(self) -> None:
        """LCA is entered, not ordered, section 4.1."""
        all_lost = all(u.link_state == "LOST" for u in self.units.values())
        if all_lost and self._comms.mode != LCA:
            self._apply_comms(LCA)
        elif not all_lost and self._comms.mode == LCA:
            self._apply_comms(self.mode_combo.currentData() or STANDARD)

    def _apply_link_cut(self, unit_ids: list[str]) -> None:
        self.mesh_link.cut()
        self.map_view.set_link_cut_visual(True, unit_ids)
        self._push_mesh_state()
        for unit_id in unit_ids:
            self.units[unit_id].link_state = "LOST"
        self._check_lost_comms_autonomous()
        self._refresh_all_rows()

    def _apply_link_restore(self, unit_ids: list[str]) -> int:
        delta = self.mesh_link.restore()
        self.map_view.set_link_cut_visual(False)
        self._push_mesh_state()
        self.map_view.flash_reconciled()
        for unit_id in unit_ids:
            self.units[unit_id].link_state = "CONNECTED"
        self._check_lost_comms_autonomous()
        self._refresh_all_rows()
        return delta

    # --- alert / confirm workflow, section 8.3 ---

    def _push_mesh_state(self) -> None:
        """Mirror the real replica state onto the map after anything changes it."""
        self.map_view.set_mesh_state(
            self.mesh_link.holdings(),
            self.mesh_link.rover_voxels,
            self.mesh_link.aircraft_voxels,
            self.mesh_link.divergence(),
        )

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
        if self._comms.mesh_traffic:
            self.mesh_link.observe(source, observation)
            self._push_mesh_state()
        if not self._comms.telemetry:
            # Found, logged on the vehicle, and not reported until the fleet
            # is allowed to talk again.
            return
        self.map_view.add_detection(detection)
        self.threat_panel.add_detection(detection)
        self.context_panel.rover_panel.sensors.register_hit(detection.get("sensor", ""))
        self.event_log.log(
            self.playback.clock_s,
            f"Detection: {detection.get('sensor')} at ({x:.0f}, {y:.0f}), confidence {confidence:.2f}",
        )
        key = (round(x, 2), round(y, 2))
        if confidence >= ALERT_CONFIDENCE_THRESHOLD and key not in self._escalated:
            self._escalated.add(key)
            self.alert_banner.show_alert(detection)
            self.event_log.log(
                self.playback.clock_s, f"ALERT escalated: confidence {confidence:.2f} at ({x:.0f}, {y:.0f})"
            )

    def _on_alert_confirmed(self, detection: dict) -> None:
        self.alert_banner.dismiss_for(detection)
        self.map_view.confirm_detection(detection["x"], detection["y"])
        self.threat_panel.set_status(detection["x"], detection["y"], "CONFIRMED")
        self.event_log.log(
            self.playback.clock_s,
            f"Threat confirmed at ({detection['x']:.0f}, {detection['y']:.0f}); marked as exclusion zone",
        )

    def _on_alert_rejected(self, detection: dict) -> None:
        self.alert_banner.dismiss_for(detection)
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
        self._update_silence_indicator()
        # Rover telemetry is a downlink like any other.
        if self.selected_unit_id == "rover" and self._comms.telemetry:
            rover = self.units["rover"]
            self.context_panel.rover_panel.update_pose(rover.pose.x, rover.pose.y, rover.pose.heading, dt)

    def _update_silence_indicator(self) -> None:
        if self._comms.telemetry or self._silence_began_s is None:
            return
        elapsed = max(0.0, self.playback.clock_s - self._silence_began_s)
        minutes, seconds = divmod(int(elapsed), 60)
        label = "CONTACT LOST" if self._comms.mode == LCA else "EMISSIONS SILENT"
        self.map_view.set_telemetry_stale(True, f"{label} \u00b7 LAST CONTACT {minutes:02d}:{seconds:02d}")

    def _on_phase_changed(self, phase: str, t: float) -> None:
        self.phase_label.setText(phase.replace('_', ' ').upper())
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
        if self._downlink_open():
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
        self.play_btn.setText("▶ DEPLOY")

    # --- transport ---

    def _on_play_clicked(self) -> None:
        if self.playback.playing:
            self.playback.pause()
            self.play_btn.setText("▶ DEPLOY")
        else:
            self.playback.play()
            self.play_btn.setText("⏸ HOLD")

    def _on_reset_clicked(self) -> None:
        self.playback.reset()
        self.map_view.clear()
        self.perception.clear()
        self.map_view.clear_perception()
        self._last_scan_s = -1e9
        self._obstacle_returns = 0
        self.event_log.clear()
        self.threat_panel.clear()
        self.phase_label.setText("STANDBY")
        self.mesh_link = MeshLink()
        self._push_mesh_state()
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
        self.play_btn.setText("▶ DEPLOY")
        self.clock_label.setText("00:00")
        self.playback.apply_initial_state()
        self._select_unit(self.selected_unit_id)
        self._refresh_all_rows()

    # --- manual driving ---

    def keyPressEvent(self, event) -> None:
        # One key back to a clean start, so a demo can always be recovered in
        # front of an audience. Modified and function keys only, so it can
        # never collide with the WASD driving keys.
        if event.key() == Qt.Key_F5 or (
            event.key() == Qt.Key_R and event.modifiers() & (Qt.ControlModifier | Qt.MetaModifier)
        ):
            self._keys_down.clear()
            self._on_reset_clicked()
            self.event_log.log(0.0, "Demo reset: fleet, map and mission returned to start")
            return
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
                unit_id, unit.control_mode, unit.battery_pct, unit.link_state, unit.current_task,
                manual, self._endurance_text(unit_id),
            )
            self.event_log.set_link(
                unit_id, unit.link_state if self._comms.telemetry else "SILENT"
            )
            feed = self.camera_feeds.get(unit_id)
            if feed is not None:
                feed.set_link_state(unit.link_state)
                if unit.kind == "rover":
                    feed.set_telemetry(f"{unit.battery_pct:.0f}%  HDG {unit.pose.heading:.0f}°")
                else:
                    feed.set_telemetry(
                        f"{unit.battery_pct:.0f}%  {self._endurance_text(unit_id)}  "
                        f"ALT {unit.pose.z:.0f}m"
                    )

        lowest = min(u.battery_pct for u in self.units.values())
        battery_color = "#C04438" if lowest <= 25 else "#C9952F" if lowest <= 50 else "#C6D0DA"
        self.battery_chip.setText(_chip("BATT", f"{lowest:.0f}%", battery_color))

        # In Ghost there is nothing on the air to be connected to, so a green
        # CONNECTED chip would contradict the mode it is sitting next to.
        if not self._comms.telemetry:
            label = "CONTACT LOST" if self._comms.mode == LCA else "SILENT"
            self.link_chip.setText(_chip("LINK", label, "#C9952F"))
        else:
            worst = max(self.units.values(), key=lambda u: LINK_SEVERITY[u.link_state])
            link_color = {"CONNECTED": "#4F9D69", "DEGRADED": "#C9952F", "LOST": "#C04438"}[
                worst.link_state
            ]
            self.link_chip.setText(_chip("LINK", worst.link_state, link_color))

        # The lock only appears when somebody actually holds the sticks; an
        # always-present padlock reading "—" is a symbol with nothing to say.
        holder = self.control_authority.manual_holder
        self.manual_chip.setVisible(holder is not None)
        if holder is not None:
            self.manual_chip.setText(_chip("\U0001F512 MANUAL", short_label(holder), "#D9B257"))
