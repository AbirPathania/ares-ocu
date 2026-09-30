"""Guided formation launch console, section 6.2 and section 9.4's Formation
selection. This is the mechanism Abir asked for directly: set one bearing,
get a formation.

Opening this console arms the sequencer (ARMED: "operator has opened the
launch console", section 6.3). Which aircraft go is the operator's choice,
not a fixed pair: the reactive launch in section 6.1 is the rover's own
planner sending one or two scouts, but a guided launch is a deliberate
command and can commit the whole bay. The bearing dial, pattern picker,
spacing and rally altitude redraw the map's live preview on every change,
before anything commits, section 6.2. Stage is the explicit verify step;
Launch is an irreversible command and takes the section 10 two-second
hold, not a dialog.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QListView,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ocu.core.constants import (
    ARES_A_CEILING_M,
    ARES_A_SCOUT_RADIUS_M_HIGH,
    ARES_A_SCOUT_RADIUS_M_LOW,
    FORMATION_PATTERNS,
    FORMATION_STANDOFF_M,
    MIN_AIRCRAFT_SEPARATION_M,
    TETHER_LIMIT_M,
)
from ocu.core.formation import Target, compute_formation_targets
from ocu.core.launch_sequencer import LaunchSequencer, LaunchStage
from ocu.ui.widgets import HoldToConfirmButton

TICK_MS = 100
DEFAULT_RALLY_ALTITUDE_M = 20.0
DEFAULT_SPACING_M = MIN_AIRCRAFT_SEPARATION_M * 2

# Section 6.2's own descriptions of what each formation is for.
PATTERN_PURPOSE = {
    "vee": "forward push",
    "line": "wide sweep",
    "column": "tight trail",
}


def short_label(unit_id: str) -> str:
    if unit_id.startswith("tricopter-"):
        return "TRI-" + unit_id.split("-")[-1]
    return unit_id.upper()


class LaunchConsole(QWidget):
    previewChanged = Signal(list, tuple)  # targets [(x,y,z)...], origin (x,y)
    previewCleared = Signal()
    bearingChanged = Signal(float)  # keeps the map's compass rose in step with the dial
    stageChanged = Signal(str)  # LaunchStage value
    aircraftPositionChanged = Signal(str, float, float, float, float)  # id, x, y, z, heading
    aircraftUndocked = Signal(list)  # tricopter_ids leaving the rover's bay
    launchComplete = Signal(list)  # tricopter_ids now flying their slots
    eventLogged = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._sequencer = LaunchSequencer()
        self._origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._available_ids: list[str] = []

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._on_tick)

        self._build_ui()
        self._update_button_states()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(4)

        title = QLabel("FORMATION LAUNCH")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        self.stage_label = QLabel("IDLE")
        self.stage_label.setObjectName("stageLabel")
        layout.addWidget(self.stage_label)

        select_label = QLabel("AIRCRAFT")
        select_label.setObjectName("sectionLabel")
        layout.addWidget(select_label)

        self.checkboxes: dict[str, QCheckBox] = {}
        checkbox_row = QHBoxLayout()
        checkbox_row.setSpacing(4)
        for i in range(3):
            unit_id = f"tricopter-{i + 1}"
            box = QCheckBox(short_label(unit_id))
            box.setChecked(True)
            box.toggled.connect(self._on_params_changed)
            # A stylesheet-drawn indicator is not included in the widget's own
            # size hint, so without this the label clips to "TRI-".
            box.setMinimumWidth(box.fontMetrics().horizontalAdvance("TRI-0 \u00b7 out") + 30)
            self.checkboxes[unit_id] = box
            checkbox_row.addWidget(box)
        checkbox_row.addStretch(1)
        layout.addLayout(checkbox_row)

        form = QFormLayout()
        form.setSpacing(4)

        self.bearing_spin = QDoubleSpinBox()
        self.bearing_spin.setRange(0.0, 359.9)
        self.bearing_spin.setSuffix("°")
        self.bearing_spin.valueChanged.connect(self._on_params_changed)
        form.addRow("Bearing", self.bearing_spin)

        self.pattern_combo = QComboBox()
        # A plain QListView takes the stylesheet; the native popup ignores it,
        # renders its own check column and clips the longest entry.
        self.pattern_combo.setView(QListView())
        self.pattern_combo.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        for value in FORMATION_PATTERNS:
            self.pattern_combo.addItem(f"{value.upper()}  \u00b7  {PATTERN_PURPOSE[value]}", value)
        self.pattern_combo.currentIndexChanged.connect(self._on_params_changed)
        self._fit_popup_to_contents(self.pattern_combo)
        form.addRow("Pattern", self.pattern_combo)

        self.spacing_spin = QDoubleSpinBox()
        self.spacing_spin.setRange(MIN_AIRCRAFT_SEPARATION_M, TETHER_LIMIT_M)
        self.spacing_spin.setValue(DEFAULT_SPACING_M)
        self.spacing_spin.setSuffix(" m")
        self.spacing_spin.valueChanged.connect(self._on_params_changed)
        form.addRow("Spacing", self.spacing_spin)

        self.standoff_spin = QDoubleSpinBox()
        self.standoff_spin.setRange(MIN_AIRCRAFT_SEPARATION_M, TETHER_LIMIT_M)
        self.standoff_spin.setValue(FORMATION_STANDOFF_M)
        self.standoff_spin.setSuffix(" m")
        self.standoff_spin.setToolTip(
            f"How far ahead of the rover the formation scouts. Section 2 puts the "
            f"ARES-A scout radius at {ARES_A_SCOUT_RADIUS_M_LOW:.0f} to "
            f"{ARES_A_SCOUT_RADIUS_M_HIGH:.0f} m forward."
        )
        self.standoff_spin.valueChanged.connect(self._on_params_changed)
        form.addRow("Standoff", self.standoff_spin)

        self.rally_altitude_spin = QDoubleSpinBox()
        self.rally_altitude_spin.setRange(1.0, ARES_A_CEILING_M)
        self.rally_altitude_spin.setValue(DEFAULT_RALLY_ALTITUDE_M)
        self.rally_altitude_spin.setSuffix(" m")
        self.rally_altitude_spin.valueChanged.connect(self._on_params_changed)
        form.addRow("Rally altitude", self.rally_altitude_spin)

        layout.addLayout(form)

        tether_note = QLabel(f"Slots are clamped to the {TETHER_LIMIT_M:.0f} m UWB tether.")
        tether_note.setObjectName("mutedNote")
        tether_note.setWordWrap(True)
        layout.addWidget(tether_note)

        button_row = QHBoxLayout()
        self.stage_btn = QPushButton("STAGE")
        self.stage_btn.clicked.connect(self._on_stage_clicked)
        button_row.addWidget(self.stage_btn)

        self.launch_btn = HoldToConfirmButton("LAUNCH")
        self.launch_btn.setObjectName("launchAircraftButton")
        self.launch_btn.confirmed.connect(self._on_launch_confirmed)
        button_row.addWidget(self.launch_btn)
        layout.addLayout(button_row)

        layout.addStretch(1)

    # --- external wiring ---

    def set_available_tricopters(
        self, tricopter_ids: list[str], origin: tuple[float, float, float]
    ) -> None:
        """Called when the operator opens the console. Only aircraft still in
        the bay can be committed; the rest are shown disabled so the operator
        can see the whole bay rather than a silently shortened list."""
        self._available_ids = list(tricopter_ids)
        self._origin = origin

        for unit_id, box in self.checkboxes.items():
            available = unit_id in tricopter_ids
            box.setEnabled(available)
            if not available:
                box.setChecked(False)
                box.setText(f"{short_label(unit_id)} · out")
            else:
                box.setText(short_label(unit_id))
                box.setChecked(True)

        if self._sequencer.stage == LaunchStage.IDLE and tricopter_ids:
            self._sequencer.arm()
            self.stage_label.setText(LaunchStage.ARMED.value)
            self.stageChanged.emit(LaunchStage.ARMED.value)

        self.bearingChanged.emit(self.bearing_spin.value())
        if tricopter_ids:
            self._emit_preview()
        else:
            self.stage_label.setText("BAY EMPTY")
            self.previewCleared.emit()
        self._update_button_states()

    @staticmethod
    def _fit_popup_to_contents(combo: QComboBox) -> None:
        """Size the popup to its longest entry.

        The popup is a separate top-level widget whose width follows the
        closed combo, not its own contents, so the longest option elides
        unless the view is widened explicitly.
        """
        view = combo.view()
        # Measure with the view's own font, and leave room for the item
        # padding, the view border and a possible scrollbar, none of which
        # the text width accounts for.
        metrics = view.fontMetrics()
        widest = max(
            (metrics.horizontalAdvance(combo.itemText(i)) for i in range(combo.count())), default=0
        )
        view.setTextElideMode(Qt.ElideNone)
        view.setMinimumWidth(widest + 72)

    def current_pattern(self) -> str:
        return self.pattern_combo.currentData() or FORMATION_PATTERNS[0]

    def set_pattern(self, value: str) -> None:
        index = self.pattern_combo.findData(value)
        if index >= 0:
            self.pattern_combo.setCurrentIndex(index)

    def selected_ids(self) -> list[str]:
        return [
            unit_id
            for unit_id, box in self.checkboxes.items()
            if box.isChecked() and box.isEnabled()
        ]

    def bearing_from_map(self, bearing_deg: float) -> None:
        self.bearing_spin.blockSignals(True)
        self.bearing_spin.setValue(bearing_deg)
        self.bearing_spin.blockSignals(False)
        self._emit_preview()

    def is_active(self) -> bool:
        return self._sequencer.stage not in (LaunchStage.IDLE, LaunchStage.FORMED)

    # --- internal ---

    def _current_targets(self) -> list[Target]:
        targets = compute_formation_targets(
            self._origin[0],
            self._origin[1],
            self.bearing_spin.value(),
            self.current_pattern(),
            self.spacing_spin.value(),
            self.rally_altitude_spin.value(),
            standoff_m=self.standoff_spin.value(),
        )
        return targets[: max(len(self.selected_ids()), 1)]

    def _on_params_changed(self, *_args) -> None:
        self.bearingChanged.emit(self.bearing_spin.value())
        self._emit_preview()
        self._update_button_states()

    def _emit_preview(self) -> None:
        if self._sequencer.stage not in (LaunchStage.ARMED, LaunchStage.STAGED):
            return
        if not self.selected_ids():
            self.previewCleared.emit()
            return
        selected = self.selected_ids()
        targets = self._current_targets()[: len(selected)]
        self.previewChanged.emit(
            [(t.x, t.y, t.z) for t in targets], (self._origin[0], self._origin[1])
        )

    def _on_stage_clicked(self) -> None:
        if self._sequencer.stage != LaunchStage.ARMED:
            return
        selected = self.selected_ids()
        if not selected:
            return
        origin = Target(*self._origin)
        targets = self._current_targets()[: len(selected)]
        self._sequencer.stage_formation(selected, targets, origin, is_reactive=False)
        self.stage_label.setText(self._sequencer.stage.value)
        self.stageChanged.emit(self._sequencer.stage.value)
        self.eventLogged.emit(
            f"Formation staged: {len(selected)} aircraft, {self.current_pattern()} "
            f"at bearing {self.bearing_spin.value():.0f}°, spacing {self.spacing_spin.value():.1f} m, "
            f"rally {self.rally_altitude_spin.value():.0f} m"
        )
        self._update_button_states()

    def _on_launch_confirmed(self) -> None:
        if self._sequencer.stage != LaunchStage.STAGED:
            return
        self._sequencer.launch()
        self.previewCleared.emit()
        self.stage_label.setText(self._sequencer.stage.value)
        self.stageChanged.emit(self._sequencer.stage.value)
        launched = [plan.tricopter_id for plan in self._sequencer.plans]
        self.eventLogged.emit(f"Launch commanded: {', '.join(short_label(i) for i in launched)}")
        self.aircraftUndocked.emit(launched)
        self._timer.start()
        self._update_button_states()

    def _on_tick(self) -> None:
        previous_stage = self._sequencer.stage
        self._sequencer.tick(TICK_MS / 1000.0)

        for plan in self._sequencer.plans:
            pos = self._sequencer.position_for(plan.tricopter_id)
            heading = self._sequencer.heading_for(plan.tricopter_id)
            self.aircraftPositionChanged.emit(plan.tricopter_id, pos.x, pos.y, pos.z, heading)

        if self._sequencer.stage != previous_stage:
            self.stage_label.setText(self._sequencer.stage.value)
            self.stageChanged.emit(self._sequencer.stage.value)
            self.eventLogged.emit(f"Launch sequence: {self._sequencer.stage.value}")

        if self._sequencer.stage == LaunchStage.FORMED:
            self._timer.stop()
            formed = [plan.tricopter_id for plan in self._sequencer.plans]
            self.launchComplete.emit(formed)
            self._sequencer.reset()
            self.stage_label.setText("FORMED · on station")
            self._update_button_states()

    def _update_button_states(self) -> None:
        stage = self._sequencer.stage
        has_selection = bool(self.selected_ids())
        self.stage_btn.setEnabled(stage == LaunchStage.ARMED and has_selection)
        self.launch_btn.setEnabled(stage == LaunchStage.STAGED)

    def reset(self) -> None:
        self._timer.stop()
        self._sequencer.reset()
        self.stage_label.setText(LaunchStage.IDLE.value)
        self.previewCleared.emit()
        for unit_id, box in self.checkboxes.items():
            box.setEnabled(True)
            box.setChecked(True)
            box.setText(short_label(unit_id))
        self._update_button_states()
