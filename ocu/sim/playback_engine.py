"""QTimer driven, reads the mission file, section 11.2.

Every position, detection, and battery percentage the interface displays
comes from mission_timeline.json (section 3: "no hardware exists yet").
This engine's only job is to play that file back on a real clock and turn
it into signals the UI panels connect to.

The one piece of genuinely live behavior in here is the reactive launch at
the "reactive_launch" phase marker: per section 6.1, it triggers the same
LaunchSequencer state machine section 6.3 describes, bearing computed
toward the scripted occlusion, no operator input. That sequencer really
runs through ARMED -> STAGED -> LAUNCHING -> CLIMBING -> TRANSLATING ->
FORMED on this engine's clock, and its stage is what the event log and the
right panel narrate. The aircraft's on-map position stays the scripted
value from mission_timeline.json (section 13 marks unit positions as
scripted, not the sequencer's own kinematics) so the map never shows two
disagreeing truths for where a tricopter is.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal

from ocu.core.constants import MIN_AIRCRAFT_SEPARATION_M
from ocu.core.formation import Target, compute_formation_targets
from ocu.core.launch_sequencer import LaunchSequencer, LaunchStage

TICK_MS = 100
DEFAULT_TIMELINE_PATH = Path(__file__).parent / "mission_timeline.json"
REACTIVE_RALLY_ALTITUDE_M = 20.0  # matches the scripted cruise altitude in mission_timeline.json


def _interpolate_series(keyframes: list[dict[str, Any]], t: float) -> dict[str, float] | None:
    """Linear interpolation between the two keyframes bounding t. Clamps at the ends."""
    if not keyframes:
        return None
    if t <= keyframes[0]["t"]:
        return {k: v for k, v in keyframes[0].items() if k != "t"}
    if t >= keyframes[-1]["t"]:
        return {k: v for k, v in keyframes[-1].items() if k != "t"}

    for a, b in zip(keyframes, keyframes[1:]):
        if a["t"] <= t <= b["t"]:
            span = b["t"] - a["t"]
            frac = 0.0 if span == 0 else (t - a["t"]) / span
            out: dict[str, float] = {}
            for key, av in a.items():
                if key == "t":
                    continue
                bv = b.get(key, av)
                out[key] = av + (bv - av) * frac
            return out
    return None


class PlaybackEngine(QObject):
    unitPoseChanged = Signal(str, float, float, float, float)  # id, x, y, z, heading
    unitBatteryChanged = Signal(str, float)
    phaseChanged = Signal(str, float)  # phase name, mission-clock seconds
    detectionOccurred = Signal(dict)
    linkEvent = Signal(str, list)  # "cut" | "restore", unit ids
    reactiveLaunchStageChanged = Signal(str)
    unitDockedChanged = Signal(str, bool)  # id, docked — stowed in the rover's single bay, section 2
    missionFinished = Signal()
    clockTick = Signal(float)

    def __init__(self, timeline_path: Path = DEFAULT_TIMELINE_PATH, parent=None) -> None:
        super().__init__(parent)
        self._timeline = json.loads(Path(timeline_path).read_text())
        self._end_time_s = max(
            kf["t"] for series in self._timeline["units"].values() for kf in series
        )
        self.reset()

    def play(self) -> None:
        self._playing = True
        self._timer.start()

    def pause(self) -> None:
        self._playing = False
        self._timer.stop()

    @property
    def playing(self) -> bool:
        return self._playing

    def set_speed(self, speed: float) -> None:
        self._speed = speed

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def clock_s(self) -> float:
        return self._clock_s

    @property
    def duration_s(self) -> float:
        return self._end_time_s

    def reset(self) -> None:
        self.pause() if getattr(self, "_timer", None) else None
        self._clock_s = 0.0
        self._speed = 1.0
        self._playing = False
        self._current_phase: dict[str, Any] | None = None
        self._emitted_detection_idx: set[int] = set()
        self._emitted_link_idx: set[int] = set()
        self._reactive_seq: LaunchSequencer | None = None
        self._reactive_triggered = False

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)

    def apply_initial_state(self) -> None:
        """Emit t=0 poses, phase, and battery once, so panels have something to
        show before Play is pressed. Call this after connecting to this
        engine's signals, not before; a signal emitted before anything is
        connected to it is simply lost, PySide does not queue it.
        """
        self._apply_at(0.0)

    def _tick(self) -> None:
        self._clock_s += (TICK_MS / 1000.0) * self._speed
        self.clockTick.emit(self._clock_s)
        self._apply_at(self._clock_s)
        if self._clock_s >= self._end_time_s:
            self.pause()
            self.missionFinished.emit()

    def _apply_at(self, t: float) -> None:
        self._advance_phase(t)
        self._emit_unit_poses(t)
        self._emit_new_detections(t)
        self._emit_new_link_events(t)
        self._tick_reactive_sequencer()

    def _advance_phase(self, t: float) -> None:
        markers = self._timeline["phase_markers"]
        current = markers[0]
        for marker in markers:
            if marker["t"] <= t:
                current = marker
            else:
                break

        if current is not self._current_phase:
            self._current_phase = current
            self.phaseChanged.emit(current["phase"], float(current["t"]))
            if current["phase"] == "reactive_launch" and not self._reactive_triggered:
                self._trigger_reactive_launch(current)

    def _emit_unit_poses(self, t: float) -> None:
        for unit_id, keyframes in self._timeline["units"].items():
            pose = _interpolate_series(keyframes, t)
            if pose is not None:
                self.unitPoseChanged.emit(
                    unit_id,
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    float(pose.get("z", 0.0)),
                    float(pose.get("heading", 0.0)),
                )

    def _emit_new_detections(self, t: float) -> None:
        for i, detection in enumerate(self._timeline.get("detections", [])):
            if i not in self._emitted_detection_idx and detection["t"] <= t:
                self._emitted_detection_idx.add(i)
                self.detectionOccurred.emit(detection)

    def _emit_new_link_events(self, t: float) -> None:
        for i, event in enumerate(self._timeline.get("link_events", [])):
            if i not in self._emitted_link_idx and event["t"] <= t:
                self._emitted_link_idx.add(i)
                self.linkEvent.emit(event["event"], event["units"])

    def _trigger_reactive_launch(self, marker: dict[str, Any]) -> None:
        self._reactive_triggered = True
        rover_pose = _interpolate_series(self._timeline["units"]["rover"], marker["t"]) or {}
        origin = Target(rover_pose.get("x", 0.0), rover_pose.get("y", 0.0), rover_pose.get("z", 0.0))

        count = marker.get("count", 2)
        tricopter_ids = [f"tricopter-{i + 1}" for i in range(count)]
        spacing = MIN_AIRCRAFT_SEPARATION_M * 2
        targets = compute_formation_targets(
            origin.x, origin.y, marker.get("bearing", 0.0), "vee", spacing, REACTIVE_RALLY_ALTITUDE_M
        )[:count]

        seq = LaunchSequencer()
        seq.arm()
        seq.stage_formation(tricopter_ids, targets, origin, is_reactive=True)
        seq.launch()
        self._reactive_seq = seq
        self.reactiveLaunchStageChanged.emit(seq.stage.value)
        for tricopter_id in tricopter_ids:
            self.unitDockedChanged.emit(tricopter_id, False)

    def _tick_reactive_sequencer(self) -> None:
        seq = self._reactive_seq
        if seq is None or seq.stage in (LaunchStage.IDLE, LaunchStage.FORMED):
            return
        previous_stage = seq.stage
        seq.tick((TICK_MS / 1000.0) * self._speed)
        if seq.stage != previous_stage:
            self.reactiveLaunchStageChanged.emit(seq.stage.value)
