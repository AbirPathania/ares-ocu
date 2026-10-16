"""QTimer driven mission clock and phase narration, section 11.2.

This engine keeps the mission clock and walks the phase markers in
mission_timeline.json (drive and scan, reactive launch, aerial recon,
fusion, alert), announcing each as it is reached. It is the mission plan's
schedule, not the world: where the rover is, where the aircraft are, what
the detection suite finds and how the batteries stand are all computed by
the simulation (flight director, LiDAR, detection suite, power model) and
no longer read from this file.

The "reactive_launch" marker is the point in the plan at which the rover
is to commit scouts. This engine only announces that the point has been
reached (reactiveLaunchRequested); *how* the launch goes is the rover's
business. It reads its own map to decide which way to look, and the
aircraft fly there under the same physics as every other aircraft.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal

TICK_MS = 100
DEFAULT_TIMELINE_PATH = Path(__file__).parent / "mission_timeline.json"
REACTIVE_RALLY_ALTITUDE_M = 20.0


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
    reactiveLaunchRequested = Signal(int)  # how many scouts the plan commits
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
        self._emit_new_link_events(t)

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

    def _emit_new_link_events(self, t: float) -> None:
        for i, event in enumerate(self._timeline.get("link_events", [])):
            if i not in self._emitted_link_idx and event["t"] <= t:
                self._emitted_link_idx.add(i)
                self.linkEvent.emit(event["event"], event["units"])

    def _trigger_reactive_launch(self, marker: dict[str, Any]) -> None:
        self._reactive_triggered = True
        self.reactiveLaunchRequested.emit(int(marker.get("count", 2)))
