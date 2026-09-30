"""Operator-commanded motion after launch: station keeping, altitude
changes, recall, return to base, and docking.

The launch sequencer (section 6.3) owns an aircraft from the bay up to
FORMED. From FORMED onward this module owns it, which is what makes the
fleet behave like one system rather than four independent icons:

- A formation slot is held *relative to the rover*, so when the rover
  drives, the formation travels with it.
- No aircraft is ever commanded outside TETHER_LIMIT_M of the rover.
  Section 2 ties the 30-50 m scout radius to the UWB ranging limit, and
  outside it the aircraft lose the relative positioning that precision
  landing depends on. Commands beyond the tether are clamped along the
  same bearing, not rejected.
- Docking waits for a stationary rover. The ArUco fiducial and the funnel
  pad in section 7 are both fixed to the deck, so the deck has to stop
  moving before stage 2 can mean anything. Return-to-base therefore
  recalls the aircraft, drives the rover home, stops it, and only then
  works through the docking queue one aircraft at a time.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from ocu.core.constants import (
    ARES_A_CEILING_M,
    HOLD_ALTITUDE_M,
    RECALL_RING_RADIUS_M,
    ROVER_STATIONARY_SPEED_MS,
    ROVER_TOP_SPEED_MS,
    STATION_KEEP_TOLERANCE_M,
    TETHER_LIMIT_M,
    TRICOPTER_CLIMB_RATE_MS,
    TRICOPTER_TRANSIT_SPEED_MS,
    TRICOPTER_TURN_RATE_DEG_S,
)
from ocu.core.formation import clamp_to_tether
from ocu.core.recovery_sequencer import RecoverySequencer, RecoveryStage


class AircraftMode(str, Enum):
    STATION = "STATION"
    RETURNING = "RETURNING"
    HOLDING = "HOLDING"
    DOCKING = "DOCKING"
    DOCKED = "DOCKED"


class RoverMode(str, Enum):
    NORMAL = "NORMAL"
    RETURNING = "RETURNING"
    AT_BASE = "AT_BASE"


@dataclass
class AircraftState:
    unit_id: str
    mode: AircraftMode = AircraftMode.STATION
    slot_dx: float = 0.0  # formation slot, as an offset from the rover
    slot_dy: float = 0.0
    commanded_altitude_m: float = 20.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    recovery: RecoverySequencer | None = None


def _turn_toward(current: float, target: float, dt_s: float) -> float:
    difference = (target - current + 180.0) % 360.0 - 180.0
    max_step = TRICOPTER_TURN_RATE_DEG_S * dt_s
    if abs(difference) <= max_step:
        return target % 360.0
    return (current + math.copysign(max_step, difference)) % 360.0


def _move_toward(
    x: float, y: float, tx: float, ty: float, speed_ms: float, dt_s: float
) -> tuple[float, float, bool]:
    dx, dy = tx - x, ty - y
    distance = math.hypot(dx, dy)
    step = speed_ms * dt_s
    if distance <= max(step, 1e-9):
        return tx, ty, True
    return x + dx / distance * step, y + dy / distance * step, False


def _climb_toward(z: float, target_z: float, dt_s: float) -> float:
    delta = target_z - z
    step = TRICOPTER_CLIMB_RATE_MS * dt_s
    if abs(delta) <= step:
        return target_z
    return z + math.copysign(step, delta)


class FlightDirector:
    def __init__(self) -> None:
        self.aircraft: dict[str, AircraftState] = {}
        self.rover_mode = RoverMode.NORMAL
        self.rover_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, heading
        self._base: tuple[float, float] = (0.0, 0.0)
        self._dock_queue: list[str] = []
        self._rover_speed_ms = 0.0
        self._last_rover_xy: tuple[float, float] | None = None

    # --- ownership ---

    def adopt(
        self,
        unit_id: str,
        x: float,
        y: float,
        z: float,
        heading: float,
        rover_x: float,
        rover_y: float,
    ) -> None:
        """Take over an aircraft at the end of its launch, holding wherever it
        ended up as a slot offset from the rover."""
        dx, dy = clamp_to_tether(x, y, rover_x, rover_y)
        self.aircraft[unit_id] = AircraftState(
            unit_id=unit_id,
            mode=AircraftMode.STATION,
            slot_dx=dx - rover_x,
            slot_dy=dy - rover_y,
            commanded_altitude_m=max(z, 1.0),
            x=x,
            y=y,
            z=z,
            heading=heading,
        )

    def release(self, unit_id: str) -> None:
        self.aircraft.pop(unit_id, None)
        if unit_id in self._dock_queue:
            self._dock_queue.remove(unit_id)

    def reset(self) -> None:
        self.aircraft.clear()
        self._dock_queue.clear()
        self.rover_mode = RoverMode.NORMAL
        self._rover_speed_ms = 0.0
        self._last_rover_xy = None

    @property
    def airborne_ids(self) -> list[str]:
        return [a.unit_id for a in self.aircraft.values() if a.mode != AircraftMode.DOCKED]

    def owns_rover(self) -> bool:
        return self.rover_mode in (RoverMode.RETURNING, RoverMode.AT_BASE)

    # --- operator commands ---

    def set_commanded_altitude(self, unit_id: str, altitude_m: float) -> None:
        """In-flight altitude change, allowed while the aircraft is flying its
        slot or transiting home, but not once it has committed to a dock."""
        state = self.aircraft.get(unit_id)
        if state is None or state.mode in (AircraftMode.DOCKING, AircraftMode.DOCKED):
            return
        state.commanded_altitude_m = max(1.0, min(altitude_m, ARES_A_CEILING_M))

    def recall(self, unit_ids: list[str] | None = None) -> list[str]:
        """Send aircraft home. They fly to a holding ring above the rover and
        wait there; docking only begins once the rover is stationary."""
        targets = unit_ids if unit_ids is not None else self.airborne_ids
        recalled = []
        for unit_id in targets:
            state = self.aircraft.get(unit_id)
            if state is None or state.mode in (
                AircraftMode.DOCKING,
                AircraftMode.DOCKED,
                AircraftMode.RETURNING,
                AircraftMode.HOLDING,
            ):
                continue
            state.mode = AircraftMode.RETURNING
            recalled.append(unit_id)
        return recalled

    def return_rover_to_base(self, base_x: float = 0.0, base_y: float = 0.0) -> list[str]:
        """Return to base. Any aircraft still out is recalled at the same time
        and travels home with the rover, because it cannot dock to a moving
        deck and must not be left outside the tether."""
        self._base = (base_x, base_y)
        self.rover_mode = RoverMode.RETURNING
        return self.recall()

    def cancel_return(self) -> None:
        if self.rover_mode == RoverMode.RETURNING:
            self.rover_mode = RoverMode.NORMAL

    # --- per-tick update ---

    def tick(
        self, dt_s: float, rover_x: float, rover_y: float, rover_heading: float
    ) -> tuple[dict[str, tuple[float, float, float, float]], tuple[float, float, float] | None, list[tuple[str, str]]]:
        """Advance every owned unit.

        Returns (aircraft poses, rover pose or None if not owned, events).
        The caller passes the rover's current position whoever owns it, so
        station keeping tracks a scripted rover just as well as one the
        director is driving itself.
        """
        events: list[tuple[str, str]] = []
        if dt_s <= 0:
            return {}, None, events

        rover_pose = self._update_rover(dt_s, rover_x, rover_y, rover_heading, events)
        effective_x, effective_y = (rover_pose[0], rover_pose[1]) if rover_pose else (rover_x, rover_y)

        poses: dict[str, tuple[float, float, float, float]] = {}
        for state in self.aircraft.values():
            if state.mode == AircraftMode.DOCKED:
                continue
            self._tick_aircraft(state, dt_s, effective_x, effective_y, events)
            poses[state.unit_id] = (state.x, state.y, state.z, state.heading)

        self._service_dock_queue(events)
        return poses, rover_pose, events

    def _update_rover(
        self, dt_s: float, rover_x: float, rover_y: float, rover_heading: float, events: list
    ) -> tuple[float, float, float] | None:
        if self.rover_mode == RoverMode.RETURNING:
            target_x, target_y = self._base
            dx, dy = target_x - rover_x, target_y - rover_y
            heading = math.degrees(math.atan2(dx, dy)) % 360.0 if (dx or dy) else rover_heading
            new_x, new_y, arrived = _move_toward(
                rover_x, rover_y, target_x, target_y, ROVER_TOP_SPEED_MS, dt_s
            )
            self._rover_speed_ms = 0.0 if arrived else ROVER_TOP_SPEED_MS
            if arrived:
                self.rover_mode = RoverMode.AT_BASE
                events.append(("rover", "AT_BASE"))
            self.rover_pose = (new_x, new_y, heading)
            self._last_rover_xy = (new_x, new_y)
            return self.rover_pose

        if self.rover_mode == RoverMode.AT_BASE:
            self._rover_speed_ms = 0.0
            self.rover_pose = (rover_x, rover_y, rover_heading)
            self._last_rover_xy = (rover_x, rover_y)
            return self.rover_pose

        # Not owned: observe how fast someone else is moving it, because the
        # docking gate depends on the deck actually being still.
        if self._last_rover_xy is not None:
            moved = math.hypot(rover_x - self._last_rover_xy[0], rover_y - self._last_rover_xy[1])
            self._rover_speed_ms = moved / dt_s
        self._last_rover_xy = (rover_x, rover_y)
        self.rover_pose = (rover_x, rover_y, rover_heading)
        return None

    def _tick_aircraft(
        self, state: AircraftState, dt_s: float, rover_x: float, rover_y: float, events: list
    ) -> None:
        if state.mode == AircraftMode.STATION:
            target_x, target_y = clamp_to_tether(
                rover_x + state.slot_dx, rover_y + state.slot_dy, rover_x, rover_y
            )
            state.x, state.y, _ = _move_toward(
                state.x, state.y, target_x, target_y, TRICOPTER_TRANSIT_SPEED_MS, dt_s
            )
            state.z = _climb_toward(state.z, state.commanded_altitude_m, dt_s)
            return

        if state.mode == AircraftMode.RETURNING:
            index = self._ring_index(state.unit_id)
            angle = math.radians(index * (360.0 / max(len(self.aircraft), 1)))
            target_x = rover_x + RECALL_RING_RADIUS_M * math.sin(angle)
            target_y = rover_y + RECALL_RING_RADIUS_M * math.cos(angle)
            bearing_to_rover = math.degrees(math.atan2(rover_x - state.x, rover_y - state.y)) % 360.0
            state.heading = _turn_toward(state.heading, bearing_to_rover, dt_s)
            state.x, state.y, arrived = _move_toward(
                state.x, state.y, target_x, target_y, TRICOPTER_TRANSIT_SPEED_MS, dt_s
            )
            state.z = _climb_toward(state.z, HOLD_ALTITUDE_M, dt_s)
            if arrived and abs(state.z - HOLD_ALTITUDE_M) < STATION_KEEP_TOLERANCE_M:
                state.mode = AircraftMode.HOLDING
                if state.unit_id not in self._dock_queue:
                    self._dock_queue.append(state.unit_id)
                events.append((state.unit_id, "HOLDING"))
            return

        if state.mode == AircraftMode.HOLDING:
            index = self._ring_index(state.unit_id)
            angle = math.radians(index * (360.0 / max(len(self.aircraft), 1)))
            target_x = rover_x + RECALL_RING_RADIUS_M * math.sin(angle)
            target_y = rover_y + RECALL_RING_RADIUS_M * math.cos(angle)
            state.x, state.y, _ = _move_toward(
                state.x, state.y, target_x, target_y, TRICOPTER_TRANSIT_SPEED_MS, dt_s
            )
            state.z = _climb_toward(state.z, HOLD_ALTITUDE_M, dt_s)
            return

        if state.mode == AircraftMode.DOCKING:
            if state.recovery is None:
                state.recovery = RecoverySequencer()
                state.recovery.recall()
            previous = state.recovery.stage
            state.recovery.tick(dt_s)
            if state.recovery.stage != previous:
                events.append((state.unit_id, f"RECOVERY:{state.recovery.stage.value}"))

            state.x, state.y, _ = _move_toward(
                state.x, state.y, rover_x, rover_y, TRICOPTER_TRANSIT_SPEED_MS, dt_s
            )
            if state.recovery.stage == RecoveryStage.DOCKED:
                state.x, state.y, state.z = rover_x, rover_y, 0.0
                state.mode = AircraftMode.DOCKED
                if state.unit_id in self._dock_queue:
                    self._dock_queue.remove(state.unit_id)
                events.append((state.unit_id, "DOCKED"))
            else:
                # Descend in step with the recovery stages rather than after them.
                fraction = state.recovery.elapsed_s / max(state.recovery.elapsed_s + 1e-6, 1e-6)
                state.z = _climb_toward(state.z, 0.0, dt_s * (1.0 + fraction))

    def _ring_index(self, unit_id: str) -> int:
        ordered = sorted(self.aircraft)
        return ordered.index(unit_id) if unit_id in ordered else 0

    def _service_dock_queue(self, events: list) -> None:
        """One aircraft docks at a time, and only onto a stationary deck."""
        if not self._dock_queue:
            return
        if self._rover_speed_ms > ROVER_STATIONARY_SPEED_MS:
            return
        already_docking = any(a.mode == AircraftMode.DOCKING for a in self.aircraft.values())
        if already_docking:
            return

        next_id = self._dock_queue[0]
        state = self.aircraft.get(next_id)
        if state is None:
            self._dock_queue.pop(0)
            return
        if state.mode == AircraftMode.HOLDING:
            state.mode = AircraftMode.DOCKING
            state.recovery = RecoverySequencer()
            state.recovery.recall()
            events.append((next_id, "DOCKING"))

    # --- introspection for the UI ---

    def mode_of(self, unit_id: str) -> str:
        state = self.aircraft.get(unit_id)
        return state.mode.value if state else "—"

    def commanded_altitude(self, unit_id: str) -> float | None:
        state = self.aircraft.get(unit_id)
        return state.commanded_altitude_m if state else None

    def tether_distance(self, unit_id: str, rover_x: float, rover_y: float) -> float | None:
        state = self.aircraft.get(unit_id)
        if state is None:
            return None
        return math.hypot(state.x - rover_x, state.y - rover_y)
