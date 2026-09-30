"""Operator-commanded motion after launch: station keeping, altitude
changes, manual flying, recall, return to base, and docking.

The launch sequencer (section 6.3) owns an aircraft from the bay up to
FORMED. From FORMED onward this module owns it, which is what makes the
fleet behave like one system rather than four independent icons:

- Everything moves through ocu.core.motion, so every unit accelerates,
  cruises and decelerates under its own limits. Nothing starts at speed,
  nothing stops dead, nothing teleports.
- A formation slot is held *relative to the rover*, so when the rover
  drives, the formation travels with it.
- No aircraft is ever commanded outside TETHER_LIMIT_M of the rover.
  Section 2 ties the 30-50 m scout radius to the UWB ranging limit, and
  outside it the aircraft lose the relative positioning that precision
  landing depends on.
- Docking waits for a stationary rover and then descends through the four
  stages of section 7, slowing as it closes, with each stage entered when
  the aircraft has actually achieved that stage's precision rather than
  when a timer says so.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from ocu.core.constants import (
    ARES_A_CEILING_M,
    DESCENT_RATE_COARSE_MS,
    DESCENT_RATE_FINE_MS,
    DESCENT_RATE_MECHANICAL_MS,
    DESCENT_RATE_TOUCHDOWN_MS,
    HOLD_ALTITUDE_M,
    RECALL_RING_RADIUS_M,
    RECOVERY_STAGE_2_PRECISION_CM,
    RECOVERY_STAGE_3_PRECISION_CM,
    ROVER_ACCEL_MS2,
    ROVER_DECEL_MS2,
    ROVER_STATIONARY_SPEED_MS,
    ROVER_TOP_SPEED_MS,
    ROVER_TURN_ACCEL_DEG_S2,
    ROVER_TURN_RATE_DEG_S,
    TETHER_LIMIT_M,
    TRICOPTER_ACCEL_MS2,
    TRICOPTER_CLIMB_RATE_MS,
    TRICOPTER_DECEL_MS2,
    TRICOPTER_DESCEND_RATE_MS,
    TRICOPTER_TRANSIT_SPEED_MS,
    TRICOPTER_TURN_ACCEL_DEG_S2,
    TRICOPTER_TURN_RATE_DEG_S,
    TRICOPTER_VERTICAL_ACCEL_MS2,
)
from ocu.core.formation import clamp_to_tether
from ocu.core.motion import KinematicBody, MotionLimits
from ocu.core.recovery_sequencer import RecoveryStage

TOUCHDOWN_ALTITUDE_M = 0.06


def tricopter_limits() -> MotionLimits:
    return MotionLimits(
        max_speed_ms=TRICOPTER_TRANSIT_SPEED_MS,
        max_accel_ms2=TRICOPTER_ACCEL_MS2,
        max_decel_ms2=TRICOPTER_DECEL_MS2,
        max_climb_ms=TRICOPTER_CLIMB_RATE_MS,
        max_descend_ms=TRICOPTER_DESCEND_RATE_MS,
        max_vertical_accel_ms2=TRICOPTER_VERTICAL_ACCEL_MS2,
        max_yaw_rate_deg_s=TRICOPTER_TURN_RATE_DEG_S,
        max_yaw_accel_deg_s2=TRICOPTER_TURN_ACCEL_DEG_S2,
    )


def rover_limits() -> MotionLimits:
    return MotionLimits(
        max_speed_ms=ROVER_TOP_SPEED_MS,
        max_accel_ms2=ROVER_ACCEL_MS2,
        max_decel_ms2=ROVER_DECEL_MS2,
        max_yaw_rate_deg_s=ROVER_TURN_RATE_DEG_S,
        max_yaw_accel_deg_s2=ROVER_TURN_ACCEL_DEG_S2,
        heading_couples_to_speed=True,
    )


class AircraftMode(str, Enum):
    STATION = "STATION"
    MANUAL = "MANUAL"
    RETURNING = "RETURNING"
    HOLDING = "HOLDING"
    DOCKING = "DOCKING"
    DOCKED = "DOCKED"


class RoverMode(str, Enum):
    NORMAL = "NORMAL"
    MANUAL = "MANUAL"
    RETURNING = "RETURNING"
    AT_BASE = "AT_BASE"


@dataclass
class ManualInput:
    """Normalised stick input, each axis in [-1, 1]."""

    forward: float = 0.0
    turn: float = 0.0
    vertical: float = 0.0

    @property
    def active(self) -> bool:
        return bool(self.forward or self.turn or self.vertical)


@dataclass
class AircraftState:
    unit_id: str
    body: KinematicBody
    mode: AircraftMode = AircraftMode.STATION
    slot_dx: float = 0.0
    slot_dy: float = 0.0
    commanded_altitude_m: float = 20.0
    manual: ManualInput = field(default_factory=ManualInput)
    recovery_stage: RecoveryStage = RecoveryStage.APPROACHING

    @property
    def x(self) -> float:
        return self.body.x

    @property
    def y(self) -> float:
        return self.body.y

    @property
    def z(self) -> float:
        return self.body.z

    @property
    def heading(self) -> float:
        return self.body.heading


class FlightDirector:
    def __init__(self) -> None:
        self.aircraft: dict[str, AircraftState] = {}
        self.rover_mode = RoverMode.NORMAL
        self.rover_body = KinematicBody(limits=rover_limits())
        self.rover_manual = ManualInput()
        self._base: tuple[float, float] = (0.0, 0.0)
        self._dock_queue: list[str] = []
        self._rover_speed_ms = 0.0
        self._rover_velocity: tuple[float, float] = (0.0, 0.0)
        self._last_rover_xy: tuple[float, float] | None = None

    # --- ownership ---

    def adopt(
        self, unit_id: str, x: float, y: float, z: float, heading: float, rover_x: float, rover_y: float
    ) -> None:
        slot_x, slot_y = clamp_to_tether(x, y, rover_x, rover_y)
        self.aircraft[unit_id] = AircraftState(
            unit_id=unit_id,
            body=KinematicBody(x=x, y=y, z=z, heading=heading, limits=tricopter_limits()),
            mode=AircraftMode.STATION,
            slot_dx=slot_x - rover_x,
            slot_dy=slot_y - rover_y,
            commanded_altitude_m=max(z, 1.0),
        )

    def release(self, unit_id: str) -> None:
        self.aircraft.pop(unit_id, None)
        if unit_id in self._dock_queue:
            self._dock_queue.remove(unit_id)

    def reset(self) -> None:
        self.aircraft.clear()
        self._dock_queue.clear()
        self.rover_mode = RoverMode.NORMAL
        self.rover_body = KinematicBody(limits=rover_limits())
        self.rover_manual = ManualInput()
        self._rover_speed_ms = 0.0
        self._rover_velocity = (0.0, 0.0)
        self._last_rover_xy = None

    @property
    def airborne_ids(self) -> list[str]:
        return [a.unit_id for a in self.aircraft.values() if a.mode != AircraftMode.DOCKED]

    def owns_rover(self) -> bool:
        return self.rover_mode in (RoverMode.RETURNING, RoverMode.AT_BASE, RoverMode.MANUAL)

    # --- operator commands ---

    def set_commanded_altitude(self, unit_id: str, altitude_m: float) -> None:
        state = self.aircraft.get(unit_id)
        if state is None or state.mode in (AircraftMode.DOCKING, AircraftMode.DOCKED):
            return
        state.commanded_altitude_m = max(1.0, min(altitude_m, ARES_A_CEILING_M))

    def take_manual(self, unit_id: str, rover_x: float, rover_y: float, rover_heading: float) -> bool:
        """Hand a unit to the sticks. Only one at a time is the caller's rule
        (section 4.2); this just switches the mode."""
        if unit_id == "rover":
            if self.rover_mode != RoverMode.MANUAL:
                self.rover_body.x, self.rover_body.y = rover_x, rover_y
                self.rover_body.heading = rover_heading
                self.rover_body.vx = self.rover_body.vy = 0.0
            self.rover_mode = RoverMode.MANUAL
            return True
        state = self.aircraft.get(unit_id)
        if state is None or state.mode in (AircraftMode.DOCKING, AircraftMode.DOCKED):
            return False
        state.mode = AircraftMode.MANUAL
        return True

    def release_manual(self, unit_id: str, rover_x: float, rover_y: float) -> None:
        if unit_id == "rover":
            if self.rover_mode == RoverMode.MANUAL:
                self.rover_mode = RoverMode.NORMAL
                self.rover_manual = ManualInput()
            return
        state = self.aircraft.get(unit_id)
        if state is not None and state.mode == AircraftMode.MANUAL:
            # Hold wherever the operator left it, as a fresh slot on the rover.
            slot_x, slot_y = clamp_to_tether(state.x, state.y, rover_x, rover_y)
            state.slot_dx, state.slot_dy = slot_x - rover_x, slot_y - rover_y
            state.commanded_altitude_m = max(state.z, 1.0)
            state.mode = AircraftMode.STATION
            state.manual = ManualInput()

    def set_manual_input(self, unit_id: str, forward: float, turn: float, vertical: float) -> None:
        clamped = ManualInput(
            forward=max(-1.0, min(forward, 1.0)),
            turn=max(-1.0, min(turn, 1.0)),
            vertical=max(-1.0, min(vertical, 1.0)),
        )
        if unit_id == "rover":
            self.rover_manual = clamped
            return
        state = self.aircraft.get(unit_id)
        if state is not None:
            state.manual = clamped

    def recall(self, unit_ids: list[str] | None = None) -> list[str]:
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
            state.manual = ManualInput()
            recalled.append(unit_id)
        return recalled

    def return_rover_to_base(
        self,
        base_x: float = 0.0,
        base_y: float = 0.0,
        rover_x: float | None = None,
        rover_y: float | None = None,
        rover_heading: float | None = None,
    ) -> list[str]:
        """Drive the rover home, recalling anything still airborne to come with it.

        The rover's position must be seeded here rather than assumed: until
        the director owns the rover its body is only a mirror of whoever does,
        and starting a return from a stale (0, 0) makes it believe it is
        already home and arrive instantly.
        """
        self._base = (base_x, base_y)
        if rover_x is not None and rover_y is not None:
            self.rover_body.x, self.rover_body.y = rover_x, rover_y
            if rover_heading is not None:
                self.rover_body.heading = rover_heading
            self.rover_body.vx = self.rover_body.vy = 0.0
        self.rover_mode = RoverMode.RETURNING
        self.rover_manual = ManualInput()
        return self.recall()

    def cancel_return(self) -> None:
        if self.rover_mode == RoverMode.RETURNING:
            self.rover_mode = RoverMode.NORMAL

    # --- per-tick update ---

    def tick(
        self, dt_s: float, rover_x: float, rover_y: float, rover_heading: float
    ) -> tuple[
        dict[str, tuple[float, float, float, float]], tuple[float, float, float] | None, list[tuple[str, str]]
    ]:
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
        body = self.rover_body

        if self.rover_mode == RoverMode.MANUAL:
            body.apply_yaw_command(self.rover_manual.turn * body.limits.max_yaw_rate_deg_s, dt_s)
            heading_rad = math.radians(body.heading)
            demand = self.rover_manual.forward * body.limits.max_speed_ms
            body.apply_horizontal_command(
                demand * math.sin(heading_rad), demand * math.cos(heading_rad), dt_s
            )
            self._rover_speed_ms = body.speed_ms
            self._rover_velocity = (body.vx, body.vy)
            self._last_rover_xy = (body.x, body.y)
            return (body.x, body.y, body.heading)

        if self.rover_mode == RoverMode.RETURNING:
            target_x, target_y = self._base
            dx, dy = target_x - body.x, target_y - body.y
            if math.hypot(dx, dy) > 0.2:
                body.step_heading(math.degrees(math.atan2(dx, dy)) % 360.0, dt_s)
            arrived = body.step_toward(target_x, target_y, dt_s)
            self._rover_velocity = (body.vx, body.vy)
            self._rover_speed_ms = body.speed_ms
            if arrived:
                self.rover_mode = RoverMode.AT_BASE
                events.append(("rover", "AT_BASE"))
            self._last_rover_xy = (body.x, body.y)
            return (body.x, body.y, body.heading)

        if self.rover_mode == RoverMode.AT_BASE:
            body.halt(dt_s)
            self._rover_speed_ms = body.speed_ms
            self._rover_velocity = (body.vx, body.vy)
            self._last_rover_xy = (body.x, body.y)
            return (body.x, body.y, body.heading)

        # Not owned: observe how fast someone else is moving it, because the
        # docking gate depends on the deck actually being still.
        if self._last_rover_xy is not None:
            vx = (rover_x - self._last_rover_xy[0]) / dt_s
            vy = (rover_y - self._last_rover_xy[1]) / dt_s
            self._rover_velocity = (vx, vy)
            self._rover_speed_ms = math.hypot(vx, vy)
        self._last_rover_xy = (rover_x, rover_y)
        body.x, body.y, body.heading = rover_x, rover_y, rover_heading
        return None

    def _tick_aircraft(
        self, state: AircraftState, dt_s: float, rover_x: float, rover_y: float, events: list
    ) -> None:
        body = state.body

        if state.mode == AircraftMode.MANUAL:
            body.apply_yaw_command(state.manual.turn * body.limits.max_yaw_rate_deg_s, dt_s)
            heading_rad = math.radians(body.heading)
            demand = state.manual.forward * body.limits.max_speed_ms
            body.apply_horizontal_command(
                demand * math.sin(heading_rad), demand * math.cos(heading_rad), dt_s
            )
            rate = state.manual.vertical * (
                body.limits.max_climb_ms if state.manual.vertical > 0 else body.limits.max_descend_ms
            )
            body.apply_vertical_command(rate, dt_s)
            # The tether is a hard limit even on the sticks.
            clamped_x, clamped_y = clamp_to_tether(body.x, body.y, rover_x, rover_y)
            if (clamped_x, clamped_y) != (body.x, body.y):
                body.x, body.y = clamped_x, clamped_y
                body.vx = body.vy = 0.0
            state.commanded_altitude_m = max(body.z, 1.0)
            return

        if state.mode == AircraftMode.STATION:
            target_x, target_y = clamp_to_tether(
                rover_x + state.slot_dx, rover_y + state.slot_dy, rover_x, rover_y
            )
            body.step_toward(target_x, target_y, dt_s, feedforward=self._rover_velocity)
            body.step_altitude(state.commanded_altitude_m, dt_s)
            body.face_travel_direction(dt_s)
            return

        if state.mode == AircraftMode.RETURNING:
            target_x, target_y = self._ring_slot(state.unit_id, rover_x, rover_y)
            body.step_heading(
                math.degrees(math.atan2(rover_x - body.x, rover_y - body.y)) % 360.0, dt_s
            )
            arrived = body.step_toward(target_x, target_y, dt_s)
            settled = body.step_altitude(HOLD_ALTITUDE_M, dt_s)
            if arrived and settled:
                state.mode = AircraftMode.HOLDING
                if state.unit_id not in self._dock_queue:
                    self._dock_queue.append(state.unit_id)
                events.append((state.unit_id, "HOLDING"))
            return

        if state.mode == AircraftMode.HOLDING:
            target_x, target_y = self._ring_slot(state.unit_id, rover_x, rover_y)
            body.step_toward(target_x, target_y, dt_s)
            body.step_altitude(HOLD_ALTITUDE_M, dt_s)
            return

        if state.mode == AircraftMode.DOCKING:
            self._tick_docking(state, dt_s, rover_x, rover_y, events)

    def _tick_docking(
        self, state: AircraftState, dt_s: float, rover_x: float, rover_y: float, events: list
    ) -> None:
        """Four-layer recovery, section 7. The stage is whatever precision the
        aircraft has actually achieved, and the descent rate falls with it, so
        3 cm is reached by flying carefully rather than by a timer expiring."""
        body = state.body
        body.step_toward(rover_x, rover_y, dt_s)
        lateral_error = math.hypot(body.x - rover_x, body.y - rover_y)

        stage = self._stage_for(lateral_error, body.z)
        if stage != state.recovery_stage:
            state.recovery_stage = stage
            events.append((state.unit_id, f"RECOVERY:{stage.value}"))

        descent_rate = {
            RecoveryStage.APPROACHING: DESCENT_RATE_COARSE_MS,
            RecoveryStage.COARSE: DESCENT_RATE_COARSE_MS,
            RecoveryStage.FINE: DESCENT_RATE_FINE_MS,
            RecoveryStage.MECHANICAL: DESCENT_RATE_MECHANICAL_MS,
            RecoveryStage.DOCKED: DESCENT_RATE_TOUCHDOWN_MS,
        }[stage]
        scale = min(1.0, descent_rate / max(body.limits.max_descend_ms, 1e-6))
        body.step_altitude(0.0, dt_s, max_rate_scale=scale)

        if body.z <= TOUCHDOWN_ALTITUDE_M and lateral_error <= RECOVERY_STAGE_3_PRECISION_CM / 100.0:
            body.x, body.y, body.z = rover_x, rover_y, 0.0
            body.vx = body.vy = body.vz = 0.0
            state.mode = AircraftMode.DOCKED
            state.recovery_stage = RecoveryStage.DOCKED
            if state.unit_id in self._dock_queue:
                self._dock_queue.remove(state.unit_id)
            events.append((state.unit_id, "DOCKED"))

    @staticmethod
    def _stage_for(lateral_error_m: float, altitude_m: float) -> RecoveryStage:
        if lateral_error_m <= RECOVERY_STAGE_3_PRECISION_CM / 100.0 and altitude_m <= 0.6:
            return RecoveryStage.MECHANICAL
        if lateral_error_m <= RECOVERY_STAGE_2_PRECISION_CM / 100.0 and altitude_m <= 2.5:
            return RecoveryStage.FINE
        return RecoveryStage.COARSE

    def _ring_slot(self, unit_id: str, rover_x: float, rover_y: float) -> tuple[float, float]:
        ordered = sorted(self.aircraft)
        index = ordered.index(unit_id) if unit_id in ordered else 0
        angle = math.radians(index * (360.0 / max(len(ordered), 1)))
        return (
            rover_x + RECALL_RING_RADIUS_M * math.sin(angle),
            rover_y + RECALL_RING_RADIUS_M * math.cos(angle),
        )

    def _service_dock_queue(self, events: list) -> None:
        if not self._dock_queue:
            return
        if self._rover_speed_ms > ROVER_STATIONARY_SPEED_MS:
            return
        if any(a.mode == AircraftMode.DOCKING for a in self.aircraft.values()):
            return

        next_id = self._dock_queue[0]
        state = self.aircraft.get(next_id)
        if state is None:
            self._dock_queue.pop(0)
            return
        if state.mode == AircraftMode.HOLDING:
            state.mode = AircraftMode.DOCKING
            state.recovery_stage = RecoveryStage.COARSE
            events.append((next_id, "DOCKING"))

    # --- introspection for the UI ---

    def mode_of(self, unit_id: str) -> str:
        state = self.aircraft.get(unit_id)
        if state is None:
            return "—"
        if state.mode == AircraftMode.DOCKING:
            return f"DOCKING · {state.recovery_stage.value}"
        return state.mode.value

    def commanded_altitude(self, unit_id: str) -> float | None:
        state = self.aircraft.get(unit_id)
        return state.commanded_altitude_m if state else None

    def speed_of(self, unit_id: str) -> float | None:
        if unit_id == "rover":
            return self.rover_body.speed_ms
        state = self.aircraft.get(unit_id)
        return state.body.speed_ms if state else None

    def tether_distance(self, unit_id: str, rover_x: float, rover_y: float) -> float | None:
        state = self.aircraft.get(unit_id)
        if state is None:
            return None
        return math.hypot(state.x - rover_x, state.y - rover_y)
