"""Kinematic motion model: acceleration limits, velocity profiles, no teleporting.

Everything in this build used to move by linear interpolation, which means
a unit reached full speed in one tick, held it exactly, and stopped dead on
arrival. Nothing physical moves like that, and on screen it reads as
sliding rather than flying or driving.

This module gives every unit a velocity and an acceleration budget:

- Horizontal motion ramps velocity toward a target at max_accel, and the
  approach speed is capped by the speed from which the body can still stop
  in the distance remaining (v = sqrt(2*a*d)). That is what produces a
  trapezoidal profile: accelerate out, cruise, decelerate in, arrive at
  zero without overshoot, all without scripting the timing.
- Vertical motion is separate and asymmetric, because a rotorcraft climbs
  faster than it descends safely.
- Yaw has its own rate and angular acceleration limits, and always turns
  the short way around.

The numbers live in constants.py. None of them come from the ARES
technical baseline, which gives endurance, ceiling and top speed but no
acceleration figures, so they are this build's flight model. See
DECISIONS.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


ARRIVAL_TOLERANCE_M = 0.05
ARRIVAL_TOLERANCE_DEG = 1.0


def wrap180(degrees: float) -> float:
    """Signed shortest angular difference, in [-180, 180)."""
    return (degrees + 180.0) % 360.0 - 180.0


def approach_speed(distance_m: float, max_speed_ms: float, max_decel_ms2: float) -> float:
    """Fastest speed from which the body can still stop exactly on target.

    This is the whole trick behind a smooth arrival: rather than scripting
    a deceleration phase, cap the speed by what the remaining distance can
    absorb, and the deceleration falls out of the geometry.
    """
    if distance_m <= 0.0:
        return 0.0
    return min(max_speed_ms, math.sqrt(2.0 * max_decel_ms2 * distance_m))


@dataclass
class MotionLimits:
    max_speed_ms: float
    max_accel_ms2: float
    max_decel_ms2: float
    max_climb_ms: float = 3.0
    max_descend_ms: float = 2.0
    max_vertical_accel_ms2: float = 2.0
    max_yaw_rate_deg_s: float = 90.0
    max_yaw_accel_deg_s2: float = 180.0
    # A wheeled vehicle cannot drive sideways: the more it has to turn, the
    # slower it may go. 0 disables the coupling (a rotorcraft can translate
    # in any direction regardless of which way it points).
    heading_couples_to_speed: bool = False


@dataclass
class KinematicBody:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    limits: MotionLimits = field(
        default_factory=lambda: MotionLimits(max_speed_ms=7.0, max_accel_ms2=3.0, max_decel_ms2=3.0)
    )

    @property
    def speed_ms(self) -> float:
        return math.hypot(self.vx, self.vy)

    def distance_to(self, tx: float, ty: float) -> float:
        return math.hypot(tx - self.x, ty - self.y)

    # --- horizontal ---

    def step_toward(
        self,
        tx: float,
        ty: float,
        dt_s: float,
        speed_scale: float = 1.0,
        feedforward: tuple[float, float] = (0.0, 0.0),
    ) -> bool:
        """Advance one tick toward (tx, ty). Returns True once effectively arrived.

        feedforward is the target's own velocity. Holding station on something
        that is moving with a position controller alone leaves a standing lag,
        because the position error is the only thing generating speed. Feeding
        the leader's velocity forward removes that lag, which is how formation
        flight actually holds a slot instead of trailing it.
        """
        if dt_s <= 0.0:
            return False

        dx, dy = tx - self.x, ty - self.y
        distance = math.hypot(dx, dy)

        max_speed = self.limits.max_speed_ms * max(0.0, min(speed_scale, 1.0))
        if self.limits.heading_couples_to_speed and distance > 1e-6:
            bearing = math.degrees(math.atan2(dx, dy))
            error = abs(wrap180(bearing - self.heading))
            # Full speed ahead, nothing sideways, and a smooth taper between.
            max_speed *= max(0.0, math.cos(math.radians(min(error, 90.0))))

        # Plan against the distance left *after* this tick: the velocity ramp
        # lags by one step, and planning against the current distance lets that
        # lag carry the body past its target.
        lookahead = max(0.0, distance - self.speed_ms * dt_s)
        target_speed = approach_speed(lookahead, max_speed, self.limits.max_decel_ms2)

        if distance > 1e-6:
            desired_vx = dx / distance * target_speed
            desired_vy = dy / distance * target_speed
        else:
            desired_vx = desired_vy = 0.0

        desired_vx += feedforward[0]
        desired_vy += feedforward[1]
        demand = math.hypot(desired_vx, desired_vy)
        if demand > self.limits.max_speed_ms:
            scale = self.limits.max_speed_ms / demand
            desired_vx *= scale
            desired_vy *= scale

        speed_before = self.speed_ms
        self._ramp_horizontal(desired_vx, desired_vy, dt_s)

        # Terminal condition. Within a few centimetres, moving slowly enough to
        # be stopped inside this tick, settle exactly on the target instead of
        # dithering around it: the ramp cannot resolve a fraction of one tick's
        # velocity step, so without this the body creeps forever.
        tracking_moving_target = feedforward[0] or feedforward[1]
        if (
            not tracking_moving_target
            and distance < ARRIVAL_TOLERANCE_M
            and speed_before <= self.limits.max_decel_ms2 * dt_s
        ):
            self.x, self.y = tx, ty
            self.vx = self.vy = 0.0
            return True

        self.x += self.vx * dt_s
        self.y += self.vy * dt_s
        return False

    def _ramp_horizontal(self, desired_vx: float, desired_vy: float, dt_s: float) -> None:
        delta_vx = desired_vx - self.vx
        delta_vy = desired_vy - self.vy
        delta = math.hypot(delta_vx, delta_vy)
        if delta < 1e-12:
            return
        # Slowing down may use the (larger) deceleration budget.
        speeding_up = math.hypot(desired_vx, desired_vy) >= self.speed_ms
        budget = self.limits.max_accel_ms2 if speeding_up else self.limits.max_decel_ms2
        max_delta = budget * dt_s
        if delta > max_delta:
            scale = max_delta / delta
            delta_vx *= scale
            delta_vy *= scale
        self.vx += delta_vx
        self.vy += delta_vy

    def apply_horizontal_command(self, cmd_vx: float, cmd_vy: float, dt_s: float) -> None:
        """Manual stick input expressed as a demanded velocity, still subject
        to the same acceleration budget so manual flying feels like flying."""
        if dt_s <= 0.0:
            return
        speed = math.hypot(cmd_vx, cmd_vy)
        if speed > self.limits.max_speed_ms:
            scale = self.limits.max_speed_ms / speed
            cmd_vx *= scale
            cmd_vy *= scale
        self._ramp_horizontal(cmd_vx, cmd_vy, dt_s)
        self.x += self.vx * dt_s
        self.y += self.vy * dt_s

    # --- vertical ---

    def step_altitude(self, target_z: float, dt_s: float, max_rate_scale: float = 1.0) -> bool:
        if dt_s <= 0.0:
            return False
        delta = target_z - self.z
        climbing = delta > 0.0
        rate_cap = (self.limits.max_climb_ms if climbing else self.limits.max_descend_ms)
        rate_cap *= max(0.0, min(max_rate_scale, 1.0))

        remaining = max(0.0, abs(delta) - abs(self.vz) * dt_s)
        target_rate = math.copysign(
            approach_speed(remaining, rate_cap, self.limits.max_vertical_accel_ms2), delta
        )
        # Settling is judged on the rate going *into* this tick. Judged after
        # the ramp, a body could ramp down by one step and then be zeroed by
        # another, which is twice the acceleration limit in a single tick.
        rate_before = self.vz
        self._ramp_vertical(target_rate, dt_s)
        if abs(delta) < ARRIVAL_TOLERANCE_M and abs(rate_before) <= self.limits.max_vertical_accel_ms2 * dt_s:
            self.z = max(0.0, target_z)
            self.vz = 0.0
            return True
        self.z = max(0.0, self.z + self.vz * dt_s)
        return False

    def _ramp_vertical(self, target_rate: float, dt_s: float) -> None:
        max_delta = self.limits.max_vertical_accel_ms2 * dt_s
        delta = target_rate - self.vz
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        self.vz += delta

    def apply_vertical_command(self, cmd_vz: float, dt_s: float) -> None:
        if dt_s <= 0.0:
            return
        cap = self.limits.max_climb_ms if cmd_vz > 0 else self.limits.max_descend_ms
        cmd_vz = max(-cap, min(cmd_vz, cap))
        self._ramp_vertical(cmd_vz, dt_s)
        self.z = max(0.0, self.z + self.vz * dt_s)

    # --- yaw ---

    def step_heading(self, target_heading: float, dt_s: float) -> bool:
        """Turn toward a heading, the short way, under rate and accel limits."""
        if dt_s <= 0.0:
            return False
        error = wrap180(target_heading - self.heading)
        remaining = max(0.0, abs(error) - abs(self.yaw_rate) * dt_s)
        target_rate = math.copysign(
            approach_speed(
                remaining, self.limits.max_yaw_rate_deg_s, self.limits.max_yaw_accel_deg_s2
            ),
            error,
        )
        self._ramp_yaw(target_rate, dt_s)
        if abs(error) < ARRIVAL_TOLERANCE_DEG and abs(self.yaw_rate) <= self.limits.max_yaw_accel_deg_s2 * dt_s:
            self.heading = target_heading % 360.0
            self.yaw_rate = 0.0
            return True
        self.heading = (self.heading + self.yaw_rate * dt_s) % 360.0
        return False

    def _ramp_yaw(self, target_rate: float, dt_s: float) -> None:
        max_delta = self.limits.max_yaw_accel_deg_s2 * dt_s
        delta = target_rate - self.yaw_rate
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        self.yaw_rate += delta

    def apply_yaw_command(self, cmd_rate_deg_s: float, dt_s: float) -> None:
        if dt_s <= 0.0:
            return
        cap = self.limits.max_yaw_rate_deg_s
        self._ramp_yaw(max(-cap, min(cmd_rate_deg_s, cap)), dt_s)
        self.heading = (self.heading + self.yaw_rate * dt_s) % 360.0

    def face_travel_direction(self, dt_s: float, min_speed_ms: float = 0.5) -> None:
        """Point the nose where the body is actually going, once it is moving
        fast enough for the direction to be meaningful."""
        if self.speed_ms < min_speed_ms:
            return
        self.step_heading(math.degrees(math.atan2(self.vx, self.vy)) % 360.0, dt_s)

    def halt(self, dt_s: float) -> None:
        """Bleed off velocity under the deceleration limit rather than snapping to zero."""
        self._ramp_horizontal(0.0, 0.0, dt_s)
        self._ramp_vertical(0.0, dt_s)
        self._ramp_yaw(0.0, dt_s)
        self.x += self.vx * dt_s
        self.y += self.vy * dt_s
        self.z = max(0.0, self.z + self.vz * dt_s)
        self.heading = (self.heading + self.yaw_rate * dt_s) % 360.0
