"""Reactive obstacle avoidance: what the rover does in the next second.

The route planner decides where to go over the whole of what has been seen. It
runs a few times a second on a grid, and what it produces is a line. A vehicle
is not a point on a line: it has width and a turning rate, it is somewhere
other than where the plan assumed by the time the plan is read, and the plan
may have been made before the thing in front of it was seen. Following a line
into an obstacle because the line said so is the failure this layer exists to
make impossible.

Every tick, from where the rover is and how it is pointing, this looks at the
obstacles near it and picks the heading to drive. The method is the one a
vector-field-histogram planner uses: lay a fan of straight tracks out from the
vehicle, find how far each can be driven before the vehicle's footprint would
touch something mapped, and take the one that gets closest to where the route
wants to go while leaving room. How fast it may drive follows from how far the
chosen track is clear, because a vehicle has to be able to stop inside the
distance it can see to be free.

It only ever refuses to come closer to an obstacle than the footprint allows.
Starting inside that margin, because something was only seen when it was near,
it will leave the way that opens clearance instead of freezing, which is what a
hard stop on its own does.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

#: Nearest the rover's centre may come to the centre of an obstacle cell. The
#: vehicle is 1.15 m by 0.85 m, so turning on the spot it sweeps a circle of
#: 0.72 m; an obstacle cell's own edge is up to a quarter of a metre from its
#: centre; and mapped footprints already err outward. This sits a little inside
#: the route planner's own margin (`navigation.ROVER_RADIUS_M`), so that a route
#: hugging the planner's line does not read as a collision.
FOOTPRINT_CLEARANCE_M = 1.0

#: Clearance below which the track is still driveable but is penalised, so the
#: vehicle takes the middle of a gap over the edge of it.
COMFORT_CLEARANCE_M = 1.5

#: How far each track is examined, and the step it is sampled at (smaller than
#: a cell so nothing is stepped over).
LOOKAHEAD_MIN_M = 2.6
SAMPLE_STEP_M = 0.25

#: Obstacles further than this from the vehicle cannot matter within the
#: lookahead and are not considered.
CONSIDER_RADIUS_M = 10.0

#: The fan of headings tried either side of the one the route wants, degrees.
FAN_STEP_DEG = 10.0
FAN_HALF_WIDTH_DEG = 180.0

#: What a turn away from the route costs, per degree, against what a change
#: from the vehicle's present heading costs. Steering round something should
#: be by the smallest turn that clears it, and not swing between sides.
DEVIATION_COST = 1.0
TURN_COST = 0.25
SIDE_MEMORY_BONUS = 40.0

#: A track shorter than this is not somewhere to go, and what it needs grows
#: with speed: turning takes time, and time is distance.
MIN_USEFUL_FREE_M = 1.2
TURN_LEAD_S = 1.6
SHORT_TRACK_COST = 60.0

#: Margin kept short of the first obstacle when working out a stopping speed.
STOP_MARGIN_M = 0.45


@dataclass
class LocalCommand:
    heading_deg: float
    #: The fastest the rover may go along that heading, m/s.
    speed_limit_ms: float
    #: Clear distance along the chosen heading, and how close the nearest
    #: obstacle is to the vehicle as it stands.
    free_m: float
    clearance_m: float
    #: True if the chosen heading is not the one the route asked for.
    deviating: bool


class LocalPlanner:
    def __init__(self) -> None:
        # Which side an obstacle is being passed on, remembered so that a
        # tie between going left and right is broken once and kept.
        self._side = 0
        self._straight_ticks = 0

    def reset(self) -> None:
        self._side = 0
        self._straight_ticks = 0

    def command(
        self,
        obstacles: np.ndarray,
        x: float,
        y: float,
        heading_deg: float,
        target_bearing_deg: float,
        speed_cap_ms: float,
        decel_ms2: float,
        speed_ms: float | None = None,
    ) -> LocalCommand:
        """The heading to drive and the speed allowed on it.

        `obstacles` is (n, 2) of obstacle cell centres in metres. Tracks are
        examined as far as the vehicle, going at the speed it is going now,
        needs to stop plus a margin. Not at the speed it is allowed: examining
        eight metres of straight track when the route curves round something in
        three makes the route's own heading look blocked, and the vehicle then
        argues with the route instead of following it.
        """
        now = speed_cap_ms if speed_ms is None else speed_ms
        look = max(LOOKAHEAD_MIN_M, now * now / (2.0 * max(decel_ms2, 0.1)) + 1.5)
        near = self._near(obstacles, x, y)
        if near.size == 0:
            self._straight_ticks += 1
            if self._straight_ticks > 25:
                self._side = 0
            return LocalCommand(target_bearing_deg % 360.0, speed_cap_ms, look, CONSIDER_RADIUS_M, False)

        clearance_now = float(np.min(np.hypot(near[:, 0] - x, near[:, 1] - y)))
        # Inside the margin already (seen late, or cut a corner): the rule is
        # to leave, not to freeze, so the track must not close in on anything.
        threshold = FOOTPRINT_CLEARANCE_M if clearance_now >= FOOTPRINT_CLEARANCE_M else max(clearance_now - 0.02, 0.2)

        offsets = np.arange(-FAN_HALF_WIDTH_DEG, FAN_HALF_WIDTH_DEG + 0.1, FAN_STEP_DEG)
        headings = (target_bearing_deg + offsets) % 360.0
        steps = np.arange(1, int(look / SAMPLE_STEP_M) + 1) * SAMPLE_STEP_M  # (K,)
        rad = np.radians(headings)
        # Sample points of every track at once: (T, K, 2).
        px = x + np.sin(rad)[:, None] * steps[None, :]
        py = y + np.cos(rad)[:, None] * steps[None, :]
        # Distance from every sample to the nearest obstacle: (T, K).
        dx = px[..., None] - near[None, None, :, 0]
        dy = py[..., None] - near[None, None, :, 1]
        nearest = np.sqrt(dx * dx + dy * dy).min(axis=2)

        # Free length: up to the first sample that would touch something.
        touching = nearest < threshold
        any_touch = touching.any(axis=2 if touching.ndim == 3 else 1)
        first = np.where(any_touch, touching.argmax(axis=1), steps.size)
        free = first * SAMPLE_STEP_M
        # Narrowest clearance along the part of the track that is free.
        masked = np.where(np.arange(steps.size)[None, :] < first[:, None], nearest, np.inf)
        track_clearance = np.where(first > 0, masked.min(axis=1), 0.0)

        deviation = np.abs(offsets)
        turn = np.abs((headings - heading_deg + 180.0) % 360.0 - 180.0)
        cost = DEVIATION_COST * deviation + TURN_COST * turn
        cost += 18.0 * np.clip(COMFORT_CLEARANCE_M - np.minimum(track_clearance, COMFORT_CLEARANCE_M), 0.0, None)
        # A track that runs out before the lookahead is worth less the sooner it
        # does, steeply enough that steering aside begins while there is still
        # room to turn, not when the obstacle is already at the bumper.
        cost += SHORT_TRACK_COST * np.clip(1.0 - free / look, 0.0, 1.0)
        # Hold the side already chosen.
        if self._side:
            cost -= SIDE_MEMORY_BONUS * (np.sign(offsets) == self._side)

        usable = free >= max(MIN_USEFUL_FREE_M, min(now * TURN_LEAD_S, look - SAMPLE_STEP_M))
        if usable.any():
            index = int(np.argmin(np.where(usable, cost, np.inf)))
        else:
            index = int(np.argmax(free))  # nowhere good: the least bad

        chosen = float(headings[index])
        offset = float(offsets[index])
        if abs(offset) < 1e-6:
            self._straight_ticks += 1
            if self._straight_ticks > 25:
                self._side = 0
        else:
            self._straight_ticks = 0
            if not self._side:
                self._side = 1 if offset > 0 else -1

        free_m = float(free[index])
        # Stop inside what can be seen to be free.
        allowed = math.sqrt(max(0.0, 2.0 * decel_ms2 * max(free_m - STOP_MARGIN_M, 0.0)))
        if free_m >= look - 1e-6:
            allowed = speed_cap_ms
        # Squeezing past: slower the tighter it is.
        pinch = min(float(track_clearance[index]) / COMFORT_CLEARANCE_M, 1.0)
        allowed *= 0.45 + 0.55 * pinch if free_m < look else 1.0
        return LocalCommand(
            chosen, min(allowed, speed_cap_ms), free_m, clearance_now, abs(offset) > 1e-6
        )

    @staticmethod
    def _near(obstacles: np.ndarray, x: float, y: float) -> np.ndarray:
        if obstacles.size == 0:
            return obstacles
        keep = (np.abs(obstacles[:, 0] - x) <= CONSIDER_RADIUS_M) & (np.abs(obstacles[:, 1] - y) <= CONSIDER_RADIUS_M)
        return obstacles[keep]
