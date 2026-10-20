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

import copy
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
    MIN_AIRCRAFT_SEPARATION_M,
    RECALL_RING_RADIUS_M,
    RECOVERY_STAGE_2_PRECISION_CM,
    RECOVERY_STAGE_3_PRECISION_CM,
    ROVER_ACCEL_MS2,
    ROVER_DECEL_MS2,
    ROVER_STATIONARY_SPEED_MS,
    ROVER_MIN_COMMAND_SPEED_MS,
    ROVER_SURVEY_SPEED_MS,
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
from ocu.core.navigation import (
    LOOKAHEAD_M,
    plan_toward,
    TIGHT_LOOKAHEAD_M,
    WAYPOINT_TOLERANCE_M,
    Corridor,
    build_costmap,
    followed_line_is_clear,
    path_is_clear,
    plan_path,
    pure_pursuit_target,
    reachable_goal,
    smooth,
    update_costmap,
)
from ocu.core.motion import KinematicBody, MotionLimits
from ocu.core.recovery_sequencer import RecoveryStage

#: A failed plan is retried at 1 Hz, not at tick rate. The map is still
#: changing so retrying is right, but a search that cannot succeed is the
#: most expensive search there is.
PLAN_RETRY_TICKS = 10

#: While exploring toward a goal the route is only as long as what has been
#: seen, so it is made again when the vehicle is this close to the end of it,
#: or this often, whichever comes first.
EXPLORE_REPLAN_NEAR_END_M = 5.0
EXPLORE_REPLAN_TICKS = 20

#: After this many consecutive failures the waypoint is called unreachable
#: and the operator is told, rather than the vehicle holding in silence.
MAX_PLAN_FAILURES = 3

#: Largest step the motion model is integrated over. Playing the mission back
#: faster makes each tick cover more mission time, and a follower that re-aims
#: once per step tracks its path more loosely the longer the step gets.
#: Splitting anything longer keeps one long tick equivalent to the same mission
#: time taken in short ones, which is what makes playback speed a choice about
#: when you watch rather than about what the vehicles do.
MAX_INTEGRATION_STEP_S = 0.1

def to_body_frame(dx: float, dy: float, heading_deg: float) -> tuple[float, float]:
    """A map offset, expressed in the vehicle's own frame.

    Compass convention: forward is (sin h, cos h) and right is 90 degrees
    clockwise of it, which is the same frame formation.py builds slots in.
    """
    heading = math.radians(heading_deg)
    forward = dx * math.sin(heading) + dy * math.cos(heading)
    right = dx * math.cos(heading) - dy * math.sin(heading)
    return forward, right


def to_map_frame(forward: float, right: float, heading_deg: float) -> tuple[float, float]:
    """The inverse: where a place on the vehicle currently is on the map."""
    heading = math.radians(heading_deg)
    dx = forward * math.sin(heading) + right * math.cos(heading)
    dy = forward * math.cos(heading) - right * math.sin(heading)
    return dx, dy


TOUCHDOWN_ALTITUDE_M = 0.06

#: An aircraft counts as on its slot when it is this close to it, this close
#: to its altitude, and moving this slowly relative to the rover. Looser than
#: the dock tolerance because a slot is a place to hold, not a place to land.
SLOT_TOLERANCE_M = 0.8
SLOT_ALTITUDE_TOLERANCE_M = 0.6
SLOT_SETTLED_SPEED_MS = 0.7

#: Above this speed relative to the rover an aircraft faces where it is
#: going; at or below it, the formation bearing. Pointing the nose along a
#: velocity that is only wobble gives a heading that jitters.
COURSE_FACING_SPEED_MS = 1.5

#: An aircraft climbing out of the bay holds its altitude rather than climb
#: into one still above it and close by. The minimum separation (6.2) is a
#: hard limit, and the launch is where it is most at risk: three aircraft
#: leave one spot within seconds of each other. The guard is only against
#: aircraft that are themselves still launching, so it can never deadlock
#: on one that is simply holding its slot.
CLIMB_GUARD_VERTICAL_M = MIN_AIRCRAFT_SEPARATION_M + 1.0
CLIMB_GUARD_HORIZONTAL_M = MIN_AIRCRAFT_SEPARATION_M * 1.6


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
    # The launch, flown rather than animated: waiting in the bay for its turn,
    # straight up to altitude, then across to its slot, then holding it.
    PREFLIGHT = "PREFLIGHT"
    CLIMBING = "CLIMBING"
    TRANSITING = "TRANSITING"
    STATION = "STATION"
    MANUAL = "MANUAL"
    RETURNING = "RETURNING"
    HOLDING = "HOLDING"
    DOCKING = "DOCKING"
    DOCKED = "DOCKED"


class RoverMode(str, Enum):
    NORMAL = "NORMAL"
    NAVIGATING = "NAVIGATING"
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
    #: Where this aircraft sits relative to the rover, in the rover's own
    #: frame: forward along its nose, right across it. Held this way rather
    #: than as a map offset because a screen flown for a vehicle belongs
    #: ahead of that vehicle, and a vehicle that turns takes its screen with
    #: it. Held as a map offset, the formation stays pinned to a compass
    #: bearing and ends up beside or behind the rover the moment it turns.
    slot_forward: float = 0.0
    slot_right: float = 0.0
    commanded_altitude_m: float = 20.0
    #: Compass heading the aircraft faces while holding its slot. Every
    #: aircraft in a formation faces the same way, so the group presents one
    #: sensor direction rather than three.
    #: Which way the aircraft faces while holding station, relative to the
    #: rover's nose. Same reason: the group's sensors point where the vehicle
    #: is going, not at a fixed compass bearing.
    bearing_deg: float = 0.0
    #: Seconds until lift-off, while waiting in the bay.
    liftoff_in_s: float = 0.0
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
        #: How fast the operator has told the rover to drive. A ceiling on
        #: every mode: the sweep, a route to a destination, the way home and
        #: the sticks all go no faster than this. It is the operator's
        #: setting, so a reset leaves it where they put it.
        self.commanded_speed_ms = ROVER_SURVEY_SPEED_MS
        self._base: tuple[float, float] = (0.0, 0.0)
        self._dock_queue: list[str] = []
        self._rover_speed_ms = 0.0
        self._rover_velocity: tuple[float, float] = (0.0, 0.0)
        self._last_rover_xy: tuple[float, float] | None = None
        self.path: list[tuple[float, float]] = []
        # The dense cell path is kept alongside the simplified world path:
        # simplify() collapses a straight run to its two endpoints, so
        # checking only those would miss an obstacle discovered in the
        # middle of it and the rover would drive straight into the thing
        # it had just mapped.
        self._path_cells: list[tuple[int, int]] = []
        self.goal: tuple[float, float] | None = None
        #: Set while the operator has the mission on hold: the rover stops
        #: where it is, and the aircraft keep station on it.
        self.hold_navigation = False
        self.lane_from: tuple[float, float] | None = None
        self._costmap = None
        self._plan_failures = 0
        self._ticks_since_attempt = 0
        self.plan_failed = False
        self._replans = 0
        #: Plan only over ground that has been seen. See `navigate_to`.
        self.explore = True
        #: Whether the route being driven reaches the goal over seen ground.
        #: False while the vehicle is still working out the way: the path is
        #: then only the part it knows, ending at the place it is going to look
        #: from next.
        self.route_known = False
        self._explore_visited: list[tuple[float, float]] = []
        self._explore_ticks = 0

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
        rover_heading: float = 0.0,
    ) -> None:
        slot_x, slot_y = clamp_to_tether(x, y, rover_x, rover_y)
        self.aircraft[unit_id] = AircraftState(
            unit_id=unit_id,
            body=KinematicBody(x=x, y=y, z=z, heading=heading, limits=tricopter_limits()),
            mode=AircraftMode.STATION,
            **dict(
                zip(
                    ("slot_forward", "slot_right"),
                    to_body_frame(slot_x - rover_x, slot_y - rover_y, rover_heading),
                )
            ),
            commanded_altitude_m=max(z, 1.0),
            bearing_deg=(heading - rover_heading) % 360.0,
        )

    def launch(
        self,
        unit_id: str,
        slot_forward: float,
        slot_right: float,
        altitude_m: float,
        bearing_deg: float,
        rover_x: float,
        rover_y: float,
        rover_heading: float,
        liftoff_in_s: float = 0.0,
    ) -> None:
        """Start an aircraft's launch from the rover's bay.

        The slot is an offset *from the rover*, not a place in the world. A
        formation is a shape held on a vehicle that is moving, so the
        target has to move with it; a world position worked out at launch
        is wrong the moment the rover has driven on.

        Until its turn the aircraft rides in the bay with the rover. Then it
        climbs straight up to its own altitude over the rover, which is also
        what deconflicts the three of them, then flies across to its slot.
        All of it through the same acceleration limits as everything else
        that flies here.
        """
        dx, dy = to_map_frame(slot_forward, slot_right, rover_heading)
        slot_x, slot_y = clamp_to_tether(rover_x + dx, rover_y + dy, rover_x, rover_y)
        held_forward, held_right = to_body_frame(
            slot_x - rover_x, slot_y - rover_y, rover_heading
        )
        self.aircraft[unit_id] = AircraftState(
            unit_id=unit_id,
            body=KinematicBody(x=rover_x, y=rover_y, z=0.0, heading=rover_heading, limits=tricopter_limits()),
            mode=AircraftMode.PREFLIGHT,
            slot_forward=held_forward,
            slot_right=held_right,
            commanded_altitude_m=max(altitude_m, 1.0),
            bearing_deg=bearing_deg % 360.0,
            liftoff_in_s=max(liftoff_in_s, 0.0),
        )

    def launch_stage(self, unit_ids: list[str]) -> str:
        """Where a launch of these aircraft has got to, read from the aircraft.

        The stage is what the aircraft are doing, not a timer running beside
        them. A launch is only FORMED when every one of them is actually
        holding its slot, which is also why it can take as long as the
        physics takes rather than as long as an animation was set to.
        """
        # An aircraft that has been recalled, taken on the sticks or docked is
        # no longer part of the launch, whatever it is doing now.
        in_launch = (
            AircraftMode.PREFLIGHT,
            AircraftMode.CLIMBING,
            AircraftMode.TRANSITING,
            AircraftMode.STATION,
        )
        modes = [
            self.aircraft[u].mode
            for u in unit_ids
            if u in self.aircraft and self.aircraft[u].mode in in_launch
        ]
        if not modes:
            return "IDLE"
        if all(m == AircraftMode.STATION for m in modes):
            return "FORMED"
        if any(m == AircraftMode.TRANSITING for m in modes):
            return "TRANSLATING"
        if any(m == AircraftMode.CLIMBING for m in modes):
            return "CLIMBING"
        return "LAUNCHING"

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
        self.path = []
        self._path_cells = []
        self.goal = None
        self.hold_navigation = False
        self.lane_from = None
        self.route_known = False
        self._explore_visited = []
        self._explore_ticks = 0
        self._replans = 0
        self._costmap = None
        self._plan_failures = 0
        self._ticks_since_attempt = 0
        self.plan_failed = False

    @property
    def airborne_ids(self) -> list[str]:
        return [
            a.unit_id
            for a in self.aircraft.values()
            if a.mode not in (AircraftMode.DOCKED, AircraftMode.PREFLIGHT)
        ]

    def owns_rover(self) -> bool:
        return self.rover_mode in (
            RoverMode.RETURNING,
            RoverMode.AT_BASE,
            RoverMode.MANUAL,
            RoverMode.NAVIGATING,
        )

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

    def release_manual(
        self, unit_id: str, rover_x: float, rover_y: float, rover_heading: float = 0.0
    ) -> None:
        if unit_id == "rover":
            if self.rover_mode == RoverMode.MANUAL:
                self.rover_mode = RoverMode.NORMAL
                self.rover_manual = ManualInput()
            return
        state = self.aircraft.get(unit_id)
        if state is not None and state.mode == AircraftMode.MANUAL:
            # Hold wherever the operator left it, as a fresh slot on the rover.
            slot_x, slot_y = clamp_to_tether(state.x, state.y, rover_x, rover_y)
            state.slot_forward, state.slot_right = to_body_frame(
                slot_x - rover_x, slot_y - rover_y, rover_heading
            )
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
            if state.mode == AircraftMode.PREFLIGHT:
                # Never left the bay. Standing down is not a recall.
                self.aircraft.pop(unit_id, None)
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

    def _cell_of(self, body: KinematicBody) -> tuple[int, int]:
        size = self._costmap.cell_size_m
        return (int(math.floor(body.x / size)), int(math.floor(body.y / size)))

    def set_commanded_speed(self, speed_ms: float) -> float:
        """Set the rover's speed ceiling, held between a crawl and its top speed."""
        self.commanded_speed_ms = max(
            ROVER_MIN_COMMAND_SPEED_MS, min(float(speed_ms), self.rover_body.limits.max_speed_ms)
        )
        return self.commanded_speed_ms

    @property
    def _pace_scale(self) -> float:
        return self.commanded_speed_ms / self.rover_body.limits.max_speed_ms

    def navigate_to(
        self,
        goal_x: float,
        goal_y: float,
        lane_from: tuple[float, float] | None = None,
        explore: bool = True,
    ) -> None:
        """Send the rover somewhere under its own navigation.

        With `lane_from`, the leg is treated as a lane to be held rather
        than two points to get between, and the planner is penalised for
        straying off it.

        With `explore` (the default) the rover plans only over ground it has
        seen: when the goal is not yet among that, it drives to the best place
        to look from and decides again from there. With it off the planner
        treats unseen ground as open, which draws a route to anywhere and is
        kept only for callers that hand it a complete map.
        """
        self.goal = (goal_x, goal_y)
        self.lane_from = lane_from
        self.explore = explore
        self.route_known = False
        self._explore_visited = []
        self._explore_ticks = 0
        self.path = []
        self._path_cells = []
        self._plan_failures = 0
        self._ticks_since_attempt = 0
        self.plan_failed = False
        self.rover_mode = RoverMode.NAVIGATING

    @property
    def display_goal(self) -> tuple[float, float] | None:
        """Where the drawn route ends: the goal if the way to it is known, and
        otherwise the frontier the rover is driving to in order to find it."""
        if self.goal is None:
            return None
        if self.explore and not self.route_known and self.path:
            return self.path[-1]
        return self.goal

    def replan(self, perception, cell_size_m: float) -> bool:
        """Plan, or replan, against what the rover currently knows.

        A route is only valid against the map that existed when it was
        made, and that map is being built as the vehicle drives. So the
        plan is checked against the current costmap every tick and rebuilt
        the moment newly discovered ground blocks it. That is the whole
        behaviour: drive, see a rock, go round it.
        """
        if self.goal is None:
            return False

        # The costmap is kept, not rebuilt. Only what the last scan
        # discovered has to be folded in, which is what makes revalidating
        # the route every tick affordable.
        if self._costmap is None or self._costmap.cell_size_m != cell_size_m:
            self._costmap = build_costmap(perception, cell_size_m)
        else:
            update_costmap(self._costmap, perception)
        costmap = self._costmap

        if self.explore:
            return self._replan_explore(costmap, cell_size_m)

        here = (
            int(math.floor(self.rover_body.x / cell_size_m)),
            int(math.floor(self.rover_body.y / cell_size_m)),
        )
        # Leaving the clearance band of something is a plan that starts inside
        # it; only the line the vehicle is following can say whether that plan
        # still holds.
        search_path_clear = costmap.is_blocked(here) or path_is_clear(costmap, self._path_cells)
        if (
            self._path_cells
            and search_path_clear
            and followed_line_is_clear(costmap, self.path, self.rover_body.x, self.rover_body.y)
        ):
            self._plan_failures = 0
            return False

        # A route the map has just invalidated must not keep being driven
        # while a replan is pending. The vehicle holds instead.
        self.path = []
        self._path_cells = []

        # Having failed once, back off. The map is still changing, so trying
        # again is right; trying again ten times a second is not, and a
        # search that cannot succeed is the most expensive one there is.
        if self._plan_failures:
            self._ticks_since_attempt += 1
            if self._ticks_since_attempt < PLAN_RETRY_TICKS:
                return False
        self._ticks_since_attempt = 0

        start = (
            int(math.floor(self.rover_body.x / cell_size_m)),
            int(math.floor(self.rover_body.y / cell_size_m)),
        )
        requested_cell = (
            int(math.floor(self.goal[0] / cell_size_m)),
            int(math.floor(self.goal[1] / cell_size_m)),
        )
        goal_cell = reachable_goal(costmap, requested_cell)
        corridor = None
        if self.lane_from is not None and goal_cell is not None:
            # Both ends in the same units the corridor measures in: world
            # metres divided by cell size. The goal end is the lane point
            # itself, not the centre of the cell it happens to fall in.
            corridor = Corridor(
                ax=self.lane_from[0] / cell_size_m,
                ay=self.lane_from[1] / cell_size_m,
                bx=self.goal[0] / cell_size_m,
                by=self.goal[1] / cell_size_m,
            )
        planned = (
            plan_path(costmap, start, goal_cell, corridor=corridor) if goal_cell else []
        )
        if not planned:
            self._plan_failures += 1
            if self._plan_failures >= MAX_PLAN_FAILURES:
                self.plan_failed = True
            return False

        # Only if the goal was actually moved for clearance does the stored
        # goal change, and then the arrival test moves with it. Rewriting it
        # unconditionally would snap every waypoint to the centre of its
        # cell -- up to a third of a metre off the lane point that was
        # asked for, on every leg, which is most of a sensor swath.
        if goal_cell != requested_cell:
            self.goal = (
                goal_cell[0] * cell_size_m + cell_size_m / 2,
                goal_cell[1] * cell_size_m + cell_size_m / 2,
            )
        self._plan_failures = 0
        self.plan_failed = False
        self._path_cells = planned

        # Followed as a smoothed line, checked for validity as the cells it
        # came from. The ends are the vehicle's actual position and the
        # actual lane point, not the centres of the cells those fall in: a
        # cell centre is up to a third of a metre from the point it stands
        # for, and the smoother holds its endpoints, so quantising them
        # would bake that error into every leg.
        world = [
            (cx * cell_size_m + cell_size_m / 2, cy * cell_size_m + cell_size_m / 2)
            for cx, cy in planned
        ]
        world[0] = (self.rover_body.x, self.rover_body.y)
        world[-1] = self.goal
        self.path = smooth(world, costmap)
        self._replans += 1
        return True

    def _replan_explore(self, costmap, cell_size_m: float) -> bool:
        body = self.rover_body
        here = (int(math.floor(body.x / cell_size_m)), int(math.floor(body.y / cell_size_m)))
        self._explore_ticks += 1

        end = self.path[-1] if self.path else None
        near_end = (
            end is not None
            and not self.route_known
            and math.hypot(end[0] - body.x, end[1] - body.y) < EXPLORE_REPLAN_NEAR_END_M
        )
        periodic = not self.route_known and self._explore_ticks >= EXPLORE_REPLAN_TICKS
        escaping = costmap.is_blocked(here)
        valid = (
            bool(self._path_cells)
            and (escaping or path_is_clear(costmap, self._path_cells))
            and followed_line_is_clear(costmap, self.path, body.x, body.y)
        )
        if valid and not near_end and not periodic:
            self._plan_failures = 0
            return False

        if not valid:
            self.path = []
            self._path_cells = []
        if self._plan_failures:
            self._ticks_since_attempt += 1
            if self._ticks_since_attempt < PLAN_RETRY_TICKS:
                return False
        self._ticks_since_attempt = 0
        self._explore_ticks = 0
        if near_end and end is not None:
            # It has stood where it meant to look from. If that edge is still
            # an edge, it is a shadow it cannot see into from here.
            self._explore_visited.append(end)

        requested = (
            int(math.floor(self.goal[0] / cell_size_m)),
            int(math.floor(self.goal[1] / cell_size_m)),
        )
        goal_cell: tuple[int, int] | None = requested
        if requested in costmap.base and costmap.is_blocked(requested):
            goal_cell = reachable_goal(costmap, requested)
        corridor = None
        if self.lane_from is not None and goal_cell is not None:
            corridor = Corridor(
                ax=self.lane_from[0] / cell_size_m,
                ay=self.lane_from[1] / cell_size_m,
                bx=self.goal[0] / cell_size_m,
                by=self.goal[1] / cell_size_m,
            )
        plan = (
            plan_toward(costmap, here, goal_cell, self._explore_visited_for(cell_size_m), corridor=corridor)
            if goal_cell is not None
            else None
        )
        if plan is None or not plan.path:
            self._plan_failures += 1
            if self._plan_failures >= MAX_PLAN_FAILURES:
                self.plan_failed = True
            return False

        if plan.reaches_goal and goal_cell != requested:
            self.goal = (
                goal_cell[0] * cell_size_m + cell_size_m / 2,
                goal_cell[1] * cell_size_m + cell_size_m / 2,
            )
        self._plan_failures = 0
        self.plan_failed = False
        self._path_cells = plan.path
        self.route_known = plan.reaches_goal
        world = [
            (cx * cell_size_m + cell_size_m / 2, cy * cell_size_m + cell_size_m / 2)
            for cx, cy in plan.path
        ]
        world[0] = (body.x, body.y)
        if plan.reaches_goal:
            world[-1] = self.goal
        self.path = smooth(world, costmap)
        self._replans += 1
        return True

    def _explore_visited_for(self, cell_size_m: float) -> list[tuple[float, float]]:
        return [
            (x / cell_size_m, y / cell_size_m) for x, y in self._explore_visited
        ]

    @property
    def replan_count(self) -> int:
        return self._replans

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

        # Split, so that one long step is the same motion as that much mission
        # time taken in short ones: the follower re-aims and the ramps resolve
        # at their own rate rather than at whatever rate the operator is
        # watching.
        if dt_s > MAX_INTEGRATION_STEP_S:
            steps = int(math.ceil(dt_s / MAX_INTEGRATION_STEP_S))
            poses: dict[str, tuple[float, float, float, float]] = {}
            rover_pose = None
            for _ in range(steps):
                step_poses, step_rover, step_events = self.tick(
                    dt_s / steps, rover_x, rover_y, rover_heading
                )
                poses.update(step_poses)
                if step_rover is not None:
                    rover_pose = step_rover
                    rover_x, rover_y, rover_heading = step_rover
                events.extend(step_events)
            return poses, rover_pose, events

        rover_pose = self._update_rover(dt_s, rover_x, rover_y, rover_heading, events)
        effective_x, effective_y, effective_heading = (
            rover_pose if rover_pose else (rover_x, rover_y, rover_heading)
        )

        poses: dict[str, tuple[float, float, float, float]] = {}
        for state in self.aircraft.values():
            if state.mode == AircraftMode.DOCKED:
                continue
            self._tick_aircraft(
                state, dt_s, effective_x, effective_y, effective_heading, events
            )
            if state.mode == AircraftMode.PREFLIGHT:
                continue  # still in the bay: the roster shows it docked
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
            demand = self.rover_manual.forward * self.commanded_speed_ms
            body.apply_horizontal_command(
                demand * math.sin(heading_rad), demand * math.cos(heading_rad), dt_s
            )
            self._rover_speed_ms = body.speed_ms
            self._rover_velocity = (body.vx, body.vy)
            self._last_rover_xy = (body.x, body.y)
            return (body.x, body.y, body.heading)

        if self.rover_mode == RoverMode.NAVIGATING:
            if self.hold_navigation:
                body.halt(dt_s)
                self._rover_speed_ms = body.speed_ms
                self._rover_velocity = (body.vx, body.vy)
                self._last_rover_xy = (body.x, body.y)
                return (body.x, body.y, body.heading)

            if self.plan_failed:
                # Nowhere to go and nowhere clear nearby. Holding here
                # silently would leave the operator watching a stopped
                # vehicle with no reason given.
                self.goal = None
                self.path = []
                self._path_cells = []
                self.plan_failed = False
                body.halt(dt_s)
                self._rover_speed_ms = body.speed_ms
                self._last_rover_xy = (body.x, body.y)
                events.append(("rover", "WAYPOINT_UNREACHABLE"))
                return (body.x, body.y, body.heading)

            if not self.path or self.goal is None:
                body.halt(dt_s)
                self._rover_speed_ms = body.speed_ms
                self._last_rover_xy = (body.x, body.y)
                return (body.x, body.y, body.heading)

            # Pure pursuit: steer at a point a fixed distance ahead on the
            # planned line. A wheeled vehicle also cannot drive sideways, so
            # it turns onto the bearing before it can build speed.
            # Close to something, follow the line closely: a long lookahead
            # chords across the corner the route was bent to go round.
            tight = self._costmap is not None and self._cell_of(body) in self._costmap.blocked
            target = pure_pursuit_target(
                self.path, body.x, body.y, lookahead_m=TIGHT_LOOKAHEAD_M if tight else LOOKAHEAD_M
            )
            before = copy.copy(body)
            if target is not None:
                dx, dy = target[0] - body.x, target[1] - body.y
                if math.hypot(dx, dy) > 0.2:
                    body.step_heading(math.degrees(math.atan2(dx, dy)) % 360.0, dt_s)
                # The pace is the operator's: a sweep is usually driven at the
                # survey crawl and a transit faster, but that is theirs to say.
                body.step_toward(target[0], target[1], dt_s, speed_scale=self._pace_scale)

            # Nothing the rover has mapped as an obstacle is ever driven onto.
            # The planner keeps a margin from it, and a vehicle that cuts a
            # corner or starts inside that margin can still end up at its
            # edge; at the edge it stops, and the route is made again from
            # there, which is a sensible thing for a vehicle to do and driving
            # through a tree is not.
            if self._costmap is not None and self._cell_of(body) in self._costmap.hard:
                body.__dict__.update(before.__dict__)
                body.vx = body.vy = 0.0
                self.path = []
                self._path_cells = []

            if math.hypot(self.goal[0] - body.x, self.goal[1] - body.y) < WAYPOINT_TOLERANCE_M:
                self.goal = None
                self.path = []
                self._path_cells = []
                events.append(("rover", "WAYPOINT_REACHED"))

            self._rover_speed_ms = body.speed_ms
            self._rover_velocity = (body.vx, body.vy)
            self._last_rover_xy = (body.x, body.y)
            return (body.x, body.y, body.heading)

        if self.rover_mode == RoverMode.RETURNING:
            target_x, target_y = self._base
            dx, dy = target_x - body.x, target_y - body.y
            if math.hypot(dx, dy) > 0.2:
                body.step_heading(math.degrees(math.atan2(dx, dy)) % 360.0, dt_s)
            arrived = body.step_toward(target_x, target_y, dt_s, speed_scale=self._pace_scale)
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
        self,
        state: AircraftState,
        dt_s: float,
        rover_x: float,
        rover_y: float,
        rover_heading: float,
        events: list,
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

        if state.mode == AircraftMode.PREFLIGHT:
            # Riding in the bay: wherever the rover is, on the ground.
            body.x, body.y, body.z = rover_x, rover_y, 0.0
            body.vx, body.vy = self._rover_velocity
            body.vz = 0.0
            state.liftoff_in_s -= dt_s
            if state.liftoff_in_s <= 0.0:
                state.mode = AircraftMode.CLIMBING
                events.append((state.unit_id, "LIFTOFF"))
            return

        if state.mode == AircraftMode.CLIMBING:
            # Straight up over the rover. Held on it, because it may be
            # driving, and an aircraft that stays where it took off from is
            # left behind the moment the vehicle moves.
            body.step_toward(rover_x, rover_y, dt_s, feedforward=self._rover_velocity)
            if self._climb_blocked(state):
                body.step_altitude(body.z, dt_s)  # hold, do not climb into it
            else:
                body.step_altitude(state.commanded_altitude_m, dt_s)
            if state.commanded_altitude_m - body.z <= SLOT_ALTITUDE_TOLERANCE_M * 2.0:
                state.mode = AircraftMode.TRANSITING
            return

        if state.mode in (AircraftMode.TRANSITING, AircraftMode.STATION):
            dx, dy = to_map_frame(state.slot_forward, state.slot_right, rover_heading)
            target_x, target_y = clamp_to_tether(
                rover_x + dx, rover_y + dy, rover_x, rover_y
            )
            body.step_toward(target_x, target_y, dt_s, feedforward=self._rover_velocity)
            body.step_altitude(state.commanded_altitude_m, dt_s)

            # Facing. Joining the formation it faces where it is going. Once on
            # station it faces the formation bearing and nothing else: the
            # rover turning swings every slot, an aircraft following its slot
            # round is moving sideways, and a multirotor does not have to point
            # where it is moving. Measured before this: a scout on the outer
            # slot yawed from 8 to 122 degrees in two seconds through a 13
            # degree turn of the rover, and swept its camera across ground
            # behind the formation.
            rel_vx = body.vx - self._rover_velocity[0]
            rel_vy = body.vy - self._rover_velocity[1]
            joining = state.mode == AircraftMode.TRANSITING
            if joining and math.hypot(rel_vx, rel_vy) > COURSE_FACING_SPEED_MS:
                body.step_heading(math.degrees(math.atan2(rel_vx, rel_vy)) % 360.0, dt_s)
            else:
                body.step_heading((rover_heading + state.bearing_deg) % 360.0, dt_s)

            if state.mode == AircraftMode.TRANSITING:
                on_slot = (
                    math.hypot(body.x - target_x, body.y - target_y) <= SLOT_TOLERANCE_M
                    and abs(body.z - state.commanded_altitude_m) <= SLOT_ALTITUDE_TOLERANCE_M
                    and math.hypot(rel_vx, rel_vy) <= SLOT_SETTLED_SPEED_MS
                )
                if on_slot:
                    state.mode = AircraftMode.STATION
                    events.append((state.unit_id, "STATION"))
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

    def _climb_blocked(self, state: AircraftState) -> bool:
        """Whether climbing now would take this aircraft into one above it."""
        for other in self.aircraft.values():
            if other is state or other.mode not in (AircraftMode.CLIMBING, AircraftMode.TRANSITING):
                continue
            above = other.z - state.z
            if 0.0 < above < CLIMB_GUARD_VERTICAL_M and (
                math.hypot(other.x - state.x, other.y - state.y) < CLIMB_GUARD_HORIZONTAL_M
            ):
                return True
        return False

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
