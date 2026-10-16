"""A launch, flown.

A launch used to be an animation: each aircraft interpolated along a fixed
line over a fixed number of seconds, aimed at a world position worked out
when the launch was staged. Measured on the real window with the rover
driving, that went from rest to 7.5 m/s in one tick (peak acceleration
about 75 m/s^2, roughly 7 g), and after it finished every aircraft was
5 m off the pattern the operator had chosen, because the rover had moved on
and the formation had been frozen where it happened to end up.

These tests hold the replacement to the properties that matter: the slot is
held on the rover as it moves, nothing exceeds its acceleration limits,
airborne aircraft stay apart, and the stage is read from what the aircraft
are actually doing.
"""

import math

from ocu.core.constants import MIN_AIRCRAFT_SEPARATION_M
from ocu.core.flight_director import (
    AircraftMode,
    FlightDirector,
    tricopter_limits,
)
from ocu.core.formation import compute_formation_targets

DT = 0.1
IDS = ["tricopter-1", "tricopter-2", "tricopter-3"]


class MovingRover:
    """A rover driven by something else at a constant velocity."""

    def __init__(self, vx=0.0, vy=0.0, x=0.0, y=0.0, heading=0.0):
        self.x, self.y, self.vx, self.vy, self.heading = x, y, vx, vy, heading

    def step(self):
        self.x += self.vx * DT
        self.y += self.vy * DT


def launch_formation(director, rover, bearing=0.0, pattern="vee", spacing=6.0, rally=20.0, standoff=30.0, ids=IDS):
    targets = compute_formation_targets(
        rover.x, rover.y, bearing, pattern, spacing, rally, standoff_m=standoff
    )[: len(ids)]
    for i, (uid, t) in enumerate(zip(ids, targets)):
        director.launch(
            uid, t.x - rover.x, t.y - rover.y, t.z, bearing,
            rover.x, rover.y, rover.heading, liftoff_in_s=i * 1.5,
        )
    return targets


def fly(director, rover, seconds, watch=None):
    for _ in range(int(seconds / DT)):
        rover.step()
        director.tick(DT, rover.x, rover.y, rover.heading)
        if watch:
            watch()


def slot_error(director, rover, uid, bearing, pattern="vee", spacing=6.0, rally=20.0, standoff=30.0, index=0):
    """Distance from where the aircraft is to where the pattern puts it
    relative to the rover *now*."""
    want = compute_formation_targets(
        rover.x, rover.y, bearing, pattern, spacing, rally, standoff_m=standoff
    )[index]
    state = director.aircraft[uid]
    return math.hypot(state.x - want.x, state.y - want.y)


# --- the stage is read from the aircraft --------------------------------------


def test_aircraft_wait_in_the_bay_until_their_turn():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    fly(director, rover, 0.5)
    modes = {u: director.aircraft[u].mode for u in IDS}
    assert modes["tricopter-1"] == AircraftMode.CLIMBING
    assert modes["tricopter-2"] == AircraftMode.PREFLIGHT
    assert modes["tricopter-3"] == AircraftMode.PREFLIGHT
    assert director.launch_stage(IDS) == "CLIMBING"


def test_an_aircraft_in_the_bay_is_not_reported_as_flying():
    """The roster shows it docked, so the director must not emit a pose for
    it; one would put an icon on the map for an aircraft that has not left."""
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    poses, _, _ = director.tick(DT, rover.x, rover.y, rover.heading)
    assert "tricopter-2" not in poses and "tricopter-3" not in poses


def test_liftoff_is_announced_once_per_aircraft():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    seen = []
    for _ in range(int(8.0 / DT)):
        _, _, events = director.tick(DT, rover.x, rover.y, rover.heading)
        seen += [e for e in events if e[1] == "LIFTOFF"]
    assert sorted(u for u, _ in seen) == IDS


def test_the_launch_is_formed_only_when_every_aircraft_is_on_its_slot():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    stages = []
    for _ in range(int(60.0 / DT)):
        rover.step()
        director.tick(DT, rover.x, rover.y, rover.heading)
        stage = director.launch_stage(IDS)
        if not stages or stages[-1] != stage:
            stages.append(stage)
    assert stages == ["CLIMBING", "TRANSLATING", "FORMED"] or stages == ["LAUNCHING", "CLIMBING", "TRANSLATING", "FORMED"]
    assert all(director.aircraft[u].mode == AircraftMode.STATION for u in IDS)


def test_launch_takes_as_long_as_the_physics_takes():
    """Twenty metres at the climb rate and thirty across at transit speed
    cannot be done in seven seconds, which is how long the animation took."""
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    elapsed = 0.0
    while director.launch_stage(IDS) != "FORMED" and elapsed < 90.0:
        rover.step()
        director.tick(DT, rover.x, rover.y, rover.heading)
        elapsed += DT
    limits = tricopter_limits()
    last_liftoff_s = 2 * 1.5
    assert elapsed > last_liftoff_s + 23.0 / limits.max_climb_ms
    assert elapsed < 60.0


# --- the formation is held on the rover, however it moves ------------------------


def test_a_formation_launched_from_a_stationary_rover_forms_on_the_pattern():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover, bearing=90.0)
    fly(director, rover, 45.0)
    for index, uid in enumerate(IDS):
        assert slot_error(director, rover, uid, 90.0, index=index) < 0.5


def test_a_formation_launched_from_a_moving_rover_forms_on_the_pattern_relative_to_where_it_is_now():
    """The regression that started this. The rover drives 2 m/s throughout;
    after the launch every aircraft must be where the pattern says relative
    to the rover's *current* position, not to where it was when staged."""
    director, rover = FlightDirector(), MovingRover(vx=1.4, vy=-1.4)
    launch_formation(director, rover, bearing=45.0)
    fly(director, rover, 50.0)
    travelled = math.hypot(rover.x, rover.y)
    assert travelled > 90.0, "the rover must actually have moved a long way"
    # Holds within 0.3 m at 2 m/s. Without the rover's velocity fed forward
    # the same formation trails by 0.8 m, which is exactly the failure of an
    # aircraft following a position that is already behind it.
    for index, uid in enumerate(IDS):
        assert slot_error(director, rover, uid, 45.0, index=index) < 0.45, uid


def test_the_slot_keeps_following_the_rover_after_the_launch():
    director, rover = FlightDirector(), MovingRover(vx=2.0)
    launch_formation(director, rover, bearing=0.0)
    fly(director, rover, 40.0)
    rover.vx, rover.vy = 0.0, 2.0
    fly(director, rover, 20.0)
    for index, uid in enumerate(IDS):
        assert slot_error(director, rover, uid, 0.0, index=index) < 0.45


def test_the_pattern_chosen_by_the_operator_is_the_pattern_flown():
    for pattern in ("vee", "line", "column"):
        director, rover = FlightDirector(), MovingRover(vx=1.0)
        launch_formation(director, rover, bearing=200.0, pattern=pattern)
        fly(director, rover, 50.0)
        for index, uid in enumerate(IDS):
            assert slot_error(director, rover, uid, 200.0, pattern=pattern, index=index) < 0.8, (pattern, uid)


# --- it is physics, not animation ---------------------------------------------------


def test_no_aircraft_exceeds_its_acceleration_limits_during_a_launch():
    director, rover = FlightDirector(), MovingRover(vx=1.5, vy=1.0)
    launch_formation(director, rover, bearing=300.0)
    limits = tricopter_limits()
    previous: dict[str, tuple[float, float, float]] = {}
    worst_h = worst_v = 0.0

    def watch():
        nonlocal worst_h, worst_v
        for uid in IDS:
            state = director.aircraft.get(uid)
            if state is None or state.mode == AircraftMode.PREFLIGHT:
                previous.pop(uid, None)
                continue
            b = state.body
            if uid in previous:
                worst_h = max(worst_h, math.hypot(b.vx - previous[uid][0], b.vy - previous[uid][1]) / DT)
                worst_v = max(worst_v, abs(b.vz - previous[uid][2]) / DT)
            previous[uid] = (b.vx, b.vy, b.vz)

    fly(director, rover, 50.0, watch)
    assert worst_h <= limits.max_accel_ms2 + 1e-6
    assert worst_v <= limits.max_vertical_accel_ms2 + 1e-6


def test_no_aircraft_turns_faster_than_it_can():
    director, rover = FlightDirector(), MovingRover(vx=1.0)
    launch_formation(director, rover, bearing=180.0)
    limits = tricopter_limits()
    previous: dict[str, float] = {}
    worst = 0.0

    def watch():
        nonlocal worst
        for uid in IDS:
            state = director.aircraft.get(uid)
            if state is None or state.mode == AircraftMode.PREFLIGHT:
                continue
            if uid in previous:
                step = abs((state.heading - previous[uid] + 180.0) % 360.0 - 180.0)
                worst = max(worst, step / DT)
            previous[uid] = state.heading

    fly(director, rover, 50.0, watch)
    assert worst <= limits.max_yaw_rate_deg_s + 1e-6


def test_airborne_aircraft_keep_the_minimum_separation_through_the_whole_launch():
    """Three aircraft leave one spot within seconds of each other. The hard
    minimum (6.2) has to hold the whole way, not only at the end."""
    for rover_velocity in ((0.0, 0.0), (2.0, 0.0), (1.0, 2.5)):
        director, rover = FlightDirector(), MovingRover(*rover_velocity)
        launch_formation(director, rover, bearing=45.0)
        closest = math.inf

        def watch():
            nonlocal closest
            flying = [
                director.aircraft[u].body
                for u in IDS
                if u in director.aircraft
                and director.aircraft[u].mode != AircraftMode.PREFLIGHT
                and director.aircraft[u].body.z > 0.5  # parked on the pad is not flying
            ]
            for i, a in enumerate(flying):
                for b in flying[i + 1 :]:
                    closest = min(closest, math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2))

        fly(director, rover, 50.0, watch)
        assert closest >= MIN_AIRCRAFT_SEPARATION_M, (rover_velocity, closest)


def test_a_climbing_aircraft_holds_rather_than_climb_into_the_one_above_it():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    fly(director, rover, 2.0)
    second = director.aircraft["tricopter-2"].body
    first = director.aircraft["tricopter-1"].body
    # The first is only a couple of metres up, so the second must still be on the pad.
    assert first.z < 4.5
    assert second.z < 0.5


def test_nothing_is_ever_below_the_ground_or_above_the_ceiling():
    from ocu.core.constants import ARES_A_CEILING_M

    director, rover = FlightDirector(), MovingRover(vx=2.0)
    launch_formation(director, rover)
    seen = []
    fly(director, rover, 50.0, lambda: seen.extend(director.aircraft[u].z for u in IDS))
    assert min(seen) >= 0.0 and max(seen) <= ARES_A_CEILING_M


# --- facing ----------------------------------------------------------------------------


def test_aircraft_hold_their_slot_facing_the_formation_bearing():
    """Every aircraft faces the formation's bearing, so the group presents one
    sensor direction rather than three, even while the rover drives another way."""
    director, rover = FlightDirector(), MovingRover(vx=0.0, vy=-2.0)
    launch_formation(director, rover, bearing=75.0)
    fly(director, rover, 50.0)
    for uid in IDS:
        error = abs((director.aircraft[uid].heading - 75.0 + 180.0) % 360.0 - 180.0)
        assert error < 3.0, (uid, director.aircraft[uid].heading)


# --- standing down and being recalled -----------------------------------------------------


def test_standing_down_before_liftoff_never_puts_an_aircraft_in_the_air():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    fly(director, rover, 0.3)
    recalled = director.recall(["tricopter-3"])
    assert recalled == [], "an aircraft that never left the bay is not recalled, it stands down"
    assert "tricopter-3" not in director.aircraft


def test_a_launch_that_loses_every_aircraft_reports_idle():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover, ids=["tricopter-2", "tricopter-3"])
    director.recall(["tricopter-2", "tricopter-3"])
    assert director.launch_stage(["tricopter-2", "tricopter-3"]) == "IDLE"


def test_an_aircraft_recalled_mid_climb_returns_instead_of_continuing():
    director, rover = FlightDirector(), MovingRover()
    launch_formation(director, rover)
    fly(director, rover, 4.0)
    assert director.recall(["tricopter-1"]) == ["tricopter-1"]
    assert director.aircraft["tricopter-1"].mode == AircraftMode.RETURNING


def test_a_slot_beyond_the_tether_is_pulled_back_onto_it():
    from ocu.core.constants import TETHER_LIMIT_M

    director, rover = FlightDirector(), MovingRover()
    director.launch("tricopter-1", 90.0, 0.0, 20.0, 90.0, 0.0, 0.0, 0.0)
    fly(director, rover, 60.0)
    state = director.aircraft["tricopter-1"]
    assert math.hypot(state.x - rover.x, state.y - rover.y) <= TETHER_LIMIT_M + 1e-6
