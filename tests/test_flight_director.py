"""The fleet-coordination rules: aircraft stay tethered to the rover, the
formation travels with the rover, and nothing docks onto a moving deck.
"""

import math

from ocu.core.constants import ROVER_STATIONARY_SPEED_MS, TETHER_LIMIT_M
from ocu.core.flight_director import AircraftMode, FlightDirector, RoverMode


def run(director, seconds, rover_x=0.0, rover_y=0.0, dt=0.1):
    """Tick for a while with the rover parked where it is."""
    for _ in range(int(seconds / dt)):
        director.tick(dt, rover_x, rover_y, 0.0)


def make_director(n=3, rover_x=0.0, rover_y=0.0):
    director = FlightDirector()
    for i in range(n):
        director.adopt(f"tricopter-{i + 1}", rover_x + 10 + i * 4, rover_y + 5, 20.0, 0.0, rover_x, rover_y)
    return director


def test_station_keeping_follows_a_moving_rover():
    director = make_director(n=1)
    state = director.aircraft["tricopter-1"]
    forward, right = state.slot_forward, state.slot_right

    for i in range(200):
        director.tick(0.1, 0.0 + i * 0.3, 0.0, 0.0)

    # The rover is driving east but still pointing north, so its own frame
    # and the map's coincide: forward is +y, right is +x.
    rover_x = 0.0 + 199 * 0.3
    assert abs(state.x - (rover_x + right)) < 1.5
    assert abs(state.y - forward) < 1.5


def test_station_keeping_follows_the_rover_round_a_turn():
    """The slot is a place on the vehicle, so it has to swing with it. A test
    that only ever drives a north-facing rover cannot tell the vehicle's
    frame from the map's, because at a heading of zero they are the same."""
    director = make_director(n=1)
    state = director.aircraft["tricopter-1"]
    forward, right = state.slot_forward, state.slot_right

    for _ in range(300):  # same place, now facing east
        director.tick(0.1, 0.0, 0.0, 90.0)

    # Facing east, the rover's forward is +x and its right is -y.
    assert abs(state.x - forward) < 1.5
    assert abs(state.y + right) < 1.5


def test_adopt_clamps_an_out_of_tether_slot():
    director = FlightDirector()
    director.adopt("tricopter-1", 500.0, 0.0, 20.0, 0.0, 0.0, 0.0)
    state = director.aircraft["tricopter-1"]
    # Frame-independent: how far out the slot is, whichever way it is held.
    assert math.hypot(state.slot_forward, state.slot_right) <= TETHER_LIMIT_M + 1e-6


def test_aircraft_never_commanded_outside_the_tether():
    director = make_director(n=3)
    for state in director.aircraft.values():
        state.slot_dx, state.slot_dy = 900.0, 900.0  # absurd command
    run(director, 40)
    for state in director.aircraft.values():
        assert math.hypot(state.x, state.y) <= TETHER_LIMIT_M + 0.5


def test_recall_brings_aircraft_to_holding():
    director = make_director(n=2)
    director.recall()
    assert all(a.mode == AircraftMode.RETURNING for a in director.aircraft.values())
    run(director, 30)
    modes = {a.mode for a in director.aircraft.values()}
    assert AircraftMode.RETURNING not in modes


def test_nothing_docks_onto_a_moving_rover():
    director = make_director(n=2)
    director.recall()
    # Rover keeps driving the whole time, well above the stationary threshold.
    x = 0.0
    for _ in range(600):
        x += ROVER_STATIONARY_SPEED_MS * 0.1 * 20
        director.tick(0.1, x, 0.0, 0.0)
    assert all(a.mode != AircraftMode.DOCKED for a in director.aircraft.values())
    assert all(a.mode != AircraftMode.DOCKING for a in director.aircraft.values())


def test_aircraft_dock_one_at_a_time():
    director = make_director(n=3)
    director.recall()
    seen_simultaneous = 0
    for _ in range(1200):
        director.tick(0.1, 0.0, 0.0, 0.0)
        docking = sum(1 for a in director.aircraft.values() if a.mode == AircraftMode.DOCKING)
        seen_simultaneous = max(seen_simultaneous, docking)
    assert seen_simultaneous <= 1
    assert all(a.mode == AircraftMode.DOCKED for a in director.aircraft.values())


def test_return_to_base_drives_rover_home_then_docks_everyone():
    director = make_director(n=3, rover_x=60.0, rover_y=40.0)
    director.return_rover_to_base(0.0, 0.0, 60.0, 40.0, 0.0)
    assert director.rover_mode == RoverMode.RETURNING
    assert director.owns_rover()

    rover_x, rover_y = 60.0, 40.0
    elapsed = 0.0
    for _ in range(6000):
        _, rover_pose, _ = director.tick(0.1, rover_x, rover_y, 0.0)
        elapsed += 0.1
        if rover_pose:
            rover_x, rover_y = rover_pose[0], rover_pose[1]

    assert director.rover_mode == RoverMode.AT_BASE
    assert math.hypot(rover_x, rover_y) < 0.5
    assert all(a.mode == AircraftMode.DOCKED for a in director.aircraft.values())


def test_return_to_base_actually_takes_time_to_drive():
    """72 m at 15 km/h cannot happen in under ~17 s, however the state machine
    is arranged. Guards against the rover teleporting home because its body
    was never seeded with where it actually was."""
    director = FlightDirector()
    director.return_rover_to_base(0.0, 0.0, 60.0, 40.0, 0.0)
    rover_x, rover_y = 60.0, 40.0
    elapsed = 0.0
    while director.rover_mode != RoverMode.AT_BASE and elapsed < 300:
        _, rover_pose, _ = director.tick(0.1, rover_x, rover_y, 0.0)
        elapsed += 0.1
        if rover_pose:
            rover_x, rover_y = rover_pose[0], rover_pose[1]
    assert director.rover_mode == RoverMode.AT_BASE
    assert elapsed > 17.0, f"drove 72 m in {elapsed:.1f}s, faster than its own top speed"


def test_in_flight_altitude_change_is_obeyed():
    director = make_director(n=1)
    director.set_commanded_altitude("tricopter-1", 45.0)
    run(director, 30)
    assert abs(director.aircraft["tricopter-1"].z - 45.0) < 0.5


def test_altitude_change_refused_once_docking():
    director = make_director(n=1)
    director.recall()
    run(director, 40)
    state = director.aircraft["tricopter-1"]
    if state.mode in (AircraftMode.DOCKING, AircraftMode.DOCKED):
        before = state.commanded_altitude_m
        director.set_commanded_altitude("tricopter-1", 300.0)
        assert state.commanded_altitude_m == before


def test_altitude_command_clamped_to_ceiling():
    director = make_director(n=1)
    director.set_commanded_altitude("tricopter-1", 99999.0)
    assert director.aircraft["tricopter-1"].commanded_altitude_m <= 4500.0


def test_recall_is_idempotent_and_safe_on_empty_fleet():
    director = FlightDirector()
    assert director.recall() == []
    director.return_rover_to_base()
    director.tick(0.1, 0.0, 0.0, 0.0)  # must not raise
    director.reset()
    assert director.aircraft == {}


def test_zero_dt_tick_is_a_noop():
    director = make_director(n=1)
    poses, rover, events = director.tick(0.0, 0.0, 0.0, 0.0)
    assert poses == {} and rover is None and events == []


def test_manual_input_accelerates_rather_than_jumping():
    director = make_director(n=1)
    director.take_manual("tricopter-1", 0.0, 0.0, 0.0)
    assert director.aircraft["tricopter-1"].mode == AircraftMode.MANUAL
    director.set_manual_input("tricopter-1", forward=1.0, turn=0.0, vertical=0.0)
    before = director.aircraft["tricopter-1"].body.speed_ms
    director.tick(0.1, 0.0, 0.0, 0.0)
    after = director.aircraft["tricopter-1"].body.speed_ms
    assert after > before
    assert after < 1.0  # one tick of acceleration, not an instant jump to cruise


def test_manual_flight_cannot_break_the_tether():
    director = make_director(n=1)
    director.take_manual("tricopter-1", 0.0, 0.0, 0.0)
    director.set_manual_input("tricopter-1", forward=1.0, turn=0.0, vertical=0.0)
    for _ in range(2000):
        director.tick(0.05, 0.0, 0.0, 0.0)
    state = director.aircraft["tricopter-1"]
    assert math.hypot(state.x, state.y) <= TETHER_LIMIT_M + 0.5


def test_releasing_manual_holds_the_new_position_as_a_slot():
    """Where the operator left it, kept as a place on the rover rather than a
    point on the map, so it travels with the vehicle like any other slot."""
    from ocu.core.flight_director import to_body_frame

    rover_heading = 55.0
    director = make_director(n=1)
    director.take_manual("tricopter-1", 0.0, 0.0, rover_heading)
    director.set_manual_input("tricopter-1", forward=1.0, turn=0.0, vertical=0.0)
    for _ in range(40):
        director.tick(0.1, 0.0, 0.0, rover_heading)
    state = director.aircraft["tricopter-1"]
    flown_to = (state.x, state.y)

    director.release_manual("tricopter-1", 0.0, 0.0, rover_heading)
    assert state.mode == AircraftMode.STATION
    forward, right = to_body_frame(flown_to[0], flown_to[1], rover_heading)
    assert abs(state.slot_forward - forward) < 0.01
    assert abs(state.slot_right - right) < 0.01


def test_manual_rover_drives_and_turns_under_limits():
    director = FlightDirector()
    director.take_manual("rover", 0.0, 0.0, 0.0)
    director.set_manual_input("rover", forward=1.0, turn=1.0, vertical=0.0)
    for _ in range(100):
        director.tick(0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading)
    body = director.rover_body
    assert body.speed_ms <= body.limits.max_speed_ms + 1e-6
    assert abs(body.yaw_rate) <= body.limits.max_yaw_rate_deg_s + 1e-6
    assert math.hypot(body.x, body.y) > 1.0  # it actually went somewhere


def test_manual_refused_on_a_docked_aircraft():
    director = make_director(n=1)
    director.recall()
    run(director, 60)
    state = director.aircraft["tricopter-1"]
    assert state.mode == AircraftMode.DOCKED
    assert director.take_manual("tricopter-1", 0.0, 0.0, 0.0) is False


def test_docking_descends_gently_through_the_stages():
    director = make_director(n=1)
    director.recall()
    rates = []
    previous_z = director.aircraft["tricopter-1"].z
    for _ in range(1500):
        director.tick(0.1, 0.0, 0.0, 0.0)
        z = director.aircraft["tricopter-1"].z
        if director.aircraft["tricopter-1"].mode == AircraftMode.DOCKING:
            rates.append(abs(z - previous_z) / 0.1)
        previous_z = z
    assert director.aircraft["tricopter-1"].mode == AircraftMode.DOCKED
    assert rates, "never entered the docking descent"
    # Nothing during recovery may fall faster than the coarse-stage rate.
    assert max(rates) <= 1.3
