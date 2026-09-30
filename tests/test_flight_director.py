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
    slot = (state.slot_dx, state.slot_dy)

    for i in range(200):
        director.tick(0.1, 0.0 + i * 0.3, 0.0, 0.0)

    rover_x = 0.0 + 199 * 0.3
    assert abs(state.x - (rover_x + slot[0])) < 1.5
    assert abs(state.y - slot[1]) < 1.5


def test_adopt_clamps_an_out_of_tether_slot():
    director = FlightDirector()
    director.adopt("tricopter-1", 500.0, 0.0, 20.0, 0.0, 0.0, 0.0)
    state = director.aircraft["tricopter-1"]
    assert math.hypot(state.slot_dx, state.slot_dy) <= TETHER_LIMIT_M + 1e-6


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
    director.return_rover_to_base(0.0, 0.0)
    assert director.rover_mode == RoverMode.RETURNING
    assert director.owns_rover()

    rover_x, rover_y = 60.0, 40.0
    for _ in range(3000):
        _, rover_pose, _ = director.tick(0.1, rover_x, rover_y, 0.0)
        if rover_pose:
            rover_x, rover_y = rover_pose[0], rover_pose[1]

    assert director.rover_mode == RoverMode.AT_BASE
    assert math.hypot(rover_x, rover_y) < 0.5
    assert all(a.mode == AircraftMode.DOCKED for a in director.aircraft.values())


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
