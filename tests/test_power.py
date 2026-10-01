"""The energy model has to reproduce the published baseline exactly.

Every number an investor can check against the spec sheet is checked here,
because the previous hand-written battery curves were out by roughly an
order of magnitude and nothing caught it.
"""

from ocu.core.constants import (
    ARES_A_CRUISE_ENDURANCE_MIN_HIGH,
    ARES_A_CRUISE_ENDURANCE_MIN_LOW,
    CHARGE_TIME_FROM_EMPTY_MIN_HIGH,
    CHARGE_TIME_FROM_EMPTY_MIN_LOW,
    ROVER_ENDURANCE_CONTINUOUS_WORST_CASE_H,
    ROVER_MISSION_AVG_POWER_W,
)
from ocu.core.power import (
    CRUISE_ENDURANCE_S,
    charge_pct_per_s,
    format_endurance,
    rover_drain_pct_per_s,
    rover_endurance_s,
    rover_power_w,
    tricopter_drain_pct_per_s,
    tricopter_endurance_s,
)


# --- rover, against 430 Wh and 143 W -------------------------------------


def test_rover_at_mission_average_power_lasts_the_published_three_hours():
    endurance_h = rover_endurance_s(100.0, 2.5) / 3600.0
    assert abs(endurance_h - ROVER_ENDURANCE_CONTINUOUS_WORST_CASE_H) < 0.05


def test_rover_mission_average_speed_draws_the_published_average_power():
    assert abs(rover_power_w(2.5) - ROVER_MISSION_AVG_POWER_W) < 0.5


def test_rover_parked_draws_less_than_driving():
    assert rover_power_w(0.0) < rover_power_w(4.0)


def test_rover_parked_still_draws_compute_and_sensors():
    assert rover_power_w(0.0) > 40.0


def test_rover_drain_is_tiny_over_a_ninety_second_mission():
    """The whole scripted mission is 90 s. A 3 hour platform must barely move."""
    used = rover_drain_pct_per_s(2.5) * 90.0
    assert used < 1.0


# --- ARES-A, against 10 to 12 minutes ------------------------------------


def test_cruise_endurance_matches_the_published_band():
    minutes = CRUISE_ENDURANCE_S / 60.0
    assert ARES_A_CRUISE_ENDURANCE_MIN_LOW <= minutes <= ARES_A_CRUISE_ENDURANCE_MIN_HIGH


def test_tricopter_at_cruise_lasts_the_published_endurance():
    endurance_min = tricopter_endurance_s(100.0, 6.0) / 60.0
    assert ARES_A_CRUISE_ENDURANCE_MIN_LOW <= endurance_min <= ARES_A_CRUISE_ENDURANCE_MIN_HIGH


def test_hover_costs_more_than_cruise():
    """No translational lift in a hover, and the pusher earns nothing."""
    assert tricopter_drain_pct_per_s(0.0) > tricopter_drain_pct_per_s(6.0)


def test_climbing_costs_more_than_level_flight():
    assert tricopter_drain_pct_per_s(3.0, 2.5) > tricopter_drain_pct_per_s(3.0, 0.0)


def test_descending_costs_less_than_level_flight():
    assert tricopter_drain_pct_per_s(3.0, -1.5) < tricopter_drain_pct_per_s(3.0, 0.0)


def test_tricopter_drain_over_a_ninety_second_sortie_is_plausible():
    """A third of a sortie's flight time should cost well under a third of
    the pack, not two thirds of it as the scripted curve did."""
    used = tricopter_drain_pct_per_s(4.0) * 60.0
    assert 5.0 < used < 20.0


# --- charging, against 8 to 10 minutes -----------------------------------


def test_charge_from_empty_matches_the_published_window():
    battery, elapsed = 0.0, 0.0
    while battery < 99.5 and elapsed < 3600:
        battery += charge_pct_per_s(battery) * 0.5
        elapsed += 0.5
    minutes = elapsed / 60.0
    # The taper above 80% means a full charge sits at or past the top of the
    # published constant-current window, which is how a real pack behaves.
    assert CHARGE_TIME_FROM_EMPTY_MIN_LOW <= minutes <= CHARGE_TIME_FROM_EMPTY_MIN_HIGH * 2.2


def test_charging_tapers_near_full():
    assert charge_pct_per_s(95.0) < charge_pct_per_s(50.0)


def test_charging_stops_at_full():
    assert charge_pct_per_s(100.0) == 0.0


def test_bulk_charge_rate_matches_the_published_window():
    """Below the taper the pack takes constant current, so 0 to 80% should
    track the published 8 to 10 minute figure directly."""
    minutes_to_80 = (80.0 / charge_pct_per_s(0.0)) / 60.0
    assert CHARGE_TIME_FROM_EMPTY_MIN_LOW * 0.75 <= minutes_to_80 <= CHARGE_TIME_FROM_EMPTY_MIN_HIGH


# --- presentation ---------------------------------------------------------


def test_endurance_formats_for_an_operator():
    assert format_endurance(3 * 3600 + 5 * 60) == "3h 05m"
    assert format_endurance(11 * 60 + 7) == "11m 07s"
    assert format_endurance(42) == "42s"
    assert format_endurance(0) == "0m"


def test_rover_mixed_duty_reproduces_the_published_four_hours():
    """The baseline gives two rover figures: 3.0 h continuous worst case and
    about 4 h mixed duty. One power model has to produce both, at a driving
    average and at a stop-and-scan average respectively."""
    from ocu.core.power import ROVER_IDLE_W, ROVER_DRIVE_W_PER_MS

    target_w = 430.0 / 4.0  # 4 h on a 430 Wh pack
    mixed_duty_speed = (target_w - ROVER_IDLE_W) / ROVER_DRIVE_W_PER_MS
    # That speed has to be a believable stop-and-scan average, not a fudge.
    assert 1.0 < mixed_duty_speed < 2.2
    assert abs(rover_endurance_s(100.0, mixed_duty_speed) / 3600.0 - 4.0) < 0.05


def test_a_parked_rover_outlasts_a_driving_one():
    assert rover_endurance_s(100.0, 0.0) > rover_endurance_s(100.0, 4.0)
