"""Energy model: battery state derived from the power budget, not scripted.

Battery used to be hand-written keyframes in the mission file, and they
were wrong by about an order of magnitude: the tricopters discharged in
73 seconds against a published cruise endurance of 10 to 12 minutes, the
rover flattened in 8 minutes against a published 3.0 hours, and a docked
aircraft charged in under two minutes against a published 8 to 10. Anyone
reading the spec sheet and then watching the screen would have caught it.

Every figure below traces to the ARES technical baseline, and the derived
endurances come back out matching it:

    rover    430 Wh usable, 143 W mission average  ->  3.0 h continuous
    ARES-A   cruise endurance 10 to 12 min         ->  11 min at cruise
    charging 8 to 10 min from empty on a 30 W pad  ->  9 min

What is *not* from the baseline is the split of the rover's 143 W between
a standing load and a speed-dependent one, and the factors by which a
rotorcraft's draw differs between hover, cruise, climb and descent. The
baseline gives totals, not breakdowns. Those are this build's model, and
they are chosen so the published totals fall straight back out. See
DECISIONS.md.

No Qt import belongs in this file.
"""

from __future__ import annotations

from ocu.core.constants import (
    ARES_A_CRUISE_ENDURANCE_MIN_HIGH,
    ARES_A_CRUISE_ENDURANCE_MIN_LOW,
    CHARGE_TIME_FROM_EMPTY_MIN_HIGH,
    CHARGE_TIME_FROM_EMPTY_MIN_LOW,
    ROVER_BATTERY_USABLE_WH,
    ROVER_COMPUTE_LOAD_W,
    ROVER_MISSION_AVG_POWER_W,
    ROVER_TOP_SPEED_MS,
    TRICOPTER_CLIMB_RATE_MS,
)

# --- rover ---------------------------------------------------------------

# Standing load: the compute pair (20 W, baseline) plus the detection suite,
# which runs continuously because the rover does not stop to scan.
ROVER_IDLE_W = 45.0
ROVER_SENSOR_W = ROVER_IDLE_W - ROVER_COMPUTE_LOAD_W

# Drive load scales with speed. The coefficient is set so that the baseline's
# 143 W mission average corresponds to a believable average driving speed
# rather than to either extreme.
ROVER_MISSION_AVG_SPEED_MS = 2.5
ROVER_DRIVE_W_PER_MS = (ROVER_MISSION_AVG_POWER_W - ROVER_IDLE_W) / ROVER_MISSION_AVG_SPEED_MS


def rover_power_w(speed_ms: float) -> float:
    """Instantaneous draw: a standing load plus a term that scales with speed."""
    return ROVER_IDLE_W + ROVER_DRIVE_W_PER_MS * max(0.0, speed_ms)


def rover_drain_pct_per_s(speed_ms: float) -> float:
    capacity_ws = ROVER_BATTERY_USABLE_WH * 3600.0
    return rover_power_w(speed_ms) / capacity_ws * 100.0


def rover_endurance_s(battery_pct: float, speed_ms: float) -> float:
    rate = rover_drain_pct_per_s(speed_ms)
    return max(0.0, battery_pct) / rate if rate > 0 else 0.0


# --- ARES-A --------------------------------------------------------------

CRUISE_ENDURANCE_S = (
    (ARES_A_CRUISE_ENDURANCE_MIN_LOW + ARES_A_CRUISE_ENDURANCE_MIN_HIGH) / 2.0 * 60.0
)
CRUISE_DRAIN_PCT_PER_S = 100.0 / CRUISE_ENDURANCE_S

# A rotorcraft in a hover has no translational lift, so it draws more than the
# same airframe in forward flight; a compound bicopter with a tail pusher more
# so, since cruise is the case its pusher exists to serve. Descent is cheaper
# than level flight. Climb costs extra in proportion to climb rate.
HOVER_FACTOR = 1.15
DESCENT_FACTOR = 0.80
CLIMB_FACTOR_AT_MAX_RATE = 0.60


def tricopter_drain_pct_per_s(speed_ms: float, vertical_speed_ms: float = 0.0) -> float:
    """Draw as a multiple of the cruise figure, from how the aircraft is flying."""
    cruise_fraction = min(1.0, max(0.0, speed_ms) / 6.0)
    factor = HOVER_FACTOR + (1.0 - HOVER_FACTOR) * cruise_fraction

    if vertical_speed_ms > 0:
        climb_fraction = min(1.0, vertical_speed_ms / TRICOPTER_CLIMB_RATE_MS)
        factor += CLIMB_FACTOR_AT_MAX_RATE * climb_fraction
    elif vertical_speed_ms < 0:
        factor *= DESCENT_FACTOR

    return CRUISE_DRAIN_PCT_PER_S * factor


def tricopter_endurance_s(battery_pct: float, speed_ms: float, vertical_speed_ms: float = 0.0) -> float:
    rate = tricopter_drain_pct_per_s(speed_ms, vertical_speed_ms)
    return max(0.0, battery_pct) / rate if rate > 0 else 0.0


# --- charging ------------------------------------------------------------

CHARGE_TIME_S = (CHARGE_TIME_FROM_EMPTY_MIN_LOW + CHARGE_TIME_FROM_EMPTY_MIN_HIGH) / 2.0 * 60.0
CHARGE_PCT_PER_S = 100.0 / CHARGE_TIME_S


def charge_pct_per_s(battery_pct: float) -> float:
    """Charge rate, tapering above 80% the way a lithium pack actually does.

    Constant-current up to roughly 80%, then constant-voltage, where current
    falls away as the cell approaches full. A pack that charges linearly to
    100% and stops dead is a giveaway that nobody modelled it.
    """
    if battery_pct >= 100.0:
        return 0.0
    if battery_pct <= 80.0:
        return CHARGE_PCT_PER_S
    taper = max(0.08, (100.0 - battery_pct) / 20.0)
    return CHARGE_PCT_PER_S * taper


def format_endurance(seconds: float) -> str:
    """Endurance remaining, as an operator reads it.

    A percentage that moves by a tenth of a point a minute tells an operator
    nothing useful; time remaining at the current draw does.
    """
    if seconds <= 0:
        return "0m"
    hours, remainder = divmod(int(seconds), 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return f"{hours}h {minutes:02d}m"
    if minutes:
        return f"{minutes}m {secs:02d}s"
    return f"{secs}s"


def flight_regime(speed_ms: float, vertical_speed_ms: float = 0.0) -> str:
    """What the aircraft is doing, in the terms the endurance depends on.

    The published 10 to 12 minutes is a *cruise* figure. A hovering aircraft
    draws more and lasts less, which looks like an error unless the display
    says which regime the number belongs to.
    """
    if vertical_speed_ms > 0.3:
        return "CLIMB"
    if vertical_speed_ms < -0.3:
        return "DESCENT"
    if speed_ms > 2.0:
        return "CRUISE"
    return "HOVER"
