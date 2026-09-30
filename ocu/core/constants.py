"""Every physical and protocol constant used anywhere in the OCU.

No number outside this file. If a module needs a figure from the build
spec (ARES_OCU_Technical_Specification.pdf, v1, 30 Sep 2026), it imports
it from here so there is exactly one place to correct it when the source
document is revised. Section references point at that spec.
"""

# --- Section 2: current authoritative specifications ---------------------

ROVER_MASS_KG_MIN = 25.0
ROVER_TOP_SPEED_KMH = 15.0
ROVER_ENDURANCE_MIXED_DUTY_H = 4.0
ROVER_ENDURANCE_CONTINUOUS_WORST_CASE_H = 3.0
ROVER_BATTERY_USABLE_WH = 430.0
ROVER_MISSION_AVG_POWER_W = 143.0
ROVER_WHEEL_COUNT = 6
ROVER_TOP_SPEED_MS = ROVER_TOP_SPEED_KMH / 3.6

ARES_A_MASS_G = 102.0
ARES_A_CRUISE_ENDURANCE_MIN_LOW = 10.0
ARES_A_CRUISE_ENDURANCE_MIN_HIGH = 12.0
ARES_A_CEILING_M = 4500.0
ARES_A_SCOUT_RADIUS_M_LOW = 30.0
ARES_A_SCOUT_RADIUS_M_HIGH = 50.0
ARES_A_COUNT = 3
ARES_A_CONFIGURATION = "compound_bicopter"  # never "nano tricopter", section 14

UWB_RANGING_LIMIT_M = 50.0
UWB_RANGING_RATE_HZ = 100.0

# The aircraft never operate outside UWB range of the rover. Section 2 makes
# this explicit: the 30-50 m scout radius is "identical to the UWB ranging
# limit, which is a known constraint". Beyond it the aircraft lose relative
# positioning and precision landing, so the tether is a hard operational
# limit, not a preference.
TETHER_LIMIT_M = UWB_RANGING_LIMIT_M

# --- Flight motion. Not baseline figures: the technical baseline gives
# endurance, ceiling and scout radius for ARES-A but no transit speed or
# climb rate, so these are this build's own motion model. See DECISIONS.md.
TRICOPTER_TRANSIT_SPEED_MS = 7.0
TRICOPTER_CLIMB_RATE_MS = 3.0
TRICOPTER_TURN_RATE_DEG_S = 120.0
STATION_KEEP_TOLERANCE_M = 0.4
RECALL_RING_RADIUS_M = 6.0  # holding ring above the rover while awaiting a dock slot
HOLD_ALTITUDE_M = 12.0  # altitude aircraft settle to while queued for docking

# Docking requires a stationary rover: the funnel pad and the ArUco fiducial
# in section 7 are both fixed to the deck, so the deck has to stop moving.
ROVER_STATIONARY_SPEED_MS = 0.15

# --- Section 6.2 / 10: formation and separation ---------------------------

LAUNCH_STAGGER_S = 1.5  # aircraft lift off one at a time, ~1.5s apart, 6.3
MIN_AIRCRAFT_SEPARATION_M = 3.0  # hard minimum, enforced in code not just UI, 6.2/10
# Not stated numerically in the spec; 3 m is a conservative placeholder pending
# a real number from Bhargav. Flagged in DECISIONS.md, do not treat as sourced.

# --- Section 7: aircraft recovery ------------------------------------------

RECOVERY_STAGE_1_PRECISION_CM = 50.0   # coarse, UWB ranging
RECOVERY_STAGE_2_PRECISION_CM = 10.0   # fine, ArUco downward camera
RECOVERY_STAGE_3_PRECISION_CM = 3.0    # mechanical, funnel pad
CHARGE_PAD_COUNT = 3
CHARGE_PAD_POWER_W = 30.0
CHARGE_TIME_FROM_EMPTY_MIN_LOW = 8.0
CHARGE_TIME_FROM_EMPTY_MIN_HIGH = 10.0
AVG_AIRBORNE_AIRCRAFT_COVERAGE = 1.5  # corrected by aerial audit, not 2, section 7

# --- Section 8: detection and the shared map -------------------------------

GPR_SCAN_WIDTH_CM = 30.0
GPR_MAX_DEPTH_CM = 40.0
PULSE_INDUCTION_SWEEP_WIDTH_CM = 40.0
PULSE_INDUCTION_SENSITIVITY_G = 5.0
PULSE_INDUCTION_SENSITIVITY_DEPTH_CM = 15.0
GRADIOMETER_SENSOR_SPACING_CM = 50.0
SAW_VOC_RANGE_M = 2.0
SAW_VOC_SAMPLE_RATE_HZ = 10.0

VOXEL_RESOLUTION_CM = 5.0
ALERT_CONFIDENCE_THRESHOLD = 0.85  # section 8.3
MESH_UPDATE_RATE_HZ = 1.0
MESH_LOAD_KBPS_LOW = 3.0
MESH_LOAD_KBPS_HIGH = 12.0

# Section 12.2 / 14: threat_type enum. The technical baseline states eight
# classes but only names seven. The eighth is an open item, confirm with
# Bhargav before locking this list, see DECISIONS.md. Do not add a guessed
# eighth value here.
THREAT_CLASSES = (
    "mine",
    "ied",
    "person",
    "vehicle",
    "disturbed_soil",
    "safe_object",
    "unknown",
)

# --- Section 9: interface -------------------------------------------------

DEFAULT_ALERT_CONFIRM_TIMEOUT_S = 30.0  # "configurable timeout", 8.3; not
# numerically specified in the spec, used as the UI default. See DECISIONS.md.

# --- Section 10: safety and arbitration ------------------------------------

IRREVERSIBLE_COMMAND_HOLD_S = 2.0  # deliberate second confirmation, 2s hold
LINK_LOSS_EMERGENCY_RETURN_S = 15.0  # per-unit Emergency Return trigger, 4.1

# --- Section 11.1: compute load -------------------------------------------

ROVER_COMPUTE_LOAD_W = 20.0  # Orin NX + STM32H7 pair, from L0.3 power budget

# --- Section 5: mission phases, in order -----------------------------------

MISSION_PHASES = (
    "deploy",
    "drive_and_scan",
    "reactive_launch",
    "aerial_recon",
    "fusion",
    "alert",
    "mark_and_route",
    "recharge_and_return",
)

# --- Section 4: modes -------------------------------------------------------

MISSION_MODES = ("STANDARD", "AUTONOMOUS", "GHOST", "LCA")
CONTROL_MODES = ("MANUAL", "GUIDED", "AUTO", "HOLD")
LINK_STATES = ("CONNECTED", "DEGRADED", "LOST")

FORMATION_PATTERNS = ("vee", "line", "column")
