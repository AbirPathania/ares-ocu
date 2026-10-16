"""The detection suite: four sensors, four physical response models, and
the cross-confirmation that section 8.1 calls the biggest lever there is
on false alarm rate.

Every figure that defines a sensor's reach comes from the baseline:

    GPR                 0 to 40 cm depth, 30 cm scan width per pass
    Pulse induction     40 cm aggregate sweep, 5 g of metal at 15 cm
    Magnetic gradiometer two fluxgates 50 cm apart, ferrous only, reaches
                        deeper than the pulse induction array
    SAW VOC             effective to 2 m downwind, 10 Hz sampling, a close
                        range confirming sensor only, never primary

What is this build's model rather than the baseline's is the *shape* of
each response between those limits: the baseline states ranges, not
falloff curves. The curves used are the physically expected ones. A
pulse induction coil sees a conductive object as an induced dipole, whose
signal falls roughly as the cube of range. A gradiometer measures the
*difference* between two magnetometers, which falls off faster again,
roughly as the fourth power, which is exactly why it rejects distant
interference and can still be run next to a vehicle. Vapour disperses, so
the VOC response falls off sharply and only downwind.

Fusion is deliberately not a weighted average. Two sensors that work on
different physics agreeing about the same point is far stronger evidence
than either of them being individually confident, and a weighted average
cannot express that. Independent evidence is combined, and co-located
hits on different physics are rewarded on top.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.constants import (
    GPR_MAX_DEPTH_CM,
    GPR_SCAN_WIDTH_CM,
    GRADIOMETER_SENSOR_SPACING_CM,
    PULSE_INDUCTION_SENSITIVITY_DEPTH_CM,
    PULSE_INDUCTION_SENSITIVITY_G,
    PULSE_INDUCTION_SWEEP_WIDTH_CM,
    SAW_VOC_RANGE_M,
)
from ocu.core.targets import BuriedTarget

#: Sensors report nothing below this; it is noise, not a detection.
REPORT_FLOOR = 0.12

#: No fused result is ever reported as certainty. A classifier that
#: outputs 1.00 is either overfitting or lying, and an operator who sees
#: it stops reading the number. The ceiling leaves the doubt visible.
MAX_FUSED_CONFIDENCE = 0.97

#: The gradiometer reaches past the pulse induction array, per section 8.1.
GRADIOMETER_DEPTH_REACH_M = 0.60


@dataclass(frozen=True)
class SensorReading:
    sensor: str
    confidence: float
    target: BuriedTarget


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


# --- individual sensors ---------------------------------------------------


def gpr_response(target: BuriedTarget, x: float, y: float) -> float:
    """Ground penetrating radar: a dielectric anomaly, any material.

    Reach is depth limited at 40 cm and laterally limited to the 30 cm
    scan width of a pass. It does not care whether the object is metal,
    which is what makes it the complement to pulse induction.
    """
    max_depth_m = GPR_MAX_DEPTH_CM / 100.0
    half_swath_m = GPR_SCAN_WIDTH_CM / 200.0

    lateral = target.lateral_offset(x, y)
    if lateral > half_swath_m or target.depth_m > max_depth_m:
        return 0.0

    depth_factor = 1.0 - (target.depth_m / max_depth_m) ** 1.6
    lateral_factor = 1.0 - (lateral / half_swath_m) ** 2
    return _clamp(0.92 * depth_factor * lateral_factor)


def pulse_induction_response(target: BuriedTarget, x: float, y: float) -> float:
    """Pulse induction: eddy currents in conductive mass.

    Calibrated against the baseline's stated sensitivity, 5 g of metal at
    15 cm, which is taken as the threshold of a reportable detection.
    Signal from an induced dipole falls as the cube of range, so mass and
    depth trade against each other steeply.
    """
    if target.metal_mass_g <= 0.0:
        return 0.0

    half_swath_m = PULSE_INDUCTION_SWEEP_WIDTH_CM / 200.0
    lateral = target.lateral_offset(x, y)
    if lateral > half_swath_m:
        return 0.0

    reference_range_m = PULSE_INDUCTION_SENSITIVITY_DEPTH_CM / 100.0
    slant = max(target.range_to(x, y), 0.02)

    # Signal relative to the calibration point, which is defined as the
    # threshold of detection.
    relative = (target.metal_mass_g / PULSE_INDUCTION_SENSITIVITY_G) * (
        reference_range_m / slant
    ) ** 3
    # Scaled so the calibration case, 5 g at 15 cm, lands just above the
    # reporting floor: the baseline calls that the threshold of detection,
    # not a confident one. Everything stronger has room to vary rather than
    # pinning at the ceiling, which a saturated model would do.
    return _clamp(0.88 * (1.0 - math.exp(-relative / 5.35)))


def gradiometer_response(target: BuriedTarget, x: float, y: float) -> float:
    """Magnetic gradiometer: ferrous anomalies only.

    Two fluxgates 50 cm apart measure a difference rather than a field, so
    Earth's field cancels and only a local anomaly survives. A difference
    falls off faster than the field itself, roughly as the fourth power of
    range, which is why it rejects distant interference but still reaches
    deeper than pulse induction on ferrous mass.
    """
    if not target.ferrous or target.metal_mass_g <= 0.0:
        return 0.0
    if target.depth_m > GRADIOMETER_DEPTH_REACH_M:
        return 0.0

    slant = max(target.range_to(x, y), 0.05)
    # Referenced to 40 g of ferrous mass at 30 cm, which sits mid-scale.
    # The fourth power is the gradient of a dipole field: a difference
    # between two sensors falls off faster than the field each one reads.
    relative = (target.metal_mass_g / 40.0) * (0.30 / slant) ** 4
    return _clamp(0.84 * (1.0 - math.exp(-relative / 1.2)))


def saw_voc_response(
    target: BuriedTarget, x: float, y: float, wind_bearing_deg: float = 0.0
) -> float:
    """SAW VOC: chemical trace, downwind, close range, confirming only.

    Section 8.1 is explicit that this is never a primary detection, so its
    output is capped below the escalation threshold. It can corroborate
    and it cannot escalate on its own.
    """
    if target.voc_emission <= 0.0:
        return 0.0

    lateral = target.lateral_offset(x, y)
    if lateral > SAW_VOC_RANGE_M:
        return 0.0

    # Vapour travels downwind: the sensor has to be on the plume side.
    bearing_to_sensor = math.degrees(math.atan2(x - target.x, y - target.y))
    offset = abs((bearing_to_sensor - wind_bearing_deg + 180.0) % 360.0 - 180.0)
    if offset > 70.0:
        return 0.0

    plume = math.cos(math.radians(offset))
    dispersal = 1.0 - (lateral / SAW_VOC_RANGE_M) ** 1.5
    depth_damping = math.exp(-target.depth_m / 0.18)
    return _clamp(0.55 * target.voc_emission * plume * dispersal * depth_damping)


SENSORS = {
    "gpr": gpr_response,
    "pulse_induction": pulse_induction_response,
    "magnetic_gradiometer": gradiometer_response,
}


def sweep(
    target: BuriedTarget, x: float, y: float, wind_bearing_deg: float = 0.0
) -> list[SensorReading]:
    """Run the whole suite over one object from one position."""
    readings = []
    for name, model in SENSORS.items():
        confidence = model(target, x, y)
        if confidence >= REPORT_FLOOR:
            readings.append(SensorReading(name, confidence, target))

    voc = saw_voc_response(target, x, y, wind_bearing_deg)
    if voc >= REPORT_FLOOR:
        readings.append(SensorReading("saw_voc", voc, target))
    return readings


# --- fusion ---------------------------------------------------------------


def fuse(confidences: dict[str, float]) -> float:
    """Combine evidence from sensors working on different physics.

    Not an average. Two independent sensors agreeing about one point is
    much stronger than either alone, so the individual confidences are
    combined as independent evidence, and cross-confirmation across
    different physics is rewarded on top of that. Section 8.1 singles out
    a co-located GPR and pulse induction hit as the biggest lever on false
    alarm rate, and that pairing gets the largest bonus here: radar and
    induction fail in different ways, so agreement between them is hard to
    produce by accident.
    """
    present = {k: v for k, v in confidences.items() if v > 0.0}
    if not present:
        return 0.0

    # Independent evidence: the chance every sensor is wrong at once.
    combined = 1.0
    for value in present.values():
        combined *= 1.0 - value
    fused = 1.0 - combined

    if "gpr" in present and "pulse_induction" in present:
        # Dielectric anomaly and conductive mass in the same place.
        fused += 0.12 * min(present["gpr"], present["pulse_induction"])
    if "magnetic_gradiometer" in present and "pulse_induction" in present:
        # Two magnetic methods agreeing is weaker corroboration than two
        # different physics, so it earns less.
        fused += 0.05 * min(present["magnetic_gradiometer"], present["pulse_induction"])
    if "saw_voc" in present and len(present) > 1:
        fused += 0.06 * present["saw_voc"]

    return min(MAX_FUSED_CONFIDENCE, _clamp(fused))


def classify(confidences: dict[str, float], target_hint: str = "") -> str:
    """The suite's best guess at what it is looking at.

    Deliberately conservative, and limited to the seven classes section
    12.2 actually names. The eighth is an open item (section 14) and
    nothing here invents it.

    The reasoning is the sensors': explosive vapour over a metal signature
    is a buried charge; metal with no vapour and no subsurface structure
    is scrap; structure in the ground with no metal at all is ground that
    has been worked. Anything thinner than that stays unknown, which is a
    real answer and the one the operator needs when the evidence is thin.
    """
    metal = max(
        confidences.get("pulse_induction", 0.0), confidences.get("magnetic_gradiometer", 0.0)
    )
    vapour = confidences.get("saw_voc", 0.0)
    anomaly = confidences.get("gpr", 0.0)

    # Telling a mine from an IED takes more than four sensors, so of the
    # two this suite reports the one it can actually support.
    if vapour > 0.0 and metal > 0.30:
        return "mine"
    if metal > 0.45 and vapour <= 0.0 and anomaly < 0.35:
        return "safe_object"
    if anomaly > 0.45 and metal < 0.25:
        return "disturbed_soil"
    return "unknown"
