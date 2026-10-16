"""The detection suite has to obey its own published limits, and the four
sensors have to fail in different ways from each other.

That second part is the whole argument for carrying four of them. If they
all responded to the same thing they would agree on the same false
positives, and cross-confirmation would be worth nothing.
"""

import math

from ocu.core.constants import (
    ALERT_CONFIDENCE_THRESHOLD,
    GPR_MAX_DEPTH_CM,
    PULSE_INDUCTION_SENSITIVITY_DEPTH_CM,
    PULSE_INDUCTION_SENSITIVITY_G,
    SAW_VOC_RANGE_M,
)
from ocu.core.detection import (
    REPORT_FLOOR,
    classify,
    fuse,
    gpr_response,
    gradiometer_response,
    pulse_induction_response,
    saw_voc_response,
    sweep,
)
from ocu.core.targets import BuriedTarget, default_field


def target(**kwargs) -> BuriedTarget:
    base = dict(
        x=0.0, y=0.0, depth_m=0.10, metal_mass_g=100.0, ferrous=True,
        voc_emission=0.5, truth_class="test",
    )
    base.update(kwargs)
    return BuriedTarget(**base)


# --- GPR: depth limited, material blind -----------------------------------


def test_gpr_sees_a_shallow_object_directly_beneath():
    assert gpr_response(target(depth_m=0.05), 0.0, 0.0) > 0.6


def test_gpr_cannot_see_past_its_published_depth():
    just_inside = GPR_MAX_DEPTH_CM / 100.0 - 0.02
    just_outside = GPR_MAX_DEPTH_CM / 100.0 + 0.02
    assert gpr_response(target(depth_m=just_inside), 0.0, 0.0) > 0.0
    assert gpr_response(target(depth_m=just_outside), 0.0, 0.0) == 0.0


def test_gpr_does_not_care_whether_the_object_is_metal():
    """A dielectric anomaly is an anomaly. This is why it complements PI."""
    metal = gpr_response(target(metal_mass_g=200.0), 0.0, 0.0)
    plastic = gpr_response(target(metal_mass_g=0.0), 0.0, 0.0)
    assert math.isclose(metal, plastic)


def test_gpr_is_limited_to_its_scan_width():
    assert gpr_response(target(), 0.0, 0.0) > 0.0
    assert gpr_response(target(), 0.0, 1.5) == 0.0


# --- pulse induction: calibrated to 5 g at 15 cm --------------------------


def test_pulse_induction_detects_its_published_sensitivity_case():
    """5 g of metal at 15 cm is the stated threshold, so it must report."""
    depth = PULSE_INDUCTION_SENSITIVITY_DEPTH_CM / 100.0
    response = pulse_induction_response(
        target(metal_mass_g=PULSE_INDUCTION_SENSITIVITY_G, depth_m=depth), 0.0, 0.0
    )
    assert response >= REPORT_FLOOR


def test_pulse_induction_ignores_objects_with_no_metal():
    assert pulse_induction_response(target(metal_mass_g=0.0), 0.0, 0.0) == 0.0


def test_pulse_induction_falls_off_steeply_with_depth():
    """An induced dipole falls as the cube of range, so depth is punishing."""
    shallow = pulse_induction_response(target(depth_m=0.05), 0.0, 0.0)
    deep = pulse_induction_response(target(depth_m=0.30), 0.0, 0.0)
    assert shallow > deep * 2.0


def test_more_metal_reads_stronger():
    small = pulse_induction_response(target(metal_mass_g=10.0), 0.0, 0.0)
    large = pulse_induction_response(target(metal_mass_g=200.0), 0.0, 0.0)
    assert large > small


# --- gradiometer: ferrous only, reaches deeper ----------------------------


def test_gradiometer_ignores_non_ferrous_metal():
    assert gradiometer_response(target(ferrous=False), 0.0, 0.0) == 0.0
    assert gradiometer_response(target(ferrous=True), 0.0, 0.0) > 0.0


def test_gradiometer_reaches_deeper_than_pulse_induction():
    """Section 8.1 states this directly."""
    deep = target(depth_m=0.45, metal_mass_g=120.0, ferrous=True)
    assert gradiometer_response(deep, 0.0, 0.0) > pulse_induction_response(deep, 0.0, 0.0)


def test_gradiometer_follows_a_fourth_power_law():
    """A gradiometer reads the *difference* between two magnetometers, which
    is the gradient of a dipole field and falls as the fourth power of
    range. That steepness is what lets it reject distant interference and
    still be run next to a vehicle.

    Checked in the unsaturated regime: a strong shallow target pins any
    real detector at full scale and hides the exponent.
    """
    weak = dict(metal_mass_g=25.0, ferrous=True)
    near = gradiometer_response(target(depth_m=0.30, **weak), 0.0, 0.0)
    far = gradiometer_response(target(depth_m=0.45, **weak), 0.0, 0.0)
    # Range ratio 1.5, so a fourth power law predicts roughly 1/5 the signal.
    assert 0.0 < far < near
    assert far / near < 0.35


# --- SAW VOC: downwind, close, and never primary --------------------------


def test_voc_requires_being_downwind():
    emitter = target(voc_emission=0.9, depth_m=0.05)
    downwind = saw_voc_response(emitter, 0.0, 1.0, wind_bearing_deg=0.0)
    upwind = saw_voc_response(emitter, 0.0, -1.0, wind_bearing_deg=0.0)
    assert downwind > 0.0
    assert upwind == 0.0


def test_voc_cannot_reach_beyond_its_published_range():
    emitter = target(voc_emission=0.9, depth_m=0.05)
    assert saw_voc_response(emitter, 0.0, SAW_VOC_RANGE_M + 0.5, 0.0) == 0.0


def test_voc_alone_can_never_escalate():
    """Section 8.1: a confirming sensor only, never primary."""
    for emission in (0.5, 0.9, 1.0):
        response = saw_voc_response(target(voc_emission=emission, depth_m=0.02), 0.0, 0.3, 0.0)
        assert response < ALERT_CONFIDENCE_THRESHOLD


# --- fusion: cross-confirmation is the point ------------------------------


def test_two_physics_agreeing_beats_either_alone():
    alone = fuse({"gpr": 0.6})
    together = fuse({"gpr": 0.6, "pulse_induction": 0.6})
    assert together > alone


def test_gpr_plus_pulse_induction_is_the_strongest_pairing():
    """Section 8.1 singles this pair out as the biggest lever there is."""
    different_physics = fuse({"gpr": 0.6, "pulse_induction": 0.6})
    same_physics = fuse({"magnetic_gradiometer": 0.6, "pulse_induction": 0.6})
    assert different_physics > same_physics


def test_fusion_never_reports_certainty():
    """A classifier that outputs 1.00 is either overfitting or lying, and an
    operator who sees it stops reading the number."""
    from ocu.core.detection import MAX_FUSED_CONFIDENCE

    saturated = fuse({"gpr": 0.99, "pulse_induction": 0.99, "magnetic_gradiometer": 0.99})
    assert saturated <= MAX_FUSED_CONFIDENCE
    assert saturated < 1.0


def test_fusion_of_nothing_is_nothing():
    assert fuse({}) == 0.0
    assert fuse({"gpr": 0.0}) == 0.0


def test_a_strong_shallow_target_escalates():
    hit = target(depth_m=0.06, metal_mass_g=150.0, ferrous=True, voc_emission=0.5)
    readings = sweep(hit, 0.0, 0.05, wind_bearing_deg=0.0)
    fused = fuse({r.sensor: r.confidence for r in readings})
    assert fused >= ALERT_CONFIDENCE_THRESHOLD


def test_metal_with_no_vapour_classifies_as_inert():
    """Scrap is the false positive the suite exists to reject."""
    scrap = target(metal_mass_g=250.0, ferrous=True, voc_emission=0.0, depth_m=0.30)
    readings = sweep(scrap, 0.0, 0.0)
    assert classify({r.sensor: r.confidence for r in readings}) == "safe_object"


def test_classification_never_invents_an_eighth_class():
    """The eighth threat class is still open, section 14."""
    from ocu.core.constants import THREAT_CLASSES

    for confidences in ({"gpr": 0.9}, {"pulse_induction": 0.9}, {"saw_voc": 0.3}, {}):
        label = classify(confidences)
        assert label in set(THREAT_CLASSES)


# --- the field ------------------------------------------------------------


def test_the_default_field_has_a_range_of_difficulty():
    """Four different outcomes, not four of the same one."""
    field = default_field()
    assert len(field) >= 8
    depths = [t.depth_m for t in field.targets]
    masses = [t.metal_mass_g for t in field.targets]
    assert max(depths) > 0.3 and min(depths) < 0.1
    assert max(masses) > 150 and min(masses) < 15
    assert any(t.ferrous for t in field.targets)
    assert any(not t.ferrous for t in field.targets)
    assert any(t.voc_emission == 0.0 for t in field.targets), "no inert scrap to reject"


def test_the_missed_object_is_beyond_every_sensor():
    """A sweep that finds everything teaches nothing about lane width."""
    from ocu.core.detection import sweep as run_sweep

    field = default_field()
    missed = next(t for t in field.targets if t.truth_class == "outside_swath")
    # The rover drives the y=0 lane past it.
    assert not run_sweep(missed, missed.x, 0.0, 0.0)


def test_nothing_is_detected_from_far_away():
    field = default_field()
    for buried in field.targets:
        assert not sweep(buried, buried.x + 40.0, buried.y + 40.0)
