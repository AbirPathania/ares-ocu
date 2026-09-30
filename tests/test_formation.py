import math

from ocu.core.constants import MIN_AIRCRAFT_SEPARATION_M
from ocu.core.formation import clamp_spacing, compute_formation_targets


def test_bearing_zero_vee_matches_local_frame_offsets():
    targets = compute_formation_targets(
        rover_x=0, rover_y=0, bearing_deg=0, pattern="vee", spacing_m=5, rally_altitude_m=20
    )
    assert targets[0].x == 0 and targets[0].y == 0
    assert math.isclose(targets[1].x, -5) and math.isclose(targets[1].y, 5)
    assert math.isclose(targets[2].x, -5) and math.isclose(targets[2].y, -5)
    assert all(t.z == 20 for t in targets)


def test_bearing_90_rotates_offsets():
    targets = compute_formation_targets(
        rover_x=0, rover_y=0, bearing_deg=90, pattern="line", spacing_m=5, rally_altitude_m=20
    )
    # line: [(0,-s),(0,0),(0,s)] rotated 90deg -> (s,0),(0,0),(-s,0)
    assert math.isclose(targets[0].x, 5, abs_tol=1e-9)
    assert math.isclose(targets[0].y, 0, abs_tol=1e-9)
    assert math.isclose(targets[2].x, -5, abs_tol=1e-9)


def test_translation_by_rover_position():
    targets = compute_formation_targets(
        rover_x=100, rover_y=50, bearing_deg=0, pattern="column", spacing_m=4, rally_altitude_m=15
    )
    assert targets[0].x == 100 and targets[0].y == 50
    assert math.isclose(targets[1].x, 96) and targets[1].y == 50
    assert math.isclose(targets[2].x, 92) and targets[2].y == 50


def test_clamp_spacing_enforces_hard_minimum_in_code():
    assert clamp_spacing(0.1) == MIN_AIRCRAFT_SEPARATION_M
    assert clamp_spacing(MIN_AIRCRAFT_SEPARATION_M + 10) == MIN_AIRCRAFT_SEPARATION_M + 10


def test_unknown_pattern_rejected():
    try:
        compute_formation_targets(0, 0, 0, "diamond", 5, 20)
        assert False, "expected ValueError"
    except ValueError:
        pass
