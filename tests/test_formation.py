"""Formation geometry. This is the part of the product that has to be exact.

Two conventions are load-bearing and easy to get wrong:

- Bearing is a compass heading. 0 is north, 90 is east. The map draws the
  bearing dial that way, so the formation has to be built that way, or the
  aircraft go somewhere other than where the operator pointed.
- The local frame is X forward along the bearing, Y to the right of it,
  which is what section 6.4 specifies.

Every pattern scouts *ahead* of the rover. An aircraft behind the rover,
or sitting on top of it, is not scouting anything.
"""

import math

from ocu.core.constants import (
    FORMATION_STANDOFF_M,
    MIN_AIRCRAFT_SEPARATION_M,
    TETHER_LIMIT_M,
)
from ocu.core.formation import clamp_spacing, clamp_to_tether, compute_formation_targets

PATTERNS = ["vee", "line", "column"]


def targets(pattern, bearing=0.0, spacing=8.0, altitude=20.0, rover=(0.0, 0.0), standoff=None):
    kwargs = {} if standoff is None else {"standoff_m": standoff}
    return compute_formation_targets(rover[0], rover[1], bearing, pattern, spacing, altitude, **kwargs)


def forward_right(target, bearing_deg, rover=(0.0, 0.0)):
    """Project a world target back into the formation's local frame."""
    bearing = math.radians(bearing_deg)
    dx, dy = target.x - rover[0], target.y - rover[1]
    forward = dx * math.sin(bearing) + dy * math.cos(bearing)
    right = dx * math.cos(bearing) - dy * math.sin(bearing)
    return forward, right


# --- the compass convention ---------------------------------------------


def test_bearing_zero_points_north():
    for pattern in PATTERNS:
        for target in targets(pattern, bearing=0.0):
            assert target.y > 0.0, f"{pattern} went south at bearing 0"
            assert abs(target.x) < TETHER_LIMIT_M


def test_bearing_ninety_points_east():
    for pattern in PATTERNS:
        for target in targets(pattern, bearing=90.0):
            assert target.x > 0.0, f"{pattern} did not go east at bearing 90"


def test_bearing_one_eighty_points_south():
    for target in targets("line", bearing=180.0):
        assert target.y < 0.0


def test_bearing_two_seventy_points_west():
    for target in targets("line", bearing=270.0):
        assert target.x < 0.0


def test_formation_rotates_rigidly_with_bearing():
    """The same formation at any bearing is the same shape, just turned."""
    reference = [forward_right(t, 0.0) for t in targets("vee", bearing=0.0)]
    for bearing in (37.0, 90.0, 180.0, 293.0):
        rotated = [forward_right(t, bearing) for t in targets("vee", bearing=bearing)]
        for (f1, r1), (f2, r2) in zip(reference, rotated):
            assert abs(f1 - f2) < 1e-6
            assert abs(r1 - r2) < 1e-6


# --- every pattern scouts ahead -------------------------------------------


def test_every_slot_is_ahead_of_the_rover():
    for pattern in PATTERNS:
        for bearing in (0.0, 45.0, 217.0):
            for target in targets(pattern, bearing=bearing):
                forward, _ = forward_right(target, bearing)
                assert forward > 0.0, f"{pattern} put a slot level with or behind the rover"


def test_no_slot_sits_on_top_of_the_rover():
    for pattern in PATTERNS:
        for target in targets(pattern):
            assert math.hypot(target.x, target.y) > MIN_AIRCRAFT_SEPARATION_M


def test_slots_respect_minimum_separation_from_each_other():
    for pattern in PATTERNS:
        slots = targets(pattern, spacing=MIN_AIRCRAFT_SEPARATION_M)
        for i, a in enumerate(slots):
            for b in slots[i + 1 :]:
                separation = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                assert separation >= MIN_AIRCRAFT_SEPARATION_M - 1e-6


# --- what each pattern is for ---------------------------------------------


def test_line_is_abreast_not_trailing():
    """A wide sweep means all three level with each other, spread across the
    track. Strung out along it is a column, which is a different order."""
    slots = targets("line", bearing=0.0, spacing=10.0)
    forwards = [forward_right(t, 0.0)[0] for t in slots]
    rights = sorted(forward_right(t, 0.0)[1] for t in slots)
    assert max(forwards) - min(forwards) < 1e-6, "line abreast must be level"
    assert rights[0] < 0 < rights[2], "line abreast must straddle the track"
    assert abs(rights[2] - rights[0]) >= 2 * 10.0 - 1e-6, "sweep must be spacing-wide"


def test_vee_has_a_leader_with_swept_back_wings():
    slots = targets("vee", bearing=0.0, spacing=10.0)
    local = [forward_right(t, 0.0) for t in slots]
    leader = max(local, key=lambda fr: fr[0])
    wings = [fr for fr in local if fr is not leader]
    assert abs(leader[1]) < 1e-6, "leader flies on the track"
    assert all(w[0] < leader[0] for w in wings), "wings must be swept back"
    assert abs(wings[0][1] + wings[1][1]) < 1e-6, "wings must be symmetric"


def test_column_is_single_file_along_the_track():
    slots = targets("column", bearing=0.0, spacing=8.0)
    local = [forward_right(t, 0.0) for t in slots]
    assert all(abs(right) < 1e-6 for _, right in local), "column must not spread sideways"
    forwards = sorted(f for f, _ in local)
    assert forwards[0] < forwards[1] < forwards[2], "column must be strung out in depth"


def test_patterns_are_actually_different_shapes():
    shapes = {p: sorted(forward_right(t, 0.0) for t in targets(p)) for p in PATTERNS}
    assert shapes["line"] != shapes["column"] != shapes["vee"] != shapes["line"]


# --- the tether -----------------------------------------------------------


def test_no_slot_ever_leaves_the_tether():
    for pattern in PATTERNS:
        for spacing in (5.0, 40.0, 500.0):
            for target in targets(pattern, spacing=spacing, standoff=TETHER_LIMIT_M):
                assert math.hypot(target.x, target.y) <= TETHER_LIMIT_M + 1e-6


def test_oversized_formation_shrinks_without_distorting():
    """Clamping each slot separately would flatten the formation against the
    tether ring. The whole shape scales instead, so a vee stays a vee."""
    small = [forward_right(t, 0.0) for t in targets("vee", spacing=6.0, standoff=10.0)]
    huge = [forward_right(t, 0.0) for t in targets("vee", spacing=400.0, standoff=400.0)]

    def aspect(local):
        spread = max(r for _, r in local) - min(r for _, r in local)
        depth = max(f for f, _ in local) - min(f for f, _ in local)
        return spread / depth

    assert abs(aspect(small) - aspect(huge)) < 1e-6


def test_every_aircraft_flies_the_altitude_the_operator_asked_for():
    """Ask for 26 m and all three read 26 m. A formation is already separated
    on the ground plane by its own spacing, so stacking it in height as well
    buys nothing and costs the operator the number they actually set."""
    for pattern in ("vee", "line", "column"):
        slots = targets(pattern, spacing=12.0, altitude=26.0)
        assert [t.z for t in slots] == [26.0, 26.0, 26.0], pattern


def test_a_formation_too_tight_to_separate_itself_is_layered_instead():
    """The exception, and the reason the layering exists: slots closer than
    the minimum separation in plan are spread in height so that two aircraft
    pushed off station cannot arrive in the same place."""
    from ocu.core.constants import MIN_AIRCRAFT_SEPARATION_M

    slots = targets("column", spacing=0.5, altitude=25.0)  # clamped, then squeezed
    plan_gaps = [
        math.hypot(a.x - b.x, a.y - b.y)
        for i, a in enumerate(slots)
        for b in slots[i + 1 :]
    ]
    if min(plan_gaps) < MIN_AIRCRAFT_SEPARATION_M:
        assert len(set(t.z for t in slots)) == 3, "nothing separates them, so height must"
        assert all(t.z >= 1.0 for t in slots)


def test_a_layered_formation_is_still_centred_on_the_commanded_altitude():
    import ocu.core.formation as formation

    altitudes = formation._slot_altitudes(30.0, [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])
    assert len(set(altitudes)) == 3
    assert abs(sum(altitudes) / 3 - 30.0) < 1e-6


def test_no_slot_is_ever_below_a_safe_height():
    import ocu.core.formation as formation

    altitudes = formation._slot_altitudes(1.0, [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])
    assert all(z >= 1.0 for z in altitudes)


# --- heading and inputs ---------------------------------------------------


def test_all_aircraft_face_the_formation_bearing():
    for bearing in (0.0, 47.0, 300.0):
        for target in targets("vee", bearing=bearing):
            assert abs(target.heading - bearing) < 1e-6


def test_standoff_defaults_into_the_scout_band():
    slots = targets("line", bearing=0.0)
    forwards = [forward_right(t, 0.0)[0] for t in slots]
    assert abs(forwards[0] - FORMATION_STANDOFF_M) < 1e-6


def test_clamp_spacing_enforces_hard_minimum_in_code():
    assert clamp_spacing(0.1) == MIN_AIRCRAFT_SEPARATION_M
    assert clamp_spacing(MIN_AIRCRAFT_SEPARATION_M + 10) == MIN_AIRCRAFT_SEPARATION_M + 10


def test_clamp_to_tether_leaves_inside_points_alone():
    assert clamp_to_tether(3.0, 4.0, 0.0, 0.0) == (3.0, 4.0)


def test_unknown_pattern_rejected():
    try:
        compute_formation_targets(0, 0, 0, "diamond", 5, 20)
        assert False, "expected ValueError"
    except ValueError:
        pass
