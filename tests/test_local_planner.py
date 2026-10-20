"""The reactive layer: the part of the rover that will not drive into things."""

import math

import numpy as np

from ocu.core.local_planner import FOOTPRINT_CLEARANCE_M, LocalPlanner


def obstacles(*points) -> np.ndarray:
    return np.array(points, dtype=np.float64).reshape(-1, 2)


def command(planner, obs, x=0.0, y=0.0, heading=0.0, bearing=0.0, cap=2.0, decel=1.5):
    return planner.command(obs, x, y, heading, bearing, cap, decel)


def test_with_nothing_near_it_goes_where_the_route_says_at_the_speed_it_is_allowed():
    result = command(LocalPlanner(), obstacles(), bearing=40.0, cap=1.7)
    assert result.heading_deg == 40.0 and result.speed_limit_ms == 1.7 and not result.deviating


def test_an_obstacle_straight_ahead_makes_it_turn_aside():
    result = command(LocalPlanner(), obstacles((0.0, 2.5)))
    assert result.deviating
    assert 10.0 <= min(result.heading_deg, 360.0 - result.heading_deg) <= 90.0


def test_it_slows_when_every_way_on_is_short():
    ring = obstacles(*[(3.0 * math.sin(t), 3.0 * math.cos(t)) for t in np.linspace(0, 2 * math.pi, 40, endpoint=False)])
    result = command(LocalPlanner(), ring, cap=3.0)
    assert result.speed_limit_ms < 3.0


def test_it_can_always_stop_inside_the_distance_it_can_see_is_clear():
    """Allowed speed is no more than the vehicle can brake from over the free
    distance, whatever the cap."""
    wall = obstacles(*[(x * 0.5, 3.0) for x in range(-30, 31)])
    result = command(LocalPlanner(), wall, cap=4.0, decel=1.5)
    braking = result.speed_limit_ms**2 / (2 * 1.5)
    assert braking <= result.free_m + 1e-6


def test_it_never_picks_a_heading_that_would_touch_something():
    planner = LocalPlanner()
    rng = np.random.default_rng(3)
    for _ in range(60):
        obs = obstacles(*[(rng.uniform(-6, 6), rng.uniform(-6, 6)) for _ in range(rng.integers(1, 25))])
        if np.min(np.hypot(obs[:, 0], obs[:, 1])) < FOOTPRINT_CLEARANCE_M + 0.1:
            continue  # starting inside the margin is the next test's business
        result = command(planner, obs, bearing=float(rng.uniform(0, 360)))
        if result.free_m < 0.5:
            continue  # boxed in: nothing is clear
        rad = math.radians(result.heading_deg)
        for d in np.arange(0.25, result.free_m + 1e-6, 0.25):
            px, py = math.sin(rad) * d, math.cos(rad) * d
            assert np.min(np.hypot(obs[:, 0] - px, obs[:, 1] - py)) >= FOOTPRINT_CLEARANCE_M - 1e-6


def test_starting_inside_the_margin_it_leaves_instead_of_freezing():
    """An obstacle seen only when it is near: the rule is to open the gap."""
    obs = obstacles((0.0, 0.8))  # 0.8 m ahead, inside the 1.0 m margin
    result = command(LocalPlanner(), obs, bearing=0.0)
    rad = math.radians(result.heading_deg)
    after = math.hypot(math.sin(rad) * 0.5 - 0.0, math.cos(rad) * 0.5 - 0.8)
    assert after >= 0.8 - 0.02, "it must not close in"
    assert result.free_m >= 1.2


def test_it_keeps_to_one_side_rather_than_swinging_between_them():
    planner = LocalPlanner()
    obs = obstacles((0.0, 3.5), (0.5, 3.5), (-0.5, 3.5))
    first = command(planner, obs, bearing=0.0)
    side = 1 if first.heading_deg < 180 else -1
    for _ in range(10):
        again = command(planner, obs, heading=first.heading_deg, bearing=0.0)
        assert (1 if again.heading_deg < 180 else -1) == side


def test_a_gap_wide_enough_is_driven_through_and_a_gap_too_narrow_is_not():
    left = [(-1.6, y * 0.5) for y in range(4, 20)]
    right_wide = [(1.6, y * 0.5) for y in range(4, 20)]
    right_narrow = [(0.2, y * 0.5) for y in range(4, 20)]  # a 1.8 m gap: less than two margins
    wide = command(LocalPlanner(), obstacles(*left, *right_wide))
    assert wide.free_m >= 2.5 and wide.heading_deg in (0.0, 10.0, 350.0, 20.0, 340.0)
    narrow = command(LocalPlanner(), obstacles(*left, *right_narrow))
    assert narrow.free_m < 2.5 or narrow.heading_deg != 0.0
