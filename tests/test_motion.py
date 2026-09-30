"""The flight model: nothing starts at speed, nothing stops dead, nothing overshoots."""

import math

from ocu.core.motion import KinematicBody, MotionLimits, approach_speed, wrap180

LIMITS = MotionLimits(max_speed_ms=7.0, max_accel_ms2=3.0, max_decel_ms2=3.0)


def test_wrap180_takes_the_short_way():
    assert wrap180(350.0) == -10.0
    assert wrap180(-350.0) == 10.0
    assert wrap180(180.0) == -180.0  # range is [-180, 180)


def test_approach_speed_is_zero_on_arrival():
    assert approach_speed(0.0, 7.0, 3.0) == 0.0


def test_approach_speed_capped_by_stopping_distance():
    # From 1 m away, at 3 m/s^2, the fastest stoppable speed is sqrt(2*3*1).
    assert abs(approach_speed(1.0, 100.0, 3.0) - math.sqrt(6.0)) < 1e-9


def test_body_accelerates_rather_than_teleporting():
    body = KinematicBody(limits=LIMITS)
    body.step_toward(100.0, 0.0, 0.1)
    # One tick at 3 m/s^2 for 0.1 s is 0.3 m/s, so it cannot have moved 1 m.
    assert body.speed_ms <= LIMITS.max_accel_ms2 * 0.1 + 1e-9
    assert body.x < 0.05


def test_body_never_exceeds_max_speed():
    body = KinematicBody(limits=LIMITS)
    for _ in range(600):
        body.step_toward(1000.0, 0.0, 0.1)
        assert body.speed_ms <= LIMITS.max_speed_ms + 1e-6


def test_body_arrives_without_overshooting():
    body = KinematicBody(limits=LIMITS)
    overshoot = 0.0
    for _ in range(600):
        body.step_toward(30.0, 0.0, 0.05)
        overshoot = max(overshoot, body.x - 30.0)
    assert overshoot < 0.05
    assert abs(body.x - 30.0) < 0.1
    assert body.speed_ms < 0.1


def test_arrival_reports_true_only_when_stopped():
    body = KinematicBody(limits=LIMITS)
    arrived = False
    for _ in range(600):
        arrived = body.step_toward(10.0, 0.0, 0.05)
        if arrived:
            break
    assert arrived
    assert body.speed_ms < 0.05


def test_vertical_climb_and_descent_are_asymmetric():
    limits = MotionLimits(
        max_speed_ms=7.0, max_accel_ms2=3.0, max_decel_ms2=3.0,
        max_climb_ms=3.0, max_descend_ms=1.0, max_vertical_accel_ms2=50.0,
    )
    climber = KinematicBody(limits=limits)
    for _ in range(100):
        climber.step_altitude(100.0, 0.1)
    descender = KinematicBody(z=100.0, limits=limits)
    for _ in range(100):
        descender.step_altitude(0.0, 0.1)
    assert climber.z > 100.0 - descender.z  # climbed further than it descended


def test_altitude_never_goes_below_ground():
    body = KinematicBody(z=1.0, limits=LIMITS)
    for _ in range(200):
        body.step_altitude(-50.0, 0.1)
    assert body.z >= 0.0


def test_yaw_turns_the_short_way_around():
    body = KinematicBody(heading=10.0, limits=LIMITS)
    headings = []
    for _ in range(200):
        body.step_heading(350.0, 0.05)
        headings.append(body.heading)
    assert abs(wrap180(body.heading - 350.0)) < 1.5
    # Never swung through the far side.
    assert all(h > 300.0 or h < 60.0 for h in headings)


def test_wheeled_body_slows_when_it_must_turn():
    wheeled = MotionLimits(
        max_speed_ms=4.0, max_accel_ms2=1.0, max_decel_ms2=1.5,
        max_yaw_rate_deg_s=25.0, heading_couples_to_speed=True,
    )
    body = KinematicBody(heading=0.0, limits=wheeled)  # facing north, target due south
    body.step_toward(0.0, -10.0, 0.1)
    assert body.speed_ms < 0.05  # cannot drive backwards; must turn first


def test_halt_bleeds_speed_off_rather_than_snapping():
    body = KinematicBody(limits=LIMITS)
    for _ in range(100):
        body.step_toward(500.0, 0.0, 0.1)
    moving = body.speed_ms
    body.halt(0.1)
    assert body.speed_ms < moving
    assert body.speed_ms > 0.0  # did not stop dead in one tick


def test_manual_command_is_acceleration_limited():
    body = KinematicBody(limits=LIMITS)
    body.apply_horizontal_command(7.0, 0.0, 0.1)
    assert body.speed_ms <= LIMITS.max_accel_ms2 * 0.1 + 1e-9


def test_manual_command_clamped_to_max_speed():
    body = KinematicBody(limits=LIMITS)
    for _ in range(300):
        body.apply_horizontal_command(500.0, 500.0, 0.1)
    assert body.speed_ms <= LIMITS.max_speed_ms + 1e-6
