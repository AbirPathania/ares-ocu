"""Which way to send scouts: toward the ground the rover cannot see.

Reactive launch is the rover's own planner reaching the end of what its
sensors can resolve. The direction has to come from its map, and the
property that makes it meaningful is that unknown ground *inside* sensor
range is hidden ground (a LiDAR cannot see through a rock), while unknown
ground beyond range is merely far.
"""

import math

from ocu.core.lidar import LIDAR_RANGE_M
from ocu.core.occlusion import FAR_M, HALF_ARC_DEG, NEAR_M, occlusion_bearing
from ocu.core.terrain import CELL_SIZE_M, PerceptionMap, Terrain


def known_disc(radius_m: float = 40.0) -> PerceptionMap:
    perception = PerceptionMap()
    reach = int(radius_m / CELL_SIZE_M)
    for cx in range(-reach, reach + 1):
        for cy in range(-reach, reach + 1):
            perception.observe((cx, cy), Terrain.GROUND)
    return perception


def forget_sector(perception: PerceptionMap, bearing_deg: float, half_width_deg: float = 8.0) -> None:
    """Make the ground in one direction unknown, as if something stood in front of it."""
    reach = int(FAR_M / CELL_SIZE_M) + 4
    for cx in range(-reach, reach + 1):
        for cy in range(-reach, reach + 1):
            x, y = cx * CELL_SIZE_M, cy * CELL_SIZE_M
            if not (NEAR_M <= math.hypot(x, y) <= FAR_M + 2):
                continue
            to_cell = math.degrees(math.atan2(x, y)) % 360.0
            if abs((to_cell - bearing_deg + 180.0) % 360.0 - 180.0) <= half_width_deg:
                perception._observed.pop((cx, cy), None)


def test_nothing_hidden_means_straight_ahead_and_nothing_to_report():
    bearing, hidden = occlusion_bearing(known_disc(), 0.0, 0.0, 130.0)
    assert hidden == 0.0
    assert bearing == 130.0


def test_the_bearing_points_at_the_hidden_ground():
    for target in (20.0, 75.0, 140.0):
        perception = known_disc()
        forget_sector(perception, target)
        bearing, hidden = occlusion_bearing(perception, 0.0, 0.0, 90.0)
        assert hidden > 0.5
        assert abs((bearing - target + 180.0) % 360.0 - 180.0) <= 10.0, (target, bearing)


def test_it_looks_ahead_not_behind():
    """A scout sent behind the vehicle looks at ground it has just driven over."""
    perception = known_disc()
    forget_sector(perception, 270.0)  # directly behind a rover heading 90
    bearing, hidden = occlusion_bearing(perception, 0.0, 0.0, 90.0)
    assert hidden == 0.0
    assert bearing == 90.0


def test_it_follows_the_vehicle_when_it_turns():
    perception = known_disc()
    forget_sector(perception, 200.0)
    assert occlusion_bearing(perception, 0.0, 0.0, 180.0)[1] > 0.5
    assert occlusion_bearing(perception, 0.0, 0.0, 0.0)[1] == 0.0, "behind it now"


def test_unknown_beyond_sensor_range_is_far_not_hidden():
    perception = known_disc(radius_m=FAR_M)
    bearing, hidden = occlusion_bearing(perception, 0.0, 0.0, 0.0)
    assert hidden == 0.0


def test_unknown_closer_than_the_near_limit_is_not_counted():
    perception = known_disc()
    reach = int(NEAR_M / CELL_SIZE_M)
    for cx in range(-reach, reach + 1):
        for cy in range(-reach, reach + 1):
            if math.hypot(cx, cy) * CELL_SIZE_M < NEAR_M - 1.0:
                perception._observed.pop((cx, cy), None)
    assert occlusion_bearing(perception, 0.0, 0.0, 0.0)[1] == 0.0


def test_with_nothing_known_at_all_it_scouts_straight_ahead():
    """Everything hidden equally: no reason to look anywhere but forward."""
    bearing, hidden = occlusion_bearing(PerceptionMap(), 0.0, 0.0, 250.0)
    assert hidden == 1.0
    assert bearing == 250.0


def test_the_arc_is_bounded_either_side_of_travel():
    assert HALF_ARC_DEG < 90.0, "never behind the vehicle"
    assert FAR_M < LIDAR_RANGE_M, "only ground the sensor should already have covered"


def test_it_works_from_wherever_the_rover_is():
    perception = known_disc(radius_m=100.0)
    for cx in range(int(60 / CELL_SIZE_M), int(70 / CELL_SIZE_M)):
        for cy in range(int(40 / CELL_SIZE_M), int(50 / CELL_SIZE_M)):
            perception._observed.pop((cx, cy), None)
    bearing, hidden = occlusion_bearing(perception, 50.0, 30.0, 0.0)
    # The hole spans bearings 27 to 63 degrees from (50, 30), 10 to 28 m away.
    assert hidden > 0.0
    assert 27.0 <= bearing <= 63.0
