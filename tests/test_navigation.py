"""The navigation stack: inflation, planning, replanning, following.

The test that matters most is that an inflated costmap refuses a gap the
vehicle cannot fit through. A planner that routes a point mass through a
1 m gap produces a plan that looks perfect and puts the rover into a rock.
"""

import math

from ocu.core.navigation import (
    INFLATION_MARGIN_M,
    ROVER_RADIUS_M,
    build_costmap,
    path_is_clear,
    plan_path,
    pure_pursuit_target,
    simplify,
)
from ocu.core.terrain import CELL_SIZE_M, TRAVERSE_COST, PerceptionMap, Terrain


def open_ground(extent: int = 40) -> PerceptionMap:
    perception = PerceptionMap()
    for x in range(-extent, extent + 1):
        for y in range(-extent, extent + 1):
            perception.observe((x, y), Terrain.GROUND)
    return perception


def wall(perception: PerceptionMap, x: int, y_range: range, gap: range = range(0)) -> None:
    for y in y_range:
        if y not in gap:
            perception.observe((x, y), Terrain.ROCK)


# --- costmap and inflation ------------------------------------------------


def test_obstacles_are_inflated_by_the_vehicle_radius():
    perception = open_ground()
    perception.observe((5, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    assert costmap.is_blocked((5, 0))
    just_inside = int(ROVER_RADIUS_M / CELL_SIZE_M)
    assert costmap.is_blocked((5 + just_inside, 0)), "inflation must cover the vehicle radius"


def test_cost_decays_across_the_margin_rather_than_stopping_dead():
    perception = open_ground()
    perception.observe((5, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    near = int((ROVER_RADIUS_M + 0.2) / CELL_SIZE_M)
    far = int((ROVER_RADIUS_M + INFLATION_MARGIN_M - 0.1) / CELL_SIZE_M)
    assert costmap.cost((5 + near, 0)) > costmap.cost((5 + far, 0))


def test_scrub_costs_more_to_cross_than_track():
    perception = PerceptionMap()
    perception.observe((0, 0), Terrain.TRACK)
    perception.observe((1, 0), Terrain.SCRUB)
    costmap = build_costmap(perception, CELL_SIZE_M)
    assert costmap.cost((1, 0)) > costmap.cost((0, 0))


def test_unknown_ground_is_neither_free_nor_blocked():
    """Free sends the rover into terrain nobody has looked at; blocked
    leaves it unable to plan at all on the first tick."""
    costmap = build_costmap(PerceptionMap(), CELL_SIZE_M)
    unknown = costmap.cost((99, 99))
    assert 0.0 < unknown < math.inf


# --- planning -------------------------------------------------------------


def test_a_straight_run_is_planned_across_open_ground():
    costmap = build_costmap(open_ground(), CELL_SIZE_M)
    path = plan_path(costmap, (0, 0), (20, 0))
    assert path
    assert path[0] == (0, 0) and path[-1] == (20, 0)


def test_the_planner_routes_around_an_obstacle():
    perception = open_ground()
    wall(perception, 10, range(-6, 7))
    costmap = build_costmap(perception, CELL_SIZE_M)

    path = plan_path(costmap, (0, 0), (20, 0))
    assert path, "a route around the wall exists and must be found"
    assert not any(costmap.is_blocked(cell) for cell in path)
    # It cannot have gone through: every cell on the wall line is inflated.
    assert all(not (cell[0] == 10 and -6 <= cell[1] <= 6) for cell in path)


def test_the_planner_refuses_a_gap_the_vehicle_cannot_fit():
    """A one metre gap with a 0.9 m vehicle radius is not a gap. This is the
    failure that inflation exists to prevent."""
    perception = open_ground()
    wall(perception, 10, range(-30, 31), gap=range(0, 2))
    costmap = build_costmap(perception, CELL_SIZE_M)

    path = plan_path(costmap, (0, 0), (20, 0))
    if path:
        assert all(cell[0] != 10 or cell[1] not in (0, 1) for cell in path)


def test_a_wide_gap_is_used():
    perception = open_ground()
    wall(perception, 10, range(-30, 31), gap=range(-6, 7))
    costmap = build_costmap(perception, CELL_SIZE_M)
    path = plan_path(costmap, (0, 0), (20, 0))
    assert path
    assert any(cell[0] == 10 for cell in path), "a gap this wide should be driven through"


def test_an_unreachable_goal_returns_no_path_rather_than_a_wrong_one():
    perception = open_ground()
    # A barrier far longer than the area the rover knows, so there is no
    # way round it through explored or unexplored ground alike.
    for y in range(-220, 221):
        perception.observe((10, y), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)
    assert plan_path(costmap, (0, 0), (20, 0), max_expansions=60_000) == []


def test_planning_to_where_you_already_are():
    costmap = build_costmap(open_ground(), CELL_SIZE_M)
    assert plan_path(costmap, (3, 3), (3, 3)) == [(3, 3)]


# --- replanning -----------------------------------------------------------


def test_a_plan_is_invalidated_by_newly_discovered_ground():
    """A plan is only valid against what was known when it was made."""
    perception = open_ground()
    costmap = build_costmap(perception, CELL_SIZE_M)
    path = plan_path(costmap, (0, 0), (20, 0))
    assert path_is_clear(costmap, path)

    # The rover drives on and finds a rock sitting on its route.
    perception.observe((10, 0), Terrain.ROCK)
    assert not path_is_clear(build_costmap(perception, CELL_SIZE_M), path)


def test_replanning_finds_a_way_round_what_was_just_discovered():
    perception = open_ground()
    wall(perception, 10, range(-6, 7))
    costmap = build_costmap(perception, CELL_SIZE_M)
    replanned = plan_path(costmap, (0, 0), (20, 0))
    assert replanned and path_is_clear(costmap, replanned)


# --- following ------------------------------------------------------------


def test_pure_pursuit_aims_ahead_not_at_the_nearest_point():
    path = [(float(i), 0.0) for i in range(20)]
    target = pure_pursuit_target(path, 5.0, 0.0, lookahead_m=3.0)
    assert target is not None
    assert target[0] >= 8.0 - 1e-9, "must aim a lookahead ahead of the vehicle"


def test_pure_pursuit_does_not_latch_onto_path_already_driven():
    """Walking from the closest point forward is what stops the vehicle
    turning back toward where it has been."""
    path = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (15.0, 0.0)]
    target = pure_pursuit_target(path, 12.0, 0.0, lookahead_m=2.0)
    assert target[0] > 12.0


def test_pure_pursuit_settles_on_the_end_of_the_path():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert pure_pursuit_target(path, 1.9, 0.0, lookahead_m=5.0) == (2.0, 0.0)


def test_pure_pursuit_on_an_empty_path_is_none():
    assert pure_pursuit_target([], 0.0, 0.0) is None


def test_simplify_keeps_corners_and_drops_straight_runs():
    straight = [(i, 0) for i in range(10)]
    assert len(simplify(straight)) == 2

    cornered = [(i, 0) for i in range(6)] + [(5, j) for j in range(1, 6)]
    simplified = simplify(cornered)
    assert 2 < len(simplified) < len(cornered)
    assert simplified[0] == cornered[0] and simplified[-1] == cornered[-1]


# --- the loop closed: drive, discover, replan ------------------------------


def test_the_rover_replans_when_it_discovers_a_rock_on_its_route():
    """The behaviour this whole stack exists for. The rover plans across
    ground it has not seen, drives, finds an obstacle on the planned line,
    and routes around it without being told."""
    from ocu.core.flight_director import FlightDirector, RoverMode
    from ocu.core.terrain import CELL_SIZE_M

    director = FlightDirector()
    perception = open_ground(extent=60)
    director.navigate_to(25.0, 0.0)
    assert director.rover_mode is RoverMode.NAVIGATING

    assert director.replan(perception, CELL_SIZE_M), "must plan across open ground"
    first_plan = list(director.path)
    plans_before = director.replan_count

    # Drive a little, then discover a wall sitting across the route.
    for _ in range(40):
        director.tick(0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading)
    wall(perception, 20, range(-7, 8))

    assert director.replan(perception, CELL_SIZE_M), "a blocked plan must be rebuilt"
    assert director.replan_count > plans_before
    assert director.path != first_plan

    costmap = build_costmap(perception, CELL_SIZE_M)
    cells = [(int(px // CELL_SIZE_M), int(py // CELL_SIZE_M)) for px, py in director.path]
    assert path_is_clear(costmap, cells), "the new route must avoid what was found"


def test_an_unobstructed_plan_is_not_rebuilt_every_tick():
    """Replanning costs real work; it should only happen when the map
    actually invalidates the route."""
    from ocu.core.flight_director import FlightDirector
    from ocu.core.terrain import CELL_SIZE_M

    director = FlightDirector()
    perception = open_ground(extent=60)
    director.navigate_to(20.0, 0.0)
    director.replan(perception, CELL_SIZE_M)
    count = director.replan_count

    for _ in range(10):
        director.replan(perception, CELL_SIZE_M)
    assert director.replan_count == count


def test_the_rover_actually_drives_its_route():
    from ocu.core.flight_director import FlightDirector
    from ocu.core.terrain import CELL_SIZE_M

    director = FlightDirector()
    perception = open_ground(extent=80)
    director.navigate_to(22.0, 0.0)
    director.replan(perception, CELL_SIZE_M)

    for _ in range(900):
        director.tick(0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading)
        if director.goal is None:
            break
    assert math.hypot(director.rover_body.x - 22.0, director.rover_body.y) < 3.0


# --- the costmap is maintained, not rebuilt --------------------------------


def test_an_incrementally_updated_costmap_matches_a_full_rebuild():
    """The property the whole optimisation rests on. If folding in only what
    changed ever disagrees with reading the whole map, the rover plans
    against a different world than the one it mapped."""
    from ocu.core.navigation import update_costmap

    perception = PerceptionMap()
    maintained = build_costmap(perception, CELL_SIZE_M)

    for step in range(6):
        for x in range(step * 4, step * 4 + 4):
            for y in range(-8, 9):
                perception.observe((x, y), Terrain.SCRUB if y % 5 else Terrain.GROUND)
        perception.observe((step * 4 + 1, 3), Terrain.ROCK)
        perception.observe((step * 4 + 2, -4), Terrain.TREE)
        update_costmap(maintained, perception)

        rebuilt = build_costmap(perception, CELL_SIZE_M)
        assert maintained.blocked == rebuilt.blocked
        for cell in set(rebuilt.base) | set(rebuilt.inflation):
            assert maintained.cost(cell) == rebuilt.cost(cell)


def test_integrating_the_same_obstacle_twice_changes_nothing():
    """Inflation takes the maximum, not the sum, so a journal replayed with
    overlap cannot inflate the same rock into a wall."""
    perception = PerceptionMap()
    perception.observe((5, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    probe = (5 + int((ROVER_RADIUS_M + 0.3) / CELL_SIZE_M), 0)
    before = costmap.cost(probe)
    costmap.integrate([((5, 0), Terrain.ROCK)] * 3)
    assert costmap.cost(probe) == before


def test_a_reclassified_cell_keeps_its_neighbours_inflation():
    """Terrain class and obstacle clearance are separate layers. Revising
    what a cell is must not erase the clearance a rock beside it imposes."""
    perception = PerceptionMap()
    perception.observe((5, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    probe = (5 + int((ROVER_RADIUS_M + 0.3) / CELL_SIZE_M), 0)
    inflated = costmap.cost(probe)
    assert inflated > TRAVERSE_COST[Terrain.GROUND]

    costmap.integrate([(probe, Terrain.GROUND)])
    assert costmap.cost(probe) > TRAVERSE_COST[Terrain.GROUND]


def test_clearing_the_map_does_not_leave_obstacles_in_the_costmap():
    """A new mission must not inherit the last one's rocks."""
    from ocu.core.navigation import update_costmap

    perception = PerceptionMap()
    perception.observe((5, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)
    assert costmap.is_blocked((5, 0))

    perception.clear()
    update_costmap(costmap, perception)
    assert not costmap.is_blocked((5, 0))


def test_the_renderer_and_the_costmap_do_not_consume_each_others_updates():
    """Two independent cursors over one journal. If they shared one, half the
    map would reach the display and the other half the planner."""
    perception = PerceptionMap()
    perception.observe((1, 1), Terrain.GROUND)
    assert perception.drain_new() == [((1, 1), Terrain.GROUND)]

    costmap = build_costmap(perception, CELL_SIZE_M)
    assert costmap.cost((1, 1)) == TRAVERSE_COST[Terrain.GROUND]


# --- a goal the vehicle cannot stand on -----------------------------------


def test_a_clear_goal_is_left_where_it_is():
    from ocu.core.navigation import reachable_goal

    costmap = build_costmap(open_ground(), CELL_SIZE_M)
    assert reachable_goal(costmap, (10, 0)) == (10, 0)


def test_a_goal_inside_an_obstacles_clearance_is_moved_to_the_nearest_clear_cell():
    """A lane plan is drawn before the obstacles are known, so some of its
    points land inside a rock's clearance envelope. Driving to within a
    metre of such a point is worth doing; abandoning the lane is not."""
    from ocu.core.navigation import reachable_goal

    perception = open_ground()
    perception.observe((10, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)
    assert costmap.is_blocked((10, 0))

    moved = reachable_goal(costmap, (10, 0))
    assert moved is not None and not costmap.is_blocked(moved)
    assert math.hypot(moved[0] - 10, moved[1]) * CELL_SIZE_M <= 2.5


def test_a_goal_with_nothing_clear_nearby_is_genuinely_unreachable():
    from ocu.core.navigation import GOAL_RELOCATION_M, reachable_goal

    perception = open_ground()
    reach = int((GOAL_RELOCATION_M + 2.0) / CELL_SIZE_M)
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            perception.observe((10 + dx, dy), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)
    assert reachable_goal(costmap, (10, 0)) is None


def test_planning_to_a_blocked_goal_fails_immediately_rather_than_exhausting_the_search():
    """The expensive failure. A search can never enter a blocked goal, so
    without this check it expands to its ceiling and returns nothing, at a
    third of a second per attempt, every tick, for the rest of the mission.
    """
    import time

    perception = open_ground(extent=60)
    perception.observe((10, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    started = time.perf_counter()
    assert plan_path(costmap, (0, 0), (10, 0)) == []
    assert (time.perf_counter() - started) < 0.01, "a blocked goal must fail without searching"


# --- what the vehicle does when it cannot plan -----------------------------


def entomb(perception: PerceptionMap, cell: tuple[int, int]) -> None:
    """Rock out far enough around a cell that nothing within the relocation
    radius is standable, so the waypoint is genuinely unreachable rather
    than merely awkward."""
    from ocu.core.navigation import GOAL_RELOCATION_M

    reach = int((GOAL_RELOCATION_M + 2.0) / CELL_SIZE_M)
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            perception.observe((cell[0] + dx, cell[1] + dy), Terrain.ROCK)


def boxed_in_director():
    """A rover with a goal it cannot possibly reach: not walled off, which
    it could drive around, but with no standable ground anywhere near the
    waypoint itself."""
    from ocu.core.flight_director import FlightDirector

    director = FlightDirector()
    perception = open_ground(extent=60)
    entomb(perception, (60, 0))
    director.navigate_to(30.0, 0.0)
    return director, perception


def test_a_failed_plan_is_not_retried_every_tick():
    """Retrying is right, because the map is still changing. Retrying ten
    times a second with the most expensive search there is, is not."""
    from ocu.core.flight_director import PLAN_RETRY_TICKS

    director, perception = boxed_in_director()
    assert not director.replan(perception, CELL_SIZE_M)

    for _ in range(PLAN_RETRY_TICKS - 2):
        assert not director.replan(perception, CELL_SIZE_M)
    assert director._plan_failures == 1, "the backoff must not count as fresh attempts"


def test_an_unreachable_waypoint_is_reported_rather_than_held_in_silence():
    from ocu.core.flight_director import MAX_PLAN_FAILURES, PLAN_RETRY_TICKS

    director, perception = boxed_in_director()
    events: list[tuple[str, str]] = []
    for _ in range(MAX_PLAN_FAILURES * (PLAN_RETRY_TICKS + 1)):
        director.replan(perception, CELL_SIZE_M)
        _, _, tick_events = director.tick(
            0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading
        )
        events.extend(tick_events)

    assert ("rover", "WAYPOINT_UNREACHABLE") in events
    assert director.goal is None


def test_a_route_the_map_invalidated_is_not_driven_while_a_replan_is_pending():
    """Holding is correct. Continuing to follow a line that is now known to
    end in a rock is how a robot drives into one."""
    from ocu.core.flight_director import FlightDirector

    director = FlightDirector()
    perception = open_ground(extent=60)
    director.navigate_to(30.0, 0.0)
    assert director.replan(perception, CELL_SIZE_M)
    assert director.path

    # The waypoint turns out to be solid rock with no standable ground
    # anywhere near it, so there is nothing to replan to.
    entomb(perception, (60, 0))
    assert not director.replan(perception, CELL_SIZE_M)
    assert director.path == [], "an invalidated route must be dropped, not followed"


def test_a_relocated_goal_can_still_be_reached():
    """If the arrival test kept the original goal, the rover would drive as
    close as clearance allows and then report never having arrived."""
    from ocu.core.flight_director import FlightDirector

    director = FlightDirector()
    perception = open_ground(extent=60)
    perception.observe((20, 0), Terrain.ROCK)
    director.navigate_to(10.0, 0.0)  # cell (20, 0), inside the rock
    assert director.replan(perception, CELL_SIZE_M)

    for _ in range(900):
        director.replan(perception, CELL_SIZE_M)
        director.tick(
            0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading
        )
        if director.goal is None:
            break
    assert director.goal is None, "the rover must register arrival at the moved goal"


# --- holding the lane ------------------------------------------------------


def lane_leg(costmap, a_m: tuple[float, float], b_m: tuple[float, float], smoothed: bool):
    """Plan a leg with and without a corridor, as world polylines."""
    from ocu.core.navigation import Corridor, smooth

    corridor = Corridor(
        ax=a_m[0] / CELL_SIZE_M, ay=a_m[1] / CELL_SIZE_M,
        bx=b_m[0] / CELL_SIZE_M, by=b_m[1] / CELL_SIZE_M,
    )
    plans = {}
    for name, corr in (("free", None), ("corridor", corridor)):
        cells = plan_path(
            costmap,
            (int(a_m[0] / CELL_SIZE_M), int(a_m[1] / CELL_SIZE_M)),
            (int(b_m[0] / CELL_SIZE_M), int(b_m[1] / CELL_SIZE_M)),
            corridor=corr,
        )
        world = [
            (cx * CELL_SIZE_M + CELL_SIZE_M / 2, cy * CELL_SIZE_M + CELL_SIZE_M / 2)
            for cx, cy in cells
        ]
        if world:
            world[0], world[-1] = a_m, b_m
        plans[name] = smooth(world, costmap) if smoothed else world
    return plans


def worst_cross_track(path, a_m, b_m) -> float:
    dx, dy = b_m[0] - a_m[0], b_m[1] - a_m[1]
    length = math.hypot(dx, dy)
    return max(
        abs((x - a_m[0]) * dy - (y - a_m[1]) * dx) / length for x, y in path
    )


def scrub_lane_with_easier_ground_beside_it() -> PerceptionMap:
    """Scrub on the lane, bare ground two metres off it. An unconstrained
    planner takes the bare ground, which is cheaper and off the lane."""
    perception = PerceptionMap()
    for x in range(-10, 70):
        for y in range(-30, 31):
            perception.observe((x, y), Terrain.GROUND)
    for x in range(-10, 70):
        for y in range(-2, 3):
            perception.observe((x, y), Terrain.SCRUB)
    return perception


def test_an_unconstrained_planner_leaves_the_lane_for_cheaper_ground():
    """The behaviour the corridor exists to correct. Worth asserting, so
    that if the corridor ever stops mattering the test says why it did."""
    costmap = build_costmap(scrub_lane_with_easier_ground_beside_it(), CELL_SIZE_M)
    a, b = (0.0, 0.0), (20.0, 0.0)
    assert worst_cross_track(lane_leg(costmap, a, b, smoothed=False)["free"], a, b) > 1.0


def test_a_corridor_holds_the_lane_across_ground_that_costs_more():
    costmap = build_costmap(scrub_lane_with_easier_ground_beside_it(), CELL_SIZE_M)
    a, b = (0.0, 0.0), (20.0, 0.0)
    planned = lane_leg(costmap, a, b, smoothed=False)["corridor"]
    assert worst_cross_track(planned, a, b) < CELL_SIZE_M


def test_a_corridor_still_yields_to_something_impassable_on_the_lane():
    """Holding the lane is a preference, not an override. A rock on the
    lane is still a rock."""
    perception = open_ground(extent=70)
    for y in range(-3, 4):
        perception.observe((20, y), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    a, b = (0.0, 0.0), (20.0, 0.0)
    path = lane_leg(costmap, a, b, smoothed=True)["corridor"]
    assert path, "it must go round rather than refuse"
    assert worst_cross_track(path, a, b) > 1.0, "it has to leave the lane to pass"
    cells = [(int(x // CELL_SIZE_M), int(y // CELL_SIZE_M)) for x, y in path]
    assert path_is_clear(costmap, cells)


def test_a_lane_point_that_is_reachable_is_not_moved_to_its_cell_centre():
    """A cell centre is up to a third of a metre from the point it stands
    for. Snapping every waypoint to one puts most of a sensor swath of
    error on every leg of the sweep."""
    from ocu.core.flight_director import FlightDirector

    director = FlightDirector()
    perception = open_ground(extent=70)
    asked_for = (12.37, 0.11)
    director.navigate_to(*asked_for)
    assert director.replan(perception, CELL_SIZE_M)
    assert director.goal == asked_for


def test_smoothing_straightens_the_grids_staircase():
    """A* can only return cell centres, so a straight route comes back as a
    staircase. Followed literally it sweeps a band half a cell wide of
    where it should be."""
    from ocu.core.navigation import smooth

    costmap = build_costmap(open_ground(extent=40), CELL_SIZE_M)
    staircase = [(i * 0.5, 0.0 if i % 2 else 0.5) for i in range(24)]
    straightened = smooth(staircase, costmap)

    a, b = straightened[0], straightened[-1]
    assert worst_cross_track(staircase, a, b) > 0.2
    assert worst_cross_track(straightened, a, b) < 0.05


def test_smoothing_pulls_taut_around_an_obstacle_rather_than_through_it():
    """The string pull is bounded by the obstacle check, not by how hard it
    is pulled. Without that check, straightening a detour cuts the corner
    the planner went round."""
    from ocu.core.navigation import smooth

    perception = open_ground(extent=40)
    perception.observe((10, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)

    # A generous detour over the rock, at the resolution a plan comes at.
    detour = []
    waypoints = [(0.0, 0.0), (3.0, 0.0), (4.5, 2.5), (5.0, 3.0), (5.5, 2.5), (7.0, 0.0), (10.0, 0.0)]
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        steps = max(int(math.hypot(x1 - x0, y1 - y0) / 0.25), 1)
        detour.extend(
            (x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps) for i in range(steps)
        )
    detour.append(waypoints[-1])
    cells = lambda path: [(int(x // CELL_SIZE_M), int(y // CELL_SIZE_M)) for x, y in path]
    assert path_is_clear(costmap, cells(detour)), "the detour itself must start out valid"

    straightened = smooth(detour, costmap)
    assert path_is_clear(costmap, cells(straightened)), "smoothing must not cut through the rock"
    assert max(y for _, y in straightened) < max(y for _, y in detour), "it should still pull taut"


# --- the line that is driven, and starting inside a clearance band --------


def test_a_tree_beside_the_search_path_but_on_the_smoothed_line_invalidates_the_route():
    """The vehicle drives the smoothed line, not the cells the search crossed.
    Measured: a rover followed a plan straight into a tree mapped after the
    plan was made, because only the search cells were checked."""
    from ocu.core.navigation import followed_line_is_clear

    perception = open_ground()
    costmap = build_costmap(perception, CELL_SIZE_M)
    line = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]
    assert followed_line_is_clear(costmap, line, 0.0, 0.0)
    costmap.integrate([((int(5.0 / CELL_SIZE_M), 0), Terrain.TREE)])
    assert not followed_line_is_clear(costmap, line, 0.0, 0.0)
    # What the vehicle has already driven past is not its concern.
    assert followed_line_is_clear(costmap, line, 8.0, 0.0)


def test_a_vehicle_inside_a_clearance_band_can_plan_its_way_out_but_never_onto_the_obstacle():
    perception = open_ground()
    perception.observe((0, 0), Terrain.ROCK)
    costmap = build_costmap(perception, CELL_SIZE_M)
    start = (1, 0)  # half a metre from the rock, well inside its clearance band
    assert costmap.is_blocked(start) and (0, 0) in costmap.hard
    path = plan_path(costmap, start, (10, 0))
    assert path, "it can get out"
    assert (0, 0) not in path
    assert not costmap.is_blocked(path[-1])


def test_the_obstacle_cells_are_tracked_apart_from_their_clearance():
    perception = open_ground()
    perception.observe((0, 0), Terrain.TREE)
    costmap = build_costmap(perception, CELL_SIZE_M)
    assert costmap.hard == {(0, 0)}
    assert len(costmap.blocked) > 1
    costmap.integrate([((0, 0), Terrain.GROUND)])
    assert costmap.hard == set(), "a cell reclassified as ground is no longer an obstacle"


def test_a_hundred_metre_route_across_open_ground_is_found():
    """The search ceiling used to make a long leg fail the moment it was asked for."""
    perception = open_ground(extent=130)
    costmap = build_costmap(perception, CELL_SIZE_M)
    path = plan_path(costmap, (0, 0), (120, 100))
    assert path and path[-1] == (120, 100)


def test_the_rover_never_drives_onto_a_mapped_obstacle():
    from ocu.core.flight_director import FlightDirector

    perception = open_ground()
    director = FlightDirector()
    director.rover_body.x = director.rover_body.y = 0.0
    director.rover_body.heading = 90.0
    director.navigate_to(20.0, 0.0)
    director.replan(perception, CELL_SIZE_M)
    # A tree the rover learns of only when it is already almost on it.
    for step in range(400):
        if step == 40:
            cx = int(director.rover_body.x / CELL_SIZE_M) + 1
            perception.observe((cx, 0), Terrain.TREE)
        director.tick(0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading)
        director.replan(perception, CELL_SIZE_M)
        cell = (math.floor(director.rover_body.x / CELL_SIZE_M), math.floor(director.rover_body.y / CELL_SIZE_M))
        assert perception.at(cell) != Terrain.TREE, f"drove onto the tree at step {step}"


def test_the_last_line_of_defence_stops_the_rover_at_the_edge_of_a_mapped_obstacle():
    """Whatever the planner does, the vehicle itself refuses to move onto a cell it
    has mapped as an obstacle. The route is left stale on purpose, so only the guard
    stands between the rover and the tree."""
    from ocu.core.flight_director import FlightDirector
    from ocu.core.navigation import update_costmap

    perception = open_ground()
    director = FlightDirector()
    director.rover_body.heading = 90.0
    director.navigate_to(20.0, 0.0)
    director.replan(perception, CELL_SIZE_M)
    perception.observe((10, 0), Terrain.TREE)
    update_costmap(director._costmap, perception)  # known, but the route is never remade

    for _ in range(600):
        director.tick(0.1, director.rover_body.x, director.rover_body.y, director.rover_body.heading)
        assert math.floor(director.rover_body.x / CELL_SIZE_M) != 10, "drove onto the tree"
