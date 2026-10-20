"""Autonomous navigation: costmap, planner, and path follower.

This is the standard ground-robot navigation stack, which is what ROS
nav2 assembles out of its own components. It is implemented directly
rather than pulled in, because the algorithms are small and the point is
to show them working on this vehicle's own discovered map:

1. **Costmap with inflation.** Obstacles are grown by the vehicle's
   radius plus a margin, so the planner can treat the rover as a point.
   Without inflation a planner happily routes a point through a gap the
   vehicle cannot fit, which is the single most common way a plan looks
   fine and puts a robot into a rock.

2. **A\\* over the costmap**, replanned whenever newly discovered ground
   invalidates the current route. The map is being built as the rover
   drives, so a plan is only ever valid against what was known when it was
   made. Replanning on change is what D\\* Lite exists to do efficiently;
   at this map size a fresh A\\* is faster than the bookkeeping to avoid
   one, so it replans outright.

3. **Pure pursuit** for following the result. The planner returns a
   polyline; the follower picks a point a fixed distance ahead on that
   line and steers at it, which produces smooth motion instead of the
   corner-cutting zigzag of driving waypoint to waypoint.

Unknown ground is given a middling cost rather than being treated as free
or blocked. Free would send the rover confidently into terrain nobody has
looked at; blocked would leave it unable to plan anywhere at the start,
when everything is unknown. A middling cost makes it prefer ground it has
already seen while still being willing to explore.

No Qt import belongs in this file.
"""

from __future__ import annotations

import heapq
import math
from collections.abc import Iterable
from dataclasses import dataclass, field

from ocu.core.terrain import (
    IMPASSABLE,
    TRAVERSE_COST,
    WORLD_HALF_EXTENT_M,
    PerceptionMap,
    Terrain,
)

#: Half the rover's width plus clearance. The ARES-G is a 25 kg six-wheel
#: platform; this is the radius a planner must keep clear of obstacles.
ROVER_RADIUS_M = 1.1

#: Extra margin beyond the vehicle radius. Costs decay across this band so
#: the planner prefers the middle of a gap rather than grazing the edge.
INFLATION_MARGIN_M = 1.1

#: A vehicle that finds itself inside an obstacle's clearance, because the
#: obstacle was not visible until it was close, may route through that band to
#: get out of it, but only this far from where it stands, and at a heavy cost,
#: so it leaves by the shortest way and does not use the band as a road.
ESCAPE_RADIUS_M = 2.4
ESCAPE_COST_FACTOR = 10.0

#: How far ahead on the path the follower aims. Shorter oscillates, longer
#: understeers and cuts corners. At this vehicle's speeds a little under two
#: metres tracks a lane to within a sensor swath without hunting.
LOOKAHEAD_M = 1.8

#: The lookahead used while inside an obstacle's clearance band, where the
#: line is bent tightly round something and a long chord would cut through it.
TIGHT_LOOKAHEAD_M = 0.6

#: Close enough to call a waypoint reached. A lane point is a point on a
#: line the vehicle is supposed to be driving along, so this is tight; the
#: slack a waypoint-to-waypoint route could afford would let it cut the
#: corner off every lane point and sweep a different line than planned.
WAYPOINT_TOLERANCE_M = 0.8

#: How far a goal may be moved to find somewhere the vehicle can stand.
#: A lane plan is drawn before the obstacles are known, so some of its
#: points land inside a rock's clearance envelope.
GOAL_RELOCATION_M = 2.5

#: Expansion ceiling for one search. On a 0.5 m grid this is a radius well
#: past anything in a local sweep. The ceiling exists so a search that
#: cannot succeed fails in milliseconds instead of stalling the tick.
MAX_PLAN_EXPANSIONS = 60_000

#: Weight on the distance-to-go estimate. Crossing a cell costs between 1.0 and
#: several times its length, so an unweighted estimate is far too hopeful and
#: the search fans out over the whole neighbourhood before committing to a
#: direction; on a leg of a hundred metres that is the ceiling above. Leaning
#: on the estimate a little makes it commit, at the price of a route that can
#: be a few percent longer than the best one, which nobody driving it can tell.
HEURISTIC_WEIGHT = 1.5


@dataclass
class Costmap:
    """Traversal cost per cell, with obstacles inflated by vehicle radius.

    Held as two layers, which is how nav2 arranges the same thing. The base
    layer is what the terrain costs to cross; the inflation layer is the
    penalty for being near something impassable. Keeping them apart is what
    makes the map maintainable in place: a cell's classification can be
    revised without destroying the inflation that neighbouring obstacles put
    on it, and an obstacle can be folded in without re-reading the map.

    Inflation takes the maximum over contributing obstacles rather than the
    sum. Summing would make a cell between two rocks look worse than the
    clearance it actually has, and it would make folding the same obstacle in
    twice change the answer.
    """

    cell_size_m: float
    base: dict[tuple[int, int], float] = field(default_factory=dict)
    inflation: dict[tuple[int, int], float] = field(default_factory=dict)
    blocked: set[tuple[int, int]] = field(default_factory=set)
    #: The obstacles themselves, without the vehicle's clearance around them.
    #: The planner keeps off everything in `blocked`; nothing may ever be
    #: driven onto a cell in `hard`.
    hard: set[tuple[int, int]] = field(default_factory=set)
    _hard_version: int = 0
    _hard_cache: tuple[int, object] | None = field(default=None, repr=False)

    #: How far into the perception map's journal this costmap has consumed,
    #: and which mission that journal belonged to.
    cursor: int = 0
    epoch: int = 0

    def cost(self, cell: tuple[int, int]) -> float:
        if cell in self.blocked:
            return math.inf
        base = self.base.get(cell, TRAVERSE_COST[Terrain.UNKNOWN])
        return base + self.inflation.get(cell, 0.0)

    def is_blocked(self, cell: tuple[int, int]) -> bool:
        return cell in self.blocked

    def obstacle_points(self):
        """The obstacle cells as (n, 2) world positions of their centres, kept
        until the set changes so the reactive layer can ask every tick."""
        import numpy as np

        if self._hard_cache is not None and self._hard_cache[0] == self._hard_version:
            return self._hard_cache[1]
        size = self.cell_size_m
        points = (
            np.array([((cx + 0.5) * size, (cy + 0.5) * size) for cx, cy in self.hard], dtype=np.float64)
            if self.hard
            else np.zeros((0, 2), dtype=np.float64)
        )
        self._hard_cache = (self._hard_version, points)
        return points

    def integrate(self, changes: Iterable[tuple[tuple[int, int], Terrain]]) -> None:
        """Fold a batch of newly classified cells into both layers.

        Every impassable cell is grown by the vehicle radius, and the penalty
        decays across a further margin so a route prefers the middle of a gap
        over grazing its edge. The rover is then a point, which is the only
        reason planning on a grid is tractable at all. Without this a planner
        happily routes a point through a gap the vehicle cannot fit, which is
        the single most common way a plan looks fine and puts a robot into a
        rock.
        """
        cell_size_m = self.cell_size_m
        hard_radius = ROVER_RADIUS_M
        soft_radius = ROVER_RADIUS_M + INFLATION_MARGIN_M
        reach = int(math.ceil(soft_radius / cell_size_m))

        for cell, kind in changes:
            self.base[cell] = TRAVERSE_COST.get(kind, TRAVERSE_COST[Terrain.UNKNOWN])
            if kind not in IMPASSABLE:
                if cell in self.hard:
                    self.hard.discard(cell)
                    self._hard_version += 1
                continue
            if cell not in self.hard:
                self.hard.add(cell)
                self._hard_version += 1
            cx, cy = cell
            for dx in range(-reach, reach + 1):
                for dy in range(-reach, reach + 1):
                    distance = math.hypot(dx, dy) * cell_size_m
                    if distance > soft_radius:
                        continue
                    neighbour = (cx + dx, cy + dy)
                    if distance <= hard_radius:
                        self.blocked.add(neighbour)
                        continue
                    # Decays to zero at the edge of the margin, so the
                    # planner is nudged toward clearance, not walled off.
                    falloff = 1.0 - (distance - hard_radius) / INFLATION_MARGIN_M
                    extra = 8.0 * falloff**2
                    if extra > self.inflation.get(neighbour, 0.0):
                        self.inflation[neighbour] = extra


def build_costmap(perception: PerceptionMap, cell_size_m: float) -> Costmap:
    """A costmap over everything the rover has seen so far."""
    costmap = Costmap(cell_size_m=cell_size_m, epoch=perception.epoch)
    update_costmap(costmap, perception)
    return costmap


def update_costmap(costmap: Costmap, perception: PerceptionMap) -> int:
    """Bring a costmap up to date with the map, reading only what changed.

    Rebuilding from scratch costs time proportional to everything ever
    mapped, every tick, forever. Reading only the journal since the last
    update costs time proportional to what the last scan actually found,
    which is what makes it affordable to revalidate the route at tick rate.
    """
    if costmap.epoch != perception.epoch:
        # A different mission's map. Nothing from the old one survives.
        costmap.base.clear()
        costmap.inflation.clear()
        costmap.blocked.clear()
        costmap.cursor = 0
        costmap.epoch = perception.epoch

    changes, cursor = perception.changes_since(costmap.cursor)
    costmap.cursor = cursor
    costmap.integrate(changes)
    return len(changes)


@dataclass(frozen=True)
class Corridor:
    """A lane the planner is expected to hold to, in cell coordinates.

    A* minimises traverse cost, which is the right objective for getting
    from A to B and the wrong one for clearance work. Given scrub on the
    lane and bare ground two metres off it, the cheapest route is off the
    lane -- so an unconstrained planner quietly sweeps a different line
    than the one that was planned, and the ground the lane was drawn over
    never gets looked at.

    Penalising distance from the lane fixes the objective rather than the
    symptom. The vehicle holds the lane, and departs only where something
    impassable leaves it no choice, which is exactly the behaviour -- and
    the visible gap in coverage -- that clearance doctrine is built around.
    """

    ax: float
    ay: float
    bx: float
    by: float
    penalty_per_cell: float = 7.0

    def cross_track(self, cell: tuple[int, int]) -> float:
        """Perpendicular distance from the lane, in cells.

        Measured from the cell's centre. A cell index names a square, not a
        point, and the square's centre is half a cell past its index. Using
        the index directly biases every route half a cell -- 25 cm here,
        which is most of a pulse induction swath, and consistently to the
        same side.
        """
        px, py = cell[0] + 0.5, cell[1] + 0.5
        dx, dy = self.bx - self.ax, self.by - self.ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 0.0:
            return math.hypot(px - self.ax, py - self.ay)
        # Clamped projection, so the ends of the lane are not penalised as
        # if the lane continued past them.
        t = ((px - self.ax) * dx + (py - self.ay) * dy) / length_sq
        t = min(max(t, 0.0), 1.0)
        return math.hypot(px - (self.ax + t * dx), py - (self.ay + t * dy))

    def penalty(self, cell: tuple[int, int]) -> float:
        return self.penalty_per_cell * self.cross_track(cell)


_NEIGHBOURS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def reachable_goal(
    costmap: Costmap, goal: tuple[int, int], max_offset_m: float = GOAL_RELOCATION_M
) -> tuple[int, int] | None:
    """The nearest cell to `goal` the vehicle can actually occupy.

    Planning to a cell inside an obstacle's clearance envelope cannot
    succeed: the search can never enter the goal, so it expands until it
    hits its ceiling and returns nothing. That is the difference between a
    plan failing in a millisecond and a plan costing a third of a second
    every tick for the rest of the mission.

    Moving the goal is also the right answer rather than a workaround. A
    lane point a metre inside a rock's clearance is still worth driving to
    within a metre; refusing it would abandon ground the vehicle could have
    swept. Returning None means there is nowhere clear nearby at all, which
    is a genuinely unreachable waypoint.
    """
    if not costmap.is_blocked(goal):
        return goal

    reach = int(math.ceil(max_offset_m / costmap.cell_size_m))
    best: tuple[int, int] | None = None
    best_distance = math.inf
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            distance = math.hypot(dx, dy) * costmap.cell_size_m
            if distance > max_offset_m or distance >= best_distance:
                continue
            candidate = (goal[0] + dx, goal[1] + dy)
            if costmap.is_blocked(candidate):
                continue
            best, best_distance = candidate, distance
    return best


def plan_path(
    costmap: Costmap,
    start: tuple[int, int],
    goal: tuple[int, int],
    max_expansions: int = MAX_PLAN_EXPANSIONS,
    corridor: Corridor | None = None,
) -> list[tuple[int, int]]:
    """A* from start to goal over the costmap. Empty list if unreachable.

    With a `corridor`, distance from that lane is added to the cost, so the
    route holds the lane instead of taking the cheapest ground between its
    ends. The penalty is non-negative, so the octile heuristic stays
    admissible and the result is still an optimal route under the costs it
    was given.
    """
    if start == goal:
        return [start]
    if costmap.is_blocked(goal):
        # Unwinnable by construction: the search can never enter the goal.
        # Callers that want the closest standable cell ask for it with
        # reachable_goal() first.
        return []

    def heuristic(cell: tuple[int, int]) -> float:
        # Octile distance: admissible on an eight-connected grid, and
        # tighter than Euclidean, so fewer cells get expanded.
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return HEURISTIC_WEIGHT * ((dx + dy) + (math.sqrt(2) - 2) * min(dx, dy))

    escaping = costmap.is_blocked(start)
    escape_cells = ESCAPE_RADIUS_M / costmap.cell_size_m

    open_heap = [(heuristic(start), 0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    best: dict[tuple[int, int], float] = {start: 0.0}
    expansions = 0

    while open_heap:
        _, travelled, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(came_from, current)
        if travelled > best.get(current, math.inf):
            continue

        expansions += 1
        if expansions > max_expansions:
            break

        for dx, dy, step in _NEIGHBOURS:
            neighbour = (current[0] + dx, current[1] + dy)
            if costmap.is_blocked(neighbour):
                # Inside the clearance band of something: only a way out, only
                # nearby, and never onto the obstacle itself.
                if (
                    not escaping
                    or neighbour in costmap.hard
                    or math.hypot(neighbour[0] - start[0], neighbour[1] - start[1]) > escape_cells
                ):
                    continue
                cell_cost = costmap.base.get(neighbour, TRAVERSE_COST[Terrain.UNKNOWN]) * ESCAPE_COST_FACTOR
            else:
                cell_cost = costmap.cost(neighbour)
            if corridor is not None:
                cell_cost += corridor.penalty(neighbour)
            tentative = travelled + step * cell_cost
            if tentative < best.get(neighbour, math.inf):
                best[neighbour] = tentative
                came_from[neighbour] = current
                heapq.heappush(open_heap, (tentative + heuristic(neighbour), tentative, neighbour))

    return []


def _reconstruct(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return list(reversed(path))


def simplify(path: list[tuple[int, int]], tolerance: float = 0.8) -> list[tuple[int, int]]:
    """Drop collinear points. A grid path is mostly straight runs, and the
    follower only needs the corners."""
    if len(path) < 3:
        return path
    simplified = [path[0]]
    for previous, current, following in zip(path, path[1:], path[2:]):
        cross = (current[0] - previous[0]) * (following[1] - previous[1]) - (
            current[1] - previous[1]
        ) * (following[0] - previous[0])
        if abs(cross) > tolerance:
            simplified.append(current)
    simplified.append(path[-1])
    return simplified


def smooth(
    path: list[tuple[float, float]],
    costmap: Costmap,
    weight_data: float = 0.0,
    weight_smooth: float = 0.45,
    iterations: int = 400,
    tolerance: float = 1e-4,
) -> list[tuple[float, float]]:
    """Pull a planned path off the grid it was planned on.

    A* returns cell centres, so even a dead straight route comes back as a
    staircase up to half a cell either side of the line it should be on.
    Half a cell here is 25 cm and the pulse induction array is 40 cm wide:
    following the staircase literally means sweeping a band that misses
    things lying on the lane. Every real planner smooths before it follows.

    The standard two-term smoother, run as a string pull: each point is
    drawn toward the midpoint of its neighbours, and by default nothing
    holds it back toward the cell the planner put it in. On a clear leg
    that converges to the straight line between the ends, which is the
    route the planner meant; the staircase was the grid's doing, not the
    planner's. A data term would instead converge on the cell centres, so a
    lane lying along a cell boundary would be followed a consistent half
    cell to one side of itself.

    What keeps the line honest is the obstacle check, not the data term.
    The endpoints are fixed, and a point that would move into a blocked
    cell keeps its previous position, so the string pulls taut around
    whatever the planner went round rather than through it.
    """
    if len(path) < 3:
        return list(path)

    cell_size_m = costmap.cell_size_m
    smoothed = [list(point) for point in path]

    for _ in range(iterations):
        shift = 0.0
        for i in range(1, len(smoothed) - 1):
            moved = []
            for axis in (0, 1):
                value = smoothed[i][axis]
                value += weight_data * (path[i][axis] - value)
                value += weight_smooth * (
                    smoothed[i - 1][axis] + smoothed[i + 1][axis] - 2.0 * value
                )
                moved.append(value)
            cell = (
                int(math.floor(moved[0] / cell_size_m)),
                int(math.floor(moved[1] / cell_size_m)),
            )
            if costmap.is_blocked(cell):
                continue
            shift += abs(moved[0] - smoothed[i][0]) + abs(moved[1] - smoothed[i][1])
            smoothed[i] = moved
        if shift < tolerance:
            break

    return [(x, y) for x, y in smoothed]


def path_is_clear(costmap: Costmap, path: list[tuple[int, int]]) -> bool:
    """Whether a plan is still valid against what is now known."""
    return not any(costmap.is_blocked(cell) for cell in path)


def _escape_exempt(costmap: Costmap, cell: tuple[int, int], x: float, y: float) -> bool:
    """A cell in the clearance band that the vehicle is already standing in.

    A plan made from inside the band has to start in it, so its first few
    metres are blocked by definition. Reading that as an invalid plan makes
    the vehicle replan every tick for as long as it is leaving.
    """
    size = costmap.cell_size_m
    here = (int(math.floor(x / size)), int(math.floor(y / size)))
    if not costmap.is_blocked(here) or cell in costmap.hard:
        return False
    return math.hypot((cell[0] + 0.5) * size - x, (cell[1] + 0.5) * size - y) <= ESCAPE_RADIUS_M + size


def followed_line_is_clear(
    costmap: Costmap, path_world: list[tuple[float, float]], x: float, y: float
) -> bool:
    """Whether the line the vehicle is actually steering along is still clear.

    `path_is_clear` asks about the cells the planner searched through. The
    vehicle does not drive those: it drives the smoothed line pulled taut
    off them, which cuts across cells the search went round. A tree mapped
    beside the search path but on the smoothed one passes the first test and
    is driven into, so this walks the line itself, from where the vehicle is
    to the end of it, at a fraction of a cell at a time.
    """
    if len(path_world) < 2:
        return True
    start = min(
        range(len(path_world)),
        key=lambda i: math.hypot(path_world[i][0] - x, path_world[i][1] - y),
    )
    step = costmap.cell_size_m * 0.25
    size = costmap.cell_size_m
    for (ax, ay), (bx, by) in zip(path_world[start:], path_world[start + 1 :]):
        length = math.hypot(bx - ax, by - ay)
        samples = max(int(length / step), 1)
        for k in range(samples + 1):
            t = k / samples
            cell = (int(math.floor((ax + (bx - ax) * t) / size)), int(math.floor((ay + (by - ay) * t) / size)))
            if costmap.is_blocked(cell) and not _escape_exempt(costmap, cell, x, y):
                return False
    return True


def pure_pursuit_target(
    path_world: list[tuple[float, float]], x: float, y: float, lookahead_m: float = LOOKAHEAD_M
) -> tuple[float, float] | None:
    """The point on the path to steer at.

    Pure pursuit: find where the path leaves a circle of the lookahead
    radius around the vehicle and aim there. Steering at a point ahead
    rather than at the next waypoint is what keeps the motion smooth
    through corners instead of zigzagging between them.
    """
    if not path_world:
        return None

    # Search strictly *forward* of the closest point. Including the closest
    # point itself lets the vehicle aim at something behind it whenever that
    # point happens to sit a lookahead away, which turns the rover around.
    closest_index = min(
        range(len(path_world)),
        key=lambda i: math.hypot(path_world[i][0] - x, path_world[i][1] - y),
    )
    for point in path_world[closest_index + 1 :]:
        if math.hypot(point[0] - x, point[1] - y) >= lookahead_m:
            return point
    return path_world[-1]


# --- exploring toward a place the vehicle has not seen the way to ---------------------------

#: How much more the distance still to go beyond a frontier counts than the
#: distance to reach it. Above one, the vehicle prefers to make progress toward
#: the goal over tidying up the nearest unexplored edge.
FRONTIER_GOAL_WEIGHT = 1.5

#: A frontier this close teaches nothing the vehicle cannot already see, and
#: one it has already stood at and found still unresolved is a shadow it
#: cannot see into from there; neither is worth driving to.
FRONTIER_MIN_DISTANCE_M = 3.0
FRONTIER_EXCLUSION_M = 4.0

#: How far from itself the vehicle will plan a route over. The scanner reports
#: ground out to 32 m, but the far end of that is thin, noisy and soon to be
#: revised; a route is laid only over ground close enough to be trusted, and
#: what lies beyond is, for planning, still unseen. A longer journey is a series
#: of these: drive to the edge of what can be trusted, look again, decide again.
PLANNING_HORIZON_M = 22.0


@dataclass
class ExplorePlan:
    """What the vehicle can honestly say about the way to a goal.

    `reaches_goal` is true only if the whole route lies over ground it has
    seen. Otherwise the path ends at the frontier it judged the best place to
    go and look from, which is as much of the way as it actually knows.
    """

    path: list[tuple[int, int]]
    reaches_goal: bool


def plan_toward(
    costmap: Costmap,
    start: tuple[int, int],
    goal: tuple[int, int],
    visited: Iterable[tuple[int, int]] = (),
    radius_m: float = PLANNING_HORIZON_M,
    max_expansions: int = MAX_PLAN_EXPANSIONS,
    corridor: Corridor | None = None,
) -> ExplorePlan:
    """Plan over known ground only, to the goal if it can be reached and to the
    most promising frontier if not, within the planning horizon either way.

    A planner that treats unseen ground as open will draw a straight line to
    anywhere, which is not navigation: it is a guess drawn as a route, and it
    is why a vehicle appears to know the way to somewhere it has never been.
    Here unseen ground is not traversable. The vehicle goes only where it has
    looked, and when the goal is not among what it has looked at, to the
    boundary of what it has looked at that leaves the least still to cover,
    where looking again will show it more.

    A frontier is a known, drivable cell with an unseen cell beside it. Each
    is scored by what it costs to get there plus the weighted straight-line
    distance on to the goal, and the cheapest wins.
    """
    size = costmap.cell_size_m
    world_cells = int(WORLD_HALF_EXTENT_M / size)
    escaping = costmap.is_blocked(start)
    escape_cells = ESCAPE_RADIUS_M / size
    radius_cells = radius_m / size
    visited_cells = [(int(v[0]), int(v[1])) for v in visited]
    exclusion_cells = FRONTIER_EXCLUSION_M / size
    min_cells = FRONTIER_MIN_DISTANCE_M / size
    known = costmap.base

    def distance_to_goal(cell: tuple[int, int]) -> float:
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    def is_frontier(cell: tuple[int, int]) -> bool:
        for dx, dy, _ in _NEIGHBOURS:
            n = (cell[0] + dx, cell[1] + dy)
            if abs(n[0]) >= world_cells or abs(n[1]) >= world_cells:
                continue
            if n not in known or math.hypot(n[0] - start[0], n[1] - start[1]) > radius_cells:
                return True
        return False

    goal_known = goal in known and not costmap.is_blocked(goal)
    open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    best: dict[tuple[int, int], float] = {start: 0.0}
    best_frontier: tuple[float, tuple[int, int]] | None = None
    expansions = 0

    while open_heap:
        travelled, current = heapq.heappop(open_heap)
        if travelled > best.get(current, math.inf):
            continue
        if goal_known and current == goal:
            return ExplorePlan(_reconstruct(came_from, current), True)
        expansions += 1
        if expansions > max_expansions:
            break

        if (
            current != start
            and math.hypot(current[0] - start[0], current[1] - start[1]) >= min_cells
            and is_frontier(current)
            and not any(
                math.hypot(current[0] - v[0], current[1] - v[1]) <= exclusion_cells for v in visited_cells
            )
        ):
            score = travelled + FRONTIER_GOAL_WEIGHT * 1.4 * distance_to_goal(current)
            if best_frontier is None or score < best_frontier[0]:
                best_frontier = (score, current)

        for dx, dy, step in _NEIGHBOURS:
            neighbour = (current[0] + dx, current[1] + dy)
            if neighbour not in known:
                continue
            if math.hypot(neighbour[0] - start[0], neighbour[1] - start[1]) > radius_cells:
                continue
            if costmap.is_blocked(neighbour):
                if (
                    not escaping
                    or neighbour in costmap.hard
                    or math.hypot(neighbour[0] - start[0], neighbour[1] - start[1]) > escape_cells
                ):
                    continue
                cell_cost = costmap.base.get(neighbour, TRAVERSE_COST[Terrain.UNKNOWN]) * ESCAPE_COST_FACTOR
            else:
                cell_cost = costmap.cost(neighbour)
            if corridor is not None:
                cell_cost += corridor.penalty(neighbour)
            tentative = travelled + step * cell_cost
            if tentative < best.get(neighbour, math.inf):
                best[neighbour] = tentative
                came_from[neighbour] = current
                heapq.heappush(open_heap, (tentative, neighbour))

    if best_frontier is not None:
        return ExplorePlan(_reconstruct(came_from, best_frontier[1]), False)
    return ExplorePlan([], False)
