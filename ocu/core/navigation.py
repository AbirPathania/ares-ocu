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

from ocu.core.terrain import IMPASSABLE, TRAVERSE_COST, PerceptionMap, Terrain

#: Half the rover's width plus clearance. The ARES-G is a 25 kg six-wheel
#: platform; this is the radius a planner must keep clear of obstacles.
ROVER_RADIUS_M = 0.9

#: Extra margin beyond the vehicle radius. Costs decay across this band so
#: the planner prefers the middle of a gap rather than grazing the edge.
INFLATION_MARGIN_M = 1.1

#: How far ahead on the path the follower aims. Shorter oscillates, longer
#: understeers and cuts corners. At this vehicle's speeds a little under two
#: metres tracks a lane to within a sensor swath without hunting.
LOOKAHEAD_M = 1.8

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
MAX_PLAN_EXPANSIONS = 20_000


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
                continue
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
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

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
                continue
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
