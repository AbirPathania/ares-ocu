"""Simulated LiDAR: a scan that is cast, occluded, and accumulated.

Phase 2 of the mission is "the rover navigates using LiDAR inertial SLAM
while running the full detection suite" (section 5). This is the sensor
half of that: a planar scan swept around the vehicle, each ray marched
outward through the terrain grid until it strikes something that stops it
or runs out of range.

The occlusion is the part that matters. A ray that hits a rock reports the
rock and nothing behind it, so the rover's map grows a shadow that is only
filled in when it drives far enough around to see into it. That behaviour
is what separates a map a vehicle built from a map someone drew: the
coverage has holes in exactly the places a real sensor would leave them,
and they close as the vehicle moves.

Range and angular resolution are not in the ARES technical baseline, which
names LiDAR in the perception stack without specifying the unit. The
figures here are this build's model, chosen to be ordinary for a ground
vehicle's scanner. See DECISIONS.md.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.terrain import PerceptionMap, Terrain, TerrainWorld

#: Effective classification range. Returns come back further out than this on
#: a real unit, but not densely enough to classify ground reliably.
LIDAR_RANGE_M = 32.0

#: One ray per degree. Dense enough that the revealed area has clean edges
#: rather than visible spokes, cheap enough to run several times a second.
LIDAR_RAYS = 360

#: Scans per second while the vehicle is moving.
LIDAR_SCAN_HZ = 4.0


@dataclass(frozen=True)
class ScanResult:
    new_cells: int
    returns: int  # rays that struck something inside range


def cast_scan(
    world: TerrainWorld,
    perception: PerceptionMap,
    origin_x: float,
    origin_y: float,
    range_m: float = LIDAR_RANGE_M,
    rays: int = LIDAR_RAYS,
) -> ScanResult:
    """Sweep a full circle from (origin_x, origin_y) and fold what is seen
    into the perception map. Returns how much of it was new.

    Rays are marched at half a cell so nothing is stepped over diagonally,
    and each ray stops at the first occluding cell: that cell is recorded,
    and everything beyond it on that bearing is left unknown.
    """
    step = world.cell_size_m * 0.5
    steps = int(range_m / step)
    new_cells = 0
    returns = 0

    for i in range(rays):
        bearing = (i / rays) * math.tau
        dx, dy = math.sin(bearing) * step, math.cos(bearing) * step
        x, y = origin_x, origin_y

        for _ in range(steps):
            x += dx
            y += dy
            cell = world.to_cell(x, y)
            kind = world.at(cell)
            if perception.observe(cell, kind):
                new_cells += 1
            if kind in (Terrain.ROCK, Terrain.TREE):
                # The beam stops here. Everything behind it stays unknown
                # until the vehicle can see round the obstacle.
                returns += 1
                break

    return ScanResult(new_cells=new_cells, returns=returns)


def has_line_of_sight(
    world: TerrainWorld, from_x: float, from_y: float, to_x: float, to_y: float
) -> bool:
    """Whether one point can see another, used for checking a shadow is real."""
    distance = math.hypot(to_x - from_x, to_y - from_y)
    if distance == 0:
        return True
    step = world.cell_size_m * 0.5
    steps = int(distance / step)
    dx, dy = (to_x - from_x) / distance * step, (to_y - from_y) / distance * step
    x, y = from_x, from_y
    for _ in range(steps):
        x += dx
        y += dy
        if world.blocks_beam(world.to_cell(x, y)):
            return False
    return True
