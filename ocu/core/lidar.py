"""Simulated LiDAR: rays cast, returns measured, a point cloud accumulated.

Phase 2 of the mission is "the rover navigates using LiDAR inertial SLAM
while running the full detection suite" (section 5). This is the sensor
half of that.

A scanner does not produce filled regions. It produces *returns*: discrete
range measurements, one per ray per surface, each with an intensity that
depends on what was hit and how far away and at what angle. Drawing the
swept area as solid colour throws all of that away and looks like a
painted map, which is what it was. So a scan here yields a list of
returns, and the display accumulates them as a point cloud.

Three physical behaviours are modelled, because they are what make the
picture read as a sensor:

- **Occlusion.** A ray stops at the first thing that blocks it. Everything
  behind that on the same bearing is never measured, so the cloud grows
  shadows that only fill in when the vehicle drives around them.
- **Grazing incidence.** Ground is hit at a shallow angle, so returns off
  it are sparse and weak. A tree trunk or a rock face is close to normal
  to the beam and returns hard. Obstacles therefore stand out in the
  cloud as bright, dense clusters, which is exactly how an operator picks
  them out on a real display.
- **Range falloff.** Returned energy drops with distance, so the cloud
  thins and dims toward the edge of its range instead of ending at a hard
  line.

Range, angular resolution and scan rate are not in the ARES technical
baseline, which names LiDAR in the perception stack without specifying
the unit. They are this build's model. See DECISIONS.md.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from ocu.core.terrain import PerceptionMap, Terrain, TerrainWorld

#: Effective classification range.
LIDAR_RANGE_M = 32.0

#: One ray per degree.
LIDAR_RAYS = 360

#: Scans per second.
LIDAR_SCAN_HZ = 4.0

#: Spacing of ground samples along a ray. Ground is grazed rather than
#: struck, so it is sampled rather than returned at every cell.
GROUND_SAMPLE_M = 1.1

#: How reflective each class is to a scanning beam. Vertical surfaces
#: return hard; vegetation scatters; open ground at a grazing angle is weak.
REFLECTIVITY = {
    Terrain.GROUND: 0.40,
    Terrain.TRACK: 0.55,
    Terrain.SCRUB: 0.60,
    Terrain.ROCK: 0.95,
    Terrain.TREE: 0.88,
    Terrain.UNKNOWN: 0.0,
}

#: Range at which returned energy has fallen by half.
RANGE_HALF_POWER_M = 14.0


@dataclass(frozen=True)
class Return:
    """One range measurement."""

    x: float
    y: float
    kind: Terrain
    intensity: float  # 0..1
    range_m: float


@dataclass(frozen=True)
class ScanResult:
    returns: list[Return]
    new_cells: int
    hits: int  # rays that struck an obstacle inside range

    @property
    def point_count(self) -> int:
        return len(self.returns)


def _intensity(kind: Terrain, range_m: float, rng: random.Random) -> float:
    falloff = 1.0 / (1.0 + range_m / RANGE_HALF_POWER_M)
    speckle = rng.uniform(0.78, 1.0)  # real returns are noisy
    return max(0.0, min(1.0, REFLECTIVITY[kind] * falloff * speckle))


def cast_scan(
    world: TerrainWorld,
    perception: PerceptionMap,
    origin_x: float,
    origin_y: float,
    range_m: float = LIDAR_RANGE_M,
    rays: int = LIDAR_RAYS,
    seed: int | None = None,
) -> ScanResult:
    """Sweep a full circle and return every measurement it produced."""
    rng = random.Random(seed if seed is not None else (int(origin_x * 97) ^ int(origin_y * 89)))
    step = world.cell_size_m * 0.5
    steps = int(range_m / step)
    returns: list[Return] = []
    new_cells = 0
    hits = 0

    for i in range(rays):
        bearing = (i / rays) * math.tau
        dx, dy = math.sin(bearing) * step, math.cos(bearing) * step
        x, y = origin_x, origin_y
        distance = 0.0
        next_ground_sample = rng.uniform(0.4, GROUND_SAMPLE_M)

        for _ in range(steps):
            x += dx
            y += dy
            distance += step
            cell = world.to_cell(x, y)
            kind = world.at(cell)
            if perception.observe(cell, kind):
                new_cells += 1

            if kind in (Terrain.ROCK, Terrain.TREE):
                # A hard return off a near-normal surface, then nothing
                # behind it on this bearing.
                hits += 1
                returns.append(
                    Return(x, y, kind, _intensity(kind, distance, rng), distance)
                )
                break

            if distance >= next_ground_sample:
                next_ground_sample = distance + rng.uniform(
                    GROUND_SAMPLE_M * 0.6, GROUND_SAMPLE_M * 1.5
                )
                returns.append(
                    Return(x, y, kind, _intensity(kind, distance, rng), distance)
                )

    return ScanResult(returns=returns, new_cells=new_cells, hits=hits)


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
