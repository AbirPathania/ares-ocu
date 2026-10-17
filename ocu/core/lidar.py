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
from dataclasses import dataclass

import numpy as np

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
    """One revolution of the scanner.

    The measurements are held as arrays, because a scan is several thousand
    of them and a list of objects costs more to build than the scan does.
    `returns` gives them as objects for anything that wants to read them one
    at a time.
    """

    xs: np.ndarray
    ys: np.ndarray
    kinds: np.ndarray  # Terrain values
    intensities: np.ndarray
    ranges: np.ndarray
    new_cells: int
    hits: int  # rays that struck an obstacle inside range

    @property
    def point_count(self) -> int:
        return int(self.xs.size)

    @property
    def returns(self) -> list[Return]:
        return [
            Return(x, y, _TERRAIN_BY_VALUE[k], i, r)
            for x, y, k, i, r in zip(
                self.xs.tolist(), self.ys.tolist(), self.kinds.tolist(),
                self.intensities.tolist(), self.ranges.tolist(),
            )
        ]


_TERRAIN_BY_VALUE = {int(t): t for t in Terrain}
_REFLECTIVITY_BY_VALUE = np.zeros(max(_TERRAIN_BY_VALUE) + 1, dtype=np.float64)
for _kind, _value in REFLECTIVITY.items():
    _REFLECTIVITY_BY_VALUE[int(_kind)] = _value


def _intensity(kinds: np.ndarray, ranges: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    falloff = 1.0 / (1.0 + ranges / RANGE_HALF_POWER_M)
    speckle = rng.uniform(0.78, 1.0, size=ranges.shape)  # real returns are noisy
    return np.clip(_REFLECTIVITY_BY_VALUE[kinds] * falloff * speckle, 0.0, 1.0)


def cast_scan(
    world: TerrainWorld,
    perception: PerceptionMap,
    origin_x: float,
    origin_y: float,
    range_m: float = LIDAR_RANGE_M,
    rays: int = LIDAR_RAYS,
    seed: int | None = None,
) -> ScanResult:
    """Sweep a full circle and return every measurement it produced.

    Every ray is sampled at once. Each ray stops at the first thing that
    blocks the beam: the cells up to and including it are observed, a hard
    return comes back off it, and nothing behind it on that bearing is
    measured. Open ground is grazed rather than struck, so it is sampled
    along the ray at irregular intervals rather than returned at every cell.
    """
    rng = np.random.default_rng(
        seed if seed is not None else (int(origin_x * 97) ^ int(origin_y * 89)) & 0xFFFFFFFF
    )
    cell = world.cell_size_m
    step = cell * 0.5
    steps = int(range_m / step)
    extent, span = world._extent, world._span

    bearing = np.arange(rays, dtype=np.float64) / rays * math.tau
    distance = step * np.arange(1, steps + 1, dtype=np.float64)
    xs = origin_x + np.sin(bearing)[:, None] * distance[None, :]  # (rays, steps)
    ys = origin_y + np.cos(bearing)[:, None] * distance[None, :]
    cx = np.floor(xs / cell).astype(np.int64)
    cy = np.floor(ys / cell).astype(np.int64)

    ix, iy = cx + extent, cy + extent
    inside = (ix >= 0) & (ix < span) & (iy >= 0) & (iy < span)
    kinds = np.full(cx.shape, int(Terrain.GROUND), dtype=np.uint8)  # beyond the world is open
    kinds[inside] = world._grid[ix[inside], iy[inside]]

    blocking = (kinds == int(Terrain.ROCK)) | (kinds == int(Terrain.TREE))
    hit = blocking.any(axis=1)
    first = np.where(hit, blocking.argmax(axis=1), steps - 1)

    # Everything up to and including the first obstruction is observed.
    seen = np.arange(steps)[None, :] <= first[:, None]
    new_cells = perception.observe_many(cx[seen], cy[seen], kinds[seen])

    # Hard returns, one per ray that struck something.
    rows = np.nonzero(hit)[0]
    hit_cols = first[rows]
    hit_x, hit_y = xs[rows, hit_cols], ys[rows, hit_cols]
    hit_kind = kinds[rows, hit_cols]
    hit_range = distance[hit_cols]

    # Ground samples: the first at a random distance, each next a random
    # interval past the last, all short of the obstruction.
    last_ground = np.where(hit, first - 1, steps - 1)
    target = rng.uniform(0.4, GROUND_SAMPLE_M, size=rays)
    every_ray = np.arange(rays)
    gx, gy, gk, gr = [], [], [], []
    while True:
        index = np.maximum(np.ceil(target / step).astype(np.int64) - 1, 0)
        valid = index <= last_ground
        if not valid.any():
            break
        r, c = every_ray[valid], index[valid]
        gx.append(xs[r, c])
        gy.append(ys[r, c])
        gk.append(kinds[r, c])
        gr.append(distance[c])
        target = (index + 1) * step + rng.uniform(
            GROUND_SAMPLE_M * 0.6, GROUND_SAMPLE_M * 1.5, size=rays
        )

    all_x = np.concatenate(gx + [hit_x])
    all_y = np.concatenate(gy + [hit_y])
    all_k = np.concatenate(gk + [hit_kind])
    all_r = np.concatenate(gr + [hit_range])
    return ScanResult(
        xs=all_x,
        ys=all_y,
        kinds=all_k,
        intensities=_intensity(all_k, all_r, rng),
        ranges=all_r,
        new_cells=new_cells,
        hits=int(hit.sum()),
    )


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
