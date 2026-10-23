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
    Terrain.POTHOLE: 0.30,  # the far wall of a hole, caught at an angle
    Terrain.BUSH: 0.55,
    Terrain.UNKNOWN: 0.0,
}

#: How far ahead a hole in the ground can be resolved. A scanning LiDAR
#: sweeping a horizontal plane looks straight over a pothole and reports the
#: ground beyond it; a negative obstacle has to be found by looking *down*,
#: which is the drive stereo pair's job and only works near the vehicle.
#: Short detection range is exactly what makes holes dangerous, and it is why
#: the planner has to replan when one appears rather than route round it from
#: the start.
NEGATIVE_OBSTACLE_RANGE_M = 8.0

#: How far away a rock can be resolved as an obstacle. A boulder half a metre
#: across subtends under a degree at thirty metres, which is a return or two
#: from a one-degree scan: not something a vehicle may plan around. Inside this
#: range it is several returns wide and classified; beyond it the beam
#: passes over it the way it passes over a hole, and the cell reads as ground
#: until the vehicle is close enough to resolve it. Trees are tall as well as
#: wide and are resolved at the full range of the scanner.
ROCK_DETECTION_RANGE_M = 16.0

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

    # A hole further out than the stereo pair can resolve reads as the ground
    # it is cut into: the horizontal beam passes over it and returns off what
    # is behind. It only becomes a hole once the rover is close enough to
    # look into it.
    too_far = distance[None, :] > NEGATIVE_OBSTACLE_RANGE_M
    kinds[(kinds == int(Terrain.POTHOLE)) & too_far] = int(Terrain.GROUND)
    # A rock further out than a return or two can describe is the same: the
    # cell reads as ground, and the rock neither blocks the ray nor is mapped.
    unresolved = distance[None, :] > ROCK_DETECTION_RANGE_M
    kinds[(kinds == int(Terrain.ROCK)) & unresolved] = int(Terrain.GROUND)
    # A shrub is too low for the horizontal beam to strike at all, and is found
    # by the drive cameras instead, which only resolve it close in.
    kinds[(kinds == int(Terrain.BUSH)) & (distance[None, :] > ROCK_DETECTION_RANGE_M)] = int(Terrain.GROUND)

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


#: How much of a body's light comes back, by what it is made of: clothing and hide, not rock.
ENTITY_REFLECTIVITY = {"person": 0.55, "animal": 0.48}

#: Standard deviation of the range error on a return off a body, in metres. A real unit's is about this at these ranges.
ENTITY_RANGE_NOISE_M = 0.02


@dataclass(frozen=True)
class EntityReturns:
    """The returns off the people and animals one revolution of the scanner struck.

    Kept apart from the scan: a body that moves must not be written into the map as if it were ground, so these
    are drawn for a moment and let go. `kinds` holds "person" or "animal"."""

    xs: np.ndarray
    ys: np.ndarray
    kinds: list[str]
    intensities: np.ndarray
    ranges: np.ndarray

    @property
    def point_count(self) -> int:
        return int(self.xs.size)


def entity_returns(
    world: TerrainWorld,
    origin_x: float,
    origin_y: float,
    bodies: list[tuple[str, float, float, float, list[tuple[float, float, float]]]],
    range_m: float = LIDAR_RANGE_M,
    rays: int = LIDAR_RAYS,
    seed: int | None = None,
) -> EntityReturns:
    """Cast the scanner's rays against the outlines of moving bodies.

    `bodies` is (kind, x, y, heading, shapes), shapes being (forward offset, half-width, half-depth) of the ellipses a
    horizontal beam cuts through the body (a torso and pack; a barrel). A ray returns off the first outline it
    reaches inside range, and only if nothing that blocks a beam stands between the scanner and that point: a person
    behind a boulder returns nothing, like the boulder's own shadow.
    """
    rng = np.random.default_rng(seed if seed is not None else (int(origin_x * 61) ^ int(origin_y * 53)) & 0xFFFFFFFF)
    bearing = np.arange(rays, dtype=np.float64) / rays * math.tau
    dx, dy = np.sin(bearing), np.cos(bearing)
    xs, ys, ks, rs = [], [], [], []
    for kind, bx, by, heading, shapes in bodies:
        h = math.radians(heading)
        fwd, side = np.array([math.sin(h), math.cos(h)]), np.array([math.cos(h), -math.sin(h)])
        nearest = np.full(rays, np.inf)
        for offset, rx, ry in shapes:
            cx, cy = bx + fwd[0] * offset, by + fwd[1] * offset
            ox, oy = origin_x - cx, origin_y - cy
            lox, loy = ox * side[0] + oy * side[1], ox * fwd[0] + oy * fwd[1]
            ldx, ldy = dx * side[0] + dy * side[1], dx * fwd[0] + dy * fwd[1]
            a = (ldx / rx) ** 2 + (ldy / ry) ** 2
            b = 2.0 * (lox * ldx / rx**2 + loy * ldy / ry**2)
            c = (lox / rx) ** 2 + (loy / ry) ** 2 - 1.0
            disc = b * b - 4.0 * a * c
            with np.errstate(invalid="ignore", divide="ignore"):
                t = (-b - np.sqrt(disc)) / (2.0 * a)
            t = np.where((disc >= 0.0) & (t > 0.0), t, np.inf)
            nearest = np.minimum(nearest, t)
        for k in np.nonzero(nearest <= range_m)[0]:
            r = float(nearest[k]) + float(rng.normal(0.0, ENTITY_RANGE_NOISE_M))
            hx, hy = origin_x + dx[k] * r, origin_y + dy[k] * r
            if has_line_of_sight(world, origin_x, origin_y, hx, hy):
                xs.append(hx)
                ys.append(hy)
                ks.append(kind)
                rs.append(r)
    ranges = np.array(rs, dtype=np.float64)
    reflect = np.array([ENTITY_REFLECTIVITY.get(k, 0.5) for k in ks], dtype=np.float64)
    falloff = 1.0 / (1.0 + ranges / RANGE_HALF_POWER_M) if ranges.size else ranges
    intensity = np.clip(reflect * falloff * rng.uniform(0.78, 1.0, size=ranges.shape), 0.0, 1.0) if ranges.size else ranges
    return EntityReturns(np.array(xs, dtype=np.float64), np.array(ys, dtype=np.float64), ks, intensity, ranges)


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
