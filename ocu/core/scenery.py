"""Dressing for the simulated world: gravel patches, cobbles, more rocks, and stepped ledges on the slopes.

The generator in terrain.py makes the country (hills, trees, scrub, a scatter of boulders); what it leaves is the small stuff that makes
ground look like ground from a few metres up, and from twenty: gravel that has washed into patches, loose cobbles lying about, rock
that has come down and stopped, and the stepped ledges a hillside breaks into. This adds them, deterministically, to a finished world.

Passable things (gravel, cobbles, ledges) go everywhere; obstacles (the extra boulders) go only where `protect` says the picture
is the simulated world's, never in the stretch near a reconstructed trail where the rover's camera shows the real photographed place
(see trail_scenario.CLEAR_ZONE_M), or the rover would be stopped by something the picture does not show.

No Qt and no GL import belongs in this file.
"""

from __future__ import annotations

import math
from typing import Callable

import numpy as np

from ocu.core.terrain import (
    BOULDER_CLEAR_RADIUS_M,
    BOULDER_OBSTACLE_MIN_TOP_M,
    ROCK_FOOTPRINT_EPS_M,
    WORLD_EDGE_MARGIN_M,
    Terrain,
    TerrainWorld,
    _field,
)

#: Gravel: patches of the ground a few metres across (the field's wavelength), covering roughly a tenth of the open ground, a little more of the track.
GRAVEL_WAVELENGTH_M = 14.0
#: (the noise field is centred on zero, about -0.4 to 0.5: its 90th percentile is 0.32, so these put gravel on roughly a tenth of the open ground and a fifth of the track)
GRAVEL_THRESHOLD = (0.26, 0.38)
GRAVEL_TRACK_THRESHOLD = (0.16, 0.28)
#: Small stones lying in a gravel patch, per square metre at its heart, and the most that are drawn in all (each is an instance).
GRAVEL_STONES_PER_M2 = 2.0
GRAVEL_STONE_RADIUS_M = (0.07, 0.17)  # the stone models are drawn at their radius, and anything smaller is under a pixel at a few metres
MAX_GRAVEL_STONES = 9000
#: Cobbles: loose stones the size of a fist to a head, driven over (lower than BOULDER_OBSTACLE_MIN_TOP_M), per square metre of open ground, gathering in patches.
COBBLES_PER_M2 = 0.035
COBBLE_RADIUS_M = (0.09, 0.30)
#: More rock that has come down: obstacles, per square metre, with the sizes of the generator's own.
EXTRA_BOULDERS_PER_M2 = 0.004
EXTRA_BOULDER_RADIUS_M = (0.45, 2.2)
#: Stepped ledges: where the ground is gently to moderately sloped, a hillside breaks into treads and risers of this height, over a patch this wide, this far from the protected line.
LEDGE_SLOPE_RANGE = (0.12, 0.45)
LEDGE_RISER_M = 0.45
LEDGE_RADIUS_M = 6.5
LEDGE_STONE_RADIUS_M = (0.25, 0.55)  # flat stones along the lip of each riser, lower than an obstacle
LEDGE_COUNT = 12
LEDGE_SPACING_M = 20.0
LEDGE_FROM_LINE_M = 12.0


def _smoothstep(lo: float, hi: float, x: np.ndarray) -> np.ndarray:
    t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def dress(world: TerrainWorld, protect: Callable[[np.ndarray], np.ndarray] | None = None, protect_m: float = 0.0, seed: int | None = None) -> dict[str, int]:
    """Add gravel, cobbles, rocks and ledges to a finished world. `protect(points)` is how far each (x, y) of an (n, 2) array is from the part of the world that has to stay clear
    of obstacles; with `protect_m`, obstacles keep that much (and their own radius) away from it. Without `protect` there are no obstacles added (only what can be driven over).
    Returns the counts of what was added."""
    rng = np.random.default_rng((world.seed if seed is None else seed) ^ 0x5CE4E)
    span, extent, cell = world._span, world._extent, world.cell_size_m
    axis = (np.arange(span) - extent + 0.5) * cell
    gx, gy = np.meshgrid(axis, axis, indexing="ij")
    grid = world._grid
    free = ~np.isin(grid, [int(Terrain.TREE), int(Terrain.ROCK), int(Terrain.POTHOLE), int(Terrain.BUSH)])
    inside = (np.abs(gx) < world.half_extent_m - WORLD_EDGE_MARGIN_M) & (np.abs(gy) < world.half_extent_m - WORLD_EDGE_MARGIN_M)
    counts = {"gravel_cells": 0, "gravel_stones": 0, "cobbles": 0, "boulders": 0, "ledges": 0}

    # --- gravel ---------------------------------------------------------------------------------------------------------------------------
    patch = _field(world.seed + 31, span, extent, cell, GRAVEL_WAVELENGTH_M, 3, coarse=2)
    gravel = _smoothstep(*GRAVEL_THRESHOLD, patch)
    gravel = np.where(grid == int(Terrain.TRACK), np.maximum(gravel, _smoothstep(*GRAVEL_TRACK_THRESHOLD, patch)), gravel)
    gravel = (gravel * free).astype(np.float32)
    world.gravel = gravel
    counts["gravel_cells"] = int((gravel > 0.5).sum())

    stone_rows: list[np.ndarray] = []
    expected = GRAVEL_STONES_PER_M2 * gravel * cell * cell * inside
    n_stones = rng.poisson(expected)
    ids = np.repeat(np.arange(n_stones.size), n_stones.ravel())
    if len(ids) > MAX_GRAVEL_STONES:
        ids = rng.choice(ids, MAX_GRAVEL_STONES, replace=False)
    ix, iy = np.unravel_index(ids, n_stones.shape)
    low, high = GRAVEL_STONE_RADIUS_M
    radius = rng.uniform(low, high, len(ids))
    stone_rows.append(np.stack([axis[ix] + rng.uniform(-cell / 2, cell / 2, len(ids)), axis[iy] + rng.uniform(-cell / 2, cell / 2, len(ids)), radius, radius * rng.uniform(0.45, 0.8, len(ids)),
                                rng.integers(0, 1 << 30, len(ids)).astype(np.float64)], axis=1))
    counts["gravel_stones"] = len(ids)

    # --- cobbles ----------------------------------------------------------------------------------------------------------------------------
    gather = np.clip(1.0 + 2.0 * _field(world.seed + 53, span, extent, cell, 22.0, 2, coarse=2), 0.05, 3.0)
    expected = COBBLES_PER_M2 * gather * cell * cell * free * inside
    n_cobbles = rng.poisson(expected)
    ids = np.repeat(np.arange(n_cobbles.size), n_cobbles.ravel())
    ix, iy = np.unravel_index(ids, n_cobbles.shape)
    low, high = COBBLE_RADIUS_M
    radius = np.exp(rng.uniform(math.log(low), math.log(high), len(ids)))
    top = np.minimum(radius * rng.uniform(0.35, 0.65, len(ids)), BOULDER_OBSTACLE_MIN_TOP_M - 0.03)
    stone_rows.append(np.stack([axis[ix] + rng.uniform(-cell / 2, cell / 2, len(ids)), axis[iy] + rng.uniform(-cell / 2, cell / 2, len(ids)), radius, top,
                                rng.integers(0, 1 << 30, len(ids)).astype(np.float64)], axis=1))
    counts["cobbles"] = len(ids)

    # --- more rock, where the picture is the simulated world's -------------------------------------------------------------------------------
    if protect is not None:
        clustering = np.clip(1.0 + 2.5 * _field(world.seed + 71, span, extent, cell, 30.0, 3, coarse=2), 0.05, 3.5)
        expected = EXTRA_BOULDERS_PER_M2 * clustering * cell * cell * free * inside
        n_rocks = rng.poisson(expected)
        ids = np.repeat(np.arange(n_rocks.size), n_rocks.ravel())
        ix, iy = np.unravel_index(ids, n_rocks.shape)
        x = axis[ix] + rng.uniform(-cell / 2, cell / 2, len(ids))
        y = axis[iy] + rng.uniform(-cell / 2, cell / 2, len(ids))
        low, high = EXTRA_BOULDER_RADIUS_M
        r = np.clip(np.exp(rng.normal(math.log(0.85), 0.45, len(ids))), low, high)
        keep = (np.hypot(x, y) > BOULDER_CLEAR_RADIUS_M) & (protect(np.stack([x, y], axis=1)) > protect_m + r)
        placed = []
        height = np.zeros_like(world._rock_height)
        for xi, yi, ri in zip(x[keep], y[keep], r[keep]):
            aspect = rng.uniform(0.55, 1.0)
            theta = rng.uniform(0.0, math.pi)
            top = ri * rng.uniform(0.5, 0.85)
            if top < BOULDER_OBSTACLE_MIN_TOP_M:
                continue
            world._raise_dome(height, xi, yi, ri, ri * aspect, theta, top)
            placed.append((xi, yi, ri, ri * aspect, theta, top))
        if placed:
            rows = np.array(placed, dtype=np.float64).reshape(-1, 6)
            world.boulders = np.concatenate([world.boulders, rows])
            world._rock_height = np.maximum(world._rock_height, height)
            footprint = world._boulder_footprint(rows) | (height > ROCK_FOOTPRINT_EPS_M)
            world._grid[footprint & (world._grid != int(Terrain.TREE))] = int(Terrain.ROCK)
        counts["boulders"] = len(placed)

    world.pebbles = np.concatenate([world.pebbles.reshape(-1, 5)] + stone_rows)

    # --- stepped ledges on the slopes ---------------------------------------------------------------------------------------------------------
    gradient = np.hypot(*np.gradient(world._elevation, cell))
    candidates = np.argwhere(free & inside & (gradient > LEDGE_SLOPE_RANGE[0]) & (gradient < LEDGE_SLOPE_RANGE[1]))
    rng.shuffle(candidates)
    chosen: list[tuple[float, float]] = []
    for cx, cy in candidates:
        x, y = float(axis[cx]), float(axis[cy])
        if protect is not None and protect(np.array([[x, y]]))[0] < LEDGE_FROM_LINE_M:
            continue
        if math.hypot(x, y) < BOULDER_CLEAR_RADIUS_M + LEDGE_RADIUS_M:
            continue
        if any(math.hypot(x - a, y - b) < LEDGE_SPACING_M for a, b in chosen):
            continue
        chosen.append((x, y))
        if len(chosen) >= LEDGE_COUNT:
            break
    elevation = world._elevation
    ledge_stones: list[np.ndarray] = []
    for x, y in chosen:
        dist = np.hypot(gx - x, gy - y)
        window = np.where(dist < LEDGE_RADIUS_M, 0.5 + 0.5 * np.cos(np.pi * dist / LEDGE_RADIUS_M), 0.0)
        reference = elevation[int(round(x / cell)) + extent, int(round(y / cell)) + extent]
        u = (elevation - reference) / LEDGE_RISER_M
        fraction = u - np.floor(u)
        stepped = LEDGE_RISER_M * (np.floor(u) + _smoothstep(0.62, 1.0, fraction))  # a flat tread over most of it, then a riser
        elevation = np.where(window > 0, elevation + window * (reference + stepped - elevation), elevation)
        lip = (window > 0.35) & (fraction > 0.80) & (fraction < 0.97) & free & inside  # the top edge of each riser
        ly, lx = np.nonzero(lip)
        pick = rng.random(len(ly)) < 0.30
        radius = rng.uniform(*LEDGE_STONE_RADIUS_M, int(pick.sum()))
        ledge_stones.append(np.stack([axis[ly[pick]] + rng.uniform(-cell / 2, cell / 2, len(radius)), axis[lx[pick]] + rng.uniform(-cell / 2, cell / 2, len(radius)), radius,
                                      radius * rng.uniform(0.4, 0.6, len(radius)), rng.integers(0, 1 << 30, len(radius)).astype(np.float64)], axis=1))
    if ledge_stones:
        world.pebbles = np.concatenate([world.pebbles.reshape(-1, 5)] + ledge_stones)
    world._elevation = elevation.astype(world._elevation.dtype)
    world.ledge_sites = list(chosen)  # where the hillside was broken into steps (for looking at)
    counts["ledges"] = len(chosen)
    return counts
