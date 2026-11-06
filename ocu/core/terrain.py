"""The ground the rover drives over: an actual landscape, not stamped shapes.

The first version scattered circular blobs and called them trees, which
looked like what it was. Real ground is not a set of independent shapes
dropped on a plane, it is correlated. Rock is exposed where the slope is
steep enough to shed soil. Vegetation grows where there is moisture and
the ground is not too steep. Dense tree cover sits in the wetter hollows,
scrub takes the margins, and a worn track follows the easiest line
through all of it. Rock is not a class painted over a region but a
population of individual boulders, each a rounded, partly buried dome of its
own size and height: sparse on flat ground, thick on slopes, and heaped into
outcrops where it is steepest. The cells it occupies, and how tall it
stands, both come from those domes.

So the landscape is generated the way terrain generators actually work:
fractal Brownian motion over simplex noise gives an elevation field, a
second field gives moisture, and the classification falls out of
elevation, slope and moisture together. Nothing is stamped. The shapes
are irregular because they are the contours of a field, which is also why
trees end up in the hollows and rock on the ridges without either being
placed there deliberately.

Two separate things live here and must not be confused:

- **Ground truth.** The generated world. The operator never sees it.
- **The rover's map.** What LiDAR has actually returned, accumulated. It
  starts empty and fills in only where the sensor reached.

Everything is referenced to the deploy point at the origin, with +y along
true north. The rover carries its own compass and inertial solution, so
the whole picture is held in that frame with no external reference.

Uses numpy and opensimplex, both open source. No Qt import belongs here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np
from opensimplex import OpenSimplex


class Terrain(IntEnum):
    UNKNOWN = 0
    GROUND = 1  # open, traversable
    TRACK = 2  # a worn path; the easiest going
    SCRUB = 3  # low vegetation, traversable but slow
    ROCK = 4  # impassable, and it blocks the beam
    TREE = 5  # impassable, and it blocks the beam
    POTHOLE = 6  # a hole: impassable, but you can see straight over it
    BUSH = 7  # a shrub: impassable, but low enough for a horizontal beam to pass over


_TERRAIN_BY_VALUE = {int(t): t for t in Terrain}

#: Classes that stop a LiDAR ray.
OCCLUDING = frozenset({Terrain.ROCK, Terrain.TREE})

#: Classes a rover cannot drive through. A hole stops the wheels without
#: stopping the beam, which is why these two sets are not the same: a
#: scanning LiDAR looks straight over a pothole and sees the ground beyond.
#: Negative obstacles are found by looking down, not across, which is what
#: the drive stereo pair is for (see lidar.NEGATIVE_OBSTACLE_RANGE_M).
IMPASSABLE = OCCLUDING | frozenset({Terrain.POTHOLE, Terrain.BUSH})

#: Relative cost of crossing each traversable class, for the planner.
TRAVERSE_COST = {
    Terrain.TRACK: 1.0,
    Terrain.GROUND: 1.4,
    Terrain.SCRUB: 3.2,
    Terrain.UNKNOWN: 2.0,  # worth probing, but not preferred over known ground
}

CELL_SIZE_M = 0.5
WORLD_HALF_EXTENT_M = 110.0

# Classification thresholds, tuned so the operating area reads as mixed
# open country: mostly traversable, with enough tree and rock to cast real
# sensor shadows and to give the planner something to route around. Too
# much impassable ground and the scanner cannot see out of its own
# clearing, which is both unrealistic and a poor demonstration.
SCRUB_MOISTURE = 0.06
TREE_MOISTURE = 0.34
TREE_MAX_SLOPE = 0.22
ROCK_SLOPE = 0.46  # slope at which scree is as thick as it gets

# Boulders. A rock is an object with a size, not a patch of ground, so the
# population is drawn the way a real one is distributed: many small, few
# large (lognormal in radius), rarely round, partly buried, and far thicker
# where the ground is steep enough to shed them.
BOULDER_MEDIAN_RADIUS_M = 0.5
BOULDER_RADIUS_SIGMA = 0.70
BOULDER_RADIUS_RANGE_M = (0.16, 3.6)
BOULDER_FLAT_DENSITY = 0.0042  # per square metre on level ground
BOULDER_SCREE_DENSITY = 0.040  # extra per square metre where it is steepest
SCREE_ONSET_SLOPE = 0.16
OUTCROP_SLOPE = 0.38
OUTCROP_CHANCE_PER_CELL = 0.0035
BOULDER_CLEAR_RADIUS_M = 8.0  # around the deploy point

#: How tall a boulder has to stand before a wheeled vehicle treats it as an
#: obstacle. Lower than this is a stone, drawn as a pebble and driven over.
#: An obstacle is impassable across its whole footprint, not only where its
#: dome is high: a rover that drove up the flank of a boulder because the
#: flank was under thirty centimetres looked, to anyone watching, like a rover
#: driving through a rock.
BOULDER_OBSTACLE_MIN_TOP_M = 0.20
ROCK_FOOTPRINT_EPS_M = 0.02

#: How far a mapped footprint is grown past the object, so a cell the object
#: only touches is on the map. Most of a cell's half-diagonal (0.35 m).
FOOTPRINT_GROWTH_M = 0.3

# Trees and bushes are individual plants, placed inside the ground that carries
# them and kept clear of each other, so what is drawn is what is there.
TREE_SPACING_M = 2.6
TREE_HEIGHT_RANGE_M = (4.5, 9.5)
TREE_CROWN_RATIO = (0.20, 0.30)  # crown radius over height
TREE_EDGE_MARGIN_M = 1.9  # a crown stays this far inside its stand
TREE_MAX_CROWN_M = 2.3
WORLD_EDGE_MARGIN_M = 5.0  # nothing is placed within this of the world's edge
BUSH_SPACING_M = 4.6
BUSH_RADIUS_RANGE_M = (0.45, 0.95)
BUSH_CHANCE_SCRUB = 0.50
BUSH_CHANCE_GROUND = 0.06
BUSH_CLEAR_RADIUS_M = 8.0

# Potholes. Washouts and collapses on and beside a worn track: wider than a
# wheel and deep enough to drop one in, which for a six-wheel platform is a
# stop rather than a bump. They gather where water runs, so they follow the
# track and the damper ground rather than falling evenly.
POTHOLE_RADIUS_RANGE_M = (0.45, 1.6)
POTHOLE_DEPTH_RANGE_M = (0.22, 0.75)
POTHOLE_DENSITY_OPEN = 0.008  # per square metre away from a track
POTHOLE_DENSITY_TRACK = 0.030  # per square metre on one
POTHOLE_CLEAR_RADIUS_M = 7.0  # around the deploy point

#: Depth at which a hole stops being a bump and becomes an obstacle.
POTHOLE_IMPASSABLE_DEPTH_M = 0.25

#: Height given to a cell painted ROCK by hand, as tests and scenarios do.
PAINTED_ROCK_HEIGHT_M = 0.9


def _fbm(seed: int, coords: np.ndarray, scale_m: float, octaves: int) -> np.ndarray:
    """Fractal Brownian motion: octaves of simplex noise at halving amplitude.

    One octave of noise is smooth and synthetic looking. Summing octaves at
    halving amplitude and doubling frequency is what gives natural terrain
    detail at every scale, which is why it is the standard approach.
    """
    noise = OpenSimplex(seed)
    total = np.zeros((coords.size, coords.size), dtype=np.float64)
    amplitude, frequency, norm = 1.0, 1.0 / scale_m, 0.0
    for _ in range(octaves):
        total += amplitude * noise.noise2array(coords * frequency, coords * frequency)
        norm += amplitude
        amplitude *= 0.5
        frequency *= 2.0
    return total / norm


def _bilinear_upsample(field: np.ndarray, span: int) -> np.ndarray:
    """Resample a coarse field up to the working grid."""
    height, width = field.shape
    yi = np.linspace(0, height - 1, span)
    xi = np.linspace(0, width - 1, span)
    y0 = np.floor(yi).astype(int)
    x0 = np.floor(xi).astype(int)
    y1 = np.minimum(y0 + 1, height - 1)
    x1 = np.minimum(x0 + 1, width - 1)
    wy = (yi - y0)[:, None]
    wx = (xi - x0)[None, :]

    top = field[np.ix_(y0, x0)] * (1 - wx) + field[np.ix_(y0, x1)] * wx
    bottom = field[np.ix_(y1, x0)] * (1 - wx) + field[np.ix_(y1, x1)] * wx
    return top * (1 - wy) + bottom * wy


def _field(seed: int, span: int, extent: int, cell_size_m: float,
           scale_m: float, octaves: int, coarse: int) -> np.ndarray:
    """A noise field generated coarsely and resampled up.

    These fields vary over tens of metres, so evaluating simplex noise at
    every half-metre cell is roughly fifteen times the work for detail that
    is not there. Generating at a few metres and resampling is visually
    identical and takes the world build from seventeen seconds to about
    one, which matters because it happens on launch.
    """
    coarse_span = max(8, span // coarse)
    coords = (np.linspace(-extent, extent, coarse_span)) * cell_size_m
    return _bilinear_upsample(_fbm(seed, coords, scale_m, octaves), span)


@dataclass
class TerrainWorld:
    """Ground truth, on a square grid centred on the deploy point."""

    cell_size_m: float = CELL_SIZE_M
    half_extent_m: float = WORLD_HALF_EXTENT_M
    seed: int = 20261016
    _grid: np.ndarray = field(default=None, repr=False)
    _elevation: np.ndarray = field(default=None, repr=False)
    #: Height of rock standing above the ground, per cell, in metres. The
    #: source of truth for rock: the ROCK class is where this exceeds
    #: ROCK_IMPASSABLE_HEIGHT_M, and the cameras draw the domes themselves.
    _rock_height: np.ndarray = field(default=None, repr=False)
    #: Depth of any hole in each cell, in metres. Like rock height, this is
    #: the source of truth: the POTHOLE class is where it exceeds
    #: POTHOLE_IMPASSABLE_DEPTH_M, and the cameras render the dip itself.
    _pothole_depth: np.ndarray = field(default=None, repr=False)
    #: The individual things standing in the world, which the cameras draw and
    #: whose footprints the class grid is made from. Rows are
    #: boulders (x, y, semi_major, semi_minor, theta, top),
    #: trees (x, y, height, crown_radius, species),
    #: bushes (x, y, radius, seed) and pebbles (x, y, radius, height, seed).
    boulders: np.ndarray = field(default=None, repr=False)
    trees: np.ndarray = field(default=None, repr=False)
    bushes: np.ndarray = field(default=None, repr=False)
    pebbles: np.ndarray = field(default=None, repr=False)
    #: How much of each cell is gravel, 0 to 1 (core/scenery.py): drawn as a patch of loose stone, driven over like any open ground. None where nothing has dressed the world.
    gravel: np.ndarray = field(default=None, repr=False)

    def __post_init__(self) -> None:
        self._extent = int(self.half_extent_m / self.cell_size_m)
        self._span = self._extent * 2 + 1
        if self._grid is None:
            self._generate()
        for name, width in (("boulders", 6), ("trees", 5), ("bushes", 4), ("pebbles", 5)):
            if getattr(self, name) is None:
                setattr(self, name, np.zeros((0, width), dtype=np.float64))
        if self._rock_height is None:
            self._rock_height = np.zeros((self._span, self._span), dtype=np.float32)
            self._rock_height[self._grid == int(Terrain.ROCK)] = PAINTED_ROCK_HEIGHT_M
        if self._pothole_depth is None:
            self._pothole_depth = np.zeros((self._span, self._span), dtype=np.float32)
            self._pothole_depth[self._grid == int(Terrain.POTHOLE)] = POTHOLE_IMPASSABLE_DEPTH_M

    @classmethod
    def blank(cls, half_extent_m: float = WORLD_HALF_EXTENT_M, cell_size_m: float = CELL_SIZE_M):
        """Featureless open ground, for controlled tests of the sensor."""
        extent = int(half_extent_m / cell_size_m)
        span = extent * 2 + 1
        return cls(
            cell_size_m=cell_size_m,
            half_extent_m=half_extent_m,
            _grid=np.full((span, span), int(Terrain.GROUND), dtype=np.uint8),
            _elevation=np.zeros((span, span), dtype=np.float64),
            _rock_height=np.zeros((span, span), dtype=np.float32),
            _pothole_depth=np.zeros((span, span), dtype=np.float32),
        )

    # --- frame conversion -------------------------------------------------

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor(x / self.cell_size_m)), int(math.floor(y / self.cell_size_m)))

    def to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        return (cell[0] * self.cell_size_m, cell[1] * self.cell_size_m)

    def _index(self, cell: tuple[int, int]) -> tuple[int, int] | None:
        ix, iy = cell[0] + self._extent, cell[1] + self._extent
        if 0 <= ix < self._span and 0 <= iy < self._span:
            return ix, iy
        return None

    def at(self, cell: tuple[int, int]) -> Terrain:
        index = self._index(cell)
        return Terrain(self._grid[index]) if index else Terrain.GROUND

    def set_cell(self, cell: tuple[int, int], kind: Terrain) -> None:
        index = self._index(cell)
        if index:
            self._grid[index] = int(kind)
            # Whatever is painted over a boulder removes it, and a cell
            # painted rock stands up, so the two views of rock never disagree.
            if kind == Terrain.ROCK:
                self._rock_height[index] = max(float(self._rock_height[index]), PAINTED_ROCK_HEIGHT_M)
                self._pothole_depth[index] = 0.0
            elif kind == Terrain.POTHOLE:
                self._pothole_depth[index] = max(
                    float(self._pothole_depth[index]), POTHOLE_IMPASSABLE_DEPTH_M
                )
                self._rock_height[index] = 0.0
            else:
                self._rock_height[index] = 0.0
                self._pothole_depth[index] = 0.0

    def stamp_object(
        self, x: float, y: float, length_m: float, width_m: float, heading_deg: float, growth_m: float = FOOTPRINT_GROWTH_M
    ) -> list[tuple[int, int]]:
        """Make something that stands on the ground, a parked vehicle, an obstacle.

        The cells an oriented rectangle covers, grown past its edge the way a boulder's
        are (a cell it only touches is on the map), become ROCK: impassable, and a
        surface a LiDAR ray stops at. The sensor cannot tell a truck from a boulder,
        and does not need to; what the thing is comes from the cameras. Heading is in
        degrees clockwise from +y, as everywhere else. Returns the cells stamped.
        """
        h = math.radians(heading_deg)
        fwd, side = (math.sin(h), math.cos(h)), (math.cos(h), -math.sin(h))
        half_l, half_w = length_m / 2.0 + growth_m, width_m / 2.0 + growth_m
        reach = int(math.ceil(math.hypot(half_l, half_w) / self.cell_size_m)) + 1
        cx, cy = self.to_cell(x, y)
        stamped = []
        for ix in range(cx - reach, cx + reach + 1):
            for iy in range(cy - reach, cy + reach + 1):
                wx, wy = (ix + 0.5) * self.cell_size_m, (iy + 0.5) * self.cell_size_m
                dx, dy = wx - x, wy - y
                if abs(dx * fwd[0] + dy * fwd[1]) <= half_l and abs(dx * side[0] + dy * side[1]) <= half_w and self._index((ix, iy)):
                    self.set_cell((ix, iy), Terrain.ROCK)
                    stamped.append((ix, iy))
        return stamped

    def rock_height_at(self, cell: tuple[int, int]) -> float:
        index = self._index(cell)
        return float(self._rock_height[index]) if index else 0.0

    def pothole_depth_at(self, cell: tuple[int, int]) -> float:
        index = self._index(cell)
        return float(self._pothole_depth[index]) if index else 0.0

    def classify(self, x: float, y: float) -> Terrain:
        return self.at(self.to_cell(x, y))

    def elevation_at(self, cell: tuple[int, int]) -> float:
        index = self._index(cell)
        return float(self._elevation[index]) if index else 0.0

    def blocks_beam(self, cell: tuple[int, int]) -> bool:
        return self.at(cell) in OCCLUDING

    # --- generation -------------------------------------------------------

    def _generate(self) -> None:
        span, extent, cell = self._span, self._extent, self.cell_size_m

        elevation = _field(self.seed, span, extent, cell, 85.0, 5, coarse=4) * 14.0
        moisture = _field(self.seed + 977, span, extent, cell, 42.0, 3, coarse=4)
        detail = _field(self.seed + 1511, span, extent, cell, 9.0, 2, coarse=2)

        # Rock is exposed where the ground is too steep to hold soil, so the
        # slope of the elevation field is what places it. The gradient is
        # taken before resampling: differentiating an interpolated field
        # gives a slope that is flat inside each coarse cell and steps at
        # the seams, which would put rock in a grid pattern.
        coarse_span = max(8, span // 4)
        coarse_coords = np.linspace(-extent, extent, coarse_span) * cell
        coarse_elevation = _fbm(self.seed, coarse_coords, 85.0, 5) * 14.0
        coarse_step = (coarse_coords[-1] - coarse_coords[0]) / max(1, coarse_span - 1)
        cgy, cgx = np.gradient(coarse_elevation, coarse_step)
        slope = _bilinear_upsample(np.hypot(cgx, cgy), span)

        grid = np.full((self._span, self._span), int(Terrain.GROUND), dtype=np.uint8)

        scrub = (moisture + 0.25 * detail) > SCRUB_MOISTURE
        grid[scrub] = int(Terrain.SCRUB)

        # Trees take the wettest ground that is not steep, which puts them
        # in the hollows rather than anywhere in particular.
        trees = (moisture + 0.30 * detail > TREE_MOISTURE) & (slope < TREE_MAX_SLOPE)
        grid[trees] = int(Terrain.TREE)

        # Boulders go where trees are not: a stand of trees and a scree
        # slope are different ground, and a boulder inside a crown is not
        # one anyone can see.
        rock_height = self._scatter_boulders(slope, detail, trees)
        grid[self._footprint | (rock_height > ROCK_FOOTPRINT_EPS_M)] = int(Terrain.ROCK)

        self._elevation = elevation
        self._grid = grid
        self._rock_height = rock_height
        # Empty for now: carving the track writes cells, and set_cell keeps
        # the hazard fields in step with the classes.
        self._pothole_depth = np.zeros((span, span), dtype=np.float32)

        self._carve_track(slope, moisture)
        self._pothole_depth = self._scatter_potholes(moisture)
        deep = self._pothole_depth >= POTHOLE_IMPASSABLE_DEPTH_M
        grid[deep & (grid != int(Terrain.ROCK)) & (grid != int(Terrain.TREE))] = int(Terrain.POTHOLE)
        # A hole is a hole in the ground, so the ground goes down with it and
        # the cameras show the dip rather than a flat patch of a new colour.
        self._elevation = self._elevation - self._pothole_depth
        self._clear_deploy_point()
        self._scatter_trees(detail)
        self._scatter_bushes()

    def _scatter_boulders(
        self, slope: np.ndarray, detail: np.ndarray, trees: np.ndarray
    ) -> np.ndarray:
        """Draw the boulder population and raise it onto a height field.

        Position comes from a density that rises with slope (scree is shed
        downhill) and is modulated by a noise field so boulders gather in
        patches instead of falling like rain. Size is lognormal. Where the
        ground is steepest, heaps of large boulders form outcrops.
        """
        rng = np.random.default_rng(self.seed ^ 0xB011D3)
        span, cell = self._span, self.cell_size_m

        steep = np.clip((slope - SCREE_ONSET_SLOPE) / (ROCK_SLOPE - SCREE_ONSET_SLOPE), 0.0, 1.0)
        steep = steep * steep * (3.0 - 2.0 * steep)  # smoothstep
        clustering = np.clip(0.55 + 0.9 * detail, 0.05, 2.0)
        density = (BOULDER_FLAT_DENSITY + BOULDER_SCREE_DENSITY * steep) * clustering
        density = np.where(trees, 0.0, density)

        counts = rng.poisson(density * cell * cell)
        cell_ids = np.repeat(np.arange(counts.size), counts.ravel())
        ix, iy = np.unravel_index(cell_ids, counts.shape)
        centre_x = (ix + rng.uniform(0.0, 1.0, ix.size) - self._extent) * cell
        centre_y = (iy + rng.uniform(0.0, 1.0, iy.size) - self._extent) * cell

        radius = np.exp(rng.normal(math.log(BOULDER_MEDIAN_RADIUS_M), BOULDER_RADIUS_SIGMA, ix.size))
        radius *= 1.0 + 0.6 * steep[ix, iy]  # scree runs larger
        boulders = [
            (x, y, r) for x, y, r in zip(centre_x, centre_y, radius)
        ]

        # Outcrops: a heap of overlapping large boulders where it is steepest.
        steep_cells = np.argwhere((slope > OUTCROP_SLOPE) & ~trees)
        chosen = steep_cells[rng.random(len(steep_cells)) < OUTCROP_CHANCE_PER_CELL]
        for cx, cy in chosen:
            ox, oy = (cx - self._extent) * cell, (cy - self._extent) * cell
            for _ in range(int(rng.integers(4, 8))):
                boulders.append(
                    (
                        ox + rng.normal(0.0, 1.8),
                        oy + rng.normal(0.0, 1.8),
                        rng.uniform(1.4, 3.6),
                    )
                )

        height = np.zeros((span, span), dtype=np.float32)
        low, high = BOULDER_RADIUS_RANGE_M
        placed: list[tuple[float, float, float, float, float, float]] = []
        stones: list[tuple[float, float, float, float, float]] = []
        for x, y, r in boulders:
            if math.hypot(x, y) < BOULDER_CLEAR_RADIUS_M:
                continue
            if abs(x) > self.half_extent_m - WORLD_EDGE_MARGIN_M or abs(y) > self.half_extent_m - WORLD_EDGE_MARGIN_M:
                continue
            r = min(max(r, low), high)
            aspect = rng.uniform(0.55, 1.0)
            theta = rng.uniform(0.0, math.pi)
            top = r * rng.uniform(0.5, 0.9)  # partly buried: lower than it is wide
            if top < BOULDER_OBSTACLE_MIN_TOP_M:
                # A stone, not an obstacle: drawn, and driven over.
                stones.append((x, y, r, top, float(rng.integers(0, 1 << 30))))
                continue
            self._raise_dome(height, x, y, r, r * aspect, theta, top)
            placed.append((x, y, r, r * aspect, theta, top))
        self.boulders = np.array(placed, dtype=np.float64).reshape(-1, 6)
        self.pebbles = np.array(stones, dtype=np.float64).reshape(-1, 5)
        self._footprint = self._boulder_footprint(self.boulders)
        return height

    def _boulder_footprint(self, boulders: np.ndarray) -> np.ndarray:
        """The cells a boulder touches, not only the cells whose centre it covers.

        Rasterising to cell centres leaves the outer third of a cell's width of
        every boulder off the map, and a vehicle keeping a margin from the
        mapped edge then touches the real one. Growing each ellipse by most of a
        cell's half-diagonal makes the map err on the side of the rock.
        """
        mask = np.zeros((self._span, self._span), dtype=bool)
        cell = self.cell_size_m
        grow = FOOTPRINT_GROWTH_M
        for x, y, a, b, theta, _top in boulders:
            reach = int(math.ceil((a + grow) / cell)) + 1
            cx = int(math.floor(x / cell)) + self._extent
            cy = int(math.floor(y / cell)) + self._extent
            x0, x1 = max(cx - reach, 0), min(cx + reach + 1, self._span)
            y0, y1 = max(cy - reach, 0), min(cy + reach + 1, self._span)
            if x0 >= x1 or y0 >= y1:
                continue
            gx = (np.arange(x0, x1) - self._extent + 0.5) * cell - x
            gy = (np.arange(y0, y1) - self._extent + 0.5) * cell - y
            dx, dy = np.meshgrid(gx, gy, indexing="ij")
            c, s = math.cos(theta), math.sin(theta)
            u = (dx * c + dy * s) / (a + grow)
            v = (-dx * s + dy * c) / (b + grow)
            mask[x0:x1, y0:y1] |= (u * u + v * v) <= 1.0
        return mask

    def _scatter_trees(self, detail: np.ndarray) -> None:
        """Individual trees, on a jittered grid, inside the stands.

        The class grid marks a stand as a region; what stands in it is trees.
        Each is placed well inside the region so its crown does not reach the
        cells outside it, which means everything a camera shows as a tree is
        something the map shows as impassable.
        """
        rng = np.random.default_rng(self.seed ^ 0x7233)
        cell = self.cell_size_m
        margin = int(math.ceil(TREE_EDGE_MARGIN_M / cell))
        mask = self._grid == int(Terrain.TREE)
        inner = mask.copy()
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                if dx * dx + dy * dy > margin * margin:
                    continue
                inner &= np.roll(np.roll(mask, dx, axis=0), dy, axis=1)

        reach = self.half_extent_m - 2.0
        axis = np.arange(-reach, reach, TREE_SPACING_M)
        gx, gy = np.meshgrid(axis, axis, indexing="ij")
        gx = gx + rng.uniform(-0.42, 0.42, gx.shape) * TREE_SPACING_M
        gy = gy + rng.uniform(-0.42, 0.42, gy.shape) * TREE_SPACING_M
        ix = np.floor(gx / cell).astype(np.int64) + self._extent
        iy = np.floor(gy / cell).astype(np.int64) + self._extent
        ok = (ix >= 0) & (ix < self._span) & (iy >= 0) & (iy < self._span)
        keep = np.zeros(gx.shape, dtype=bool)
        keep[ok] = inner[ix[ok], iy[ok]]
        keep &= np.hypot(gx, gy) > 6.5  # not on the deploy point
        keep &= (np.abs(gx) < self.half_extent_m - WORLD_EDGE_MARGIN_M) & (np.abs(gy) < self.half_extent_m - WORLD_EDGE_MARGIN_M)

        xs, ys = gx[keep], gy[keep]
        count = xs.size
        low, high = TREE_HEIGHT_RANGE_M
        # Height varies smoothly across a stand, with a little per tree.
        patch = np.sin(xs * 0.11 + 1.3) * np.cos(ys * 0.09 - 0.7)
        heights = low + (high - low) * np.clip(0.5 + 0.35 * patch + rng.normal(0.0, 0.16, count), 0.0, 1.0)
        crowns = np.minimum(heights * rng.uniform(*TREE_CROWN_RATIO, size=count), TREE_MAX_CROWN_M)
        # The footage's forest: acacia throughout (0), a rounder broadleaf in the wetter parts of a stand (1), and trees
        # standing leafless in the dry season (2), more of them on the drier, higher side.
        field = np.sin(xs * 0.045 - ys * 0.037 + 0.4) + rng.normal(0.0, 0.25, count)
        species = np.where(field > 0.55, 2.0, np.where(field < -0.35, 1.0, 0.0))
        self.trees = np.stack([xs, ys, heights, crowns, species], axis=1)

    def _scatter_bushes(self) -> None:
        """Shrubs, each a small impassable patch of its own.

        Broad scrub stays what it was, low cover that costs more to cross. A
        bush is a thing with an edge: the camera draws it, the LiDAR resolves it
        at close range, and the planner goes round it.
        """
        rng = np.random.default_rng(self.seed ^ 0xB05E)
        cell = self.cell_size_m
        reach = self.half_extent_m - 4.0
        axis = np.arange(-reach, reach, BUSH_SPACING_M)
        gx, gy = np.meshgrid(axis, axis, indexing="ij")
        gx = (gx + rng.uniform(-0.4, 0.4, gx.shape) * BUSH_SPACING_M).ravel()
        gy = (gy + rng.uniform(-0.4, 0.4, gy.shape) * BUSH_SPACING_M).ravel()
        roll = rng.random(gx.size)
        radii = rng.uniform(*BUSH_RADIUS_RANGE_M, size=gx.size)
        seeds = rng.integers(0, 1 << 30, gx.size)

        placed = []
        for x, y, chance_roll, r, seed in zip(gx, gy, roll, radii, seeds):
            if math.hypot(x, y) < BUSH_CLEAR_RADIUS_M:
                continue
            here = self.classify(x, y)
            chance = {Terrain.SCRUB: BUSH_CHANCE_SCRUB, Terrain.GROUND: BUSH_CHANCE_GROUND}.get(here, 0.0)
            if chance_roll > chance:
                continue
            # Clear of rock, trees and holes all round, so it is a bush and
            # not a bush half inside something else.
            blocked = False
            span_cells = int(math.ceil(r / cell)) + 1
            ci, cj = self.to_cell(x, y)
            cells = []
            for dx in range(-span_cells, span_cells + 1):
                for dy in range(-span_cells, span_cells + 1):
                    if math.hypot(dx, dy) * cell > r + FOOTPRINT_GROWTH_M:
                        continue
                    kind = self.at((ci + dx, cj + dy))
                    if kind in (Terrain.ROCK, Terrain.TREE, Terrain.POTHOLE, Terrain.TRACK):
                        blocked = True
                    cells.append((ci + dx, cj + dy))
            if blocked:
                continue
            for c in cells:
                index = self._index(c)
                if index is not None:
                    self._grid[index] = int(Terrain.BUSH)
            placed.append((x, y, r, float(seed)))
        self.bushes = np.array(placed, dtype=np.float64).reshape(-1, 4)

    def _scatter_potholes(self, moisture: np.ndarray) -> np.ndarray:
        """Washouts, thicker along the track where water runs and wheels pass."""
        rng = np.random.default_rng(self.seed ^ 0x9014E)
        cell = self.cell_size_m
        on_track = self._grid == int(Terrain.TRACK)
        wet = np.clip(0.5 + moisture, 0.0, 2.0)
        density = np.where(on_track, POTHOLE_DENSITY_TRACK, POTHOLE_DENSITY_OPEN) * wet

        counts = rng.poisson(density * cell * cell)
        ids = np.repeat(np.arange(counts.size), counts.ravel())
        ix, iy = np.unravel_index(ids, counts.shape)
        xs = (ix + rng.uniform(0.0, 1.0, ix.size) - self._extent) * cell
        ys = (iy + rng.uniform(0.0, 1.0, iy.size) - self._extent) * cell
        radii = rng.uniform(*POTHOLE_RADIUS_RANGE_M, size=ix.size)
        depths = rng.uniform(*POTHOLE_DEPTH_RANGE_M, size=ix.size)

        depth_field = np.zeros((self._span, self._span), dtype=np.float32)
        for x, y, radius, depth in zip(xs, ys, radii, depths):
            if math.hypot(x, y) < POTHOLE_CLEAR_RADIUS_M:
                continue
            self._sink_dish(depth_field, x, y, radius, depth)
        return depth_field

    def cut_washout(self, x: float, y: float, radius_m: float, depth_m: float) -> None:
        """Cut one washout into the ground at a world position.

        Scenario setup: the generated terrain decides where water has cut
        the ground, and this puts one somewhere specific on top of that.
        The hole itself is no different from a generated one.
        """
        self._sink_dish(self._pothole_depth, x, y, radius_m, depth_m)
        before = self._pothole_depth >= POTHOLE_IMPASSABLE_DEPTH_M
        fresh = before & (self._grid != int(Terrain.ROCK)) & (self._grid != int(Terrain.TREE))
        newly = fresh & (self._grid != int(Terrain.POTHOLE))
        self._grid[fresh] = int(Terrain.POTHOLE)
        self._elevation[newly] -= self._pothole_depth[newly]

    def _sink_dish(
        self, depth: np.ndarray, x: float, y: float, radius: float, deepest: float
    ) -> None:
        """Sink one dish-shaped hollow into the depth field, deepest at its
        centre and feathering to nothing at the rim."""
        cell = self.cell_size_m
        reach = int(math.ceil(radius / cell)) + 1
        cx = int(math.floor(x / cell)) + self._extent
        cy = int(math.floor(y / cell)) + self._extent
        x0, x1 = max(cx - reach, 0), min(cx + reach + 1, self._span)
        y0, y1 = max(cy - reach, 0), min(cy + reach + 1, self._span)
        if x0 >= x1 or y0 >= y1:
            return

        gx = (np.arange(x0, x1) - self._extent + 0.5) * cell - x
        gy = (np.arange(y0, y1) - self._extent + 0.5) * cell - y
        dx, dy = np.meshgrid(gx, gy, indexing="ij")
        inside = 1.0 - (dx * dx + dy * dy) / (radius * radius)
        dish = deepest * np.clip(inside, 0.0, None)
        window = depth[x0:x1, y0:y1]
        np.maximum(window, dish.astype(np.float32), out=window)

    def _raise_dome(
        self,
        height: np.ndarray,
        x: float,
        y: float,
        semi_major: float,
        semi_minor: float,
        theta: float,
        top: float,
    ) -> None:
        """Raise one half-ellipsoid onto the height field, keeping the taller
        of it and whatever is already there so overlapping boulders merge."""
        cell = self.cell_size_m
        reach = int(math.ceil(semi_major / cell)) + 1
        cx = int(math.floor(x / cell)) + self._extent
        cy = int(math.floor(y / cell)) + self._extent
        x0, x1 = max(cx - reach, 0), min(cx + reach + 1, self._span)
        y0, y1 = max(cy - reach, 0), min(cy + reach + 1, self._span)
        if x0 >= x1 or y0 >= y1:
            return

        gx = (np.arange(x0, x1) - self._extent + 0.5) * cell - x
        gy = (np.arange(y0, y1) - self._extent + 0.5) * cell - y
        dx, dy = np.meshgrid(gx, gy, indexing="ij")
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        u = (dx * cos_t + dy * sin_t) / semi_major
        v = (-dx * sin_t + dy * cos_t) / semi_minor
        inside = 1.0 - (u * u + v * v)
        dome = top * np.sqrt(np.clip(inside, 0.0, None))
        window = height[x0:x1, y0:y1]
        np.maximum(window, dome.astype(np.float32), out=window)

    def _carve_track(self, slope: np.ndarray, moisture: np.ndarray) -> None:
        """A worn path follows the easiest line, so it is routed by cost.

        At each step the track takes whichever forward heading crosses the
        least difficult ground. That is why a real path bends around a bank
        instead of going straight over it.
        """
        x, y = 0.0, 0.0
        heading = 0.35
        half_width = max(1, int(1.1 / self.cell_size_m))

        for _ in range(1200):
            best, best_cost = heading, float("inf")
            for turn in (-0.22, -0.11, 0.0, 0.11, 0.22):
                candidate = heading + turn
                index = self._index(
                    self.to_cell(x + math.sin(candidate) * 3.0, y + math.cos(candidate) * 3.0)
                )
                if index is None:
                    continue
                cost = slope[index] * 10.0 + max(0.0, moisture[index]) * 2.0 + abs(turn) * 0.6
                if cost < best_cost:
                    best, best_cost = candidate, cost
            heading = best

            x += math.sin(heading) * self.cell_size_m
            y += math.cos(heading) * self.cell_size_m
            if abs(x) > self.half_extent_m - 2 or abs(y) > self.half_extent_m - 2:
                break

            centre = self.to_cell(x, y)
            for dx in range(-half_width, half_width + 1):
                for dy in range(-half_width, half_width + 1):
                    cell = (centre[0] + dx, centre[1] + dy)
                    # A path goes round a boulder or a stand of trees; it does not
                    # erase them, which would leave the cameras drawing a rock the
                    # map no longer has.
                    if self.at(cell) in (Terrain.ROCK, Terrain.TREE):
                        continue
                    self.set_cell(cell, Terrain.TRACK)

    def _clear_deploy_point(self) -> None:
        """A rover does not start inside a tree."""
        clear = int(6.0 / self.cell_size_m)
        for dx in range(-clear, clear + 1):
            for dy in range(-clear, clear + 1):
                if math.hypot(dx, dy) * self.cell_size_m <= 6.0 and self.at((dx, dy)) in IMPASSABLE:
                    self.set_cell((dx, dy), Terrain.GROUND)


@dataclass
class PerceptionMap:
    """What the rover has observed. Empty until something is scanned."""

    cell_size_m: float = CELL_SIZE_M
    _observed: dict[tuple[int, int], Terrain] = field(default_factory=dict)

    #: Append-only record of every classification change, in the order it
    #: happened. Several things consume the map as a stream rather than
    #: re-reading all of it: the renderer blits new points into its cached
    #: cloud, and the costmap folds new obstacles into its inflation layer.
    #: Each keeps its own cursor into this journal, so neither consumes the
    #: other's updates.
    _journal: list[tuple[tuple[int, int], Terrain]] = field(default_factory=list)
    _render_cursor: int = 0

    #: Bumped on clear(). A consumer holding a cursor from a previous
    #: mission has to notice that the map underneath it was replaced rather
    #: than silently carrying stale obstacles into the new one.
    epoch: int = 0

    def observe(self, cell: tuple[int, int], kind: Terrain) -> bool:
        known = self._observed.get(cell)
        if known == kind or not self._supersedes(known, kind):
            return False
        self._observed[cell] = kind
        self._journal.append((cell, kind))
        return True

    def observe_many(self, cx: np.ndarray, cy: np.ndarray, kinds: np.ndarray) -> int:
        """Observe a batch of cells at once; returns how many changed.

        A scan covers the same cell from several rays, so the batch is reduced
        to unique cells first. A cell has one true class, so which ray saw it
        does not matter.
        """
        if cx.size == 0:
            return 0
        # Two coordinates packed into one 64-bit key, because a one-dimensional
        # unique is several times faster than a row-wise one. The offset keeps
        # the shifted value below the sign bit; a billion cells either way is
        # more ground than any mission covers.
        offset = 1 << 30
        keys = ((cx.astype(np.int64) + offset) << 32) | (cy.astype(np.int64) + offset)
        unique, first = np.unique(keys, return_index=True)
        xs = (unique >> 32) - offset
        ys = (unique & 0xFFFFFFFF) - offset
        observed, journal = self._observed, self._journal
        changed = 0
        for x, y, value in zip(xs.tolist(), ys.tolist(), kinds[first].tolist()):
            kind = _TERRAIN_BY_VALUE[value]
            cell = (x, y)
            known = observed.get(cell)
            if known is kind or not self._supersedes(known, kind):
                continue
            observed[cell] = kind
            journal.append((cell, kind))
            changed += 1
        return changed

    @staticmethod
    def _supersedes(known: Terrain | None, fresh: Terrain) -> bool:
        """Whether a new reading should replace what is already mapped.

        Almost always yes: the ground is what the sensor last said it was.
        The exception is a hole already mapped. A scanning LiDAR can only
        resolve a negative obstacle close up; from further off it passes over
        the hole and returns off the ground beyond, reporting open ground
        perfectly honestly. That reading is not evidence the hole is gone, and
        letting it overwrite one means the rover forgets every hazard the
        moment it drives on -- and then drives back into it. A rock is the same:
        it is only resolved close up, and the rover turning away from one it
        mapped a moment ago must not unmap it on the strength of the ground
        it reads beyond.
        """
        return not (known in (Terrain.POTHOLE, Terrain.ROCK, Terrain.BUSH) and fresh is Terrain.GROUND)

    def at(self, cell: tuple[int, int]) -> Terrain:
        return self._observed.get(cell, Terrain.UNKNOWN)

    def is_known(self, cell: tuple[int, int]) -> bool:
        return cell in self._observed

    def cells(self) -> dict[tuple[int, int], Terrain]:
        return self._observed

    def coverage_m2(self) -> float:
        return len(self._observed) * self.cell_size_m**2

    def drain_new(self) -> list[tuple[tuple[int, int], Terrain]]:
        """Changes since this was last called. For the renderer's cursor."""
        new, self._render_cursor = self._journal[self._render_cursor :], len(self._journal)
        return new

    def changes_since(self, cursor: int) -> tuple[list[tuple[tuple[int, int], Terrain]], int]:
        """Changes after `cursor`, and the cursor to pass in next time."""
        return self._journal[cursor:], len(self._journal)

    def clear(self) -> None:
        self._observed.clear()
        self._journal.clear()
        self._render_cursor = 0
        self.epoch += 1

    def __len__(self) -> int:
        return len(self._observed)
