"""Camera views rendered from the simulated world.

A camera sees the world, not the map. The perception map holds what the
rover has *classified*; a camera sees what is *there*, including ground the
LiDAR has not reached and objects it has not yet resolved. So these views
are drawn from the same ground truth the sensors are simulated against, and
nothing is drawn that does not exist in it. A rig is built from a world and
reflects it as it was then; the terrain is fixed for a mission, so there is
nothing to keep in step, but a changed world needs a new rig.

Two views, because the vehicles look at the world differently:

  * The rover's forward camera: a pinhole view at wheel-mast height,
    rendered by marching a ray down every pixel column and projecting the
    ground and whatever stands on it. This is the standard heightfield
    method. It resolves occlusion for free, because a column that has
    already been filled by something nearer is not drawn over by something
    farther, and that is exactly why a tree hides the ground behind it.

  * The tricopters' nadir camera: a straight-down view whose footprint is
    set by altitude, so climbing widens what is seen and costs resolution,
    which is the real trade. Tall things cast shadows away from the sun,
    because from above a tree is a green shape and its shadow is the only
    thing that says it is tall.

Colour is natural, not the map's classification palette. The map is a
product; the camera is a window.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math

import numpy as np

from ocu.core.terrain import Terrain, TerrainWorld

# --- the scene --------------------------------------------------------------

#: How tall each class stands above the ground, in metres, at the middle of
#: a stand of it. Trees are the only thing here meaningfully taller than the
#: camera.
TREE_HEIGHT_M = 6.5
ROCK_HEIGHT_M = 1.1
SCRUB_HEIGHT_M = 0.6

#: How much of the surrounding ground has to be a class before it stands up
#: at all, and how much before it stands at full height. The classification
#: grid is half-metre cells; a tree is not a half-metre cell, it is a crown
#: that thins toward its edge, and this is what produces that taper.
STAND_ONSET = 0.30
STAND_FULL = 0.80

#: Blur radii, in cells, for turning the grid into smooth fields.
GROUND_BLUR_CELLS = 2
TREE_BLUR_CELLS = 3
ROCK_BLUR_CELLS = 1
SCRUB_BLUR_CELLS = 2

#: Height variation across a stand, as a fraction. Smooth, so neighbouring
#: trees are similar and distant ones differ, not random per cell.
HEIGHT_VARIATION = 0.32

#: Surface colours, 0-255 RGB. Dry scrubland: pale compacted track, tan
#: ground, olive brush, grey rock, dark canopy over a dark trunk.
GROUND_RGB = {
    Terrain.UNKNOWN: (139, 121, 92),
    Terrain.GROUND: (139, 121, 92),
    Terrain.TRACK: (170, 153, 124),
    Terrain.SCRUB: (101, 108, 62),
    Terrain.ROCK: (124, 122, 116),
    Terrain.TREE: (74, 84, 52),  # leaf litter beneath a canopy
}
CANOPY_RGB = (48, 78, 42)
TRUNK_RGB = (62, 47, 34)
ROCK_RGB = (128, 126, 120)
SCRUB_RGB = (96, 106, 58)

SKY_ZENITH_RGB = (96, 138, 186)
HAZE_RGB = (186, 200, 208)

#: Atmospheric extinction length. Distant ground fades into haze, which is
#: what gives a view its depth and hides the edge of the rendered range.
HAZE_LENGTH_M = 95.0

#: Fraction of a tree's height that is trunk, measured from the ground.
TRUNK_FRACTION = 0.28

# --- the light --------------------------------------------------------------

#: Sun azimuth (compass, clockwise from north) and elevation. A fixed late
#: morning sun: low enough that shadows read, high enough not to blind.
SUN_AZIMUTH_DEG = 150.0
SUN_ELEVATION_DEG = 42.0

#: Distances toward the sun at which a ground point is tested for an
#: occluder. A shadow is as long as the thing casting it divided by the tan
#: of the sun's elevation, so trees need several samples to be caught.
SHADOW_PROBES_M = (0.7, 1.5, 2.5, 3.6, 5.0, 6.5)
SHADOW_STRENGTH = 0.52

#: Gain applied to the finished frame. A camera meters for its scene; an
#: unmetered render of dark vegetation under a mid-height sun reads as dusk.
EXPOSURE = 1.14

#: World-anchored texture scale. Fine texture is hashed from world position
#: rather than screen position, or it would swim across the image as the
#: vehicle moves.
TEXTURE_CELL_M = 0.13

# --- the rover, as seen from above -------------------------------------------

ROVER_LENGTH_M = 1.15
ROVER_WIDTH_M = 0.85
ROVER_BODY_RGB = (46, 52, 60)
ROVER_NOSE_RGB = (120, 140, 156)


def _hash01(ix: np.ndarray, iy: np.ndarray, salt: int) -> np.ndarray:
    """Cheap deterministic per-cell noise in [0, 1).

    An integer hash, not a random generator, so the same world position
    always gives the same value. That is what keeps ground texture planted
    in the world instead of crawling across it.
    """
    h = (ix.astype(np.int64) * 73856093) ^ (iy.astype(np.int64) * 19349663) ^ (salt * 83492791)
    h = (h ^ (h >> 13)) * 1274126177
    h = h ^ (h >> 16)
    return (h & 0xFFFF).astype(np.float32) / 65536.0


def _value_noise(u: np.ndarray, v: np.ndarray, salt: int) -> np.ndarray:
    """Smooth noise in [0, 1): hashed lattice values, interpolated.

    The constant-per-cell hash above is right for grit, which should be
    pixel-fine. Anything larger than a pixel needs this instead: held
    constant across a cell it becomes a visible square as soon as the
    camera is close enough for a cell to cover several pixels, which on
    a slope face a metre from the lens is every cell.
    """
    iu, iv = np.floor(u), np.floor(v)
    fu, fv = u - iu, v - iv
    fu, fv = fu * fu * (3.0 - 2.0 * fu), fv * fv * (3.0 - 2.0 * fv)
    a = _hash01(iu, iv, salt)
    b = _hash01(iu + 1, iv, salt)
    c = _hash01(iu, iv + 1, salt)
    d = _hash01(iu + 1, iv + 1, salt)
    return (a * (1 - fu) + b * fu) * (1 - fv) + (c * (1 - fu) + d * fu) * fv


def _box_blur(values: np.ndarray, radius: int) -> np.ndarray:
    """Separable box blur with clamped edges, over the first two axes."""
    if radius <= 0:
        return values
    out = values.astype(np.float64)
    width = 2 * radius + 1
    for axis in (0, 1):
        pad = [(0, 0)] * out.ndim
        pad[axis] = (radius, radius)
        padded = np.pad(out, pad, mode="edge")
        moved = np.moveaxis(padded, axis, 0)
        cumulative = np.cumsum(moved, axis=0)
        cumulative = np.concatenate([np.zeros_like(cumulative[:1]), cumulative], axis=0)
        out = np.moveaxis((cumulative[width:] - cumulative[:-width]) / width, 0, axis)
    return out.astype(np.float32)


def _stand(density: np.ndarray) -> np.ndarray:
    """Turn local density of a class into how fully it stands up, 0 to 1."""
    return np.clip((density - STAND_ONSET) / (STAND_FULL - STAND_ONSET), 0.0, 1.0)


def _sun_vector() -> tuple[float, float, float]:
    azimuth = math.radians(SUN_AZIMUTH_DEG)
    elevation = math.radians(SUN_ELEVATION_DEG)
    return (
        math.sin(azimuth) * math.cos(elevation),
        math.cos(azimuth) * math.cos(elevation),
        math.sin(elevation),
    )


class CameraRig:
    """Renders camera views of one world.

    Everything that depends only on the terrain -- the class grid, the
    elevation, the hillshade, object heights -- is computed once here, so a
    frame costs only the projection.
    """

    def __init__(self, world: TerrainWorld) -> None:
        self._cell = world.cell_size_m
        self._extent = world._extent
        self._span = world._span
        grid = world._grid
        self._grid = grid
        self._elevation = world._elevation.astype(np.float32)

        # Ground colour as a smooth field. Painting a class colour per cell
        # makes every boundary a staircase on the half-metre grid, however
        # the image is sampled. Blurring the colours themselves keeps the
        # boundaries where the classification put them and rounds them off.
        flat = np.zeros(grid.shape + (3,), dtype=np.float32)
        for terrain, rgb in GROUND_RGB.items():
            flat[grid == int(terrain)] = rgb
        self._colour = _box_blur(flat, GROUND_BLUR_CELLS)

        # Hillshade from the elevation field. The grid is indexed [x, y], so
        # the gradient's first output is d/dx.
        gx, gy = np.gradient(self._elevation, self._cell)
        normal = np.stack([-gx, -gy, np.ones_like(gx)], axis=-1)
        normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
        lit = np.clip(normal @ np.array(_sun_vector(), dtype=np.float32), 0.0, 1.0)
        self._shade = np.clip(0.50 + 0.78 * lit, 0.55, 1.22).astype(np.float32)

        # Standing things as smooth height fields, one per kind. A stand of
        # trees is tall in the middle and tapers to nothing at its edge;
        # each cell having its own random height instead gives a row of
        # isolated towers.
        rng = np.random.default_rng(world.seed ^ 0x5EED)

        def variation(radius: int) -> np.ndarray:
            noise = _box_blur(rng.normal(size=grid.shape).astype(np.float32), radius)
            noise /= max(float(noise.std()), 1e-6)
            return np.clip(1.0 + HEIGHT_VARIATION * noise, 0.5, 1.5).astype(np.float32)

        tree = _box_blur((grid == int(Terrain.TREE)).astype(np.float32), TREE_BLUR_CELLS)
        rock = _box_blur((grid == int(Terrain.ROCK)).astype(np.float32), ROCK_BLUR_CELLS)
        scrub = _box_blur((grid == int(Terrain.SCRUB)).astype(np.float32), SCRUB_BLUR_CELLS)
        tree_h = TREE_HEIGHT_M * _stand(tree) * variation(3)
        rock_h = ROCK_HEIGHT_M * _stand(rock) * variation(1)
        scrub_h = SCRUB_HEIGHT_M * _stand(scrub) * variation(2)

        # Fused fields. A frame is dominated by the cost of calling into
        # numpy, not by arithmetic, so what is sampled together is stored
        # together and read with one bilinear lookup instead of four.
        #   relief  = elevation, tree, rock, scrub heights
        #   surface = lit ground colour (r, g, b), then the hillshade
        self._relief = np.stack([self._elevation, tree_h, rock_h, scrub_h], axis=-1)
        self._standing = np.maximum(np.maximum(tree_h, rock_h), scrub_h)
        self._surface = np.concatenate(
            [self._colour * self._shade[..., None], self._shade[..., None]], axis=-1
        )

    # --- sampling ---------------------------------------------------------

    def _bilerp(self, field: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Bilinear sample of a per-cell field at world points.

        Cell (i, j) holds the value at its centre, which is half a cell past
        its index. Sampling at the nearest cell instead turns every smooth
        slope into terraces half a metre wide.
        """
        fx = xs / self._cell + self._extent - 0.5
        fy = ys / self._cell + self._extent - 0.5
        x0 = np.floor(fx)
        y0 = np.floor(fy)
        tx = (fx - x0).astype(np.float32)
        ty = (fy - y0).astype(np.float32)
        top = self._span - 1
        x0 = np.clip(x0.astype(np.int64), 0, top)
        y0 = np.clip(y0.astype(np.int64), 0, top)
        x1 = np.minimum(x0 + 1, top)
        y1 = np.minimum(y0 + 1, top)
        if field.ndim == 3:
            tx, ty = tx[..., None], ty[..., None]
        return (
            field[x0, y0] * (1 - tx) * (1 - ty)
            + field[x1, y0] * tx * (1 - ty)
            + field[x0, y1] * (1 - tx) * ty
            + field[x1, y1] * tx * ty
        )

    def elevation_at(self, x: float, y: float) -> float:
        return float(self._bilerp(self._elevation, np.array([x]), np.array([y]))[0])

    def _ground_colour(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Lit, textured ground colour at world points."""
        return self._textured(self._bilerp(self._surface, xs, ys)[..., :3], xs, ys)

    @staticmethod
    def _textured(lit: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Multi-scale, world-anchored speckle: grit, patches and mottling.

        Three scales, because bare ground is not uniform at any one of them.
        Without the larger two a slope face is a flat colour with grain.
        """
        grit = _value_noise(xs / TEXTURE_CELL_M, ys / TEXTURE_CELL_M, 11)
        patch = _value_noise(xs / 0.55, ys / 0.55, 29)
        mottle = _value_noise(xs / 2.1, ys / 2.1, 53)
        texture = 0.80 + 0.16 * grit + 0.14 * patch + 0.12 * mottle
        return lit * texture[..., None]

    # --- the rover's forward camera -----------------------------------------

    def forward_view(
        self,
        x: float,
        y: float,
        heading_deg: float,
        width: int = 192,
        height: int = 76,
        fov_deg: float = 112.0,
        camera_height_m: float = 1.5,
        max_range_m: float = 65.0,
        sensor_noise: float = 0.0,
    ) -> np.ndarray:
        """What the rover's forward camera sees. (height, width, 3) uint8."""
        focal = (width / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        horizon = height * 0.43

        columns = np.arange(width, dtype=np.float32)
        theta = np.arctan((columns + 0.5 - width / 2.0) / focal)
        cos_theta = np.cos(theta)
        bearing = math.radians(heading_deg) + theta  # clockwise-positive, so right of centre
        dir_x, dir_y = np.sin(bearing), np.cos(bearing)

        camera_z = self.elevation_at(x, y) + camera_height_m
        rows = np.arange(height, dtype=np.float32)
        haze_rgb = np.array(HAZE_RGB, dtype=np.float32)

        # Sample depths: about a cell apart near the camera, spread
        # geometrically further out, since a half-metre step at sixty metres
        # is far finer than a pixel is wide.
        depths = []
        depth = 0.8
        while depth < max_range_m:
            depth += max(self._cell * 0.8, depth * 0.018)
            depths.append(depth)
        depth = np.array(depths, dtype=np.float32)[:, None]  # (S, 1)
        step = np.diff(depth[:, 0], prepend=depth[0, 0] - self._cell)[:, None]

        # Every sample along every column's ray, in one lookup.
        sx = x + dir_x[None, :] * depth  # (S, W)
        sy = y + dir_y[None, :] * depth
        sample = self._bilerp(self._relief, sx, sy)  # (S, W, 4)
        ground_z = sample[..., 0]
        heights = sample[..., 1:]  # tree, rock, scrub
        standing = heights.max(axis=-1)
        zd = depth * cos_theta[None, :]  # perpendicular depth is what projects

        y_top = horizon - focal * (ground_z + standing - camera_z) / zd
        y_base = horizon - focal * (ground_z - camera_z) / zd

        # Occlusion. A pixel belongs to the nearest sample whose top has
        # reached it, because anything farther is behind that. The running
        # minimum of the tops is how far up each column the nearer samples
        # have already covered, so the first sample whose running minimum
        # reaches a row is the one that row shows.
        first = np.clip(np.ceil(y_top), 0, height)
        covered_to = np.minimum.accumulate(first, axis=0)  # (S, W)
        reached = covered_to[:, None, :] <= rows[None, :, None]  # (S, H, W)
        painted = reached.any(axis=0)  # (H, W)
        owner = reached.argmax(axis=0)  # (H, W), which sample owns each pixel

        r, c = np.nonzero(painted)
        k = owner[r, c]
        rf = rows[r] + 0.5
        d = depth[k, 0]
        on_object = rf < y_base[k, c]
        colour = np.empty((len(r), 3), dtype=np.float32)

        # Ground. Its colour is looked up at the world point each pixel row
        # actually looks at, not at the one sample the strip was marched
        # from, so near ground has texture instead of one flat colour per step.
        ground = ~on_object
        gk, gc = k[ground], c[ground]
        below = np.maximum(rf[ground] - horizon, 0.5)
        row_depth = np.clip(
            focal * (camera_z - ground_z[gk, gc]) / below, d[ground] - step[gk, 0], d[ground] + step[gk, 0]
        )
        row_dist = row_depth / cos_theta[gc]
        colour[ground] = self._ground_colour(x + dir_x[gc] * row_dist, y + dir_y[gc] * row_dist)

        # Standing things.
        ok, oc = k[on_object], c[on_object]
        if len(ok):
            span = np.maximum(y_base[ok, oc] - y_top[ok, oc], 1e-3)
            t = np.clip((rf[on_object] - y_top[ok, oc]) / span, 0.0, 1.0)  # 0 top .. 1 foot
            kind = np.argmax(heights[ok, oc], axis=-1)  # 0 tree, 1 rock, 2 scrub
            lit = self._bilerp(self._surface[..., 3], sx[ok, oc], sy[ok, oc])
            colour[on_object] = self._object_colour(kind, t, lit, sx[ok, oc], sy[ok, oc])

        haze = (1.0 - np.exp(-d / HAZE_LENGTH_M))[:, None]
        image = np.zeros((height, width, 3), dtype=np.float32)
        image[r, c] = colour * (1.0 - haze) + haze_rgb * haze

        # Whatever nothing reached is sky.
        sky_t = np.clip(rows / max(horizon, 1.0), 0.0, 1.0)[:, None, None]  # 0 zenith .. 1 horizon
        sky = (
            np.array(SKY_ZENITH_RGB, dtype=np.float32) * (1.0 - sky_t**1.6)
            + haze_rgb * sky_t**1.6
        )
        image = np.where(painted[..., None], image, sky)
        return self._finish(image, sensor_noise)

    @staticmethod
    def _object_colour(kind, t, lit, sx, sy) -> np.ndarray:
        """Colour of a standing thing at fraction `t` down from its top."""
        out = np.zeros((len(kind), 3), dtype=np.float32)

        tree = kind == 0
        if tree.any():
            tt = t[tree]
            # Smooth in world position and in height, so a crown shows soft
            # clumps of foliage rather than stripes, one per sample step.
            leaf = _value_noise(
                (sx[tree] + 0.6 * sy[tree]) / 0.9, tt * 3.2 + 0.4 * sy[tree], 5
            )
            canopy = np.array(CANOPY_RGB, dtype=np.float32) * (0.72 + 0.62 * leaf)[:, None]
            canopy = canopy * (1.16 - 0.5 * tt)[:, None]  # darker toward the underside
            trunk = np.array(TRUNK_RGB, dtype=np.float32) * (0.85 + 0.3 * leaf)[:, None]
            is_trunk = tt > (1.0 - TRUNK_FRACTION)
            out[tree] = np.where(is_trunk[:, None], trunk, canopy) * lit[tree][:, None]

        rock = kind == 1
        if rock.any():
            tr = t[rock]
            facet = _value_noise((sx[rock] + 0.8 * sy[rock]) / 0.45, tr * 2.4 + sy[rock] / 0.6, 7)
            shade = (1.18 - 0.5 * tr) * (0.78 + 0.44 * facet)  # lit top, dark foot
            out[rock] = np.array(ROCK_RGB, dtype=np.float32) * shade[:, None] * lit[rock][:, None]

        scrub = kind == 2
        if scrub.any():
            ts = t[scrub]
            tuft = _value_noise(sx[scrub] / 0.3, sy[scrub] / 0.3, 3)
            shade = (1.08 - 0.35 * ts) * (0.72 + 0.56 * tuft)
            out[scrub] = np.array(SCRUB_RGB, dtype=np.float32) * shade[:, None] * lit[scrub][:, None]
        return out

    # --- the tricopters' nadir camera ---------------------------------------

    def nadir_view(
        self,
        x: float,
        y: float,
        altitude_m: float,
        heading_deg: float,
        width: int = 192,
        height: int = 76,
        fov_deg: float = 84.0,
        rover: tuple[float, float, float] | None = None,
        sensor_noise: float = 0.0,
    ) -> np.ndarray:
        """What a straight-down camera sees. (height, width, 3) uint8.

        Footprint follows altitude: the half-width on the ground is
        altitude times tan(half fov), so climbing widens the view and
        coarsens it. `rover` is (x, y, heading_deg) of the ground vehicle,
        drawn if it falls inside the frame, since a vehicle is part of the
        scene an aircraft looks down on.
        """
        altitude = max(altitude_m, 1.5)
        half_w = altitude * math.tan(math.radians(fov_deg / 2.0))
        half_h = half_w * height / width

        u = (np.arange(width, dtype=np.float32) + 0.5 - width / 2.0) / (width / 2.0)
        v = (height / 2.0 - np.arange(height, dtype=np.float32) - 0.5) / (height / 2.0)
        right_m = (u * half_w)[None, :]
        fwd_m = (v * half_h)[:, None]

        heading = math.radians(heading_deg)
        fx, fy = math.sin(heading), math.cos(heading)
        rx, ry = math.cos(heading), -math.sin(heading)
        px = x + fwd_m * fx + right_m * rx
        py = y + fwd_m * fy + right_m * ry

        surface = self._bilerp(self._surface, px, py)
        colour = self._textured(surface[..., :3], px, py)
        light = surface[..., 3:4]
        heights = self._bilerp(self._relief, px, py)[..., 1:]  # tree, rock, scrub

        # Crowns, rocks and brush. Each has a height field that tapers to
        # nothing at its edge; thresholding that against fine world-anchored
        # noise gives an edge that follows the field but is ragged the way a
        # real crown is, instead of following the half-metre grid.
        fine = _hash01(np.floor(px / TEXTURE_CELL_M), np.floor(py / TEXTURE_CELL_M), 41)
        tree = heights[..., 0] > 1.2 + 2.2 * fine
        rock = heights[..., 1] > 0.18 + 0.45 * fine
        scrub = heights[..., 2] > 0.10 + 0.30 * fine

        crown = _value_noise(px / 0.7, py / 0.7, 17)
        tone = (0.66 + 0.74 * crown) * (0.9 + 0.2 * fine)
        colour = np.where(
            tree[..., None],
            np.array(CANOPY_RGB, dtype=np.float32) * (tone * light[..., 0] * 1.18)[..., None],
            colour,
        )
        facet = _value_noise(px / 0.4, py / 0.4, 23)
        colour = np.where(
            (rock & ~tree)[..., None],
            np.array(ROCK_RGB, dtype=np.float32) * ((0.82 + 0.4 * facet) * light[..., 0])[..., None],
            colour,
        )
        tuft = _value_noise(px / 0.3, py / 0.3, 31)
        colour = np.where(
            (scrub & ~tree & ~rock)[..., None],
            np.array(SCRUB_RGB, dtype=np.float32) * ((0.8 + 0.4 * tuft) * light[..., 0])[..., None],
            colour,
        )

        # Shadows. A point is shaded if something toward the sun is tall
        # enough to reach it: height > distance * tan(elevation). Soft at the
        # edge, because the height fields taper rather than step.
        sun_x, sun_y = math.sin(math.radians(SUN_AZIMUTH_DEG)), math.cos(math.radians(SUN_AZIMUTH_DEG))
        tan_sun = math.tan(math.radians(SUN_ELEVATION_DEG))
        shade_amount = np.zeros(px.shape, dtype=np.float32)
        for probe in SHADOW_PROBES_M:
            reach = self._bilerp(self._standing, px + sun_x * probe, py + sun_y * probe)
            shade_amount = np.maximum(shade_amount, np.clip((reach - probe * tan_sun) / 1.2 + 0.5, 0.0, 1.0))
        shade_amount *= ~(tree | rock)  # a crown is lit on top; only the ground beside it is shaded
        colour = colour * (1.0 - (1.0 - SHADOW_STRENGTH) * shade_amount)[..., None]

        if rover is not None:
            colour = self._paint_rover(colour, px, py, rover)
        return self._finish(colour, sensor_noise)

    @staticmethod
    def _paint_rover(colour: np.ndarray, px: np.ndarray, py: np.ndarray, rover) -> np.ndarray:
        """Draw the ground vehicle where the frame covers it."""
        rover_x, rover_y, rover_heading = rover
        h = math.radians(rover_heading)
        fx, fy = math.sin(h), math.cos(h)
        rx, ry = math.cos(h), -math.sin(h)
        dx, dy = px - rover_x, py - rover_y
        along = dx * fx + dy * fy
        across = dx * rx + dy * ry

        # Its shadow first, thrown away from the sun and underneath it.
        sun_x, sun_y = math.sin(math.radians(SUN_AZIMUTH_DEG)), math.cos(math.radians(SUN_AZIMUTH_DEG))
        sdx, sdy = dx + sun_x * 0.45, dy + sun_y * 0.45
        s_along = sdx * fx + sdy * fy
        s_across = sdx * rx + sdy * ry
        shadow = (np.abs(s_along) < ROVER_LENGTH_M / 2) & (np.abs(s_across) < ROVER_WIDTH_M / 2)
        colour = np.where(shadow[..., None], colour * 0.55, colour)

        body = (np.abs(along) < ROVER_LENGTH_M / 2) & (np.abs(across) < ROVER_WIDTH_M / 2)
        nose = body & (along > ROVER_LENGTH_M / 2 - 0.22)
        colour = np.where(body[..., None], np.array(ROVER_BODY_RGB, dtype=np.float32), colour)
        return np.where(nose[..., None], np.array(ROVER_NOSE_RGB, dtype=np.float32), colour)

    # --- output -----------------------------------------------------------

    @staticmethod
    def _finish(image: np.ndarray, sensor_noise: float) -> np.ndarray:
        """Sensor noise and quantisation. A camera is never clean."""
        if sensor_noise > 0.0:
            image = image + np.random.normal(0.0, sensor_noise, size=image.shape).astype(np.float32)
        return np.clip(image * EXPOSURE, 0.0, 255.0).astype(np.uint8)
