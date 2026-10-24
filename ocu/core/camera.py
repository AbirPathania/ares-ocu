"""Camera views rendered from the simulated world.

A camera sees the world, not the map. The perception map holds what the
rover has *classified*; a camera sees what is *there*, including ground the
LiDAR has not reached and objects it has not yet resolved. So these views
are drawn from the same ground truth the sensors are simulated against, and
nothing is drawn that does not exist in it. A rig is built from a world and
reflects it as it was then; the terrain is fixed for a mission, so there is
nothing to keep in step, but a changed world needs a new rig.

Three views, because the vehicles look at the world differently:

  * The rover's forward camera: a pinhole view at mast height, rendered by
    marching a ray down every azimuth and projecting the ground and whatever
    stands on it. This is the standard heightfield method.

  * The tricopters' scout camera: the same renderer from the air, pitched
    down. A nano-class scout carries a gimballed forward-looking camera, not
    a straight-down one, because what it is for is seeing ahead of the
    formation: ground, the far edge of a clearing, the thing behind the
    rock. Pitch and field of view are parameters; the defaults look along
    the direction the formation faces, tipped down enough to see the ground
    it is crossing.

  * A straight-down nadir view, for a camera that really does look down.
    Its footprint is set by altitude, so climbing widens what is seen and
    costs resolution. Tall things cast shadows away from the sun, because
    from above a tree is a green shape and its shadow is what says it is tall.

Both oblique views are resolved in angle space. Every pixel has a true ray,
an azimuth and an elevation, and every sample along a ground azimuth has the
elevation at which it appears. A pixel belongs to the nearest sample whose
top has reached its elevation, which is occlusion, and the same rule is exact
for a pitched camera where treating image columns as constant-azimuth rays
(the usual shortcut) is only right with the camera level.

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
SCRUB_BLUR_CELLS = 2

#: Height variation across a stand, as a fraction. Smooth, so neighbouring
#: trees are similar and distant ones differ, not random per cell.
HEIGHT_VARIATION = 0.32

#: The scouts' camera: field of view, how far it is tipped down, and the
#: shape of the frame it is rendered into (height over width). Chosen for this
#: build, not published figures of any airframe. Detection uses the same numbers
#: the picture is drawn with, so what a scout reports is what is in its view.
SCOUT_HFOV_DEG = 84.0
SCOUT_PITCH_DOWN_DEG = 18.0
SCOUT_NOMINAL_ASPECT = 0.38

#: Surface colours, 0-255 RGB. Dry scrubland: pale compacted track, tan
#: ground, olive brush, grey rock, dark canopy over a dark trunk.
GROUND_RGB = {
    Terrain.UNKNOWN: (139, 121, 92),
    Terrain.GROUND: (139, 121, 92),
    Terrain.TRACK: (170, 153, 124),
    Terrain.SCRUB: (101, 108, 62),
    Terrain.ROCK: (124, 122, 116),
    Terrain.TREE: (74, 84, 52),  # leaf litter beneath a canopy
    Terrain.POTHOLE: (74, 62, 50),  # bare earth in shadow, down inside it
    Terrain.BUSH: (101, 108, 62),
}
CANOPY_RGB = (48, 78, 42)
TRUNK_RGB = (62, 47, 34)
ROCK_RGB = (128, 126, 120)
SCRUB_RGB = (96, 106, 58)

SKY_ZENITH_RGB = (96, 138, 186)
HAZE_RGB = (186, 200, 208)

#: How finely the scene is sampled, and what that costs.
#:
#: AZIMUTH_OVERSAMPLE: ray bearings per pixel column. Above one, neighbouring
#: pixels looking along slightly different bearings get their own ray.
#: NEAR_STEP_CELLS / FAR_STEP_FRACTION: ground sampled about a cell apart
#: close in, then spreading geometrically, because a half-metre step at sixty
#: metres is far finer than a pixel is wide. These set the cost of a frame
#: almost entirely: the scene is sampled rays x steps times.
AZIMUTH_OVERSAMPLE = 0.85
NEAR_STEP_CELLS = 1.0
FAR_STEP_FRACTION = 0.030

#: Atmospheric extinction length. Distant ground fades into haze, which is
#: what gives a view its depth and hides the edge of the rendered range.
HAZE_LENGTH_M = 170.0

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
ROVER_HEIGHT_M = 0.8
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
        self._world = world
        #: The GPU scene, once `enable_gpu` has made one. While it is None the
        #: views are drawn by the numpy renderer below.
        self.gpu = None
        #: (kind, x, y, heading) of the people, vehicles and animals to draw; set by whoever owns the mission clock
        self.entities: list = []
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
        scrub = _box_blur((grid == int(Terrain.SCRUB)).astype(np.float32), SCRUB_BLUR_CELLS)
        tree_h = TREE_HEIGHT_M * _stand(tree) * variation(3)
        scrub_h = SCRUB_HEIGHT_M * _stand(scrub) * variation(2)
        # Rock is not a blurred class. The world holds each boulder as a
        # dome with a real height, so the camera draws that, round and of the
        # size it actually is, instead of inferring a shape from a mask.
        rock_h = world._rock_height.astype(np.float32)

        # Fused fields. A frame is dominated by the cost of calling into
        # numpy, not by arithmetic, so what is sampled together is stored
        # together and read with one bilinear lookup instead of four.
        #   relief  = elevation, tree, rock, scrub heights
        #   surface = lit ground colour (r, g, b), then the hillshade
        self._ray_cache: dict = {}
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

    def line_of_sight(
        self, x0: float, y0: float, z0: float, x1: float, y1: float, z1: float
    ) -> bool:
        """Whether the straight line between two points clears the ground and
        everything standing on it. The same heights the picture is drawn from."""
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(int(length / self._cell), 2)
        t = np.linspace(0.0, 1.0, steps + 1)[1:-1]
        xs, ys = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        sample = self._bilerp(self._relief, xs, ys)
        surface = sample[:, 0] + sample[:, 1:].max(axis=1)
        return bool(np.all(z0 + (z1 - z0) * t > surface))

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

    def enable_gpu(self) -> bool:
        """Draw the views on the GPU, as 3D geometry, if a GPU can be had."""
        from ocu.render.gpu_scene import GpuScene

        self.gpu = GpuScene.create(self._world)
        return self.gpu is not None

    # --- oblique cameras: the rover's forward camera, and the scouts' ---------

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
        pitch_down_deg: float = 3.0,
        sensor_noise: float = 0.0,
    ) -> np.ndarray:
        """What the rover's forward camera sees. (height, width, 3) uint8."""
        if self.gpu is not None:
            eye = (x, y, self.elevation_at(x, y) + camera_height_m)
            return self.gpu.render(eye, heading_deg, pitch_down_deg, fov_deg, width, height, grain=sensor_noise / 255.0, entities=self.entities or None)
        return self._render(
            x, y, self.elevation_at(x, y) + camera_height_m, heading_deg, width, height,
            fov_deg, pitch_down_deg, max_range_m, None, sensor_noise,
        )

    def scout_view(
        self,
        x: float,
        y: float,
        altitude_agl_m: float,
        heading_deg: float,
        width: int = 192,
        height: int = 76,
        fov_deg: float = SCOUT_HFOV_DEG,
        pitch_down_deg: float = SCOUT_PITCH_DOWN_DEG,
        max_range_m: float = 140.0,
        rover: tuple[float, float, float] | None = None,
        sensor_noise: float = 0.0,
    ) -> np.ndarray:
        """What a scout's forward-looking, gimballed camera sees.

        The camera faces `heading_deg`, tipped down by `pitch_down_deg`, at
        `altitude_agl_m` above the ground beneath it (scouts hold height over
        terrain rather than over a datum). `rover` is (x, y, heading_deg) of
        the ground vehicle, drawn if it is in view: a vehicle is part of the
        scene an aircraft looks at.
        """
        if self.gpu is not None:
            eye = (x, y, self.elevation_at(x, y) + altitude_agl_m)
            return self.gpu.render(
                eye, heading_deg, pitch_down_deg, fov_deg, width, height, rover=rover, grain=sensor_noise / 255.0, entities=self.entities or None
            )
        return self._render(
            x, y, self.elevation_at(x, y) + altitude_agl_m, heading_deg, width, height,
            fov_deg, pitch_down_deg, max_range_m, rover, sensor_noise,
        )

    def _pixel_rays(self, width: int, height: int, fov_deg: float, pitch_down_deg: float):
        """Azimuth offset and elevation of every pixel's ray, and its bin.

        Camera axes are (right, up, forward). Tipping the camera down by the
        pitch turns the forward axis toward the ground, so a pixel's ray in
        the level frame is the camera ray rotated about the lateral axis.
        Depends only on the lens and the pitch, so it is computed once.
        """
        key = (width, height, round(fov_deg, 3), round(pitch_down_deg, 3))
        cached = self._ray_cache.get(key)
        if cached is not None:
            return cached

        focal = (width / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        pitch = math.radians(pitch_down_deg)
        cols = np.arange(width, dtype=np.float64) + 0.5 - width / 2.0
        rows = height / 2.0 - (np.arange(height, dtype=np.float64) + 0.5)
        right, up = np.meshgrid(cols, rows)  # (H, W)

        horizontal = focal * math.cos(pitch) + up * math.sin(pitch)  # along the heading
        vertical = up * math.cos(pitch) - focal * math.sin(pitch)
        azimuth = np.arctan2(right, horizontal)
        elevation = np.arctan2(vertical, np.hypot(right, horizontal))

        # Azimuth is sampled finer than the pixel grid, so adjacent pixels
        # that look along slightly different azimuths do not share a ray.
        bins = max(int(width * AZIMUTH_OVERSAMPLE), 8)
        lo, hi = float(azimuth.min()), float(azimuth.max())
        edges = np.linspace(lo, hi, bins)
        spacing = (hi - lo) / max(bins - 1, 1)
        bin_of = np.clip(np.rint((azimuth - lo) / max(spacing, 1e-9)).astype(np.int64), 0, bins - 1)
        cached = (azimuth.astype(np.float32), elevation.astype(np.float32), edges.astype(np.float32), bin_of)
        self._ray_cache[key] = cached
        return cached

    def resolve_view(
        self,
        x: float,
        y: float,
        camera_z: float,
        heading_deg: float,
        width: int,
        height: int,
        fov_deg: float,
        pitch_down_deg: float,
        max_range_m: float = 140.0,
    ) -> dict:
        """What every pixel's ray meets first, by the rule `_render` uses for occlusion, without painting anything.

        Returns flat arrays of length width x height (row-major): `painted` (the ray met ground or something standing
        on it), `depth` (horizontal distance to what it met, inf where nothing), `kind` (-1 sky, 0 ground, 1 tree,
        2 rock, 3 scrub), `hx`, `hy`, `hz` (where it met it), and each pixel's `azimuth` (from the heading) and `elevation`.
        A camera that is not the visible one, such as the thermal imager, reads the same scene through this.
        """
        azimuth, elevation, bin_azimuth, bin_of = self._pixel_rays(width, height, fov_deg, pitch_down_deg)
        heading = math.radians(heading_deg)
        depths = []
        distance = 0.6
        while distance < max_range_m:
            distance += max(self._cell * NEAR_STEP_CELLS, distance * FAR_STEP_FRACTION)
            depths.append(distance)
        depth = np.array(depths, dtype=np.float32)[:, None]
        step = np.diff(depth[:, 0], prepend=depth[0, 0] - self._cell)[:, None]
        ray_bearing = heading + bin_azimuth
        sx = x + np.sin(ray_bearing)[None, :] * depth
        sy = y + np.cos(ray_bearing)[None, :] * depth
        sample = self._bilerp(self._relief, sx, sy)
        ground_z = sample[..., 0]
        heights = sample[..., 1:]
        standing = heights.max(axis=-1)
        angle_top = np.arctan2(ground_z + standing - camera_z, depth)
        angle_base = np.arctan2(ground_z - camera_z, depth)
        reach = np.maximum.accumulate(angle_top, axis=0)
        flat_bins = bin_of.ravel()
        flat_elev = elevation.ravel()
        samples, rays = reach.shape
        separation = 4.0
        offsets = np.arange(rays, dtype=np.float64) * separation
        ordered = (reach.astype(np.float64) + offsets[None, :]).T.ravel()
        found = np.searchsorted(ordered, flat_elev.astype(np.float64) + flat_bins * separation)
        owner = found - flat_bins * samples
        painted = owner < samples
        owner = np.clip(owner, 0, samples - 1)

        n = width * height
        out_depth = np.full(n, np.inf, dtype=np.float32)
        kind = np.full(n, -1, dtype=np.int8)
        hx, hy, hz = np.zeros(n, np.float32), np.zeros(n, np.float32), np.zeros(n, np.float32)
        pix = np.nonzero(painted)[0]
        k, j, e = owner[pix], flat_bins[pix], flat_elev[pix]
        d = depth[k, 0]
        on_object = e > angle_base[k, j]
        out_depth[pix] = d
        pix_az = azimuth.ravel()[pix]
        bearing = heading + pix_az
        ground = ~on_object
        if ground.any():
            gk, gj, ge = k[ground], j[ground], e[ground]
            tan_e = np.tan(ge)
            tan_e = np.where(np.abs(tan_e) < 1e-4, -1e-4, tan_e)
            ray_d = np.clip((ground_z[gk, gj] - camera_z) / tan_e, d[ground] - step[gk, 0], d[ground] + step[gk, 0])
            gp = pix[ground]
            kind[gp] = 0
            hx[gp] = x + np.sin(bearing[ground]) * ray_d
            hy[gp] = y + np.cos(bearing[ground]) * ray_d
            hz[gp] = ground_z[gk, gj]
        if on_object.any():
            ok, oj, oe = k[on_object], j[on_object], e[on_object]
            op = pix[on_object]
            kind[op] = 1 + np.argmax(heights[ok, oj], axis=-1)  # 1 tree, 2 rock, 3 scrub
            hx[op], hy[op] = sx[ok, oj], sy[ok, oj]
            hz[op] = camera_z + d[on_object] * np.tan(oe)
        return {"painted": painted, "depth": out_depth, "kind": kind, "hx": hx, "hy": hy, "hz": hz,
                "azimuth": azimuth.ravel(), "elevation": flat_elev, "heading": heading}

    def _render(
        self,
        x: float,
        y: float,
        camera_z: float,
        heading_deg: float,
        width: int,
        height: int,
        fov_deg: float,
        pitch_down_deg: float,
        max_range_m: float,
        rover: tuple[float, float, float] | None,
        sensor_noise: float,
    ) -> np.ndarray:
        azimuth, elevation, bin_azimuth, bin_of = self._pixel_rays(width, height, fov_deg, pitch_down_deg)
        haze_rgb = np.array(HAZE_RGB, dtype=np.float32)
        heading = math.radians(heading_deg)

        # Ground distances to sample: about a cell apart near the camera and
        # spread geometrically beyond, since a half-metre step at sixty metres
        # is far finer than a pixel is wide.
        depths = []
        distance = 0.6
        while distance < max_range_m:
            distance += max(self._cell * NEAR_STEP_CELLS, distance * FAR_STEP_FRACTION)
            depths.append(distance)
        depth = np.array(depths, dtype=np.float32)[:, None]  # (S, 1)
        step = np.diff(depth[:, 0], prepend=depth[0, 0] - self._cell)[:, None]

        ray_bearing = heading + bin_azimuth  # (B,)
        sx = x + np.sin(ray_bearing)[None, :] * depth  # (S, B)
        sy = y + np.cos(ray_bearing)[None, :] * depth
        sample = self._bilerp(self._relief, sx, sy)  # (S, B, 4)
        ground_z = sample[..., 0]
        heights = sample[..., 1:]  # tree, rock, scrub
        standing = heights.max(axis=-1)

        # Elevation at which the top and the foot of each sample appear.
        angle_top = np.arctan2(ground_z + standing - camera_z, depth)
        angle_base = np.arctan2(ground_z - camera_z, depth)

        # Occlusion. A pixel belongs to the nearest sample whose top has
        # reached its elevation, because anything farther is behind it. The
        # running maximum of the tops is how high nearer samples already
        # reach along this azimuth.
        reach = np.maximum.accumulate(angle_top, axis=0)  # (S, B)
        flat_bins = bin_of.ravel()
        flat_elev = elevation.ravel()

        # That running maximum only ever rises, so the owning sample can be
        # found by binary search rather than by testing every sample against
        # every pixel. Each ray's samples are offset into one ascending array
        # -- the offset is far wider than the range of an elevation, so the
        # rays cannot interleave -- and the whole frame resolves in one
        # searchsorted instead of a comparison per pixel per sample.
        samples, rays = reach.shape
        # Elevations are angles, so a separation of four radians is already
        # wider than any two can span. Done in double precision: the offset
        # for a ray far across the frame is thousands of times larger than
        # the angles being compared, and in single precision that gap
        # swallows them.
        separation = 4.0
        offsets = np.arange(rays, dtype=np.float64) * separation
        ordered = (reach.astype(np.float64) + offsets[None, :]).T.ravel()
        found = np.searchsorted(
            ordered, flat_elev.astype(np.float64) + flat_bins * separation
        )
        owner = found - flat_bins * samples
        painted = owner < samples
        owner = np.clip(owner, 0, samples - 1)

        pix = np.nonzero(painted)[0]
        k = owner[pix]
        j = flat_bins[pix]
        e = flat_elev[pix]
        pix_az = azimuth.ravel()[pix]
        d = depth[k, 0]
        on_object = e > angle_base[k, j]
        colour = np.empty((len(pix), 3), dtype=np.float32)

        # Ground. Where this pixel's own ray meets the ground plane at that
        # sample's elevation, clipped to the neighbourhood of the sample so
        # slopes cannot send it somewhere unrelated. Texture is looked up
        # there, so near ground is textured rather than one colour per step.
        ground = ~on_object
        if ground.any():
            gk, gj, ge = k[ground], j[ground], e[ground]
            tan_e = np.tan(ge)
            tan_e = np.where(np.abs(tan_e) < 1e-4, -1e-4, tan_e)
            ray_d = (ground_z[gk, gj] - camera_z) / tan_e
            ray_d = np.clip(ray_d, d[ground] - step[gk, 0], d[ground] + step[gk, 0])
            bearing = heading + pix_az[ground]
            colour[ground] = self._ground_colour(x + np.sin(bearing) * ray_d, y + np.cos(bearing) * ray_d)

        if on_object.any():
            ok, oj, oe = k[on_object], j[on_object], e[on_object]
            span = np.maximum(angle_top[ok, oj] - angle_base[ok, oj], 1e-6)
            t = np.clip((angle_top[ok, oj] - oe) / span, 0.0, 1.0)  # 0 top .. 1 foot
            kind = np.argmax(heights[ok, oj], axis=-1)  # 0 tree, 1 rock, 2 scrub
            lit = self._bilerp(self._surface[..., 3], sx[ok, oj], sy[ok, oj])
            colour[on_object] = self._object_colour(kind, t, lit, sx[ok, oj], sy[ok, oj])

        fog = (1.0 - np.exp(-d / HAZE_LENGTH_M))[:, None]
        image = np.zeros((height * width, 3), dtype=np.float32)
        image[pix] = colour * (1.0 - fog) + haze_rgb * fog

        # Whatever nothing reached is sky, or haze below the horizon where
        # the range ran out before the ground did.
        sky_t = np.clip(flat_elev / math.radians(42.0), 0.0, 1.0)[:, None] ** 0.7
        sky = np.array(SKY_ZENITH_RGB, dtype=np.float32) * sky_t + haze_rgb * (1.0 - sky_t)
        image = np.where(painted[:, None], image, sky)

        if rover is not None:
            nearest = np.full(height * width, np.inf, dtype=np.float32)
            nearest[pix] = d
            image = self._paint_rover_oblique(
                image, x, y, camera_z, heading, azimuth.ravel(), flat_elev, nearest, rover
            )
        return self._finish(image.reshape(height, width, 3), sensor_noise)

    def _paint_rover_oblique(
        self, image, x, y, camera_z, heading, azimuth, elevation, nearest, rover
    ) -> np.ndarray:
        """Draw the ground vehicle in an oblique view, if it is in front of
        what each pixel would otherwise show."""
        rx, ry, rheading = rover
        dx, dy = rx - x, ry - y
        distance = math.hypot(dx, dy)
        if distance < 1.0:
            return image
        bearing = math.atan2(dx, dy)
        offset = (bearing - heading + math.pi) % math.tau - math.pi
        ground = self.elevation_at(rx, ry)

        # Seen from the side or end, the vehicle presents a different width.
        yaw = math.radians(rheading) - bearing
        presented = abs(math.sin(yaw)) * ROVER_LENGTH_M + abs(math.cos(yaw)) * ROVER_WIDTH_M
        half_width = math.atan2(presented / 2.0, distance)
        top = math.atan2(ground + ROVER_HEIGHT_M - camera_z, distance)
        base = math.atan2(ground - camera_z, distance)

        in_view = (
            (np.abs(azimuth - offset) <= half_width)
            & (elevation <= top)
            & (elevation >= base)
            & (distance < nearest)
        )
        if not in_view.any():
            return image
        # Dark body, lighter roof line, so it reads as a vehicle not a smudge.
        roof = in_view & (elevation > top - (top - base) * 0.28)
        image = np.where(in_view[:, None], np.array(ROVER_BODY_RGB, dtype=np.float32), image)
        return np.where(roof[:, None], np.array(ROVER_NOSE_RGB, dtype=np.float32), image)

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
        if self.gpu is not None:
            eye = (x, y, self.elevation_at(x, y) + max(altitude_m, 1.5))
            return self.gpu.render(eye, heading_deg, 90.0, fov_deg, width, height, rover=rover, grain=sensor_noise / 255.0, entities=self.entities or None)
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
