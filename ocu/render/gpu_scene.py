"""The GPU scene: the world drawn as geometry, from any camera.

Built once from a `TerrainWorld`. Terrain is a heightfield mesh shaded from the
class grid; rocks, bushes and trees are instanced meshes at the positions the
world generated them; shadows come from one shadow map of the whole world,
rendered at the start, since the sun does not move and nothing that casts a
shadow does either. Each camera view is then a single pass with 4x multisampling
read back as an image.

If a GL context cannot be made (no GPU, no display session) `create` returns
None and the caller keeps the numpy renderer.
"""

from __future__ import annotations

import math
import time

import numpy as np

from ocu.core.terrain import Terrain, TerrainWorld
from ocu.render import assets, figures, meshes, palette, shaders, splats, trail

SHADOW_SIZE = 4096
MSAA_SAMPLES = 4
#: How high up a figure the dust it has driven or walked through reaches, for the shader. Chosen for how each looks.
DUST_HEIGHT_M = {"vehicle": 1.0, "person": 0.5, "animal": 0.4}
NEAR_M = 0.12
FAR_M = 5000.0
FOG_PER_M = 1.0 / 520.0
EXPOSURE = palette.EXPOSURE

#: Things further than this are not drawn: past it they are fog.
DRAW_DISTANCE = {"rock": 150.0, "pebble": 34.0, "bush": 120.0, "acacia": 190.0, "leafy": 190.0, "snag": 190.0, "grass": 22.0}

ROCK_VARIANTS = 5
FOLIAGE_VARIANTS = 3
PEBBLE_COUNT = 9000
HALF_FOV_MARGIN_DEG = 14.0


# --- matrices -----------------------------------------------------------------


def perspective(fovy_deg: float, aspect: float, near: float, far: float) -> np.ndarray:
    f = 1.0 / math.tan(math.radians(fovy_deg) / 2.0)
    m = np.zeros((4, 4), dtype=np.float64)
    m[0, 0] = f / aspect
    m[1, 1] = f
    m[2, 2] = (far + near) / (near - far)
    m[2, 3] = 2.0 * far * near / (near - far)
    m[3, 2] = -1.0
    return m


def look_at(eye, target, up) -> np.ndarray:
    eye, target, up = (np.asarray(v, dtype=np.float64) for v in (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.eye(4)
    m[0, :3], m[1, :3], m[2, :3] = s, u, -f
    m[:3, 3] = -m[:3, :3] @ eye
    return m


def orthographic(left, right, bottom, top, near, far) -> np.ndarray:
    m = np.eye(4)
    m[0, 0], m[1, 1], m[2, 2] = 2.0 / (right - left), 2.0 / (top - bottom), -2.0 / (far - near)
    m[0, 3], m[1, 3], m[2, 3] = -(right + left) / (right - left), -(top + bottom) / (top - bottom), -(far + near) / (far - near)
    return m


def _bytes(matrix: np.ndarray) -> bytes:
    return np.ascontiguousarray(matrix.T, dtype=np.float32).tobytes()


class GpuScene:
    @classmethod
    def create(cls, world: TerrainWorld, use_assets: bool = True) -> "GpuScene | None":
        try:
            return cls(world, use_assets)
        except Exception:  # no GPU or no way to make a context: fall back
            return None

    def __init__(self, world: TerrainWorld, use_assets: bool = True) -> None:
        import moderngl

        self.mgl = moderngl
        self.ctx = moderngl.create_standalone_context()
        self._world = world
        self.trail = None  # a reconstructed trail, once load_trail has found one
        self._t0 = time.monotonic()
        self._cell = world.cell_size_m
        self._extent = world._extent
        self._span = world._span
        self._elevation = world._elevation.astype(np.float64)

        self._use_assets = bool(use_assets and assets.available())
        self._build_programs()
        self._build_ground_textures()
        self._build_terrain()
        self._build_templates()
        self._build_pbr_models()
        self._build_instances()
        self._build_shadow()
        self._targets: dict[tuple[int, int], tuple] = {}

    # --- building ---------------------------------------------------------

    def _program(self, vertex: str, fragment: str):
        if not vertex.lstrip().startswith("#version"):
            vertex = "#version 330\n" + vertex
        return self.ctx.program(vertex_shader=vertex, fragment_shader=fragment)

    def _build_programs(self) -> None:
        self.p_terrain = self._program(shaders.TERRAIN_VERT, shaders.TERRAIN_FRAG)
        self.p_inst = self._program(shaders.INSTANCE_VERT, shaders.INSTANCE_FRAG)
        self.p_pbr = self._program(shaders.PBR_VERT, shaders.PBR_FRAG)
        self.p_sky = self._program(shaders.SKY_VERT, shaders.SKY_FRAG)
        self.p_shadow_terrain = self._program(shaders.SHADOW_TERRAIN_VERT, shaders.SHADOW_FRAG)
        self.p_shadow_inst = self._program(shaders.SHADOW_INSTANCE_VERT, shaders.SHADOW_FRAG)

    def _texture(self, image: np.ndarray):
        tex = self.ctx.texture((image.shape[1], image.shape[0]), 3, np.ascontiguousarray(image).tobytes())
        tex.build_mipmaps()
        tex.filter = (self.mgl.LINEAR_MIPMAP_LINEAR, self.mgl.LINEAR)
        tex.anisotropy = 8.0
        return tex

    def _build_ground_textures(self) -> None:
        self.ground_albedo = self.ground_normal = None
        if not self._use_assets:
            return
        ground = assets.load_ground_textures()
        if ground is None:
            return
        layers = ("ground", "scrub", "track", "dark", "slope")
        ctx = self.ctx
        for attribute, field in (("ground_albedo", "albedo"), ("ground_normal", "normal")):
            stack = np.stack([getattr(ground[name], field) for name in layers])  # (5, h, w, 3)
            size = stack.shape[1]
            array = ctx.texture_array((size, size, len(layers)), 3, np.ascontiguousarray(stack).tobytes())
            array.build_mipmaps()
            array.filter = (self.mgl.LINEAR_MIPMAP_LINEAR, self.mgl.LINEAR)
            array.anisotropy = 8.0
            setattr(self, attribute, array)

    def _build_pbr_models(self) -> None:
        """Upload the downloaded models: each variant as a near and a far mesh."""
        self._pbr_models: dict[str, dict] = {}
        if not self._use_assets:
            return
        for name in assets.PLAN:
            model = assets.load_model(name)
            if model is None or model.textures is None:
                continue
            kind = assets.PLAN[name][0]
            textures = (
                self._texture(model.textures.albedo),
                self._texture(model.textures.normal),
                self._texture(model.textures.arm),
            )
            variants = []
            for variant in model.variants:
                lods = []
                for mesh in variant.lods:
                    vertex = np.concatenate([mesh.positions, mesh.normals, mesh.uvs], axis=1).astype(np.float32)
                    lods.append((self.ctx.buffer(vertex.tobytes()), self.ctx.buffer(mesh.indices.astype(np.uint32).tobytes())))
                variants.append({"variant": variant, "lods": lods})
            self._pbr_models[name] = {"kind": kind, "textures": textures, "variants": variants}

    def _build_terrain(self) -> None:
        from ocu.core.camera import _box_blur

        world = self._world
        span, cell, extent = self._span, self._cell, self._extent
        grid = world._grid
        z = self._elevation

        gx, gy = np.gradient(z, cell)
        normal = np.stack([-gx, -gy, np.ones_like(gx)], axis=-1)
        normal /= np.linalg.norm(normal, axis=-1, keepdims=True)

        ground = np.isin(grid, [int(Terrain.GROUND), int(Terrain.UNKNOWN)]).astype(np.float32)
        scrub = np.isin(grid, [int(Terrain.SCRUB), int(Terrain.BUSH)]).astype(np.float32)
        track = (grid == int(Terrain.TRACK)).astype(np.float32)
        dark = np.isin(grid, [int(Terrain.TREE), int(Terrain.ROCK), int(Terrain.POTHOLE)]).astype(np.float32)
        weights = np.stack([ground, scrub, track, dark], axis=-1)
        weights = _box_blur(weights, 2)
        weights /= np.maximum(weights.sum(axis=-1, keepdims=True), 1e-4)
        hole = np.clip(world._pothole_depth / 0.75, 0.0, 1.0).astype(np.float32)
        extra = np.stack([hole, np.zeros_like(hole), np.zeros_like(hole), np.zeros_like(hole)], axis=-1)

        # Extend past the edge of the world so the horizon is country and not the end of a table: ground that rises from the
        # edge into rolling hills, forested as the footage's are, fading into the haze.
        axis = (np.arange(span) - extent + 0.5) * cell
        half = axis[-1]
        outer = half + np.array([0.5, 6.0, 12.0, 20.0, 30.0, 45.0, 65.0, 90.0, 120.0, 155.0, 195.0, 240.0, 290.0, 345.0, 405.0, 470.0, 540.0, 615.0, 700.0,
                                800.0, 920.0, 1050.0, 1200.0, 1400.0, 1650.0, 2000.0, 2500.0, 3300.0, 5000.0])
        pad = len(outer)
        xs = np.concatenate([-outer[::-1], axis, outer])

        def padded(a):
            widths = [(pad, pad), (pad, pad)] + [(0, 0)] * (a.ndim - 2)
            return np.pad(a, widths, mode="edge")

        def padded_with(a, value):
            out = padded(a)
            out[:pad], out[-pad:], out[:, :pad], out[:, -pad:] = value, value, value, value
            return out

        X, Y = np.meshgrid(xs, xs, indexing="ij")
        beyond = np.maximum(np.abs(X), np.abs(Y)) - half  # metres past the edge of the world, zero or less inside it
        rise = np.clip(beyond / 170.0, 0.0, 1.0)
        rise = rise * rise * (3.0 - 2.0 * rise) * (1.0 - np.clip((beyond - 2200.0) / 2800.0, 0.0, 1.0))
        big = meshes.fbm3(np.stack([X / 330.0, Y / 330.0, np.zeros_like(X)], axis=-1).reshape(-1, 3), 77, 4).reshape(X.shape)
        small = meshes.fbm3(np.stack([X / 105.0, Y / 105.0, np.zeros_like(X)], axis=-1).reshape(-1, 3), 78, 3).reshape(X.shape)
        hills = rise * (95.0 * (big - 0.36) + 16.0 * (small - 0.5))

        # What is past the edge is not the edge of the world smeared out: its materials are constants, and a flag in the
        # extra channel says "forest" (the shader paints canopy there).
        zp = padded(z) + hills
        np_ = padded_with(normal, (0.0, 0.0, 1.0))
        far_gx, far_gy = np.gradient(zp, xs, xs)
        far_normal = np.stack([-far_gx, -far_gy, np.ones_like(far_gx)], axis=-1)
        far_normal /= np.linalg.norm(far_normal, axis=-1, keepdims=True)
        outside = (beyond > 0.0)[..., None]
        np_ = np.where(outside, far_normal, np_)
        wp = padded_with(weights, (0.75, 0.25, 0.0, 0.0))
        ep = padded_with(extra, (0.0, 0.0, 0.0, 0.0))
        forest = np.clip(beyond / 45.0, 0.0, 1.0)
        ep[..., 1] = forest * forest * (3.0 - 2.0 * forest)
        positions = np.stack([X, Y, zp], axis=-1).reshape(-1, 3)
        vertex = np.concatenate(
            [positions, np_.reshape(-1, 3), wp.reshape(-1, 4), ep.reshape(-1, 4)], axis=1
        ).astype(np.float32)
        n = len(xs)
        idx = np.arange(n * n, dtype=np.uint32).reshape(n, n)
        a, b, c, d = idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]
        indices = np.concatenate([np.stack([a, b, c], -1).reshape(-1, 3), np.stack([a, c, d], -1).reshape(-1, 3)]).reshape(-1)

        ctx = self.ctx
        self.terrain_vbo = ctx.buffer(vertex.tobytes())
        # The picture is drawn from every second vertex: a half-metre triangle is
        # smaller than a pixel a few metres out, and the detail that matters up
        # close (grain, bumps, hollows) is in the shader and the normals, which
        # are still the full-resolution ones.
        coarse = idx[::2, ::2]
        ca, cb, cc, cd = coarse[:-1, :-1], coarse[1:, :-1], coarse[1:, 1:], coarse[:-1, 1:]
        coarse_indices = np.concatenate([np.stack([ca, cb, cc], -1).reshape(-1, 3), np.stack([ca, cc, cd], -1).reshape(-1, 3)]).reshape(-1)
        self.terrain_ibo = ctx.buffer(indices.astype(np.uint32).tobytes())
        self.terrain_coarse_ibo = ctx.buffer(coarse_indices.astype(np.uint32).tobytes())
        self.terrain_vao = ctx.vertex_array(
            self.p_terrain,
            [(self.terrain_vbo, "3f 3f 4f 4f", "in_pos", "in_normal", "in_mat", "in_extra")],
            self.terrain_ibo,
        )
        self.terrain_coarse_vao = ctx.vertex_array(
            self.p_terrain,
            [(self.terrain_vbo, "3f 3f 4f 4f", "in_pos", "in_normal", "in_mat", "in_extra")],
            self.terrain_coarse_ibo,
        )
        # The shadow pass only needs the near world, not the extension.
        inner = idx[pad - 1 : n - pad + 1, pad - 1 : n - pad + 1]
        a, b, c, d = inner[:-1, :-1], inner[1:, :-1], inner[1:, 1:], inner[:-1, 1:]
        inner_indices = np.concatenate([np.stack([a, b, c], -1).reshape(-1, 3), np.stack([a, c, d], -1).reshape(-1, 3)]).reshape(-1)
        self.terrain_shadow_ibo = ctx.buffer(inner_indices.astype(np.uint32).tobytes())
        self.terrain_shadow_vao = ctx.vertex_array(
            self.p_shadow_terrain, [(self.terrain_vbo, "3f 11x4", "in_pos")], self.terrain_shadow_ibo
        )

    def _template_vao(self, vertices: np.ndarray, indices: np.ndarray, instance_buffer):
        ctx = self.ctx
        vbo = ctx.buffer(vertices.tobytes())
        ibo = ctx.buffer(indices.astype(np.uint32).tobytes())
        layout = [
            (vbo, "3f 3f 3f 1f", "in_pos", "in_normal", "in_rgb", "in_ao"),
            (instance_buffer, "4f 4f 4f/i", "i_pos_yaw", "i_scale", "i_tint"),
        ]
        vao = ctx.vertex_array(self.p_inst, layout, ibo)
        shadow = ctx.vertex_array(
            self.p_shadow_inst,
            [(vbo, "3f 7x4", "in_pos"), (instance_buffer, "4f 4f 4x4/i", "i_pos_yaw", "i_scale")],
            ibo,
        )
        return vao, shadow

    def load_trail(self, name: str | None = None) -> bool:
        """Load a reconstructed trail (a splat and its route) from assets/footage/trail, if there is one. True if loaded.

        Must run on the thread that owns the GL context."""
        folder = trail.TRAIL_DIR
        candidates = [folder / f"{name}.ply"] if name else sorted(folder.glob("*.ply"))
        for ply in candidates:
            route_file = ply.with_suffix(".npz")
            if not (ply.exists() and route_file.exists()):
                continue
            try:
                route, k = trail.TrailRoute.load(route_file)
                renderer = splats.SplatRenderer(self.ctx, splats.load_ply(ply))
            except (OSError, ValueError, KeyError):
                continue
            self.trail = {"name": ply.stem, "route": route, "intrinsics": k, "renderer": renderer}
            return True
        return False

    def trail_length_m(self) -> float:
        return self.trail["route"].length_m if self.trail else 0.0

    def _trail_view(self, s_m: float, lateral_m: float, yaw_deg: float, height_m: float | None, width: int, height: int):
        """The splat camera for a point on the route, and the heading and pitch to draw the sky with.

        The sky is drawn by the same code as the rest of the world, which has z up, so the camera's direction is turned from the
        reconstruction's frame into a z-up one first. Roll is ignored: a phone held by a walker is nearly level."""
        t = self.trail
        route, k = t["route"], t["intrinsics"]
        pose = route.pose_at(s_m, lateral_m, yaw_deg, height_m)
        up = route.up / np.linalg.norm(route.up)
        axis = np.cross(up, [0.0, 0.0, 1.0])
        sin, cos = float(np.linalg.norm(axis)), float(up @ [0.0, 0.0, 1.0])
        if sin < 1e-8:
            turn = np.eye(3) if cos > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            a = axis / sin
            ax = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            turn = np.eye(3) + sin * ax + (1 - cos) * (ax @ ax)  # Rodrigues: takes `up` to +z
        f = turn @ pose.rotation[2]
        heading = math.degrees(math.atan2(f[0], f[1])) % 360.0
        pitch_down = math.degrees(-math.asin(float(np.clip(f[2], -1.0, 1.0))))
        fx = k.fx * width / k.width
        return pose, splats.Intrinsics(width, height, fx, fx, width / 2.0, height / 2.0), heading, pitch_down

    def _build_templates(self) -> None:
        self._template_meshes = {
            "rock": [meshes.rock_template(100 + k) for k in range(ROCK_VARIANTS)],
            "pebble": [meshes.rock_template(300 + k, 2) for k in range(3)],
            "bush": [meshes.bush_template(500 + k) for k in range(FOLIAGE_VARIANTS)],
            "acacia": [meshes.acacia_template(700 + k) for k in range(FOLIAGE_VARIANTS)],
            "leafy": [meshes.leafy_template(900 + k) for k in range(FOLIAGE_VARIANTS)],
            "snag": [meshes.snag_template(1300 + k) for k in range(FOLIAGE_VARIANTS)],
            "grass": [meshes.grass_template(1100 + k) for k in range(4)],
        }
        self._rover_mesh = meshes.rover_mesh()

    def _ground(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        fx = xs / self._cell + self._extent - 0.5
        fy = ys / self._cell + self._extent - 0.5
        x0, y0 = np.floor(fx), np.floor(fy)
        tx, ty = fx - x0, fy - y0
        top = self._span - 1
        x0 = np.clip(x0.astype(int), 0, top)
        y0 = np.clip(y0.astype(int), 0, top)
        x1, y1 = np.minimum(x0 + 1, top), np.minimum(y0 + 1, top)
        z = self._elevation
        return z[x0, y0] * (1 - tx) * (1 - ty) + z[x1, y0] * tx * (1 - ty) + z[x0, y1] * (1 - tx) * ty + z[x1, y1] * tx * ty

    def _build_instances(self) -> None:
        world = self._world
        rng = np.random.default_rng(world.seed ^ 0x5CE9E)
        groups: dict[tuple[str, int], np.ndarray] = {}

        def add(kind: str, variant: int, rows: np.ndarray) -> None:
            if len(rows):
                groups[(kind, variant)] = np.asarray(rows, dtype=np.float32)

        b = world.boulders
        if len(b):
            gz = self._ground(b[:, 0], b[:, 1])
            tint = rng.uniform(0.86, 1.10, (len(b), 1))
            rows = np.concatenate(
                [
                    np.stack([b[:, 0], b[:, 1], gz - 0.05 * b[:, 5], b[:, 4]], axis=1),
                    np.stack([b[:, 2], b[:, 3], b[:, 5], rng.uniform(0, 100, len(b))], axis=1),
                    np.concatenate([tint, tint * rng.uniform(0.97, 1.03, (len(b), 1)), tint * rng.uniform(0.93, 1.02, (len(b), 1)), np.zeros((len(b), 1))], axis=1),
                ],
                axis=1,
            )
            variant = rng.integers(0, ROCK_VARIANTS, len(b))
            for v in range(ROCK_VARIANTS):
                add("rock", v, rows[variant == v])

        # Stones and loose pebbles: a few from the world, many scattered.
        stones = []
        p = world.pebbles
        if len(p):
            stones.append(np.stack([p[:, 0], p[:, 1], p[:, 2], p[:, 3]], axis=1))
        reach = world.half_extent_m - 3.0
        sx = rng.uniform(-reach, reach, PEBBLE_COUNT)
        sy = rng.uniform(-reach, reach, PEBBLE_COUNT)
        ok = np.array([world.classify(x, y) in (Terrain.GROUND, Terrain.SCRUB, Terrain.TRACK) for x, y in zip(sx, sy)])
        sx, sy = sx[ok], sy[ok]
        sr = np.exp(rng.normal(math.log(0.07), 0.45, len(sx)))
        stones.append(np.stack([sx, sy, sr, sr * rng.uniform(0.5, 0.8, len(sx))], axis=1))
        stones = np.concatenate(stones)
        gz = self._ground(stones[:, 0], stones[:, 1])
        tint = rng.uniform(0.7, 1.15, (len(stones), 1))
        rows = np.concatenate(
            [
                np.stack([stones[:, 0], stones[:, 1], gz - 0.2 * stones[:, 3], rng.uniform(0, math.tau, len(stones))], axis=1),
                np.stack([stones[:, 2], stones[:, 2] * rng.uniform(0.7, 1.0, len(stones)), stones[:, 3], rng.uniform(0, 100, len(stones))], axis=1),
                np.concatenate([tint * 1.02, tint, tint * 0.94, np.zeros((len(stones), 1))], axis=1),
            ],
            axis=1,
        )
        variant = rng.integers(0, 3, len(rows))
        for v in range(3):
            add("pebble", v, rows[variant == v])

        k = world.bushes
        if len(k):
            gz = self._ground(k[:, 0], k[:, 1])
            tone = rng.uniform(0.80, 1.20, (len(k), 1))
            rows = np.concatenate(
                [
                    np.stack([k[:, 0], k[:, 1], gz - 0.04, rng.uniform(0, math.tau, len(k))], axis=1),
                    np.stack([k[:, 2], k[:, 2], k[:, 2] * rng.uniform(0.85, 1.1, len(k)), rng.uniform(0, 100, len(k))], axis=1),
                    np.concatenate([tone, tone, tone * rng.uniform(0.85, 1.0, (len(k), 1)), np.full((len(k), 1), 0.6)], axis=1),
                ],
                axis=1,
            )
            variant = rng.integers(0, FOLIAGE_VARIANTS, len(k))
            for v in range(FOLIAGE_VARIANTS):
                add("bush", v, rows[variant == v])

        t = world.trees
        if len(t):
            gz = self._ground(t[:, 0], t[:, 1])
            for species, kind, crown_in_template in ((0, "acacia", 0.40), (1, "leafy", 0.34), (2, "snag", 0.34)):
                sel = t[:, 4] == species
                if not sel.any():
                    continue
                tt, gg = t[sel], gz[sel]
                n = len(tt)
                tone = rng.uniform(0.85, 1.15, (n, 1))
                warm = rng.uniform(0.94, 1.08, (n, 1))
                rows = np.concatenate(
                    [
                        np.stack([tt[:, 0], tt[:, 1], gg - 0.05, rng.uniform(0, math.tau, n)], axis=1),
                        np.stack([tt[:, 3] / crown_in_template, tt[:, 3] / crown_in_template, tt[:, 2], rng.uniform(0, 100, n)], axis=1),
                        np.concatenate([tone * warm, tone, tone / warm, np.full((n, 1), 1.0)], axis=1),
                    ],
                    axis=1,
                )
                variant = rng.integers(0, FOLIAGE_VARIANTS, n)
                for v in range(FOLIAGE_VARIANTS):
                    add(kind, v, rows[variant == v])

        # Grass: a tuft per cell where the ground carries it, thicker in scrub.
        grid = world._grid
        for terrain, per_cell in ((Terrain.SCRUB, 4.2), (Terrain.GROUND, 2.0), (Terrain.BUSH, 2.4)):
            ix, iy = np.nonzero(grid == int(terrain))
            count = rng.poisson(per_cell, len(ix))
            ix, iy = np.repeat(ix, count), np.repeat(iy, count)
            if not len(ix):
                continue
            gx = (ix - self._extent + rng.uniform(0, 1, len(ix))) * self._cell
            gy = (iy - self._extent + rng.uniform(0, 1, len(ix))) * self._cell
            # Grass grows in patches and in clumps, not evenly: thin it by a slow noise, then give each survivor a few
            # close neighbours.
            patch = meshes.fbm3(np.stack([gx * 0.30, gy * 0.30, np.zeros_like(gx)], axis=1), 4242, 3)
            keep = rng.uniform(0.0, 1.0, len(gx)) < np.clip((patch - 0.28) / 0.34, 0.06, 1.0)
            gx, gy = gx[keep], gy[keep]
            rep = rng.integers(1, 4, len(gx))
            gx = np.repeat(gx, rep) + rng.normal(0.0, 0.20, int(rep.sum()))
            gy = np.repeat(gy, rep) + rng.normal(0.0, 0.20, int(rep.sum()))
            ground = self._ground(gx, gy)
            n = len(gx)
            # the template is the colour of dry grass; an instance only varies from it, a little greener or more bleached
            straw = rng.uniform(0.0, 1.0, n) < 0.85
            tone = rng.uniform(0.70, 1.20, (n, 1))
            colour = np.where(straw[:, None], np.array([1.08, 0.97, 0.76]), np.array([0.82, 1.02, 0.70])) * tone
            height = np.exp(rng.normal(np.log(0.62 if terrain == Terrain.SCRUB else 0.40), 0.32, n))
            rows = np.concatenate(
                [
                    np.stack([gx, gy, ground - 0.02, rng.uniform(0, math.tau, n)], axis=1),
                    np.stack([height * 0.62, height * 0.62, height, rng.uniform(0, 100, n)], axis=1),
                    np.concatenate([colour, np.full((n, 1), 0.8)], axis=1),
                ],
                axis=1,
            )
            variant = rng.integers(0, 4, n)
            for v in range(4):
                key = ("grass", v)
                part = rows[variant == v]
                groups[key] = np.concatenate([groups[key], part.astype(np.float32)]) if key in groups else part.astype(np.float32)

        self._pbr_groups: list[dict] = []
        for kind in ("rock", "pebble", "grass", "bush"):
            if any(m["kind"] == {"pebble": "stone", "bush": "shrub"}.get(kind, kind) for m in self._pbr_models.values()):
                for key in [k for k in groups if k[0] == kind]:
                    del groups[key]
        if self._pbr_models:
            self._build_pbr_groups(rng)

        ctx = self.ctx
        self._groups: dict[tuple[str, int], dict] = {}
        for (kind, variant), rows in groups.items():
            # Sorted along x, so a view can take the slice of the world it
            # could possibly see without examining all of it.
            rows = rows[np.argsort(rows[:, 0])]
            vertices, indices = self._template_meshes[kind][variant]
            buffer = ctx.buffer(reserve=max(rows.nbytes, 48))
            buffer.write(rows.tobytes())
            vao, shadow = self._template_vao(vertices, indices, buffer)
            self._groups[(kind, variant)] = {
                "rows": rows, "buffer": buffer, "vao": vao, "shadow": shadow,
                "kind": kind, "shader_kind": {"rock": 0, "pebble": 0, "grass": 3}.get(kind, 1),
                "positions": rows[:, :2], "count": len(rows),
            }
        # One rover, rewritten each time it is drawn.
        self._rover_buffer = ctx.buffer(reserve=48)
        vertices, indices = self._rover_mesh
        self._rover_vao, _ = self._template_vao(vertices, indices, self._rover_buffer)
        # People, vehicles and animals: every pose of each, drawn from one small instance buffer per kind that is
        # rewritten each time one is drawn.
        self._entity_buffers = {}
        self._entity_frames = {}
        for kind, frames in figures.build_all().items():
            buffer = ctx.buffer(reserve=48)
            self._entity_buffers[kind] = buffer
            self._entity_frames[kind] = [self._template_vao(v, i, buffer) for v, i in frames]

    def _pbr_variants(self, *kinds: str, models: tuple[str, ...] | None = None) -> list[tuple[str, dict]]:
        out = []
        for name, model in self._pbr_models.items():
            if model["kind"] in kinds and (models is None or name in models):
                for index, entry in enumerate(model["variants"]):
                    out.append((name, entry))
        return out

    def _add_pbr_group(self, name: str, entry: dict, rows: np.ndarray, mode: int, lod_distance: float, draw: float,
                       shadow: bool = False) -> None:
        if not len(rows):
            return
        ctx = self.ctx
        rows = rows[np.argsort(rows[:, 0])].astype(np.float32)
        lods = []
        for vbo_ibo in entry["lods"]:
            vbo, ibo = vbo_ibo
            buffer = ctx.buffer(reserve=max(rows.nbytes, 48))
            vao = ctx.vertex_array(
                self.p_pbr,
                [(vbo, "3f 3f 2f", "in_pos", "in_normal", "in_uv"), (buffer, "4f 4f 4f/i", "i_pos_yaw", "i_scale", "i_tint")],
                ibo,
            )
            lods.append({"vao": vao, "buffer": buffer})
        group = {
            "name": name, "rows": rows, "mode": mode, "lods": lods,
            "lod_distance": lod_distance, "draw": draw,
            "textures": self._pbr_models[name]["textures"],
        }
        if shadow:
            # The shadow map is drawn once, from the far mesh and every instance.
            vbo, ibo = entry["lods"][-1]
            buffer = ctx.buffer(rows.tobytes())
            group["shadow"] = ctx.vertex_array(
                self.p_shadow_inst, [(vbo, "3f 5x4", "in_pos"), (buffer, "4f 4f 4x4/i", "i_pos_yaw", "i_scale")], ibo
            )
            group["shadow_count"] = len(rows)
        self._pbr_groups.append(group)

    def _build_pbr_groups(self, rng: np.random.Generator) -> None:
        world = self._world

        # Boulders: each takes the downloaded rock whose proportions fit its
        # footprint best, turned so its long side lies along the boulder's.
        rocks = self._pbr_variants("rock", models=tuple(n for n in self._pbr_models if "boulder" in n))
        b = world.boulders
        if rocks and len(b):
            aspect = np.array([e["variant"].half_extent[1] / e["variant"].half_extent[0] for _, e in rocks])
            want = b[:, 3] / b[:, 2]
            choice = np.argmin(np.abs(np.log(want[:, None]) - np.log(aspect[None, :])) + rng.uniform(0, 0.25, (len(b), len(rocks))), axis=1)
            gz = self._ground(b[:, 0], b[:, 1])
            for index, (name, entry) in enumerate(rocks):
                sel = choice == index
                if not sel.any():
                    continue
                v = entry["variant"]
                hx, hy = v.half_extent
                bb = b[sel]
                along_y = hy > hx
                yaw = bb[:, 4] + (math.pi / 2 if along_y else 0.0) + rng.normal(0, 0.05, len(bb))
                a_axis, b_axis = (bb[:, 3], bb[:, 2]) if along_y else (bb[:, 2], bb[:, 3])
                sx, sy = 0.94 * a_axis / hx, 0.94 * b_axis / hy
                sz = bb[:, 5] / max(v.height, 1e-3)
                tint = rng.uniform(0.86, 1.12, (len(bb), 1))
                rows = np.concatenate(
                    [
                        np.stack([bb[:, 0], bb[:, 1], gz[sel] - 0.02, yaw], axis=1),
                        np.stack([sx, sy, sz, rng.uniform(0, 100, len(bb))], axis=1),
                        np.concatenate([tint, tint, tint, np.zeros((len(bb), 1))], axis=1),
                    ],
                    axis=1,
                )
                self._add_pbr_group(name, entry, rows, 0, 24.0, DRAW_DISTANCE["rock"], shadow=True)

        # Stones and small rocks scattered over the ground.
        stones = self._pbr_variants("stone") + self._pbr_variants("rock", models=("namaqualand_rocks_01",))
        if stones:
            reach = world.half_extent_m - 3.0
            sx_ = rng.uniform(-reach, reach, PEBBLE_COUNT)
            sy_ = rng.uniform(-reach, reach, PEBBLE_COUNT)
            ok = np.array([world.classify(x, y) in (Terrain.GROUND, Terrain.SCRUB, Terrain.TRACK) for x, y in zip(sx_, sy_)])
            sx_, sy_ = sx_[ok], sy_[ok]
            radius = np.exp(rng.normal(math.log(0.09), 0.5, len(sx_)))
            gz = self._ground(sx_, sy_)
            pick = rng.integers(0, len(stones), len(sx_))
            for index, (name, entry) in enumerate(stones):
                sel = pick == index
                if not sel.any():
                    continue
                v = entry["variant"]
                scale = radius[sel] / max(max(v.half_extent), 1e-3)
                tint = rng.uniform(0.8, 1.15, (sel.sum(), 1))
                rows = np.concatenate(
                    [
                        np.stack([sx_[sel], sy_[sel], gz[sel] - 0.01, rng.uniform(0, math.tau, sel.sum())], axis=1),
                        np.stack([scale, scale, scale, rng.uniform(0, 100, sel.sum())], axis=1),
                        np.concatenate([tint, tint, tint, np.zeros((sel.sum(), 1))], axis=1),
                    ],
                    axis=1,
                )
                self._add_pbr_group(name, entry, rows, 1, 9.0, DRAW_DISTANCE["pebble"])

        # Bushes: the downloaded shrub, one per bush the world has.
        shrubs = self._pbr_variants("shrub")
        k = world.bushes
        if shrubs and len(k):
            gz = self._ground(k[:, 0], k[:, 1])
            pick = rng.integers(0, len(shrubs), len(k))
            for index, (name, entry) in enumerate(shrubs):
                sel = pick == index
                n = int(sel.sum())
                if not n:
                    continue
                v = entry["variant"]
                scale = k[sel, 2] / max(max(v.half_extent), 1e-3) * rng.uniform(0.95, 1.2, n)
                tint = rng.uniform(0.8, 1.15, (n, 1))
                rows = np.concatenate(
                    [
                        np.stack([k[sel, 0], k[sel, 1], gz[sel] - 0.03, rng.uniform(0, math.tau, n)], axis=1),
                        np.stack([scale, scale, scale * rng.uniform(0.8, 1.1, n), rng.uniform(0, 100, n)], axis=1),
                        np.concatenate([tint, tint, tint, np.full((n, 1), 0.35)], axis=1),
                    ],
                    axis=1,
                )
                self._add_pbr_group(name, entry, rows, 3, 30.0, DRAW_DISTANCE["bush"])

        # Grass clumps, thickest in scrub.
        clumps = self._pbr_variants("grass", models=("grass_medium_01", "grass_medium_02"))
        if clumps:
            grid = world._grid
            gx_all, gy_all = [], []
            for terrain, per_cell in ((Terrain.SCRUB, 0.55), (Terrain.GROUND, 0.14), (Terrain.BUSH, 0.3)):
                ix, iy = np.nonzero(grid == int(terrain))
                count = rng.poisson(per_cell, len(ix))
                ix, iy = np.repeat(ix, count), np.repeat(iy, count)
                gx_all.append((ix - self._extent + rng.uniform(0, 1, len(ix))) * self._cell)
                gy_all.append((iy - self._extent + rng.uniform(0, 1, len(ix))) * self._cell)
            gx, gy = np.concatenate(gx_all), np.concatenate(gy_all)
            gz = self._ground(gx, gy)
            pick = rng.integers(0, len(clumps), len(gx))
            for index, (name, entry) in enumerate(clumps):
                sel = pick == index
                n = int(sel.sum())
                if not n:
                    continue
                scale = rng.uniform(1.8, 3.0, n)
                straw = rng.uniform(0, 1, n) < 0.5
                tint = np.where(straw[:, None], np.array([1.9, 1.5, 0.85]), np.array([1.35, 1.5, 1.05])) * rng.uniform(0.8, 1.2, (n, 1))
                rows = np.concatenate(
                    [
                        np.stack([gx[sel], gy[sel], gz[sel] - 0.01, rng.uniform(0, math.tau, n)], axis=1),
                        np.stack([scale, scale, scale * rng.uniform(0.8, 1.3, n), rng.uniform(0, 100, n)], axis=1),
                        np.concatenate([tint, np.full((n, 1), 0.9)], axis=1),
                    ],
                    axis=1,
                )
                self._add_pbr_group(name, entry, rows, 2, 7.0, DRAW_DISTANCE["grass"])

    def _build_shadow(self) -> None:
        from ocu.core.camera import _sun_vector

        ctx = self.ctx
        self._sun = np.array(_sun_vector(), dtype=np.float64)
        half = self._world.half_extent_m + 6.0
        zmin, zmax = float(self._elevation.min()) - 2.0, float(self._elevation.max()) + 14.0
        centre = np.array([0.0, 0.0, (zmin + zmax) / 2.0])
        view = look_at(centre + self._sun * 600.0, centre, (0.0, 0.0, 1.0))
        corners = np.array([[sx * half, sy * half, z] for sx in (-1, 1) for sy in (-1, 1) for z in (zmin, zmax)])
        light = (view @ np.c_[corners, np.ones(len(corners))].T).T[:, :3]
        lo, hi = light.min(axis=0), light.max(axis=0)
        proj = orthographic(lo[0], hi[0], lo[1], hi[1], -hi[2] - 5.0, -lo[2] + 5.0)
        self._shadow_vp = proj @ view

        self.shadow_tex = ctx.depth_texture((SHADOW_SIZE, SHADOW_SIZE))
        self.shadow_tex.compare_func = "<="
        self.shadow_tex.filter = (self.mgl.LINEAR, self.mgl.LINEAR)
        self.shadow_tex.repeat_x = self.shadow_tex.repeat_y = False
        fbo = ctx.framebuffer(depth_attachment=self.shadow_tex)
        fbo.use()
        ctx.viewport = (0, 0, SHADOW_SIZE, SHADOW_SIZE)
        fbo.clear(depth=1.0)
        ctx.enable(self.mgl.DEPTH_TEST)
        ctx.disable(self.mgl.CULL_FACE)
        for program in (self.p_shadow_terrain, self.p_shadow_inst):
            program["u_vp"].write(_bytes(self._shadow_vp))
        self.terrain_shadow_vao.render()
        for group in self._groups.values():
            group["shadow"].render(instances=group["count"])
        for group in self._pbr_groups:
            if "shadow" in group:
                group["shadow"].render(instances=group["shadow_count"])
        # Keep the finished map, and a copy of it with nothing moving in it, so that each frame can start from the
        # copy and add whatever is walking about.
        self._shadow_fbo = fbo
        self._shadow_base_tex = ctx.depth_texture((SHADOW_SIZE, SHADOW_SIZE))
        self._shadow_base_fbo = ctx.framebuffer(depth_attachment=self._shadow_base_tex)
        ctx.copy_framebuffer(self._shadow_base_fbo, fbo)
        self._shadow_has_entities = False

    # --- people, vehicles and animals -------------------------------------

    def _entity_ground(self, kind: str, x: float, y: float, heading_deg: float) -> float:
        """Where a figure stands. A vehicle rests on its four wheels, so it takes the average ground under them."""
        if kind != "vehicle":
            return float(self._ground(np.array([x]), np.array([y]))[0])
        h = math.radians(heading_deg)
        fwd, side = np.array([math.sin(h), math.cos(h)]), np.array([math.cos(h), -math.sin(h)])
        pts = [np.array([x, y]) + fwd * ay + side * ax for ay in (1.55, -1.62) for ax in (-0.92, 0.92)]
        return float(np.mean(self._ground(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))))

    def _draw_entity(self, entity, shadow: bool) -> None:
        kind, x, y, heading, *rest = entity
        phase = rest[0] if rest else 0.0
        style = rest[1] if len(rest) > 1 else ""
        key = f"{kind}:{style}" if style and f"{kind}:{style}" in self._entity_frames else kind
        frames = self._entity_frames[key]
        vao, shadow_vao = frames[int(phase * len(frames)) % len(frames)]
        row = np.array([[x, y, self._entity_ground(kind, x, y, heading), -math.radians(heading), 1, 1, 1, DUST_HEIGHT_M[kind], 1, 1, 1, 0]], dtype=np.float32)
        self._entity_buffers[key].write(row.tobytes())
        if shadow:
            shadow_vao.render(instances=1)
        else:
            self._set(self.p_inst, "u_kind", 4)
            vao.render(instances=1)

    def _update_entity_shadows(self, entities) -> None:
        """Start from the shadow map of the still scene and add the figures, so that they cast shadows."""
        ctx = self.ctx
        ctx.copy_framebuffer(self._shadow_fbo, self._shadow_base_fbo)
        self._shadow_fbo.use()
        ctx.viewport = (0, 0, SHADOW_SIZE, SHADOW_SIZE)
        ctx.enable(self.mgl.DEPTH_TEST)
        ctx.disable(self.mgl.CULL_FACE)
        self.p_shadow_inst["u_vp"].write(_bytes(self._shadow_vp))
        for e in entities:
            self._draw_entity(e, shadow=True)
        self._shadow_has_entities = bool(entities)

    # --- drawing ----------------------------------------------------------

    def _target(self, width: int, height: int):
        key = (width, height)
        if key not in self._targets:
            ctx = self.ctx
            color = ctx.texture((width, height), 4, samples=MSAA_SAMPLES)
            depth = ctx.depth_renderbuffer((width, height), samples=MSAA_SAMPLES)
            msaa = ctx.framebuffer(color_attachments=[color], depth_attachment=depth)
            out_color = ctx.texture((width, height), 4)
            resolve = ctx.framebuffer(color_attachments=[out_color])
            self._targets[key] = (msaa, resolve, color, depth, out_color)
            if len(self._targets) > 12:  # sizes change as tiles are resized
                old = next(iter(self._targets))
                for obj in self._targets.pop(old):
                    obj.release()
        return self._targets[key][:2]

    def _set(self, program, name: str, value) -> None:
        if name not in program:
            return
        if isinstance(value, np.ndarray) and value.shape == (4, 4):
            program[name].write(_bytes(value))
        else:
            program[name].value = value

    def _draw_pbr(self, eye_v: np.ndarray, forward: np.ndarray, cone: float) -> None:
        self._set(self.p_pbr, "u_albedo", 3)
        self._set(self.p_pbr, "u_normal", 4)
        self._set(self.p_pbr, "u_arm", 5)
        for group in self._pbr_groups:
            rows = group["rows"]
            reach = group["draw"]
            xs = rows[:, 0]
            lo = int(np.searchsorted(xs, eye_v[0] - reach))
            hi = int(np.searchsorted(xs, eye_v[0] + reach))
            if hi <= lo:
                continue
            rows = rows[lo:hi]
            d = rows[:, :3].astype(np.float64) - eye_v
            dist = np.linalg.norm(d, axis=1)
            with np.errstate(invalid="ignore", divide="ignore"):
                facing = (d @ forward) / np.maximum(dist, 1e-6) > cone - 0.12
            visible = (dist < reach) & (facing | (dist < 5.0))
            if not visible.any():
                continue
            albedo, normal, arm = group["textures"]
            albedo.use(location=3)
            normal.use(location=4)
            arm.use(location=5)
            self._set(self.p_pbr, "u_mode", group["mode"])
            near = visible & (dist < group["lod_distance"])
            far = visible & ~near
            for mask, lod in ((near, group["lods"][0]), (far, group["lods"][-1])):
                count = int(mask.sum())
                if not count:
                    continue
                lod["buffer"].write(np.ascontiguousarray(rows[mask]).tobytes())
                lod["vao"].render(instances=count)

    def render(
        self,
        eye: tuple[float, float, float],
        heading_deg: float,
        pitch_down_deg: float,
        hfov_deg: float,
        width: int,
        height: int,
        rover: tuple[float, float, float] | None = None,
        grain: float = 0.0,
        entities: list[tuple[str, float, float, float]] | None = None,
        trail_s: float | None = None,
        trail_lateral_m: float = 0.0,
        trail_yaw_deg: float = 0.0,
        trail_height_m: float | None = None,
        trail_world_view: bool = False,
    ) -> np.ndarray:
        """One frame, (height, width, 3) uint8. `eye` is absolute (x, y, z).

        With `trail_s` (metres along the reconstructed trail) and a trail loaded, the picture is the real place, drawn
        from the splats, over the sky, and the simulated world's terrain and plants are not drawn. By default `eye`, the
        heading and the pitch are then ignored (a replay of the walk, with no figures). With `trail_world_view` they are the
        camera's place in the simulated world, which the caller has worked out to be where the trail camera is, and the
        figures standing in that world are drawn, with depth, so that the splats in front of one hide it."""
        ctx = self.ctx
        width, height = max(int(width), 8), max(int(height), 8)
        view_pose = None
        if trail_s is not None and self.trail:
            view_pose, view_k, inferred_heading, inferred_pitch = self._trail_view(trail_s, trail_lateral_m, trail_yaw_deg, trail_height_m, width, height)
            hfov_deg = math.degrees(2.0 * math.atan(width / (2.0 * view_k.fx)))
            if not trail_world_view:
                eye, heading_deg, pitch_down_deg, entities = (0.0, 0.0, 2.0), inferred_heading, inferred_pitch, None
        h, pitch = math.radians(heading_deg), math.radians(pitch_down_deg)
        forward = np.array([math.sin(h) * math.cos(pitch), math.cos(h) * math.cos(pitch), -math.sin(pitch)])
        up = (0.0, 0.0, 1.0) if pitch_down_deg < 80.0 else (math.sin(h), math.cos(h), 0.0)
        eye_v = np.array(eye, dtype=np.float64)
        view = look_at(eye_v, eye_v + forward, up)
        aspect = width / height
        vfov = math.degrees(2.0 * math.atan(math.tan(math.radians(hfov_deg) / 2.0) / aspect))
        proj = perspective(vfov, aspect, NEAR_M, FAR_M)
        vp = proj @ view
        t = time.monotonic() - self._t0

        if entities or self._shadow_has_entities:
            self._update_entity_shadows(entities or [])
        msaa, resolve = self._target(width, height)
        msaa.use()
        ctx.viewport = (0, 0, width, height)
        msaa.clear(0.0, 0.0, 0.0, 1.0, depth=1.0)

        common = {
            "u_cam": tuple(eye_v), "u_sun": tuple(self._sun), "u_time": t,
            "u_fog": FOG_PER_M, "u_exposure": EXPOSURE, "u_grain": grain,
        }
        # Sky first, behind everything.
        ctx.disable(self.mgl.DEPTH_TEST)
        for name, value in common.items():
            self._set(self.p_sky, name, value)
        self._set(self.p_sky, "u_inv_vp", np.linalg.inv(vp))
        ctx.vertex_array(self.p_sky, []).render(self.mgl.TRIANGLES, vertices=3)

        ctx.enable(self.mgl.DEPTH_TEST)
        ctx.disable(self.mgl.CULL_FACE)
        self.shadow_tex.use(location=0)
        self._set(self.p_inst, "u_kind", 0)
        for program in (self.p_terrain, self.p_inst, self.p_pbr):
            for name, value in common.items():
                self._set(program, name, value)
            self._set(program, "u_vp", vp)
            self._set(program, "u_shadow_vp", self._shadow_vp)
            self._set(program, "u_shadow", 0)
        textured = self.ground_albedo is not None
        self._set(self.p_terrain, "u_has_tex", 1 if textured else 0)
        self._set(self.p_terrain, "u_footage", 1.0 if (textured and assets.ground_is_footage()) else 0.0)
        if textured:
            self.ground_albedo.use(location=1)
            self.ground_normal.use(location=2)
            self._set(self.p_terrain, "u_g_albedo", 1)
            self._set(self.p_terrain, "u_g_normal", 2)
        if view_pose is None:
            self.terrain_coarse_vao.render()

        # Cull instances to what could be in view.
        diagonal = math.degrees(math.atan(math.hypot(math.tan(math.radians(hfov_deg / 2)), math.tan(math.radians(vfov / 2)))))
        cone = math.cos(math.radians(min(diagonal + HALF_FOV_MARGIN_DEG, 179.0)))
        self._set(self.p_inst, "u_kind", 0)
        for key, group in ({} if view_pose is not None else self._groups).items():
            kind = group["kind"]
            rows = group["rows"]
            reach = DRAW_DISTANCE[kind]
            xs = rows[:, 0]
            lo = int(np.searchsorted(xs, eye_v[0] - reach))
            hi = int(np.searchsorted(xs, eye_v[0] + reach))
            if hi <= lo:
                continue
            rows = rows[lo:hi]
            d = rows[:, :3].astype(np.float64) - eye_v
            dist = np.linalg.norm(d, axis=1)
            near = dist < DRAW_DISTANCE[kind]
            # Anything big enough to matter a little outside the cone still counts.
            with np.errstate(invalid="ignore", divide="ignore"):
                facing = (d @ forward) / np.maximum(dist, 1e-6) > cone - 0.12
            keep = near & (facing | (dist < 6.0))
            count = int(keep.sum())
            if not count:
                continue
            visible = np.ascontiguousarray(rows[keep])
            group["buffer"].write(visible.tobytes())
            self._set(self.p_inst, "u_kind", group["shader_kind"])
            group["vao"].render(instances=count)

        if view_pose is None:
            self._draw_pbr(eye_v, forward, cone)

        if rover is not None and view_pose is None:
            rx, ry, rheading = rover
            gz = float(self._ground(np.array([rx]), np.array([ry]))[0])
            row = np.array([[rx, ry, gz, -math.radians(rheading), 1, 1, 1, 0, 1, 1, 1, 0]], dtype=np.float32)
            self._rover_buffer.write(row.tobytes())
            self._set(self.p_inst, "u_kind", 2)
            self._rover_vao.render(instances=1)

        for e in entities or ():
            self._draw_entity(e, shadow=False)

        if view_pose is not None:
            # the real place, over the sky and under nothing; against the figures' depth when there are any
            scale = float(self.trail["route"].scale)
            self.trail["renderer"].render(view_pose, view_k, depth=(scale, NEAR_M, FAR_M) if trail_world_view else None)
            ctx.disable(self.mgl.BLEND)
            ctx.enable(self.mgl.DEPTH_TEST)

        ctx.copy_framebuffer(resolve, msaa)
        resolve.use()
        raw = resolve.read(components=3, alignment=1)
        return np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)[::-1].copy()
