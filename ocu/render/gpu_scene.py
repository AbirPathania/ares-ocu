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
from ocu.render import meshes, shaders

SHADOW_SIZE = 4096
MSAA_SAMPLES = 4
NEAR_M = 0.12
FAR_M = 5000.0
FOG_PER_M = 1.0 / 520.0
EXPOSURE = 0.85

#: Things further than this are not drawn: past it they are fog.
DRAW_DISTANCE = {"rock": 150.0, "pebble": 34.0, "bush": 120.0, "conifer": 190.0, "broadleaf": 190.0, "grass": 22.0}

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
    def create(cls, world: TerrainWorld) -> "GpuScene | None":
        try:
            return cls(world)
        except Exception:  # no GPU or no way to make a context: fall back
            return None

    def __init__(self, world: TerrainWorld) -> None:
        import moderngl

        self.mgl = moderngl
        self.ctx = moderngl.create_standalone_context()
        self._world = world
        self._t0 = time.monotonic()
        self._cell = world.cell_size_m
        self._extent = world._extent
        self._span = world._span
        self._elevation = world._elevation.astype(np.float64)

        self._build_programs()
        self._build_terrain()
        self._build_templates()
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
        self.p_sky = self._program(shaders.SKY_VERT, shaders.SKY_FRAG)
        self.p_shadow_terrain = self._program(shaders.SHADOW_TERRAIN_VERT, shaders.SHADOW_FRAG)
        self.p_shadow_inst = self._program(shaders.SHADOW_INSTANCE_VERT, shaders.SHADOW_FRAG)

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

        # Extend past the edge of the world, flat, so the horizon is ground and
        # not the end of a table.
        pad = 5
        axis = (np.arange(span) - extent + 0.5) * cell
        half = axis[-1]
        outer = np.array([half + 0.5, half + 30.0, half + 300.0, half + 1200.0, half + 5000.0])
        xs = np.concatenate([-outer[::-1], axis, outer])

        def padded(a):
            widths = [(pad, pad), (pad, pad)] + [(0, 0)] * (a.ndim - 2)
            return np.pad(a, widths, mode="edge")

        def padded_with(a, value):
            out = padded(a)
            out[:pad], out[-pad:], out[:, :pad], out[:, -pad:] = value, value, value, value
            return out

        # The extension is plain ground: its materials are constants, not the
        # edge of the world smeared out for five kilometres.
        zp = padded(z)
        np_ = padded_with(normal, (0.0, 0.0, 1.0))
        wp = padded_with(weights, (0.35, 0.65, 0.0, 0.0))
        ep = padded_with(extra, (0.0, 0.0, 0.0, 0.0))
        X, Y = np.meshgrid(xs, xs, indexing="ij")
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

    def _build_templates(self) -> None:
        self._template_meshes = {
            "rock": [meshes.rock_template(100 + k) for k in range(ROCK_VARIANTS)],
            "pebble": [meshes.rock_template(300 + k, 2) for k in range(3)],
            "bush": [meshes.bush_template(500 + k) for k in range(FOLIAGE_VARIANTS)],
            "conifer": [meshes.conifer_template(700 + k) for k in range(FOLIAGE_VARIANTS)],
            "broadleaf": [meshes.broadleaf_template(900 + k) for k in range(FOLIAGE_VARIANTS)],
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
                    np.concatenate([tone * rng.uniform(0.40, 0.58, (len(k), 1)), tone * 0.50, tone * rng.uniform(0.30, 0.45, (len(k), 1)), np.full((len(k), 1), 0.6)], axis=1),
                ],
                axis=1,
            )
            variant = rng.integers(0, FOLIAGE_VARIANTS, len(k))
            for v in range(FOLIAGE_VARIANTS):
                add("bush", v, rows[variant == v])

        t = world.trees
        if len(t):
            gz = self._ground(t[:, 0], t[:, 1])
            for species, kind, crown_in_template in ((0, "broadleaf", 0.34), (1, "conifer", 0.30)):
                sel = t[:, 4] == species
                if not sel.any():
                    continue
                tt, gg = t[sel], gz[sel]
                n = len(tt)
                tone = rng.uniform(0.82, 1.18, (n, 1))
                warm = rng.uniform(0.9, 1.12, (n, 1))
                rows = np.concatenate(
                    [
                        np.stack([tt[:, 0], tt[:, 1], gg - 0.05, rng.uniform(0, math.tau, n)], axis=1),
                        np.stack([tt[:, 3] / crown_in_template, tt[:, 3] / crown_in_template, tt[:, 2], rng.uniform(0, 100, n)], axis=1),
                        np.concatenate([tone * warm * 0.72, tone * 0.74, tone * 0.62, np.full((n, 1), 1.0)], axis=1),
                    ],
                    axis=1,
                )
                variant = rng.integers(0, FOLIAGE_VARIANTS, n)
                for v in range(FOLIAGE_VARIANTS):
                    add(kind, v, rows[variant == v])

        # Grass: a tuft per cell where the ground carries it, thicker in scrub.
        grid = world._grid
        for terrain, per_cell in ((Terrain.SCRUB, 1.4), (Terrain.GROUND, 0.36), (Terrain.BUSH, 0.7)):
            ix, iy = np.nonzero(grid == int(terrain))
            count = rng.poisson(per_cell, len(ix))
            ix, iy = np.repeat(ix, count), np.repeat(iy, count)
            if not len(ix):
                continue
            gx = (ix - self._extent + rng.uniform(0, 1, len(ix))) * self._cell
            gy = (iy - self._extent + rng.uniform(0, 1, len(ix))) * self._cell
            ground = self._ground(gx, gy)
            n = len(gx)
            straw = rng.uniform(0.0, 1.0, n) < (0.55 if terrain == Terrain.GROUND else 0.3)
            tone = rng.uniform(0.75, 1.2, (n, 1))
            colour = np.where(straw[:, None], np.array([1.05, 0.92, 0.62]), np.array([0.62, 0.95, 0.5])) * tone
            height = rng.uniform(0.25, 0.55, n) * (1.3 if terrain == Terrain.SCRUB else 0.9)
            rows = np.concatenate(
                [
                    np.stack([gx, gy, ground - 0.02, rng.uniform(0, math.tau, n)], axis=1),
                    np.stack([height * 0.9, height * 0.9, height, rng.uniform(0, 100, n)], axis=1),
                    np.concatenate([colour, np.full((n, 1), 0.8)], axis=1),
                ],
                axis=1,
            )
            variant = rng.integers(0, 4, n)
            for v in range(4):
                key = ("grass", v)
                part = rows[variant == v]
                groups[key] = np.concatenate([groups[key], part.astype(np.float32)]) if key in groups else part.astype(np.float32)

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
        fbo.release()

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
    ) -> np.ndarray:
        """One frame, (height, width, 3) uint8. `eye` is absolute (x, y, z)."""
        ctx = self.ctx
        width, height = max(int(width), 8), max(int(height), 8)
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
        for program in (self.p_terrain, self.p_inst):
            for name, value in common.items():
                self._set(program, name, value)
            self._set(program, "u_vp", vp)
            self._set(program, "u_shadow_vp", self._shadow_vp)
            self._set(program, "u_shadow", 0)
        self.terrain_coarse_vao.render()

        # Cull instances to what could be in view.
        diagonal = math.degrees(math.atan(math.hypot(math.tan(math.radians(hfov_deg / 2)), math.tan(math.radians(vfov / 2)))))
        cone = math.cos(math.radians(min(diagonal + HALF_FOV_MARGIN_DEG, 179.0)))
        self._set(self.p_inst, "u_kind", 0)
        for key, group in self._groups.items():
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

        if rover is not None:
            rx, ry, rheading = rover
            gz = float(self._ground(np.array([rx]), np.array([ry]))[0])
            row = np.array([[rx, ry, gz, -math.radians(rheading), 1, 1, 1, 0, 1, 1, 1, 0]], dtype=np.float32)
            self._rover_buffer.write(row.tobytes())
            self._set(self.p_inst, "u_kind", 2)
            self._rover_vao.render(instances=1)

        ctx.copy_framebuffer(resolve, msaa)
        resolve.use()
        raw = resolve.read(components=3, alignment=1)
        return np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)[::-1].copy()
