"""People, vehicles and animals, built as real shapes.

There are no photographed models of these in the free, licence-clean sources this build draws on (Poly Haven
has rocks, plants and props, not a pickup truck, a person or a deer), so they are made here: smooth surfaces
from a small set of primitives, with the proportions of the real thing, painted in colours it really has, and
each vertex tagged with what it is made of (paint, glass, rubber, cloth, skin, fur ...). The renderer shades by
material (shaders.py), so glass reflects the sky, paint has a clear coat under a layer of dust, and cloth and
fur are soft. Walkers come as a cycle of poses.

Every figure faces +y, stands on z = 0, is x to its right, and is in metres. A vertex is
(x, y, z, nx, ny, nz, r, g, b, ao_and_material), where the last number is ambient occlusion (0 to 1) plus
2 for each step of the material number, which the shader unpacks.

No Qt import belongs in this package.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from functools import lru_cache

import numpy as np

# Material numbers, shared with the fragment shader.
PAINT, GLASS, RUBBER, DARK, FABRIC, SKIN, FUR, CHROME, LAMP, HAIR, BONE = range(11)


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Builder:
    """Collects parts into one mesh."""

    def __init__(self) -> None:
        self.P: list[np.ndarray] = []
        self.N: list[np.ndarray] = []
        self.C: list[np.ndarray] = []
        self.A: list[np.ndarray] = []
        self.F: list[np.ndarray] = []
        self._n = 0

    def add(self, pos, nrm, col, ao, mat, faces) -> None:
        pos = np.asarray(pos, dtype=np.float64)
        n = len(pos)
        col = np.broadcast_to(np.asarray(col, dtype=np.float64), (n, 3)).copy()
        ao = np.broadcast_to(np.asarray(ao, dtype=np.float64), (n,)).copy()
        self.P.append(pos)
        self.N.append(np.asarray(nrm, dtype=np.float64))
        self.C.append(col)
        self.A.append(np.clip(ao, 0.0, 1.0) + 2.0 * mat)
        self.F.append(np.asarray(faces, dtype=np.int64).reshape(-1, 3) + self._n)
        self._n += n

    def build(self) -> tuple[np.ndarray, np.ndarray]:
        pos, nrm, col, ao = (np.concatenate(x) for x in (self.P, self.N, self.C, self.A))
        verts = np.concatenate([pos, nrm, col, ao[:, None]], axis=1).astype(np.float32)
        return verts, np.concatenate(self.F).astype(np.uint32).reshape(-1)


def _vertex_normals(pos: np.ndarray, faces: np.ndarray) -> np.ndarray:
    tri = pos[faces]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nrm = np.zeros_like(pos)
    for k in range(3):
        np.add.at(nrm, faces[:, k], fn)
    return _unit(nrm)


def _resolve(value, pos, nrm):
    return value(pos, nrm) if callable(value) else value


def _grid_faces(rows: int, cols: int, wrap: bool) -> np.ndarray:
    """Triangles for a (rows, cols) grid of vertices, columns wrapping round if `wrap`."""
    faces = []
    last = cols if wrap else cols - 1
    for i in range(rows - 1):
        for j in range(last):
            a, b = i * cols + j, i * cols + (j + 1) % cols
            c, d = (i + 1) * cols + j, (i + 1) * cols + (j + 1) % cols
            faces += [(a, c, b), (b, c, d)]
    return np.array(faces, dtype=np.int64)


def superellipsoid(b: Builder, centre, size, mat, colour, ao=1.0, e1=2.0, e2=2.0, rot=None, lat=14, lon=24, flip_y_taper=None):
    """A rounded box (e1, e2 large) or an ellipsoid (both 2). `size` is the full extents (x, y, z)."""
    v = np.linspace(-math.pi / 2, math.pi / 2, lat)
    u = np.linspace(-math.pi, math.pi, lon, endpoint=False)
    uu, vv = np.meshgrid(u, v)

    def sp(x, n):  # n is how square the shape is: 2 is an ellipse, larger is a box with rounded edges
        return np.sign(x) * np.abs(x) ** (2.0 / n)

    x = sp(np.cos(vv), e1) * sp(np.cos(uu), e2)
    y = sp(np.cos(vv), e1) * sp(np.sin(uu), e2)
    z = sp(np.sin(vv), e1)
    pts = np.stack([x, y, z], axis=-1).reshape(-1, 3) * (np.asarray(size) / 2.0)
    if flip_y_taper is not None:  # narrow the section toward one end: (width factor at -y, at +y)
        t = (pts[:, 1] / (size[1] / 2.0) + 1.0) / 2.0
        pts[:, 0] *= flip_y_taper[0] + (flip_y_taper[1] - flip_y_taper[0]) * t
    if rot is not None:
        pts = pts @ np.asarray(rot).T
    pts = pts + np.asarray(centre)
    faces = _grid_faces(lat, lon, wrap=True)
    nrm = _vertex_normals(pts, faces)
    b.add(pts, nrm, _resolve(colour, pts, nrm), _resolve(ao, pts, nrm), mat, faces)


def tube(b: Builder, stations, mat, colour, ao=1.0, sides=18, caps=True):
    """A surface lofted through cross-sections. Each station is (centre, u_axis, v_axis, ru, rv, power)."""
    t = np.linspace(0, 2 * math.pi, sides, endpoint=False)
    rings = []
    for centre, ua, va, ru, rv, power in stations:
        c, s = np.cos(t), np.sin(t)
        e = 2.0 / power
        px = np.sign(c) * np.abs(c) ** e * ru
        py = np.sign(s) * np.abs(s) ** e * rv
        rings.append(np.asarray(centre) + px[:, None] * np.asarray(ua) + py[:, None] * np.asarray(va))
    pts = np.concatenate(rings)
    faces = _grid_faces(len(stations), sides, wrap=True)
    nrm = _vertex_normals(pts, faces)
    b.add(pts, nrm, _resolve(colour, pts, nrm), _resolve(ao, pts, nrm), mat, faces)
    if caps:
        for ring, outward in ((rings[0], -1), (rings[-1], 1)):
            centre = ring.mean(axis=0)
            cap = np.vstack([centre, ring])
            f = np.array([(0, 1 + (k + 1) % sides, 1 + k) if outward > 0 else (0, 1 + k, 1 + (k + 1) % sides) for k in range(sides)])
            axis = _unit(np.cross(ring[1] - ring[0], ring[sides // 4] - ring[0]))
            cn = np.tile(axis * (1 if outward > 0 else -1), (len(cap), 1))
            b.add(cap, cn, _resolve(colour, cap, cn), _resolve(ao, cap, cn), mat, f)


def limb(b: Builder, p0, p1, r0, r1, mat, colour, ao=1.0, sides=14, ends=True, squash=1.0):
    """A tapered limb from p0 to p1, rounded at both ends, slightly flattened front to back by `squash`."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    axis = _unit(p1 - p0)
    helper = np.array([0.0, 1.0, 0.0]) if abs(axis[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    ua = _unit(np.cross(axis, helper))
    va = _unit(np.cross(axis, ua))
    stations = [(p0 + (p1 - p0) * f, ua, va, r0 + (r1 - r0) * f, (r0 + (r1 - r0) * f) * squash, 2.0) for f in np.linspace(0, 1, 5)]
    tube(b, stations, mat, colour, ao, sides=sides, caps=False)
    if ends:
        for p, r in ((p0, r0), (p1, r1)):
            superellipsoid(b, p, (2 * r, 2 * r * squash, 2 * r), mat, colour, ao, lat=8, lon=sides, rot=np.column_stack([ua, va, axis]))


def lathe(b: Builder, centre, axis_index, profile, mat, colour, ao=1.0, sides=32):
    """A surface of revolution about the x (0), y (1) or z (2) axis through `centre`; profile is (radius, position along the axis)."""
    t = np.linspace(0, 2 * math.pi, sides, endpoint=False)
    rows = []
    for r, h in profile:
        c, s = r * np.cos(t), r * np.sin(t)
        if axis_index == 0:
            rows.append(np.stack([np.full_like(t, h), c, s], axis=1))
        elif axis_index == 1:
            rows.append(np.stack([c, np.full_like(t, h), s], axis=1))
        else:
            rows.append(np.stack([c, s, np.full_like(t, h)], axis=1))
    pts = np.concatenate(rows) + np.asarray(centre)
    faces = _grid_faces(len(profile), sides, wrap=True)
    nrm = _vertex_normals(pts, faces)
    b.add(pts, nrm, _resolve(colour, pts, nrm), _resolve(ao, pts, nrm), mat, faces)


def panel(b: Builder, corners, mat, colour, ao=1.0, normal=None):
    """A flat four-cornered panel (glass, a lamp, a plate)."""
    pts = np.asarray(corners, dtype=float)
    faces = np.array([(0, 1, 2), (0, 2, 3)])
    n = _unit(np.cross(pts[1] - pts[0], pts[2] - pts[0])) if normal is None else np.asarray(normal, dtype=float)
    b.add(pts, np.tile(n, (4, 1)), _resolve(colour, pts, np.tile(n, (4, 1))), ao, mat, faces)


def noise_tint(base, amount=0.06, scale=9.0, seed=0) -> Callable:
    """A colour that varies slowly across the surface, the way real paint, cloth and hide do not stay uniform."""
    base = np.asarray(base, dtype=float)

    def fn(pos, _n):
        ph = np.sin(pos[:, 0] * scale + seed) * np.sin(pos[:, 1] * scale * 0.7 + seed * 2.0) * np.sin(pos[:, 2] * scale * 1.3 + seed * 3.0)
        return np.clip(base[None, :] * (1.0 + amount * ph[:, None]), 0.0, 1.0)

    return fn


def height_ao(z0: float, z1: float, lo: float = 0.42) -> Callable:
    """Ambient occlusion that is darkest at the ground and open higher up."""

    def fn(pos, _n):
        return lo + (1.0 - lo) * np.clip((pos[:, 2] - z0) / max(z1 - z0, 1e-6), 0.0, 1.0) ** 0.7

    return fn


def countershade(top, belly, axis=2, soft=0.35) -> Callable:
    """Animal colouring: darker on top, paler underneath, by which way the surface faces."""
    top, belly = np.asarray(top, dtype=float), np.asarray(belly, dtype=float)

    def fn(_pos, n):
        f = np.clip((n[:, axis] + 1.0 - soft) / (2.0 - soft), 0.0, 1.0)[:, None]
        f = np.clip(f * 1.6 - 0.15, 0.0, 1.0)  # most of the coat is the top colour; only the underside is pale
        return belly * (1 - f) + top * f

    return fn


# ====================================================================================== the truck


def truck(length: float = 5.3, height: float = 1.85) -> tuple[np.ndarray, np.ndarray]:
    """A long-wheelbase utility pickup of the kind used for field and relief work: dusty sand-white paint,
    a single cab, an open bed, a roof rack, a bull bar and a snorkel."""
    b = Builder()
    k = length / 5.3
    paint = noise_tint((0.80, 0.79, 0.73), 0.05, 7.0, 1.0)
    ao = height_ao(0.3, 1.5, 0.45)
    steel = (0.30, 0.31, 0.33)
    black = (0.04, 0.04, 0.045)

    # underbody, chassis rails, tank
    superellipsoid(b, (0, 0.0, 0.52), (1.40, 4.7 * k, 0.28), DARK, black, 0.5, 6, 6)
    # lower body and fenders
    superellipsoid(b, (0, 0.20 * k, 0.98), (1.78, 2.05 * k, 0.74), PAINT, paint, ao, 6, 6)
    superellipsoid(b, (0, 1.95 * k, 0.88), (1.80, 1.50 * k, 0.52), PAINT, paint, ao, 6, 6, flip_y_taper=(1.0, 0.96))
    # bonnet, sloping down toward the grille
    superellipsoid(b, (0, 1.72 * k, 1.22), (1.70, 1.65 * k, 0.34), PAINT, paint, ao, 5, 6, rot=rot_x(0.06), flip_y_taper=(1.0, 0.94))
    # cab: belt, then the greenhouse above it
    superellipsoid(b, (0, 0.18 * k, 1.28), (1.78, 1.55 * k, 0.5), PAINT, paint, ao, 5, 6)
    # the greenhouse is glass; the roof and the pillars are paint standing round it
    superellipsoid(b, (0, 0.04 * k, 1.60), (1.62, 1.40 * k, 0.58), GLASS, (0.02, 0.03, 0.04), 1.0, 3, 4, flip_y_taper=(0.97, 0.86))
    superellipsoid(b, (0, 0.02 * k, 1.93), (1.68, 1.50 * k, 0.11), PAINT, noise_tint((0.84, 0.83, 0.78), 0.04, 5.0, 3.0), ao, 4, 5, flip_y_taper=(0.98, 0.88))
    for sx in (-1, 1):
        limb(b, (sx * 0.77, 0.66 * k, 1.30), (sx * 0.70, 0.30 * k, 1.90), 0.036, 0.034, PAINT, paint, ao, sides=10)  # A pillar
        limb(b, (sx * 0.83, -0.10 * k, 1.30), (sx * 0.78, -0.10 * k, 1.90), 0.034, 0.034, PAINT, paint, ao, sides=10)  # B pillar
        limb(b, (sx * 0.72, -0.50 * k, 1.34), (sx * 0.62, -0.48 * k, 1.90), 0.036, 0.034, PAINT, paint, ao, sides=10)  # C pillar
    # bed: floor, sides, headboard, tailgate
    superellipsoid(b, (0, -1.60 * k, 0.88), (1.62, 2.05 * k, 0.10), DARK, (0.10, 0.10, 0.11), 0.55, 8, 8)
    for sx in (-1, 1):
        superellipsoid(b, (sx * 0.86, -1.60 * k, 1.12), (0.10, 2.15 * k, 0.56), PAINT, paint, ao, 8, 8)
    superellipsoid(b, (0, -0.68 * k, 1.28), (1.76, 0.10, 0.88), PAINT, paint, ao, 8, 8)
    superellipsoid(b, (0, -2.66 * k, 1.12), (1.78, 0.10, 0.56), PAINT, paint, ao, 8, 8)
    # bumpers, bull bar, grille, lamps
    superellipsoid(b, (0, 2.70 * k, 0.62), (1.86, 0.28, 0.26), CHROME, steel, 0.8, 6, 6)
    superellipsoid(b, (0, -2.74 * k, 0.60), (1.84, 0.24, 0.22), CHROME, steel, 0.8, 6, 6)
    for sx in (-0.45, 0.45):
        limb(b, (sx, 2.74 * k, 0.55), (sx, 2.72 * k, 1.10), 0.028, 0.028, CHROME, steel, 0.9)
    limb(b, (-0.45, 2.74 * k, 1.10), (0.45, 2.74 * k, 1.10), 0.028, 0.028, CHROME, steel, 0.9)
    superellipsoid(b, (0, 2.80 * k, 0.99), (0.86, 0.06, 0.30), DARK, black, 0.7, 5, 5)
    for sx in (-1, 1):
        superellipsoid(b, (sx * 0.68, 2.80 * k, 1.04), (0.26, 0.09, 0.22), LAMP, (0.95, 0.93, 0.85), 1.0, 4, 4)
        superellipsoid(b, (sx * 0.84, 2.78 * k, 0.86), (0.10, 0.06, 0.07), LAMP, (0.95, 0.55, 0.12), 1.0, 4, 4)
        superellipsoid(b, (sx * 0.78, -2.74 * k, 1.04), (0.14, 0.05, 0.26), LAMP, (0.65, 0.05, 0.04), 1.0, 4, 4)
        # mirrors and handles
        superellipsoid(b, (sx * 1.02, 0.62 * k, 1.50), (0.08, 0.15, 0.20), PAINT, (0.07, 0.07, 0.08), 1.0, 4, 4)
        limb(b, (sx * 0.90, 0.58 * k, 1.42), (sx * 0.97, 0.62 * k, 1.48), 0.012, 0.012, DARK, black)
        superellipsoid(b, (sx * 0.895, 0.02 * k, 1.34), (0.03, 0.14, 0.03), CHROME, steel, 1.0, 4, 4)
    # snorkel
    limb(b, (0.86, 0.60 * k, 1.18), (0.86, 0.50 * k, 1.98), 0.04, 0.04, DARK, black, 0.9)
    superellipsoid(b, (0.86, 0.50 * k, 2.03), (0.11, 0.11, 0.09), DARK, black, 1.0, 3, 3)
    # roof rack with a load
    for sx in (-0.62, 0.62):
        limb(b, (sx, 0.55 * k, 1.99), (sx, -0.60 * k, 1.99), 0.016, 0.016, DARK, black)
    for yy in (0.5, 0.1, -0.3, -0.62):
        limb(b, (-0.62, yy * k, 1.99), (0.62, yy * k, 1.99), 0.014, 0.014, DARK, black)
    superellipsoid(b, (0, -0.1 * k, 2.08), (0.50, 0.70 * k, 0.20), FABRIC, (0.20, 0.23, 0.14), 0.9, 6, 6)  # a rolled load
    # bed contents: a spare wheel lying flat and a jerrycan
    lathe(b, (0.35, -1.70 * k, 0.95), 2, [(0.0, 0.0), (0.30, 0.0), (0.32, 0.05), (0.30, 0.20), (0.0, 0.20)], RUBBER, (0.05, 0.05, 0.055), 0.7, 28)
    superellipsoid(b, (-0.45, -1.30 * k, 1.12), (0.32, 0.18, 0.40), DARK, (0.18, 0.22, 0.12), 0.7, 5, 5)
    # wheels: arch, tyre with tread blocks, rim
    for sy in (1.55 * k, -1.62 * k):
        for sx in (-1, 1):
            cx = sx * 0.92
            lathe(b, (sx * 0.885, sy, 0.46), 0, [(0.0, 0.0), (0.46, 0.0), (0.46, sx * 0.02)], DARK, black, 0.4, 36)
            tyre = [(0.18, -0.13), (0.33, -0.14), (0.385, -0.115), (0.405, -0.06), (0.405, 0.06), (0.385, 0.115), (0.33, 0.14), (0.18, 0.13)]
            lathe(b, (cx, sy, 0.40), 0, tyre, RUBBER, (0.05, 0.05, 0.055), 0.55, 36)
            for a in np.linspace(0, 2 * math.pi, 34, endpoint=False):  # tread blocks
                superellipsoid(b, (cx, sy + 0.405 * math.sin(a), 0.40 + 0.405 * math.cos(a)), (0.24, 0.085, 0.04), RUBBER, (0.045, 0.045, 0.05), 0.6, 4, 4, rot=rot_x(-a), lat=5, lon=8)
            lathe(b, (cx, sy, 0.40), 0, [(0.0, sx * 0.10), (0.10, sx * 0.10), (0.17, sx * 0.08), (0.20, sx * 0.03), (0.20, -sx * 0.05)], CHROME, (0.50, 0.52, 0.55), 0.8, 24)
            superellipsoid(b, (cx + sx * 0.075, sy, 0.40), (0.05, 0.14, 0.14), CHROME, (0.62, 0.64, 0.66), 1.0, 2, 2, lat=8, lon=14)
    return b.build()


# ====================================================================================== the person

SKIN_TONE = (0.50, 0.34, 0.25)


def person(phase: float = 0.0, height: float = 1.75) -> tuple[np.ndarray, np.ndarray]:
    """A person walking, in field clothes with a pack. `phase` is where in the stride (0 to 1)."""
    b = Builder()
    k = height / 1.75
    f = 2.0 * math.pi * phase
    jacket = noise_tint((0.085, 0.145, 0.175), 0.07, 14.0, 2.0)
    trousers = noise_tint((0.27, 0.235, 0.170), 0.06, 12.0, 4.0)
    boots = (0.13, 0.09, 0.07)
    pack = noise_tint((0.22, 0.185, 0.125), 0.06, 10.0, 6.0)
    bob = (0.018 * math.cos(2 * f) - 0.018) * k
    lean = 0.06
    hip_z, hip_x = 0.93 * k + bob, 0.095 * k
    sh_z, sh_x = 1.46 * k + bob, 0.205 * k
    ao = height_ao(0.0, 1.7 * k, 0.55)

    def leg(side: int, ph: float):
        ang = 0.42 * math.sin(ph)
        bend = 0.06 + 0.80 * max(0.0, math.cos(ph)) ** 2
        hip = np.array([side * hip_x, 0.0, hip_z])
        thigh_len, shin_len = 0.44 * k, 0.45 * k
        knee = hip + np.array([0.0, math.sin(ang) * thigh_len, -math.cos(ang) * thigh_len])
        shin_ang = ang - bend
        ankle = knee + np.array([0.0, math.sin(shin_ang) * shin_len, -math.cos(shin_ang) * shin_len])
        limb(b, hip, knee, 0.088 * k, 0.065 * k, FABRIC, trousers, ao, squash=1.0)
        limb(b, knee, ankle, 0.062 * k, 0.050 * k, FABRIC, trousers, ao)
        foot_pitch = shin_ang * 0.7
        boot_c = ankle + np.array([0.0, 0.05 * k * math.cos(foot_pitch), -0.035 * k])
        superellipsoid(b, boot_c, (0.105 * k, 0.285 * k, 0.095 * k), DARK, boots, 0.55, 5, 4, rot=rot_x(-foot_pitch * 0.6), lat=8, lon=14)
        superellipsoid(b, ankle + np.array([0, -0.01 * k, 0.06 * k]), (0.10 * k, 0.11 * k, 0.17 * k), DARK, boots, 0.6, 4, 3, lat=6, lon=12)

    leg(-1, f)
    leg(1, f + math.pi)

    # pelvis and torso: a jacket, longer than the waist, wider at the shoulder than the hip
    centre = lambda z, y: np.array([0.0, y, z])
    stations = []
    for z, ru, rv, y in [(0.86, 0.168, 0.115, 0.0), (1.00, 0.165, 0.110, 0.0), (1.18, 0.170, 0.108, 0.01), (1.36, 0.200, 0.115, 0.02), (1.48, 0.205, 0.105, 0.02), (1.54, 0.10, 0.075, 0.02)]:
        stations.append((centre(z * k + bob, y + (z - 0.86) * lean), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), ru * k, rv * k, 2.3))
    tube(b, stations, FABRIC, jacket, ao, sides=22, caps=True)
    # a high collar
    superellipsoid(b, (0, 0.02 + 0.55 * lean * 0, 1.57 * k + bob), (0.17 * k, 0.15 * k, 0.07 * k), FABRIC, jacket, 0.9, 3, 3, lat=6, lon=14)
    # arms, swinging against the legs
    for side, ph in ((-1, f + math.pi), (1, f)):
        shoulder = np.array([side * sh_x, 0.02, sh_z])
        sw = 0.40 * math.sin(ph)
        elbow_bend = 0.22 + 0.34 * max(0.0, -math.sin(ph))
        up, fore = 0.30 * k, 0.27 * k
        elbow = shoulder + np.array([side * 0.015, math.sin(sw) * up, -math.cos(sw) * up])
        wrist_ang = sw + elbow_bend
        wrist = elbow + np.array([side * 0.01, math.sin(wrist_ang) * fore, -math.cos(wrist_ang) * fore])
        limb(b, shoulder, elbow, 0.052 * k, 0.045 * k, FABRIC, jacket, ao)
        limb(b, elbow, wrist, 0.044 * k, 0.036 * k, FABRIC, jacket, ao)
        superellipsoid(b, wrist + np.array([0, 0.02 * k, -0.045 * k]), (0.055 * k, 0.095 * k, 0.095 * k), SKIN, SKIN_TONE, 1.0, 2.6, 2.4, lat=8, lon=12)
    # neck and head
    limb(b, (0, 0.03, 1.50 * k + bob), (0, 0.04, 1.60 * k + bob), 0.05 * k, 0.047 * k, SKIN, SKIN_TONE, 1.0)
    head_c = np.array([0.0, 0.045, 1.655 * k + bob])
    superellipsoid(b, head_c, (0.165 * k, 0.205 * k, 0.225 * k), SKIN, SKIN_TONE, 1.0, 2.1, 2.1, lat=14, lon=22)
    superellipsoid(b, head_c + np.array([0, 0.105 * k, -0.03 * k]), (0.03 * k, 0.04 * k, 0.045 * k), SKIN, SKIN_TONE, 1.0, 2, 2, lat=6, lon=10)  # nose
    for sx in (-1, 1):
        superellipsoid(b, head_c + np.array([sx * 0.04 * k, 0.088 * k, 0.025 * k]), (0.022 * k, 0.012 * k, 0.014 * k), DARK, (0.05, 0.04, 0.035), 1.0, 2, 2, lat=5, lon=8)  # eyes
        superellipsoid(b, head_c + np.array([sx * 0.088 * k, -0.01 * k, -0.005 * k]), (0.02 * k, 0.04 * k, 0.07 * k), SKIN, SKIN_TONE, 0.9, 2, 2, lat=6, lon=8)  # ears
    # a wide-brimmed field hat
    superellipsoid(b, head_c + np.array([0, 0.0, 0.07 * k]), (0.19 * k, 0.225 * k, 0.12 * k), FABRIC, (0.22, 0.20, 0.14), 1.0, 2.4, 2.4, lat=10, lon=18)
    superellipsoid(b, head_c + np.array([0, 0.0, 0.035 * k]), (0.36 * k, 0.38 * k, 0.025 * k), FABRIC, (0.21, 0.19, 0.13), 1.0, 2.2, 2.2, lat=6, lon=24)
    # a rucksack, with a sleeping mat strapped on top
    superellipsoid(b, (0, -0.145 * k - 0.04 * lean, 1.28 * k + bob), (0.31 * k, 0.20 * k, 0.48 * k), FABRIC, pack, 0.85, 3.5, 3.5, lat=10, lon=18)
    superellipsoid(b, (0, -0.15 * k, 1.56 * k + bob), (0.34 * k, 0.15 * k, 0.12 * k), FABRIC, (0.36, 0.15, 0.06), 0.9, 2.4, 3.0, lat=8, lon=14)
    for sx in (-1, 1):  # shoulder straps
        limb(b, (sx * 0.13 * k, -0.02 * k, 1.50 * k + bob), (sx * 0.10 * k, 0.07 * k, 1.20 * k + bob), 0.018 * k, 0.018 * k, FABRIC, (0.10, 0.10, 0.10), 0.9, sides=8, squash=0.5)
    return b.build()


# ====================================================================================== the deer

HIDE_TOP, HIDE_BELLY = (0.215, 0.125, 0.062), (0.50, 0.42, 0.32)
BONE_COLOUR = (0.43, 0.35, 0.25)
IBEX_TOP, IBEX_BELLY = (0.30, 0.215, 0.125), (0.66, 0.58, 0.46)
HORN_COLOUR = (0.27, 0.215, 0.15)
IBEX_BODY_SCALE = (1.10, 0.78, 0.86)  # across, along and up, from the deer it is built on: stockier, shorter in the body and the leg
IBEX_HEIGHT = 1.0


def _bezier(points, n=7):
    p = np.asarray(points, dtype=float)
    out = []
    for tt in np.linspace(0.0, 1.0, n):
        pts = p.copy()
        while len(pts) > 1:
            pts = pts[:-1] * (1 - tt) + pts[1:] * tt
        out.append(pts[0])
    return np.array(out)


def _antler(b: Builder, base: np.ndarray, side: float, scale: float) -> None:
    """One antler: a curved main beam sweeping up and forward, a brow tine, and tines branching off it."""
    s_ = scale
    beam = _bezier([base, base + np.array([side * 0.10, -0.06, 0.12]) * s_, base + np.array([side * 0.22, -0.02, 0.25]) * s_, base + np.array([side * 0.26, 0.15, 0.36]) * s_], 9)
    ru = np.array([1.0, 0.0, 0.0])
    stations = []
    for i, p in enumerate(beam):
        t = i / (len(beam) - 1)
        d = beam[min(i + 1, len(beam) - 1)] - beam[max(i - 1, 0)]
        d = d / max(np.linalg.norm(d), 1e-9)
        v = _unit(np.cross(d, ru))
        u = _unit(np.cross(d, v))
        r = (0.036 - 0.024 * t) * s_ + 0.004
        stations.append((p, u, v, r, r * 0.9, 2.0))
    tube(b, stations, BONE, BONE_COLOUR, 0.95, sides=12, caps=True)
    for f0, direction, length in ((0.0, np.array([side * 0.15, 0.85, 0.45]), 0.30), (0.28, np.array([side * 0.45, 0.35, 0.85]), 0.30), (0.52, np.array([side * 0.55, 0.20, 0.80]), 0.28), (0.76, np.array([side * 0.30, 0.55, 0.78]), 0.22), (1.0, np.array([side * 0.10, 0.50, 0.85]), 0.16)):
        idx = int(f0 * (len(beam) - 1))
        origin = beam[idx]
        tip = origin + _unit(np.asarray(direction, dtype=float)) * length * s_
        limb(b, origin, tip, 0.024 * s_, 0.008 * s_, BONE, BONE_COLOUR, 0.95, sides=8)


def _horn(b: Builder, base: np.ndarray, side: float, scale: float) -> None:
    """An ibex's horn: flattened, sweeping up and then far back in a scimitar, with a transverse ridge every few centimetres."""
    s_ = scale
    arc = _bezier([base, base + np.array([side * 0.06, -0.04, 0.16]) * s_, base + np.array([side * 0.13, -0.22, 0.34]) * s_,
                   base + np.array([side * 0.16, -0.50, 0.38]) * s_, base + np.array([side * 0.15, -0.72, 0.27]) * s_], 16)
    ru = np.array([1.0, 0.0, 0.0])
    stations = []
    for i, p in enumerate(arc):
        t = i / (len(arc) - 1)
        d = arc[min(i + 1, len(arc) - 1)] - arc[max(i - 1, 0)]
        d = d / max(np.linalg.norm(d), 1e-9)
        v = _unit(np.cross(d, ru))
        u = _unit(np.cross(d, v))
        ridge = 1.0 + 0.14 * math.cos(2.0 * math.pi * 9.0 * t) * (1.0 - 0.6 * t)
        r = (0.052 * (1.0 - t) ** 0.8 + 0.006) * s_ * ridge
        stations.append((p, u, v, r * 1.3, r * 0.75, 2.0))
    tube(b, stations, BONE, HORN_COLOUR, 0.95, sides=12, caps=True)


def deer(phase: float = 0.0, height: float = 1.0, buck: bool = False, species: str = "deer") -> tuple[np.ndarray, np.ndarray]:
    """A deer walking: lateral-sequence gait, a countershaded coat with a dark dorsal stripe, white rump, tail, throat
    and chin, large ears with pale insides, a face with a brow and ringed eyes, jointed legs with split hooves and
    dew claws. A buck carries antlers and a heavier neck."""
    b = Builder()
    k = height / 1.0
    f = 2.0 * math.pi * phase
    ibex = species == "ibex"
    top = np.array(IBEX_TOP if ibex else HIDE_TOP)
    belly_c = np.array(IBEX_BELLY if ibex else HIDE_BELLY)

    def hide(pos, n):
        base = countershade(top, belly_c)(pos, n)
        stripe = np.exp(-((pos[:, 0] / (0.045 * k)) ** 2)) * np.clip(n[:, 2], 0.0, 1.0) * np.clip((pos[:, 1] + 0.7 * k) / (1.3 * k), 0.0, 1.0)
        base = base * (1.0 - 0.28 * stripe[:, None])  # a darker line down the back
        flank = np.clip(1.0 - np.abs(pos[:, 2] - 0.66 * k) / (0.12 * k), 0.0, 1.0) * (1.0 - np.abs(n[:, 2]))
        base = base * (1.0 + 0.10 * flank[:, None])  # a paler band along the flank
        ph = np.sin(pos[:, 0] * 31.0) * np.sin(pos[:, 1] * 23.0) * np.sin(pos[:, 2] * 37.0)
        return np.clip(base * (1.0 + 0.08 * ph[:, None]), 0.0, 1.0)

    lower = (0.12, 0.085, 0.06) if ibex else (0.16, 0.105, 0.065)
    pale = (0.62, 0.52, 0.43)
    white = (0.80, 0.78, 0.72)
    dark = (0.04, 0.035, 0.032)
    girth = 1.22 if ibex else 1.07 if buck else 1.0
    bob = 0.012 * k * math.cos(2 * f)
    ao = height_ao(0.0, 1.1 * k, 0.5)

    # barrel: deep at the chest, tucked up at the flank, with a shoulder and a haunch
    body = []
    for y, z, ru, rv in [(-0.755, 0.80, 0.025, 0.04), (-0.71, 0.82, 0.075, 0.115), (-0.63, 0.85, 0.118, 0.185), (-0.44, 0.84, 0.158, 0.225), (-0.18, 0.80, 0.150, 0.212),
                         (0.08, 0.78, 0.150, 0.212), (0.30, 0.785, 0.170, 0.262), (0.50, 0.82, 0.146, 0.235), (0.63, 0.875, 0.10, 0.185), (0.72, 0.905, 0.04, 0.09)]:
        body.append((np.array([0.0, y * k, z * k + bob]), np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), ru * k * girth, rv * k, 2.3))
    tube(b, body, FUR, hide, ao, sides=30, caps=True)
    for sx in (-1, 1):
        superellipsoid(b, (sx * 0.095 * k, -0.42 * k, 0.77 * k + bob), (0.13 * k, 0.34 * k, 0.36 * k), FUR, hide, ao, 2.2, 2.2, lat=12, lon=16)  # haunch
        superellipsoid(b, (sx * 0.095 * k, 0.34 * k, 0.76 * k + bob), (0.12 * k * girth, 0.30 * k, 0.34 * k), FUR, hide, ao, 2.2, 2.2, lat=12, lon=16)  # shoulder
    # belly line and a paler chest
    superellipsoid(b, (0, 0.05 * k, 0.60 * k + bob), (0.20 * k, 0.9 * k, 0.07 * k), FUR, pale, 0.8, 2.2, 2.2, lat=8, lon=16)
    # white rump patch, a tail that is dark above and white beneath, flicking as it walks
    superellipsoid(b, (0, -0.66 * k, 0.84 * k + bob), (0.20 * k, 0.12 * k, 0.27 * k), FUR, (0.62, 0.50, 0.38) if ibex else (0.78, 0.75, 0.68), 0.9, 2.2, 2.2, lat=12, lon=16)
    flick = 0.25 * math.sin(f * 2.0)
    tail_rot = rot_x(0.55 + flick)
    superellipsoid(b, (0, -0.71 * k, 0.80 * k + bob), (0.06 * k, 0.11 * k, 0.15 * k) if ibex else (0.07 * k, 0.15 * k, 0.19 * k), FUR, (0.10, 0.07, 0.05) if ibex else (0.30, 0.18, 0.10), 0.9, 2.2, 2.2, rot=tail_rot, lat=8, lon=12)
    superellipsoid(b, (0, -0.725 * k, 0.775 * k + bob), (0.05 * k, 0.09 * k, 0.12 * k) if ibex else (0.06 * k, 0.13 * k, 0.17 * k), FUR, (0.20, 0.15, 0.10) if ibex else (0.90, 0.89, 0.86), 0.9, 2.2, 2.2, rot=tail_rot, lat=8, lon=12)

    # neck, with a ruff at the throat (heavier on a buck)
    head_bob = 0.02 * k * math.sin(2 * f)
    neck_dir = np.array([0.0, 0.50, 0.34]) / math.hypot(0.50, 0.34)
    neck_v = np.array([0.0, -neck_dir[2], neck_dir[1]])
    nk = 1.32 if ibex else 1.18 if buck else 1.0
    neck = [(np.array([0.0, (0.44 + 0.52 * t) * k, (0.88 + 0.36 * t) * k + bob + head_bob * t]), np.array([1.0, 0, 0]), neck_v, (0.100 - 0.052 * t) * k * nk, (0.125 - 0.068 * t) * k * nk, 2.2) for t in np.linspace(0, 1, 7)]
    tube(b, neck, FUR, hide, ao, sides=22, caps=False)
    superellipsoid(b, (0, 0.70 * k, 0.98 * k + bob), (0.15 * k * nk, 0.14 * k, 0.17 * k), FUR, (0.60, 0.50, 0.41), 0.9, 2.2, 2.2, lat=8, lon=14)  # the throat patch
    if buck:
        for t_, r_ in ((0.25, 0.14), (0.50, 0.12), (0.75, 0.10)):
            superellipsoid(b, (0, (0.44 + 0.52 * t_) * k, (0.88 + 0.36 * t_) * k + bob + 0.05 * k), (0.15 * k, 0.17 * k, r_ * k), FUR, (0.20, 0.12, 0.07), 0.9, 2.2, 2.2, lat=8, lon=12)  # a mane

    # head
    head_c = np.array([0.0, 1.07 * k, 1.28 * k + bob + head_bob])
    pitch = -0.35
    R = rot_x(pitch)
    hp = lambda dx, dy, dz: head_c + R @ (np.array([dx, dy, dz]) * k)  # noqa: E731
    superellipsoid(b, head_c, (0.135 * k, 0.24 * k, 0.16 * k), FUR, lower, 1.0, 2.2, 2.2, rot=R, lat=14, lon=20)
    superellipsoid(b, hp(0, 0.01, 0.075), (0.105 * k, 0.12 * k, 0.07 * k), FUR, (0.19, 0.125, 0.08), 1.0, 2.2, 2.2, rot=R, lat=8, lon=12)  # the forehead
    muzzle_a, muzzle_b = hp(0, 0.10, -0.01), hp(0, 0.25, -0.035)
    limb(b, muzzle_a, muzzle_b, 0.054 * k, 0.036 * k, FUR, (0.30, 0.22, 0.16), 1.0, sides=14)
    superellipsoid(b, hp(0, 0.215, -0.012), (0.075 * k, 0.09 * k, 0.045 * k), FUR, (0.27, 0.19, 0.13), 1.0, 2.2, 2.2, rot=R, lat=8, lon=12)  # the upper lip
    superellipsoid(b, hp(0, 0.255, -0.04), (0.052 * k, 0.04 * k, 0.04 * k), DARK, dark, 1.0, 2, 2, rot=R, lat=8, lon=12)  # nose pad
    for sx in (-1, 1):
        superellipsoid(b, hp(sx * 0.014, 0.27, -0.025), (0.012 * k, 0.014 * k, 0.012 * k), DARK, (0.0, 0.0, 0.0), 1.0, 2, 2, lat=5, lon=8)  # nostrils
    superellipsoid(b, hp(0, 0.215, -0.085), (0.06 * k, 0.11 * k, 0.04 * k), FUR, white, 0.95, 2.2, 2.2, rot=R, lat=8, lon=12)  # a pale chin
    limb(b, hp(-0.045, 0.255, -0.05), hp(-0.045, 0.16, -0.062), 0.006 * k, 0.006 * k, DARK, dark, 1.0, sides=6)  # the line of the mouth
    limb(b, hp(0.045, 0.255, -0.05), hp(0.045, 0.16, -0.062), 0.006 * k, 0.006 * k, DARK, dark, 1.0, sides=6)
    for sx in (-1, 1):
        superellipsoid(b, hp(sx * 0.064, 0.045, 0.03), (0.05 * k, 0.034 * k, 0.05 * k), FUR, (0.48, 0.40, 0.32), 1.0, 2, 2, rot=R, lat=8, lon=12)  # the pale ring round the eye
        superellipsoid(b, hp(sx * 0.068, 0.05, 0.03), (0.030 * k, 0.024 * k, 0.032 * k), GLASS, (0.015, 0.011, 0.009), 1.0, 2, 2, rot=R, lat=8, lon=12)  # the eye
        superellipsoid(b, hp(sx * 0.052, 0.085, 0.015), (0.012 * k, 0.04 * k, 0.012 * k), DARK, (0.10, 0.07, 0.05), 1.0, 2, 2, rot=R, lat=4, lon=8)  # a dark mark below it
        twitch = 0.07 * math.sin(3.0 * f + sx)
        ear_c = head_c + np.array([sx * 0.085 * k, -0.07 * k, 0.095 * k])
        er = rot_z(sx * (0.50 + twitch)) @ rot_x(0.2)
        ek = 0.62 if ibex else 1.0  # an ibex's ears are short
        superellipsoid(b, ear_c, (0.048 * k * ek, 0.09 * k * ek, 0.185 * k * ek), FUR, lower, 1.0, 2.2, 2.2, rot=er, lat=10, lon=14)
        superellipsoid(b, ear_c + er @ np.array([-sx * 0.012, 0.012, 0.0]) * k, (0.030 * k * ek, 0.05 * k * ek, 0.14 * k * ek), FUR, (0.70, 0.60, 0.50), 1.0, 2, 2, rot=er, lat=8, lon=12)  # the pale inside
        superellipsoid(b, ear_c + er @ np.array([-sx * 0.012, 0.012, 0.085]) * k * ek, (0.022 * k * ek, 0.04 * k * ek, 0.04 * k * ek), FUR, (0.80, 0.75, 0.66), 1.0, 2, 2, rot=er, lat=6, lon=8)  # the tuft of hair
    if buck:
        for sx in (-1, 1):
            base = hp(sx * 0.045, -0.04, 0.10)
            _antler(b, base, float(sx), k)
            superellipsoid(b, base, (0.05 * k, 0.05 * k, 0.04 * k), FUR, BONE_COLOUR, 1.0, 2, 2, lat=6, lon=10)  # the burr at its foot

    # legs: knee or hock, cannon, dew claws and a split hoof, in the order hind-left, fore-left, hind-right, fore-right
    def leg(x: float, y: float, z: float, offset: float, front: bool):
        ph = f + 2 * math.pi * offset
        swing = 0.36 * math.sin(ph)
        lift = 0.75 * max(0.0, math.cos(ph)) ** 2
        top_len, low_len = (0.30 if front else 0.33) * k, (0.31 if front else 0.30) * k
        hip = np.array([x * k, y * k, z * k + bob])
        knee = hip + np.array([0.0, math.sin(swing) * top_len, -math.cos(swing) * top_len])
        low_ang = swing + (-lift if front else lift * 0.8)
        fetlock = knee + np.array([0.0, math.sin(low_ang) * low_len, -math.cos(low_ang) * low_len])
        limb(b, hip, knee, 0.074 * k, 0.036 * k, FUR, lower if front else hide, ao)
        limb(b, knee, fetlock, 0.030 * k, 0.021 * k, FUR, lower, ao)
        joint = (0.05 * k, 0.055 * k, 0.075 * k) if front else (0.045 * k, 0.07 * k, 0.085 * k)
        superellipsoid(b, knee + np.array([0, 0.0 if front else -0.012 * k, 0]), joint, FUR, lower, ao, 2.2, 2.2, lat=8, lon=12)  # the knee, or the point of the hock
        superellipsoid(b, fetlock + np.array([0, 0, 0.01 * k]), (0.036 * k, 0.04 * k, 0.04 * k), FUR, lower, ao, 2, 2, lat=6, lon=10)
        for dx in (-0.016, 0.016):
            superellipsoid(b, fetlock + np.array([dx * k, -0.03 * k * math.cos(low_ang), -0.012 * k]), (0.012 * k, 0.022 * k, 0.018 * k), DARK, (0.07, 0.06, 0.055), 0.7, 2, 2, lat=5, lon=8)  # dew claws
        toe_c = fetlock + np.array([0.0, 0.012 * k * math.cos(low_ang), -0.03 * k])
        for dx in (-0.013, 0.013):
            superellipsoid(b, toe_c + np.array([dx * k, 0, 0]), (0.024 * k, 0.07 * k, 0.062 * k), DARK, (0.07, 0.06, 0.055), 0.7, 3, 3, lat=6, lon=10)  # the two halves of the hoof

    leg(-0.085, -0.48, 0.70, 0.0, False)
    leg(-0.085, 0.38, 0.68, 0.25, True)
    leg(0.085, -0.48, 0.70, 0.5, False)
    leg(0.085, 0.38, 0.68, 0.75, True)
    if ibex:
        # Shorter in the leg and stockier than the deer drawn above, then the horns and the beard, which are not squashed
        squash = np.array(IBEX_BODY_SCALE)
        for arr in b.P:
            arr *= squash
        for sx in (-1, 1):
            _horn(b, hp(sx * 0.050, -0.03, 0.115) * squash, float(sx), k * 1.15)
        limb(b, hp(0, 0.20, -0.11) * squash, hp(0, 0.16, -0.30) * squash, 0.032 * k, 0.012 * k, FUR, (0.11, 0.08, 0.055), 1.0, sides=10)  # the beard
    return b.build()


GAIT_FRAMES = 12


@lru_cache(maxsize=1)
def build_all() -> dict[str, list[tuple[np.ndarray, np.ndarray]]]:
    """Every pose of every figure: one truck, and a stride of people and deer."""
    return {
        "vehicle": [truck()],
        "person": [person(i / GAIT_FRAMES) for i in range(GAIT_FRAMES)],
        "animal": [deer(i / GAIT_FRAMES) for i in range(GAIT_FRAMES)],
        "animal:buck": [deer(i / GAIT_FRAMES, buck=True) for i in range(GAIT_FRAMES)],
        "animal:ibex": [deer(i / GAIT_FRAMES, height=IBEX_HEIGHT, species="ibex") for i in range(GAIT_FRAMES)],
    }
