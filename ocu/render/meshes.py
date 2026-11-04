"""Procedural meshes for the things that stand in the world.

Every mesh is built from numbers, so the build needs no asset files and the
same seed gives the same rock. Each is returned as vertex rows of
(position xyz, normal xyz, colour rgb, ambient occlusion) in a unit frame:
rocks have a radius of one and sit with their centre at the origin, trees and
bushes stand on the origin with their trunk along +z.

Rocks are convex polyhedra with softened edges and a little noise on top,
which is what weathered boulders are: a sphere pushed in by a handful of
cleavage planes. Trees are built in tiers and crowns of displaced spheres with
their darkness baked in toward the middle, because a crown is dark where it is
deep and bright where it faces the sky, and no lighting model gets that from
a smooth blob.
"""

from __future__ import annotations

import math

import numpy as np

VERTEX_FLOATS = 10  # position 3, normal 3, colour 3, occlusion 1


# --- primitives -----------------------------------------------------------


def icosphere(subdivisions: int) -> tuple[np.ndarray, np.ndarray]:
    """Unit sphere vertices and triangle indices."""
    t = (1.0 + math.sqrt(5.0)) / 2.0
    verts = [
        (-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
        (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
        (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1),
    ]
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]
    vertices = [np.array(v, dtype=np.float64) / np.linalg.norm(v) for v in verts]
    cache: dict[tuple[int, int], int] = {}

    def midpoint(a: int, b: int) -> int:
        key = (min(a, b), max(a, b))
        if key in cache:
            return cache[key]
        m = (vertices[a] + vertices[b]) / 2.0
        vertices.append(m / np.linalg.norm(m))
        cache[key] = len(vertices) - 1
        return cache[key]

    for _ in range(subdivisions):
        refined = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            refined += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = refined
    return np.array(vertices, dtype=np.float64), np.array(faces, dtype=np.uint32)


def vertex_normals(positions: np.ndarray, faces: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(positions)
    a, b, c = positions[faces[:, 0]], positions[faces[:, 1]], positions[faces[:, 2]]
    face_normal = np.cross(b - a, c - a)
    for k in range(3):
        np.add.at(normals, faces[:, k], face_normal)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.maximum(length, 1e-9)


def _hash3(p: np.ndarray, seed: int) -> np.ndarray:
    """Cheap deterministic hash of integer lattice points to [0, 1)."""
    q = p.astype(np.int64)
    n = (q[..., 0] * 374761393 + q[..., 1] * 668265263 + q[..., 2] * 2147483647 + seed * 1274126177) & 0xFFFFFFFF
    n = (n ^ (n >> 13)) * 1274126177 & 0xFFFFFFFF
    n = n ^ (n >> 16)
    return (n & 0xFFFFFF) / float(0x1000000)


def value_noise3(p: np.ndarray, seed: int) -> np.ndarray:
    """Smooth 3D value noise in [0, 1] at points p of shape (n, 3)."""
    i = np.floor(p)
    f = p - i
    u = f * f * (3.0 - 2.0 * f)
    out = 0.0
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                corner = i + np.array([dx, dy, dz])
                weight = (
                    (u[:, 0] if dx else 1 - u[:, 0])
                    * (u[:, 1] if dy else 1 - u[:, 1])
                    * (u[:, 2] if dz else 1 - u[:, 2])
                )
                out = out + weight * _hash3(corner, seed)
    return out


def fbm3(p: np.ndarray, seed: int, octaves: int = 4) -> np.ndarray:
    total, amplitude, norm = 0.0, 0.5, 0.0
    for octave in range(octaves):
        total = total + amplitude * value_noise3(p * (2.0**octave), seed + octave * 31)
        norm += amplitude
        amplitude *= 0.5
    return total / norm


def _pack(positions, normals, colours, occlusion) -> np.ndarray:
    return np.concatenate(
        [positions, normals, colours, occlusion.reshape(-1, 1)], axis=1
    ).astype(np.float32)


def _merge(parts: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    vertices, indices, offset = [], [], 0
    for v, i in parts:
        vertices.append(v)
        indices.append(i + offset)
        offset += len(v)
    return np.concatenate(vertices), np.concatenate(indices).astype(np.uint32)


# --- rocks ------------------------------------------------------------------


def rock_template(seed: int, subdivisions: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """A boulder: radius one, centre at the origin.

    The shape is a sphere cut by several planes (so it has flat weathered
    faces and ridges between them), then roughened at three scales. The lower
    half is meant to be buried and is drawn anyway.
    """
    rng = np.random.default_rng(seed)
    directions, faces = icosphere(subdivisions)
    d = directions

    planes = rng.normal(size=(rng.integers(5, 9), 3))
    planes /= np.linalg.norm(planes, axis=1, keepdims=True)
    offsets = rng.uniform(0.70, 0.93, len(planes))
    along = d @ planes.T  # (n, planes)
    # Distance to the cleavage plane along each direction; the rock is whichever
    # surface is nearest, which makes a convex solid. A soft minimum keeps the
    # ridges rounded instead of knife-edged.
    reach = np.where(along > 1e-3, offsets[None, :] / np.maximum(along, 1e-3), 4.0)
    k = 14.0
    radius = -np.log(np.sum(np.exp(-k * np.minimum(reach, 1.6)), axis=1)) / k
    radius = np.minimum(radius, 1.0)

    rough = (
        0.10 * (fbm3(d * 2.1 + seed * 0.37, seed, 3) - 0.5)
        + 0.045 * (fbm3(d * 6.5 + seed * 0.11, seed + 7, 3) - 0.5)
        + 0.015 * (fbm3(d * 18.0, seed + 13, 2) - 0.5)
    )
    radius = np.clip(radius * (1.0 + rough * 2.0), 0.45, 1.05)
    positions = d * radius[:, None]
    normals = vertex_normals(positions, faces)

    shade = 0.78 + 0.34 * fbm3(d * 3.0 + 5.0, seed + 3, 3)
    colours = np.stack([shade * 0.50, shade * 0.495, shade * 0.47], axis=1)
    # Darker underneath, where it meets the ground and is shaded by it.
    occlusion = np.clip(0.55 + 0.6 * (d[:, 2] * 0.5 + 0.5), 0.35, 1.0)
    return _pack(positions, normals, colours, occlusion), faces.reshape(-1)


# --- bushes -----------------------------------------------------------------


def bush_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """A dry-season shrub (lantana, a thorn bush): footprint radius one, about 0.9 tall, standing on the origin.

    Several thin stems from the root, a loose mass of olive leaf sprays held on them, a share of the leaves brown and
    dead, and gaps you can see through. Not a lobed ball: the footage's shrubs are open and ragged."""
    rng = np.random.default_rng(seed)
    parts = []
    for k in range(int(rng.integers(5, 8))):
        a = rng.uniform(0.0, math.tau)
        out = np.array([math.cos(a), math.sin(a), 0.0])
        end = np.array([0.0, 0.0, 0.0])
        points = [end]
        d = out * 0.55 + np.array([0.0, 0.0, 0.85])
        for _ in range(4):
            d = d + rng.normal(0.0, 0.12, 3)
            d /= np.linalg.norm(d)
            points.append(points[-1] + d * 0.22)
        points = np.array(points)
        parts.append(_tube(points, np.linspace(0.028, 0.012, len(points)), 5, BARK_LIGHT, 0.9))
        parts.append(_fronds(points[-1] + np.array([0.0, 0.0, 0.05]), np.array([0.40, 0.40, 0.28]), 60, 0.105, rng, dry=0.30))
    return _merge(parts)


# --- trees ------------------------------------------------------------------


# Colours are linear albedo, set from the reference footage (scripts/footage_palette.py): a dry-season foothill forest,
# its leaves olive and dull, its bark grey-brown, its dead leaves the colour of the straw under them.
LEAF_LIT = np.array([0.105, 0.128, 0.050])
LEAF_DEEP = np.array([0.040, 0.055, 0.024])
LEAF_DRY = np.array([0.26, 0.20, 0.09])
BARK = np.array([0.070, 0.055, 0.045])
BARK_LIGHT = np.array([0.13, 0.105, 0.085])
#: Wood is marked by normals of length two, which the instance shader reads as "do not wobble this": the wobble that
#: breaks up a crown would bend a twig.
WOOD = 2.0


def _tube(points: np.ndarray, radii: np.ndarray, sides: int, colour: np.ndarray, occlusion: float) -> tuple[np.ndarray, np.ndarray]:
    """A tapering tube along a polyline, rings joined by quads, ends left open (a branch ends in leaves or in a thinner branch)."""
    n = len(points)
    rings, normals = [], []
    angles = np.linspace(0.0, math.tau, sides, endpoint=False)
    previous_side = None
    for i in range(n):
        tangent = points[min(i + 1, n - 1)] - points[max(i - 1, 0)]
        tangent = tangent / max(float(np.linalg.norm(tangent)), 1e-9)
        helper = np.array([0.0, 0.0, 1.0]) if abs(tangent[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
        side = np.cross(tangent, helper)
        side /= max(float(np.linalg.norm(side)), 1e-9)
        if previous_side is not None and float(np.dot(side, previous_side)) < 0.0:
            side = -side
        previous_side = side
        up = np.cross(tangent, side)
        ring = np.cos(angles)[:, None] * side + np.sin(angles)[:, None] * up
        rings.append(points[i] + ring * radii[i])
        normals.append(ring)
    positions = np.concatenate(rings)
    normal = np.concatenate(normals) * WOOD
    faces = []
    for i in range(n - 1):
        for k in range(sides):
            j = (k + 1) % sides
            a, b, c, d = i * sides + k, i * sides + j, (i + 1) * sides + j, (i + 1) * sides + k
            faces += [(a, b, c), (a, c, d)]
    rng_tone = 0.85 + 0.3 * np.linspace(0.0, 1.0, n).repeat(sides)
    colours = np.tile(colour, (len(positions), 1)) * rng_tone[:, None]
    return _pack(positions, normal, colours, np.full(len(positions), occlusion)), np.array(faces, dtype=np.uint32).reshape(-1)


def _fronds(centre: np.ndarray, radii: np.ndarray, count: int, size: float, rng: np.random.Generator, dry: float = 0.0,
            top_bias: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """A cloud of small leaf sprays in a flattened ellipsoid: each a pair of triangles turned a random way, so the
    outline of the crown is ragged and the light comes through it. Lit on the outside, dark in the middle."""
    positions, normals, colours, occlusion, faces = [], [], [], [], []
    for _ in range(count):
        while True:
            q = rng.uniform(-1.0, 1.0, 3)
            if float(q @ q) <= 1.0:
                break
        p = centre + q * radii
        depth = 1.0 - float(np.linalg.norm(q))  # 0 at the surface, 1 in the middle
        facing = q / max(float(np.linalg.norm(q)), 1e-6)
        a = rng.uniform(0.0, math.tau)
        axis_u = np.array([math.cos(a), math.sin(a), rng.uniform(-0.45, 0.65)])
        axis_u /= np.linalg.norm(axis_u)
        axis_v = np.cross(axis_u, np.array([0.0, 0.0, 1.0]))
        axis_v /= max(float(np.linalg.norm(axis_v)), 1e-6)
        s = size * rng.uniform(0.7, 1.3)
        corners = [p - axis_u * s * 0.6, p + axis_v * s * 0.45, p + axis_u * s, p - axis_v * s * 0.45]
        n = facing * 0.55 + np.array([0.0, 0.0, 0.55 + top_bias]) + rng.normal(0.0, 0.18, 3)
        n /= np.linalg.norm(n)
        height_lit = float(np.clip(0.5 + 0.5 * q[2] + 0.25 * rng.normal(), 0.0, 1.0))
        is_dry = rng.uniform() < dry
        colour = LEAF_DRY * rng.uniform(0.7, 1.2) if is_dry else (LEAF_DEEP + (LEAF_LIT - LEAF_DEEP) * height_lit) * rng.uniform(0.8, 1.25)
        ao = float(np.clip(0.38 + 0.62 * (1.0 - depth) ** 0.7 + 0.15 * q[2], 0.3, 1.0))
        base = len(positions)
        for corner in corners:
            positions.append(corner)
            normals.append(n)
            colours.append(colour)
            occlusion.append(ao)
        faces += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return _pack(np.array(positions), np.array(normals), np.array(colours), np.array(occlusion)), np.array(faces, dtype=np.uint32).reshape(-1)


class _Tree:
    """Accumulates limbs and foliage for one tree."""

    def __init__(self, seed: int) -> None:
        self.rng = np.random.default_rng(seed)
        self.parts: list[tuple[np.ndarray, np.ndarray]] = []
        self.tips: list[tuple[np.ndarray, np.ndarray, float]] = []  # where a limb ended: position, direction, radius

    def limb(self, start, direction, length, radius, bend, segments=5, colour=BARK, taper=0.55) -> tuple[np.ndarray, np.ndarray]:
        rng = self.rng
        direction = np.asarray(direction, dtype=np.float64)
        direction /= np.linalg.norm(direction)
        points = [np.asarray(start, dtype=np.float64)]
        for _ in range(segments):
            direction = direction + rng.normal(0.0, bend, 3) + np.array([0.0, 0.0, 0.05])
            direction /= np.linalg.norm(direction)
            points.append(points[-1] + direction * length / segments)
        points = np.array(points)
        radii = np.linspace(radius, radius * taper, len(points))
        self.parts.append(_tube(points, radii, 6 if radius > 0.006 else 4, colour, 0.9))
        return points[-1], direction

    def grow(self, start, direction, length, radius, depth, spread, lean=0.0, fork=(2, 3)):
        """A limb, then at its end forks of shorter, thinner limbs that go out and up."""
        end, d = self.limb(start, direction, length, radius, 0.10)
        if depth == 0:
            self.tips.append((end, d, radius))
            return
        rng = self.rng
        for k in range(int(rng.integers(fork[0], fork[1] + 1))):
            a = rng.uniform(0.0, math.tau)
            out = np.array([math.cos(a), math.sin(a), 0.0])
            child = d * 0.55 + out * spread * rng.uniform(0.7, 1.2) + np.array([0.0, 0.0, 0.35])
            self.grow(end, child, length * rng.uniform(0.62, 0.8), radius * 0.62, depth - 1, spread, lean, fork)

    def finish(self) -> tuple[np.ndarray, np.ndarray]:
        return _merge(self.parts)


def acacia_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """An acacia of the kind that lines the footage's trails: height one, crown radius about 0.40, trunk on the origin.

    A slender, leaning trunk that forks low into three or four limbs, which fork again and carry thin, flat layers of
    feathery leaves at their ends: an open crown you can see sky through, not a ball."""
    tree = _Tree(seed)
    rng = tree.rng
    lean = np.array([rng.normal(0.0, 0.10), rng.normal(0.0, 0.10), 1.0])
    end, d = tree.limb([0, 0, 0], lean, 0.26, 0.024, 0.07, segments=5, taper=0.72)
    for k in range(int(rng.integers(3, 6))):
        a = rng.uniform(0.0, math.tau)
        out = np.array([math.cos(a), math.sin(a), 0.0])
        tree.grow(end, d * 0.7 + out * 0.85 + np.array([0.0, 0.0, 0.3]), 0.36, 0.0125, 2, 0.85)
    parts = tree.parts
    for tip, direction, radius in tree.tips:
        centre = tip + np.array([0.0, 0.0, 0.02])
        parts.append(_fronds(centre, np.array([0.155, 0.155, 0.045]), 46, 0.078, rng, dry=0.06))
    return _merge(parts)


def leafy_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """A rounder broadleaf of the same forest (a bauhinia, a terminalia): height one, crown radius about 0.34."""
    tree = _Tree(seed)
    rng = tree.rng
    lean = np.array([rng.normal(0.0, 0.08), rng.normal(0.0, 0.08), 1.0])
    end, d = tree.limb([0, 0, 0], lean, 0.32, 0.028, 0.06, segments=5, taper=0.72)
    for k in range(int(rng.integers(2, 4))):
        a = rng.uniform(0.0, math.tau)
        out = np.array([math.cos(a), math.sin(a), 0.0])
        tree.grow(end, d * 0.9 + out * 0.6 + np.array([0.0, 0.0, 0.25]), 0.30, 0.014, 2, 0.65)
    parts = tree.parts
    for tip, direction, radius in tree.tips:
        parts.append(_fronds(tip + np.array([0.0, 0.0, 0.04]), np.array([0.135, 0.135, 0.105]), 60, 0.074, rng, dry=0.05))
    return _merge(parts)


def snag_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """A tree with its leaves off, as many of them are in the dry season: height one, crown radius about 0.34. Fine
    bare twigs against the sky, and a few dead leaves hanging on."""
    tree = _Tree(seed)
    rng = tree.rng
    lean = np.array([rng.normal(0.0, 0.10), rng.normal(0.0, 0.10), 1.0])
    end, d = tree.limb([0, 0, 0], lean, 0.30, 0.024, 0.08, segments=5, taper=0.72)
    for k in range(int(rng.integers(3, 5))):
        a = rng.uniform(0.0, math.tau)
        out = np.array([math.cos(a), math.sin(a), 0.0])
        tree.grow(end, d * 0.75 + out * 0.85 + np.array([0.0, 0.0, 0.3]), 0.30, 0.011, 3, 0.9, fork=(2, 3))
    parts = tree.parts
    for tip, direction, radius in tree.tips[: len(tree.tips) // 3]:
        parts.append(_fronds(tip, np.array([0.05, 0.05, 0.03]), 6, 0.040, rng, dry=1.0))
    return _merge(parts)


# --- the rover ----------------------------------------------------------------


def box(centre, size, colour, occlusion=1.0) -> tuple[np.ndarray, np.ndarray]:
    cx, cy, cz = centre
    sx, sy, sz = (s / 2.0 for s in size)
    corners = np.array(
        [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
        dtype=np.float64,
    ) * np.array([sx, sy, sz]) + np.array([cx, cy, cz])
    quads = [
        ((0, 3, 2, 1), (0, 0, -1)), ((4, 5, 6, 7), (0, 0, 1)),
        ((0, 1, 5, 4), (0, -1, 0)), ((2, 3, 7, 6), (0, 1, 0)),
        ((1, 2, 6, 5), (1, 0, 0)), ((3, 0, 4, 7), (-1, 0, 0)),
    ]
    positions, normals, faces = [], [], []
    for quad, normal in quads:
        base = len(positions)
        for index in quad:
            positions.append(corners[index])
            normals.append(normal)
        faces += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    positions = np.array(positions)
    colours = np.tile(np.array(colour, dtype=np.float64), (len(positions), 1))
    return (
        _pack(positions, np.array(normals, dtype=np.float64), colours, np.full(len(positions), occlusion)),
        np.array(faces, dtype=np.uint32).reshape(-1),
    )


def wheel(centre, radius, width, colour) -> tuple[np.ndarray, np.ndarray]:
    sides = 14
    angles = np.linspace(0, math.tau, sides, endpoint=False)
    cx, cy, cz = centre
    positions, normals, faces = [], [], []
    for side in (-1, 1):
        for a in angles:
            positions.append((cx + side * width / 2, cy + math.cos(a) * radius, cz + math.sin(a) * radius))
            normals.append((side, 0, 0))
    centre_a, centre_b = len(positions), len(positions) + 1
    positions += [(cx - width / 2, cy, cz), (cx + width / 2, cy, cz)]
    normals += [(-1, 0, 0), (1, 0, 0)]
    for i in range(sides):
        j = (i + 1) % sides
        faces += [(centre_a, j, i), (centre_b, sides + i, sides + j)]
    rim_start = len(positions)
    for side in (-1, 1):
        for a in angles:
            positions.append((cx + side * width / 2, cy + math.cos(a) * radius, cz + math.sin(a) * radius))
            normals.append((0, math.cos(a), math.sin(a)))
    for i in range(sides):
        j = (i + 1) % sides
        faces += [(rim_start + i, rim_start + sides + i, rim_start + sides + j), (rim_start + i, rim_start + sides + j, rim_start + j)]
    positions = np.array(positions)
    colours = np.tile(np.array(colour, dtype=np.float64), (len(positions), 1))
    return (
        _pack(positions, np.array(normals, dtype=np.float64), colours, np.full(len(positions), 0.8)),
        np.array(faces, dtype=np.uint32).reshape(-1),
    )


def rover_mesh(length: float = 1.15, width: float = 0.85, height: float = 0.8) -> tuple[np.ndarray, np.ndarray]:
    """The ground vehicle, facing +y: a chassis, a sensor deck, a mast and six wheels."""
    parts = [
        box((0, 0, 0.36), (width * 0.82, length * 0.9, 0.26), (0.20, 0.23, 0.27)),
        box((0, 0, 0.55), (width * 0.74, length * 0.74, 0.16), (0.78, 0.80, 0.82)),  # light deck
        box((0, 0.30, 0.60), (width * 0.5, 0.2, 0.1), (0.10, 0.11, 0.13)),  # sensor head
        box((0, -0.12, 0.76), (0.04, 0.04, 0.42), (0.16, 0.17, 0.19)),  # mast
        box((0, -0.12, 1.00), (0.14, 0.08, 0.08), (0.10, 0.11, 0.13)),
        box((0, -0.05, 0.64), (width * 0.66, length * 0.46, 0.03), (0.12, 0.16, 0.24)),  # panel
    ]
    for y in (-0.38, 0.0, 0.38):
        for side in (-1, 1):
            parts.append(wheel((side * (width * 0.5 + 0.01), y, 0.17), 0.17, 0.12, (0.07, 0.07, 0.08)))
    return _merge(parts)


# --- grass ------------------------------------------------------------------


def grass_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """A tussock of dry-season grass (kans, the tall grass that closes in over the footage's trails): height one,
    footprint radius about 0.5.

    Many long narrow blades that rise from a tight base and arch outward, olive-green at the root and straw to pale at
    the tip, a few stiff stems standing above them with a pale plume. Narrow, because real grass blades are."""
    rng = np.random.default_rng(seed)
    positions, normals, colours, occlusion, faces = [], [], [], [], []
    blades = int(rng.integers(34, 48))
    for _ in range(blades):
        angle = rng.uniform(0.0, math.tau)
        reach = rng.uniform(0.0, 0.10)
        base = np.array([math.cos(angle), math.sin(angle), 0.0]) * reach
        lean = np.array([math.cos(angle + rng.normal(0.0, 0.35)), math.sin(angle + rng.normal(0.0, 0.35)), 0.0])
        height = rng.uniform(0.55, 1.0)
        bend = rng.uniform(0.25, 0.75)  # how far the tip falls outward, as a fraction of height
        width = rng.uniform(0.020, 0.034)
        across = np.array([-lean[1], lean[0], 0.0])
        first = len(positions)
        levels = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
        for t in levels:
            centre = base + lean * (bend * t * t * height) + np.array([0.0, 0.0, t * height * (1.0 - 0.25 * bend * t)])
            w = width * (1.0 - t * 0.95)
            for side in (-1, 1) if t < 1.0 else (0,):
                positions.append(centre + across * w * side)
                normals.append(np.array([lean[0] * 0.5, lean[1] * 0.5, 0.85]))
                green = np.array([0.050, 0.060, 0.024])
                straw = np.array([0.205, 0.158, 0.068])
                pale = np.array([0.290, 0.245, 0.135])
                colours.append(green * (1 - t) ** 1.5 + straw * (1 - abs(2 * t - 1.0)) * 0.9 + pale * t ** 2.0 * 0.8 + straw * 0.22)
                occlusion.append(0.35 + 0.65 * t)
        for k in range(len(levels) - 2):
            a, b = first + 2 * k, first + 2 * k + 1
            c, d = first + 2 * k + 2, first + 2 * k + 3
            faces += [(a, b, d), (a, d, c)]
        faces += [(first + 2 * (len(levels) - 2), first + 2 * (len(levels) - 2) + 1, first + 2 * (len(levels) - 1))]
    for _ in range(int(rng.integers(2, 5))):  # flowering stems
        angle = rng.uniform(0.0, math.tau)
        top = np.array([math.cos(angle), math.sin(angle), 0.0]) * rng.uniform(0.05, 0.25) + np.array([0.0, 0.0, rng.uniform(1.0, 1.3)])
        first = len(positions)
        for t in (0.0, 0.6, 1.0):
            centre = np.array([0.0, 0.0, 0.0]) * (1 - t) + top * t
            positions += [centre + np.array([0.008, 0.0, 0.0]) * (1 - 0.5 * t), centre - np.array([0.008, 0.0, 0.0]) * (1 - 0.5 * t)]
            normals += [np.array([0.0, 0.0, 1.0])] * 2
            colours += [np.array([0.22, 0.18, 0.10])] * 2
            occlusion += [0.9, 0.9]
        faces += [(first, first + 1, first + 3), (first, first + 3, first + 2), (first + 2, first + 3, first + 5), (first + 2, first + 5, first + 4)]
        # the plume: a soft fan of pale strands at the top
        for k in range(5):
            a = angle + (k - 2) * 0.35
            tip = top + np.array([math.cos(a) * 0.035, math.sin(a) * 0.035, 0.11])
            i0 = len(positions)
            positions += [top + np.array([0.005, 0.0, 0.0]), top - np.array([0.005, 0.0, 0.0]), tip]
            normals += [np.array([0.0, 0.0, 1.0])] * 3
            colours += [np.array([0.34, 0.31, 0.23])] * 3
            occlusion += [0.9, 0.95, 1.0]
            faces += [(i0, i0 + 1, i0 + 2)]
    positions = np.array(positions)
    normals = np.array(normals)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    return _pack(positions, normals, np.array(colours), np.array(occlusion)), np.array(faces, dtype=np.uint32).reshape(-1)
