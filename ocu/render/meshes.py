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
    """A shrub: footprint radius one, about 0.9 tall, standing on the origin.

    A clump of overlapping leafy lobes. Each is a displaced sphere, darker
    toward its inside and underneath, so the mass reads as foliage with depth
    and not as one green ball.
    """
    rng = np.random.default_rng(seed)
    parts = []
    lobes = int(rng.integers(5, 8))
    for k in range(lobes):
        radius = rng.uniform(0.38, 0.62)
        angle = rng.uniform(0, math.tau)
        spread = 0.52 if k else 0.0
        centre = np.array(
            [math.cos(angle) * spread * rng.uniform(0.5, 1.0), math.sin(angle) * spread * rng.uniform(0.5, 1.0), radius * 0.72 + rng.uniform(0, 0.18)]
        )
        d, faces = icosphere(2)
        wobble = 1.0 + 0.30 * (fbm3(d * 3.2 + k * 3.7, seed + k, 3) - 0.5) + 0.12 * (fbm3(d * 9.0, seed + 5 + k, 2) - 0.5)
        positions = d * radius * wobble[:, None] * np.array([1.0, 1.0, 0.86]) + centre
        normals = vertex_normals(positions, faces)
        # Leaves face outward and up, which is what a crown catches light on.
        normals = normals + np.array([0.0, 0.0, 0.35])
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        top = np.clip((positions[:, 2] - 0.1) / 0.9, 0.0, 1.0)
        tone = 0.70 + 0.55 * fbm3(positions * 4.0, seed + 9 + k, 3)
        colours = np.stack([0.24 * tone, 0.31 * tone, 0.10 * tone], axis=1)
        occlusion = np.clip(0.42 + 0.72 * top, 0.3, 1.0)
        # Cut off below the ground so nothing hangs under it.
        keep = np.ones(len(positions), dtype=bool)
        parts.append((_pack(positions, normals, colours, occlusion), faces.reshape(-1).astype(np.uint32)))
    vertices, indices = _merge(parts)
    return vertices, indices


# --- trees ------------------------------------------------------------------


def _cylinder(radius_bottom, radius_top, height, sides, z0=0.0) -> tuple[np.ndarray, np.ndarray]:
    angles = np.linspace(0, math.tau, sides, endpoint=False)
    cos, sin = np.cos(angles), np.sin(angles)
    bottom = np.stack([cos * radius_bottom, sin * radius_bottom, np.full(sides, z0)], axis=1)
    top = np.stack([cos * radius_top, sin * radius_top, np.full(sides, z0 + height)], axis=1)
    positions = np.concatenate([bottom, top])
    faces = []
    for i in range(sides):
        j = (i + 1) % sides
        faces += [(i, j, sides + j), (i, sides + j, sides + i)]
    faces = np.array(faces, dtype=np.uint32)
    normals = np.concatenate([np.stack([cos, sin, np.zeros(sides)], axis=1)] * 2)
    return positions, faces, normals


def conifer_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """A pine or fir: height one, crown radius about 0.30, trunk on the origin.

    A trunk with whorls of boughs up it, each bough a drooping fan of needles,
    long and low on the tree and short at the top. Built from boughs, not from
    cones, so the outline is ragged the way a conifer's is and the light gets
    in between them.
    """
    rng = np.random.default_rng(seed)
    parts = []

    trunk_positions, trunk_faces, trunk_normals = _cylinder(0.030, 0.006, 1.0, 7)
    trunk_colour = np.tile(np.array([0.27, 0.19, 0.12]), (len(trunk_positions), 1))
    parts.append(
        (_pack(trunk_positions, trunk_normals, trunk_colour, np.full(len(trunk_positions), 0.8)), trunk_faces.reshape(-1))
    )

    positions, normals, colours, occlusion, faces = [], [], [], [], []
    whorls = 30
    first_whorl = 0.15
    for w in range(whorls):
        f = w / (whorls - 1)
        z = first_whorl + f * (0.985 - first_whorl)
        length = 0.34 * (1.0 - f) ** 0.85 + 0.03
        boughs = max(4, int(round(8 - 4 * f)))
        offset = rng.uniform(0, math.tau)
        for k in range(boughs):
            angle = offset + k * math.tau / boughs + rng.normal(0, 0.18)
            L = length * rng.uniform(0.78, 1.14)
            droop = L * rng.uniform(0.18, 0.36)
            width = max(L * 0.34, 0.022)
            direction = np.array([math.cos(angle), math.sin(angle), 0.0])
            across = np.array([-direction[1], direction[0], 0.0])
            base_index = len(positions)
            levels = (0.0, 0.33, 0.66, 1.0)
            for t in levels:
                centre = direction * (L * t) + np.array([0.0, 0.0, z - droop * t * t + 0.02 * (1 - t)])
                half = width * (np.sin(math.pi * min(t * 0.85 + 0.1, 1.0)) * 0.9 + 0.1) * (1.0 - 0.55 * t)
                tint = 0.75 + 0.5 * t
                shade = 0.30 + 0.70 * t  # the inner boughs are in shadow
                for side in (-1, 1) if t < 1.0 else (0,):
                    positions.append(centre + across * half * side)
                    n = np.array([direction[0] * 0.25, direction[1] * 0.25, 0.92])
                    normals.append(n / np.linalg.norm(n))
                    green = np.array([0.15, 0.27, 0.13]) * (0.8 + 0.4 * rng.uniform())
                    colours.append(green * tint)
                    occlusion.append(shade * (0.62 + 0.38 * f))
            for q in range(3):
                a, b = base_index + 2 * q, base_index + 2 * q + 1
                if q < 2:
                    c, d = base_index + 2 * q + 2, base_index + 2 * q + 3
                    faces += [(a, b, d), (a, d, c)]
                else:
                    faces += [(a, b, base_index + 6)]
    parts.append(
        (
            _pack(np.array(positions), np.array(normals), np.array(colours), np.array(occlusion)),
            np.array(faces, dtype=np.uint32).reshape(-1),
        )
    )
    return _merge(parts)


def broadleaf_template(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """A broadleaf: height one, crown radius about 0.34, trunk on the origin."""
    rng = np.random.default_rng(seed)
    parts = []
    trunk_positions, trunk_faces, trunk_normals = _cylinder(0.045, 0.02, 0.5, 8)
    trunk_colour = np.tile(np.array([0.34, 0.25, 0.17]), (len(trunk_positions), 1))
    parts.append(
        (_pack(trunk_positions, trunk_normals, trunk_colour, np.full(len(trunk_positions), 0.85)), trunk_faces.reshape(-1))
    )

    clusters = int(rng.integers(6, 10))
    for k in range(clusters):
        angle = rng.uniform(0, math.tau)
        spread = rng.uniform(0.0, 0.22) if k else 0.0
        centre = np.array([math.cos(angle) * spread, math.sin(angle) * spread, rng.uniform(0.52, 0.80)])
        radius = rng.uniform(0.17, 0.26)
        d, faces = icosphere(2)
        wobble = 1.0 + 0.34 * (fbm3(d * 3.0 + k * 2.9, seed + k, 3) - 0.5) + 0.16 * (fbm3(d * 8.0, seed + 3 + k, 2) - 0.5)
        positions = d * radius * wobble[:, None] * np.array([1.0, 1.0, 0.82]) + centre
        normals = vertex_normals(positions, faces)
        normals = normals + np.array([0.0, 0.0, 0.3])
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        top = np.clip((positions[:, 2] - 0.45) / 0.5, 0.0, 1.0)
        shade = 0.62 + 0.6 * fbm3(positions * 5.5, seed + 5 + k, 3)
        green = np.array([0.24, 0.36, 0.14])
        colours = np.tile(green, (len(positions), 1)) * shade[:, None]
        occlusion = np.clip(0.34 + 0.78 * top + 0.2 * np.clip(d[:, 2], 0, 1), 0.28, 1.0)
        parts.append((_pack(positions, normals, colours, occlusion), faces.reshape(-1).astype(np.uint32)))
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
    """A tuft: height one, a handful of curved blades fanning from the origin."""
    rng = np.random.default_rng(seed)
    positions, normals, colours, occlusion, faces = [], [], [], [], []
    blades = int(rng.integers(8, 12))
    for _ in range(blades):
        angle = rng.uniform(0, math.tau)
        base = np.array([math.cos(angle), math.sin(angle), 0.0]) * rng.uniform(0.0, 0.09)
        lean = np.array([math.cos(angle + rng.normal(0, 0.5)), math.sin(angle + rng.normal(0, 0.5)), 0.0])
        height = rng.uniform(0.55, 1.0)
        bend = rng.uniform(0.15, 0.55)
        width = rng.uniform(0.030, 0.045)
        across = np.array([-lean[1], lean[0], 0.0])
        first = len(positions)
        levels = (0.0, 0.4, 0.75, 1.0)
        for k, t in enumerate(levels):
            centre = base + lean * (bend * t * t * height) + np.array([0.0, 0.0, t * height])
            w = width * (1.0 - t * 0.92)
            tint = 0.30 + 0.70 * t
            for side in (-1, 1) if t < 1.0 else (0,):
                positions.append(centre + across * w * side)
                normals.append(np.array([lean[0] * 0.5, lean[1] * 0.5, 0.85]))
                colours.append(np.array([0.22, 0.25, 0.10]) * (1 - t) + np.array([0.70, 0.66, 0.34]) * t)
                occlusion.append(0.45 + 0.55 * t)
        # Strip: three quads then the tip triangle.
        for k in range(3):
            a, b = first + 2 * k, first + 2 * k + 1
            if k < 2:
                c, d = first + 2 * k + 2, first + 2 * k + 3
                faces += [(a, b, d), (a, d, c)]
            else:
                tip = first + 6
                faces += [(a, b, tip)]
    positions = np.array(positions)
    normals = np.array(normals)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    return _pack(positions, normals, np.array(colours), np.array(occlusion)), np.array(faces, dtype=np.uint32).reshape(-1)
