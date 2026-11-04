"""Loading the Poly Haven models and textures, and making them cheap enough to
instance hundreds of times.

The files are photoscanned or sculpted at tens of thousands of triangles each,
because that is what they are for. A boulder here is drawn a few hundred times a
frame, so each model is simplified (with its texture coordinates and normals
weighed in, so the map still lies on it) to a near and a far version, once, and
the result is cached beside the download.

A downloaded model is data from outside, so it is read defensively: every
accessor is range-checked against its buffer, nothing is executed, and a file
that is not what it says is rejected, not trusted.

No Qt import belongs in this package.
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent / "assets" / "polyhaven"
#: What was made from the reference footage (scripts/footage_textures.py): the user's own ground, kept in the repository.
FOOTAGE = Path(__file__).resolve().parent.parent.parent / "assets" / "footage"

_COMPONENT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


class AssetError(Exception):
    pass


@dataclass
class Mesh:
    positions: np.ndarray  # (n, 3) float32, z up
    normals: np.ndarray  # (n, 3)
    uvs: np.ndarray  # (n, 2)
    indices: np.ndarray  # (m,) uint32

    @property
    def triangles(self) -> int:
        return len(self.indices) // 3


@dataclass
class Variant:
    """One drawable thing from a model, centred on its footprint, resting on z = 0."""

    name: str
    lods: list[Mesh]  # near first
    half_extent: tuple[float, float]  # half the width and depth of the unscaled mesh
    height: float


@dataclass
class Textures:
    albedo: np.ndarray  # (h, w, 3) uint8
    normal: np.ndarray  # (h, w, 3) uint8, OpenGL convention
    arm: np.ndarray  # (h, w, 3) uint8: occlusion, roughness, metal


@dataclass
class Model:
    name: str
    variants: list[Variant] = field(default_factory=list)
    textures: Textures | None = None


# --- glTF ---------------------------------------------------------------------


def _accessor(gltf: dict, buffers: list[bytes], index: int) -> np.ndarray:
    accessor = gltf["accessors"][index]
    view = gltf["bufferViews"][accessor["bufferView"]]
    data = buffers[view["buffer"]]
    dtype = np.dtype(_COMPONENT[accessor["componentType"]])
    width = _WIDTH[accessor["type"]]
    count = int(accessor["count"])
    start = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    stride = int(view.get("byteStride", 0)) or dtype.itemsize * width
    last = start + stride * (count - 1) + dtype.itemsize * width
    if count < 0 or start < 0 or last > len(data):
        raise AssetError("accessor reads outside its buffer")
    if stride == dtype.itemsize * width:
        return np.frombuffer(data, dtype=dtype, count=count * width, offset=start).reshape(count, width).copy()
    out = np.empty((count, width), dtype=dtype)
    for i in range(count):
        out[i] = np.frombuffer(data, dtype=dtype, count=width, offset=start + i * stride)
    return out


def _node_matrix(node: dict) -> np.ndarray:
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    t = np.array(node.get("translation", (0, 0, 0)), dtype=np.float64)
    q = np.array(node.get("rotation", (0, 0, 0, 1)), dtype=np.float64)
    s = np.array(node.get("scale", (1, 1, 1)), dtype=np.float64)
    x, y, z, w = q
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    m = np.eye(4)
    m[:3, :3] = r * s
    m[:3, 3] = t
    return m


def _to_z_up(v: np.ndarray) -> np.ndarray:
    """glTF is y-up and right-handed; the world is z-up."""
    return np.stack([v[:, 0], -v[:, 2], v[:, 1]], axis=1)


def _read_gltf(path: Path) -> list[tuple[str, Mesh]]:
    gltf = json.loads(path.read_text())
    if gltf.get("asset", {}).get("version", "").split(".")[0] != "2":
        raise AssetError("not a glTF 2 file")
    buffers = []
    for entry in gltf["buffers"]:
        uri = entry["uri"]
        target = (path.parent / uri).resolve()
        if path.parent.resolve() not in target.parents:
            raise AssetError("buffer outside the asset folder")
        data = target.read_bytes()
        if len(data) < entry["byteLength"]:
            raise AssetError("buffer shorter than it says")
        buffers.append(data)

    out = []
    for node in gltf["nodes"]:
        if "mesh" not in node:
            continue
        matrix = _node_matrix(node)
        merged = []
        for primitive in gltf["meshes"][node["mesh"]]["primitives"]:
            attributes = primitive["attributes"]
            pos = _accessor(gltf, buffers, attributes["POSITION"]).astype(np.float64)
            nor = _accessor(gltf, buffers, attributes["NORMAL"]).astype(np.float64)
            uv = _accessor(gltf, buffers, attributes["TEXCOORD_0"]).astype(np.float32)
            idx = _accessor(gltf, buffers, primitive["indices"]).reshape(-1).astype(np.uint32)
            if idx.size and int(idx.max()) >= len(pos):
                raise AssetError("index past the end of its vertices")
            pos = (np.c_[pos, np.ones(len(pos))] @ matrix.T)[:, :3]
            nor = nor @ matrix[:3, :3].T
            nor /= np.maximum(np.linalg.norm(nor, axis=1, keepdims=True), 1e-9)
            merged.append((_to_z_up(pos).astype(np.float32), _to_z_up(nor).astype(np.float32), uv, idx))
        offset, parts = 0, []
        for pos, nor, uv, idx in merged:
            parts.append((pos, nor, uv, idx + offset))
            offset += len(pos)
        mesh = Mesh(
            np.concatenate([p[0] for p in parts]),
            np.concatenate([p[1] for p in parts]),
            np.concatenate([p[2] for p in parts]),
            np.concatenate([p[3] for p in parts]).astype(np.uint32),
        )
        out.append((node.get("name", "node"), mesh))
    return out


# --- simplification --------------------------------------------------------------


def weld(mesh: Mesh) -> Mesh:
    """Merge vertices that are the same point with the same normal and texture
    coordinate. A mesh stored as loose triangles cannot be simplified: no edge
    is shared, so none can collapse."""
    key = np.concatenate([mesh.positions, mesh.normals, mesh.uvs], axis=1)
    key = np.round(key, 5)
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inverse = inverse.reshape(-1)
    return Mesh(mesh.positions[first], mesh.normals[first], mesh.uvs[first], inverse[mesh.indices].astype(np.uint32))


def simplify(mesh: Mesh, target_triangles: int) -> Mesh:
    """Fewer triangles, keeping the silhouette and the texture on it."""
    import ctypes

    import meshoptimizer as mo
    from meshoptimizer import simplifier

    if mesh.triangles <= target_triangles:
        return mesh
    mesh = weld(mesh)
    # The binding does not declare this function's argument types, so a float
    # error bound cannot be passed to it; say what they are.
    fn = simplifier.lib.meshopt_simplifyWithAttributes
    fn.argtypes = [
        ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_float), ctypes.c_size_t, ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_float), ctypes.c_size_t, ctypes.POINTER(ctypes.c_float), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_ubyte), ctypes.c_size_t, ctypes.c_float, ctypes.c_uint,
        ctypes.POINTER(ctypes.c_float),
    ]
    fn.restype = ctypes.c_size_t
    attributes = np.concatenate([mesh.normals, mesh.uvs], axis=1).astype(np.float32)
    weights = np.array([0.4, 0.4, 0.4, 1.2, 1.2], dtype=np.float32)
    destination = np.zeros(len(mesh.indices), dtype=np.uint32)
    count = mo.simplify_with_attributes(
        destination,
        mesh.indices,
        np.ascontiguousarray(mesh.positions, dtype=np.float32),
        attributes,
        weights,
        target_index_count=int(target_triangles) * 3,
        target_error=0.05,
        options=mo.SIMPLIFY_SPARSE,
    )
    if count > target_triangles * 3 * 1.6:
        # The texture seams held it up (a model unwrapped into many small
        # islands cannot lose triangles along them). Let the shape lead.
        count = mo.simplify_with_attributes(
            destination, mesh.indices, np.ascontiguousarray(mesh.positions, dtype=np.float32), attributes,
            np.array([0.05, 0.05, 0.05, 0.15, 0.15], dtype=np.float32),
            target_index_count=int(target_triangles) * 3, target_error=0.4, options=mo.SIMPLIFY_SPARSE,
        )
    kept = destination[:count]
    used, inverse = np.unique(kept, return_inverse=True)
    return Mesh(mesh.positions[used], mesh.normals[used], mesh.uvs[used], inverse.astype(np.uint32))


def _rest_on_ground(mesh: Mesh, centre_xy: np.ndarray, bottom: float, burial: float) -> Mesh:
    shift = np.array([centre_xy[0], centre_xy[1], bottom + burial], dtype=np.float32)
    return Mesh(mesh.positions - shift, mesh.normals, mesh.uvs, mesh.indices)


# --- models ---------------------------------------------------------------------


def _load_image(path: Path, size: int = 1024) -> np.ndarray:
    from PIL import Image

    with Image.open(path) as image:
        image = image.convert("RGB")
        if image.size != (size, size):
            image = image.resize((size, size), Image.LANCZOS)
        return np.asarray(image, dtype=np.uint8).copy()


#: The stone in the footage is grey-tan: river cobbles in the cut banks, pale weathered sandstone, nothing orange. The
#: boulders downloaded for this scene are lichen-covered South African granite, so their colour is taken out and
#: replaced, keeping the grain (brightness variation) of the photographs.
STONE_TINT = np.array([1.00, 0.91, 0.78], dtype=np.float32)
STONE_CHROMA_KEPT = 0.10
STONE_LEVEL = 0.92


def grade_stone(albedo: np.ndarray) -> np.ndarray:
    f = albedo.astype(np.float32) / 255.0
    lum = f @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
    grey = lum[..., None] * STONE_TINT * STONE_LEVEL
    graded = grey * (1.0 - STONE_CHROMA_KEPT) + f * STONE_CHROMA_KEPT
    return (np.clip(graded, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


def load_textures(model_dir: Path, name: str) -> Textures:
    base = model_dir / "textures"
    albedo = _load_image(base / f"{name}_diff_1k.jpg")
    if footage_ground_available() and PLAN.get(name, ("",))[0] in ("rock", "stone"):
        albedo = grade_stone(albedo)
    return Textures(
        albedo=albedo,
        normal=_load_image(base / f"{name}_nor_gl_1k.jpg"),
        arm=_load_image(base / f"{name}_arm_1k.jpg"),
    )


#: What each downloaded model is used for, and how far to simplify it:
#: (near triangles, far triangles, how much of it is buried, how many of its
#: parts to use).
PLAN = {
    "namaqualand_boulder_02": ("rock", 5500, 900, 0.10, 1),
    "namaqualand_boulder_03": ("rock", 5500, 900, 0.10, 1),
    "namaqualand_boulder_04": ("rock", 5500, 900, 0.10, 1),
    "namaqualand_boulder_05": ("rock", 5500, 900, 0.10, 1),
    "namaqualand_boulder_06": ("rock", 5500, 900, 0.10, 1),
    "namaqualand_rocks_01": ("rock", 4500, 700, 0.10, 4),
    "namaqualand_stones_01": ("stone", 500, 140, 0.20, 5),
}
# Not used any more: the fynbos shrub and the green grass clumps (othonna_cerarioides, grass_medium_01/02,
# grass_bermuda_01) do not belong in a dry foothill forest. The footage's kans tussocks and dry shrubs are built in
# render/meshes.py, which is what the renderer falls back to for any kind with no downloaded model.


def _cache_path(name: str) -> Path:
    return ROOT / "cache" / f"{name}.npz"


def _cache_key(name: str, gltf_path: Path) -> str:
    return f"{PLAN[name]}|{gltf_path.stat().st_size}|{int(gltf_path.stat().st_mtime)}|v1"


def _read_cache(name: str, key: str) -> list[Variant] | None:
    path = _cache_path(name)
    if not path.exists():
        return None
    try:
        with np.load(path, allow_pickle=False) as data:
            if str(data["key"]) != key:
                return None
            variants = []
            for i in range(int(data["count"])):
                lods = []
                for lod in range(2):
                    prefix = f"v{i}_l{lod}_"
                    lods.append(Mesh(data[prefix + "p"], data[prefix + "n"], data[prefix + "u"], data[prefix + "i"]))
                extent = data[f"v{i}_extent"]
                variants.append(Variant(str(data[f"v{i}_name"]), lods, (float(extent[0]), float(extent[1])), float(data[f"v{i}_height"])))
            return variants
    except (OSError, ValueError, KeyError):
        return None


def _write_cache(name: str, key: str, variants: list[Variant]) -> None:
    arrays: dict[str, np.ndarray] = {"key": np.array(key), "count": np.array(len(variants))}
    for i, variant in enumerate(variants):
        arrays[f"v{i}_name"] = np.array(variant.name)
        arrays[f"v{i}_extent"] = np.array(variant.half_extent, dtype=np.float64)
        arrays[f"v{i}_height"] = np.array(variant.height)
        for lod, mesh in enumerate(variant.lods):
            prefix = f"v{i}_l{lod}_"
            arrays[prefix + "p"], arrays[prefix + "n"] = mesh.positions, mesh.normals
            arrays[prefix + "u"], arrays[prefix + "i"] = mesh.uvs, mesh.indices
    try:
        _cache_path(name).parent.mkdir(parents=True, exist_ok=True)
        np.savez(_cache_path(name), **arrays)
    except OSError:
        pass  # a cache that cannot be written only costs time


def load_model(name: str) -> Model | None:
    """The model ready to draw, or None if it was not downloaded or is not what it claims.

    The simplified meshes are cached beside the download, since simplifying is
    most of what loading costs and the answer does not change.
    """
    directory = ROOT / "models" / name
    gltf_path = directory / f"{name}_1k.gltf"
    if not gltf_path.exists() or name not in PLAN:
        return None
    _kind, near_tris, far_tris, burial, parts = PLAN[name]
    try:
        textures = load_textures(directory, name)
        key = _cache_key(name, gltf_path)
        cached = _read_cache(name, key)
        if cached is not None:
            return Model(name=name, variants=cached, textures=textures)
        meshes = _read_gltf(gltf_path)[:parts]
    except (AssetError, KeyError, OSError, ValueError, json.JSONDecodeError):
        return None
    model = Model(name=name, textures=textures)
    for part_name, mesh in meshes:
        low, high = mesh.positions.min(axis=0), mesh.positions.max(axis=0)
        centre = (low + high) / 2.0
        height = float(high[2] - low[2])
        lods = [simplify(mesh, near_tris), simplify(mesh, far_tris)]
        lods = [_rest_on_ground(m, centre[:2], float(low[2]), height * burial) for m in lods]
        model.variants.append(
            Variant(part_name, lods, (float(high[0] - low[0]) / 2.0, float(high[1] - low[1]) / 2.0), height * (1.0 - burial))
        )
    _write_cache(name, key, model.variants)
    return model


GROUND_ROLES = ("ground", "scrub", "track", "dark", "slope")


def footage_ground_available() -> bool:
    return all((FOOTAGE / "ground" / f"{role}_{kind}.jpg").exists() for role in GROUND_ROLES for kind in ("albedo", "normal"))


def load_footage_ground() -> dict[str, Textures] | None:
    """The terrain's textures as made from the reference footage, or None if they have not been made."""
    if not footage_ground_available():
        return None
    out = {}
    try:
        for role in GROUND_ROLES:
            albedo = _load_image(FOOTAGE / "ground" / f"{role}_albedo.jpg")
            normal = _load_image(FOOTAGE / "ground" / f"{role}_normal.jpg")
            arm = np.full_like(albedo, (255, 230, 0))  # occlusion, roughness, metal: dry ground, not shiny; the terrain does not read it
            out[role] = Textures(albedo, normal, arm)
    except OSError:
        return None
    return out


def load_ground_textures() -> dict[str, Textures] | None:
    """The terrain's textures, keyed by what they are for: the footage's if made, otherwise Poly Haven's."""
    footage = load_footage_ground()
    if footage is not None:
        return footage
    wanted = {
        "ground": "dry_ground_01",
        "scrub": "aerial_grass_rock",
        "track": "gravelly_sand",
        "dark": "brown_mud_dry",
        "slope": "rocky_terrain_02",
    }
    out = {}
    for role, folder in wanted.items():
        base = ROOT / "textures" / folder
        try:
            diffuse = next(base.glob("Diffuse.*"))
            normal = next(base.glob("nor_gl.*"))
            rough = next(base.glob("Rough.*"))
        except StopIteration:
            return None
        try:
            out[role] = Textures(_load_image(diffuse), _load_image(normal), _load_image(rough))
        except OSError:
            return None
    return out


def available() -> bool:
    return (ROOT / "models").exists() and (ROOT / "textures").exists()


def ground_is_footage() -> bool:
    return footage_ground_available()
