"""Make the renderer's ground textures from the reference footage.

    .venv/bin/python scripts/footage_textures.py            # -> assets/footage/ground/<role>_albedo.jpg, <role>_normal.jpg

For each of the five ground roles the renderer uses (ground, scrub, track, dark, slope), a patch of real ground is cut
out of a clip at the clip's own resolution and turned into a tile that repeats without a seam:

  1. Un-lit. A photograph carries the light of the moment: a shadow of a branch, the sun on one side. The picture is
     divided by its own slow variation in brightness (a wide Gaussian), so that what is left is the ground's colour
     and grain, which the renderer then lights for itself.
  2. Made to tile. The patch is blended with itself shifted by half a tile, with a weight that is one at the middle and
     zero at the edges, so there is no seam. A plain average of two unrelated crops loses contrast in the middle;
     this one takes the means out first and puts the variance back (after Heitz and Neyret's histogram-preserving
     blending), so the join keeps the grain of the rest.
  3. Given a surface. A normal map comes from the high-pass of the same brightness (dark is low, light is high), at a
     strength chosen by eye. It is a stand-in for a scan: the footage has no depth, and says so in DECISIONS.md.

What goes in is the user's own footage; nothing is downloaded. The patches are chosen by eye from the footage
(`SOURCES`), because no colour rule tells a good bit of ground from a good bit of grass.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import footage_ingest as fi  # noqa: E402

OUT = fi.ROOT / "assets" / "footage" / "ground"
TILE = 1024


@dataclass(frozen=True)
class Source:
    role: str
    clip: str
    t: float
    box: tuple[float, float, float, float]  # x0, y0, x1, y1 as fractions of the upright frame
    upright: bool = True
    normal_strength: float = 2.0
    delight_sigma: float = 0.12  # as a fraction of the patch: how slowly a brightness has to vary to be called light
    mean_srgb: tuple[int, int, int] = (160, 130, 100)  # what the finished tile averages to


#: What each role's tile averages to, as a display colour. The footage's own measured colours (footage_palette.py) are
#: the sunlit trail, sunlit straw and so on; a tile is the ground *un-lit*, so these are those colours less the
#: shadows and shade between the stems and stones that the patch carries.
SOURCES = [
    Source("track", "IMG_5108", 169.5, (0.25, 0.60, 0.65, 1.00), normal_strength=3.0, mean_srgb=(186, 156, 118)),
    Source("ground", "IMG_5132", 43.5, (0.40, 0.62, 0.62, 1.00), normal_strength=2.0, mean_srgb=(176, 146, 112)),
    Source("scrub", "IMG_5132", 13.5, (0.30, 0.60, 0.70, 1.00), normal_strength=2.5, mean_srgb=(150, 126, 88)),
    Source("dark", "IMG_5129", 8.0, (0.20, 0.62, 0.45, 1.00), normal_strength=2.5, mean_srgb=(112, 92, 70)),
    Source("slope", "IMG_5109", 43.5, (0.35, 0.50, 0.80, 1.00), normal_strength=3.5, mean_srgb=(132, 110, 92)),
]


def read_patch(source: Source) -> np.ndarray:
    """The source's patch at full resolution, the right way up."""
    clip = fi.probe(fi.FOOTAGE / f"{source.clip}.MOV")
    x0, y0, x1, y1 = source.box
    if source.upright:
        return fi.read_frame(clip, source.t, crop=(x0, y0, x1, y1))
    # the phone was upside down: ask for the same patch in the frame as stored, then turn it
    raw = fi.read_frame(clip, source.t, crop=(1 - x1, 1 - y1, 1 - x0, 1 - y0))
    return raw[::-1, ::-1].copy()


def to_linear(rgb8: np.ndarray) -> np.ndarray:
    return np.power(rgb8.astype(np.float32) / 255.0, 2.2)


def to_srgb8(linear: np.ndarray) -> np.ndarray:
    return (np.clip(np.power(np.clip(linear, 0.0, 1.0), 1 / 2.2), 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


def luminance(linear: np.ndarray) -> np.ndarray:
    return linear @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)


def delight(linear: np.ndarray, sigma_fraction: float) -> np.ndarray:
    """Divide out the slow variation in brightness. The ratio is limited so that a hard shadow edge does not become a ridge."""
    lum = luminance(linear)
    sigma = sigma_fraction * min(lum.shape)
    low = ndimage.gaussian_filter(lum, sigma, mode="reflect")
    ratio = np.clip(low / max(float(np.median(low)), 1e-6), 0.35, 2.5)
    return linear / ratio[..., None]


def centre_square(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    s = min(h, w)
    y, x = (h - s) // 2, (w - s) // 2
    return img[y:y + s, x:x + s]


def resize(img: np.ndarray, size: int) -> np.ndarray:
    channels = [np.array(Image.fromarray(img[..., c]).resize((size, size), Image.LANCZOS)) for c in range(img.shape[2])]
    return np.stack(channels, axis=2)


def make_tileable(img: np.ndarray) -> np.ndarray:
    """Variance-preserving blend of the image with itself shifted by half a tile (see the module docstring)."""
    n = img.shape[0]
    shifted = np.roll(np.roll(img, n // 2, axis=0), n // 2, axis=1)
    ax = np.abs(np.linspace(-1.0, 1.0, n))
    edge = np.maximum(ax[:, None], ax[None, :])  # 0 in the middle, 1 at the border
    w = np.clip(1.0 - edge, 0.0, 1.0)
    w = w * w * (3.0 - 2.0 * w)  # smoothstep: 1 in the middle, 0 at the border
    w = w[..., None]
    mean = img.mean(axis=(0, 1), keepdims=True)
    blended = w * (img - mean) + (1.0 - w) * (shifted - mean)
    # the blend of two unrelated crops has variance (w^2 + (1-w)^2) of one crop; restore it
    blended /= np.sqrt(w * w + (1.0 - w) * (1.0 - w))
    return blended + mean


def normal_map(lum: np.ndarray, strength: float) -> np.ndarray:
    """OpenGL-convention tangent-space normal from a brightness image, dark = low. Wraps at the edges so the map tiles."""
    high = lum - ndimage.gaussian_filter(lum, 24.0, mode="wrap")
    high = ndimage.gaussian_filter(high, 0.8, mode="wrap")
    scale = strength / max(float(np.std(high)), 1e-6) * 0.18
    dx = (np.roll(high, -1, axis=1) - np.roll(high, 1, axis=1)) * 0.5 * scale
    dy = (np.roll(high, -1, axis=0) - np.roll(high, 1, axis=0)) * 0.5 * scale
    nx, ny, nz = -dx, dy, np.ones_like(dx)  # image y runs down, OpenGL green runs up
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    return ((np.stack([nx, ny, nz], axis=2) / norm[..., None] * 0.5 + 0.5) * 255.0 + 0.5).astype(np.uint8)


def build(source: Source) -> tuple[np.ndarray, np.ndarray]:
    patch = to_linear(read_patch(source))
    patch = delight(patch, source.delight_sigma)
    patch = centre_square(patch)
    small = resize(to_srgb8(patch), TILE)
    tile = make_tileable(to_linear(small))
    # set the tile's average colour: each channel scaled, so the grain and the colour balance of the patch stay as they were
    target = np.power(np.array(source.mean_srgb, dtype=np.float32) / 255.0, 2.2)
    tile = tile * (target / np.maximum(tile.mean(axis=(0, 1)), 1e-6))
    albedo = to_srgb8(tile)
    return albedo, normal_map(luminance(tile), source.normal_strength)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    only = set(sys.argv[1:])
    for source in SOURCES:
        if only and source.role not in only:
            continue
        albedo, normal = build(source)
        Image.fromarray(albedo).save(OUT / f"{source.role}_albedo.jpg", quality=94)
        Image.fromarray(normal).save(OUT / f"{source.role}_normal.jpg", quality=94)
        print(source.role, "albedo mean", albedo.reshape(-1, 3).mean(axis=0).round(1), flush=True)


if __name__ == "__main__":
    main()
