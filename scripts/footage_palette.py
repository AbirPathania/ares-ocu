"""Measure the colours of the reference footage: sky, haze, trail, grass, foliage, and how deep its shadows are.

    .venv/bin/python scripts/footage_palette.py     # reads reference_footage/derived/, writes ocu/data/footage_palette.json

The numbers are display colours (sRGB) of tone-mapped frames, because that is what the renderer's finished picture is
matched against; the renderer converts them to its own scene colours (see render/shaders.py and DECISIONS.md,
"Reference footage"). Each class is found by colour and position alone, in frames that are the right way up, and the
reported value is a median over every frame that has enough of it, so one odd frame does not move it.
"""

from __future__ import annotations

import colorsys
import json
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
DERIVED = ROOT / "reference_footage" / "derived"
OUT = ROOT / "ocu" / "data" / "footage_palette.json"


def hsv(rgb: np.ndarray):
    f = rgb.astype(np.float32) / 255.0
    mx, mn = f.max(axis=2), f.min(axis=2)
    d = mx - mn
    h = np.zeros_like(mx)
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    safe = np.where(d == 0, 1, d)
    h = np.where(mx == r, ((g - b) / safe) % 6, np.where(mx == g, (b - r) / safe + 2, (r - g) / safe + 4)) * 60.0
    s = np.where(mx == 0, 0, d / np.where(mx == 0, 1, mx))
    return h, s, mx


def masks(rgb: np.ndarray) -> dict[str, np.ndarray]:
    h, s, v = hsv(rgb)
    rows = np.arange(rgb.shape[0])[:, None] / rgb.shape[0]
    f = rgb.astype(np.float32)
    sky = (f[..., 2] > f[..., 0] + 12) & (f[..., 2] > f[..., 1] + 4) & (v > 0.45) & (rows < 0.6)
    sand = (h > 18) & (h < 42) & (s > 0.18) & (s < 0.55) & (v > 0.28) & (rows > 0.45)
    grass = (h >= 38) & (h < 58) & (s > 0.28) & (v > 0.35)
    leaf = (h >= 70) & (h < 140) & (s > 0.25) & (v > 0.18)
    return {"sky": sky, "sand": sand, "grass": grass, "leaf": leaf}


def median_colour(rgb: np.ndarray, mask: np.ndarray, lo_q=None, hi_q=None) -> list[float]:
    px = rgb[mask].astype(np.float32)
    if lo_q is not None:
        lum = px @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
        a, b = np.percentile(lum, [lo_q, hi_q])
        px = px[(lum >= a) & (lum <= b)]
    return [round(float(c), 1) for c in np.median(px, axis=0)]


def sky_profile(catalogue: dict, degrees=(1, 3, 6, 10, 14), vfov_deg: float = 40.5) -> dict[str, list[float]]:
    """Sky colour at a number of degrees above the horizon, from the hilltop clip (IMG_5117), which looks out level.

    The horizon in each frame is the first row, from the top, whose middle half is no longer mostly sky; the camera's
    vertical field of view (about 40 degrees for the phone's main lens at 16:9) turns rows into degrees."""
    found: dict[int, list[np.ndarray]] = {d: [] for d in degrees}
    for row in catalogue["frames"]:
        if row["clip"] != "IMG_5117":
            continue
        rgb = np.array(Image.open(DERIVED / row["file"]))
        h, w = rgb.shape[:2]
        sky = masks(rgb)["sky"]
        edge = int(np.argmax(sky[:, w // 4: 3 * w // 4].mean(axis=1) < 0.3))
        if edge < 60:
            continue
        for d in degrees:
            r = edge - int(d * h / vfov_deg)
            if r >= 4:
                found[d].append(np.median(rgb[max(r - 3, 0): r + 3, w // 4: 3 * w // 4].reshape(-1, 3).astype(float), axis=0))
    return {str(d): [round(float(c), 1) for c in np.median(np.array(v), axis=0)] for d, v in found.items() if v}


def main() -> None:
    catalogue = json.loads((DERIVED / "catalogue.json").read_text())
    samples: dict[str, list[list[float]]] = {k: [] for k in ("sky_top", "sky_low", "sand_lit", "sand_shade", "grass_lit", "grass_shade", "leaf_lit", "leaf_shade")}
    ridge = []
    for row in catalogue["frames"]:
        if row["sharpness"] < 4.0:
            continue
        rgb = np.array(Image.open(DERIVED / row["file"]))
        m = masks(rgb)
        h = rgb.shape[0]
        sky = m["sky"]
        if sky[: h // 5].mean() > 0.25:
            samples["sky_top"].append(median_colour(rgb[: h // 8], sky[: h // 8])) if sky[: h // 8].sum() > 500 else None
        for name, key in (("sand", "sand"), ("grass", "grass"), ("leaf", "leaf")):
            mask = m[key]
            if mask.mean() > 0.04:
                samples[f"{name}_lit"].append(median_colour(rgb, mask, 70, 97))
                samples[f"{name}_shade"].append(median_colour(rgb, mask, 3, 30))
        if row["clip"] == "IMG_5117" and sky.mean() > 0.15:
            # the horizon: sky just above the first non-sky row, and the ridge just below it, in the column middle
            col = sky[:, rgb.shape[1] // 4: 3 * rgb.shape[1] // 4].mean(axis=1)
            edge = int(np.argmax(col < 0.3))
            if 20 < edge < h - 40:
                samples["sky_low"].append(median_colour(rgb[edge - 12: edge - 2], sky[edge - 12: edge - 2]) if sky[edge - 12: edge - 2].sum() > 200 else [0, 0, 0])
                ridge.append([median_colour(rgb[edge + k: edge + k + 8], np.ones((8, rgb.shape[1]), bool)) for k in (6, 60, 160) if edge + k + 8 < h])
    out_profile = sky_profile(catalogue)
    out = {k: [round(float(c), 1) for c in np.median(np.array([s for s in v if sum(s) > 0]), axis=0)] if v else None for k, v in samples.items()}
    out["sky_by_elevation_deg"] = out_profile
    out["frames_used"] = {k: len([s for s in v if sum(s) > 0]) for k, v in samples.items()}
    if ridge:
        depth = min(len(r) for r in ridge)
        out["ridge_by_depth"] = [[round(float(c), 1) for c in np.median(np.array([r[i] for r in ridge]), axis=0)] for i in range(depth)]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
