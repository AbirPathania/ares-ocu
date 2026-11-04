"""The scene's colours of sky and haze, taken from the reference footage.

What was measured (scripts/footage_palette.py) is how the footage looks on a screen: a sky 142/185/218 ten degrees above
the horizon, a haze 178/188/195 at it. The renderer works in linear light and ends in an exposure, a filmic curve, a
touch of saturation and a gamma. So the measured display colour is run backwards through that same ending, and what
comes out is the scene colour that the renderer will turn back into the measured display colour.

The measurements stop at 14 degrees above the horizon (the hilltop clip looks out level). Above that the sky is
continued toward a deeper blue by assumption, with the usual clear-sky behaviour; the points are listed in `ABOVE_DEG`
and are marked as chosen, not measured, in DECISIONS.md.

No Qt and no GL import belongs in this file.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

EXPOSURE = 0.78  # the renderer's exposure; the one place it is set
SATURATION = 1.10  # what the finishing pass in shaders.py does to chroma
PALETTE_PATH = Path(__file__).resolve().parent.parent / "data" / "footage_palette.json"

#: Used if the palette file is missing: the sky and haze this renderer had before the footage.
DEFAULT_SKY_DEG = {1: (190, 200, 208), 3: (180, 196, 210), 6: (160, 190, 218), 10: (142, 185, 218), 14: (128, 166, 214)}
#: Chosen, not measured: how the sky goes on above where the footage stops.
ABOVE_DEG = {24: (116, 158, 213), 45: (98, 144, 209), 90: (82, 130, 204)}
HORIZON_HAZE = (180, 186, 187)  # the far ridge in the hilltop clip


def _aces(x: np.ndarray) -> np.ndarray:
    return np.clip((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0)


def scene_from_display(display_rgb, exposure: float = EXPOSURE) -> np.ndarray:
    """Linear scene colour that finishes (exposure, filmic curve, saturation, gamma) as the given 0..255 display colour."""
    c = np.power(np.clip(np.asarray(display_rgb, dtype=np.float64) / 255.0, 1e-4, 0.999), 2.2)
    lum = float(c @ np.array([0.299, 0.587, 0.114]))
    c = lum + (c - lum) / SATURATION
    lo, hi = np.zeros(3), np.full(3, 60.0)
    for _ in range(60):
        mid = (lo + hi) / 2.0
        below = _aces(mid) < c
        lo, hi = np.where(below, mid, lo), np.where(below, hi, mid)
    return np.maximum((lo + hi) / 2.0 / exposure, 0.0)


def sky_points() -> list[tuple[float, tuple[float, float, float]]]:
    """(degrees above the horizon, linear scene colour), lowest first."""
    try:
        measured = {int(k): tuple(v) for k, v in json.loads(PALETTE_PATH.read_text())["sky_by_elevation_deg"].items()}
    except (OSError, KeyError, ValueError):
        measured = dict(DEFAULT_SKY_DEG)
    display = {0: HORIZON_HAZE, **measured, **ABOVE_DEG}
    return [(float(d), tuple(float(x) for x in scene_from_display(display[d]))) for d in sorted(display)]


def haze_colour() -> tuple[float, float, float]:
    return tuple(float(x) for x in scene_from_display(HORIZON_HAZE))


def ambient_sky() -> tuple[float, float, float]:
    """Colour of the light the sky gives the ground: the sky at mid height, at the brightness the renderer already used."""
    colour = scene_from_display(ABOVE_DEG[45])
    old = np.array([0.30, 0.42, 0.62])
    colour = colour * float(old @ np.array([0.299, 0.587, 0.114])) / float(colour @ np.array([0.299, 0.587, 0.114]))
    # light from a whole sky is bluer than the ground it falls on looks, because the ground is not grey; a third of the way
    return tuple(float(x) for x in 0.65 * old + 0.35 * colour)


def sky_glsl() -> str:
    """GLSL for the body of `vec3 skyColor(vec3 d)`'s gradient: a piecewise-linear table in elevation."""
    points = sky_points()
    lines = ["    float e = degrees(asin(clamp(d.z, 0.0, 1.0)));", "    vec3 col = vec3({:.5f}, {:.5f}, {:.5f});".format(*points[0][1])]
    for (d0, c0), (d1, c1) in zip(points, points[1:]):
        lines.append(
            "    col = mix(col, vec3({:.5f}, {:.5f}, {:.5f}), smoothstep({:.1f}, {:.1f}, e));".format(*c1, d0, d1)
        )
    return "\n".join(lines)
