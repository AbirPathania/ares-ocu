"""How a thermal picture is shown: gain, palette and scale.

A thermal camera has no colour. What it measures is an apparent temperature, and what an operator sees is that
temperature between two levels mapped to brightness or to a palette. The levels come from an automatic gain control
(the 1st and 99th percentile of the frame, so one hot engine does not flatten everything else) or are set by hand.

No Qt import belongs in this file.
"""

from __future__ import annotations

import numpy as np

PALETTES = ("white_hot", "black_hot", "iron")
PALETTE_LABELS = {"white_hot": "WHITE-HOT", "black_hot": "BLACK-HOT", "iron": "IRON"}

#: The classic "ironbow": black through violet and red to yellow and white. Anchor colours at even steps.
_IRON = np.array(
    [(0, 0, 0), (20, 0, 60), (80, 0, 110), (150, 20, 100), (210, 60, 50), (240, 120, 20), (252, 190, 40), (255, 245, 150), (255, 255, 255)],
    dtype=np.float32,
)
LUT_SIZE = 256
AGC_PERCENTILES = (1.0, 99.0)
MIN_SPAN_K = 12.0  # never stretch a frame narrower than this: noise, and a patch of sun on a slope, would become picture


def _lut(name: str) -> np.ndarray:
    x = np.linspace(0.0, 1.0, LUT_SIZE)
    if name == "iron":
        anchors = np.linspace(0.0, 1.0, len(_IRON))
        return np.stack([np.interp(x, anchors, _IRON[:, c]) for c in range(3)], axis=1).astype(np.uint8)
    grey = (x * 255.0).astype(np.uint8)
    if name == "black_hot":
        grey = grey[::-1]
    return np.stack([grey, grey, grey], axis=1)


LUTS = {name: _lut(name) for name in PALETTES}


def auto_range(temp_c: np.ndarray, valid: np.ndarray | None = None) -> tuple[float, float]:
    """The two temperatures that map to black and white: percentiles of the frame, at least MIN_SPAN_K apart."""
    values = temp_c[valid] if valid is not None and valid.any() else temp_c.ravel()
    lo, hi = np.percentile(values, AGC_PERCENTILES)
    if hi - lo < MIN_SPAN_K:
        mid = (hi + lo) / 2.0
        lo, hi = mid - MIN_SPAN_K / 2.0, mid + MIN_SPAN_K / 2.0
    return float(lo), float(hi)


def to_rgb(temp_c: np.ndarray, lo: float, hi: float, palette: str = "white_hot") -> np.ndarray:
    """(H, W, 3) uint8 from apparent temperatures between two levels."""
    idx = np.clip((temp_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0) * (LUT_SIZE - 1)
    return LUTS[palette][idx.astype(np.int32)]


def colour_of(temp_c: float, lo: float, hi: float, palette: str) -> tuple[int, int, int]:
    r, g, b = LUTS[palette][int(np.clip((temp_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0) * (LUT_SIZE - 1))]
    return int(r), int(g), int(b)


def scale_ticks(lo: float, hi: float, count: int = 5) -> list[tuple[float, float]]:
    """(position 0..1, degrees C) along a scale, at round numbers."""
    span = hi - lo
    raw = span / max(count - 1, 1)
    step = next(s for s in (0.5, 1, 2, 5, 10, 20, 50) if s >= raw) if raw <= 50 else 100
    first = np.ceil(lo / step) * step
    values = np.arange(first, hi + 1e-9, step)
    return [((v - lo) / span, float(v)) for v in values]
