"""The thermal map: a mosaic of what the thermal cameras have actually seen.

Like the LiDAR cloud, it holds only what a sensor has covered. Unseen ground is dark. A cell keeps the temperature it
had when it was last imaged and the mission time of that picture, and fades as it goes stale: the ground changes and
the mosaic does not until it is seen again. Moving bodies are never written into it (a walker would leave a streak);
they are kept as short-lived heat contacts.

Resolution is finer than the world's cells so that a person is more than a pixel. No Qt import belongs in this file.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ocu.core.thermal_display import colour_of, to_rgb
from ocu.core.thermal_scene import ThermalFrame
from ocu.core.terrain import WORLD_HALF_EXTENT_M

CELL_M = 0.25
#: A pixel is only written if the ground it covers is no longer than this along the line of sight: a rover's camera
#: sits low, so beyond some distance one pixel covers metres and a mosaic of it means nothing.
MAX_FOOTPRINT_M = 2.0
MAX_SPLAT_RADIUS_CELLS = 6
#: How long a picture of the ground stays as it was drawn before it starts to fade, and how long the fade takes.
FRESH_S = 60.0
FADE_S = 600.0
MIN_STALE_ALPHA = 0.45
CONTACT_LIFE_S = 2.5
CONTACT_CAP = 40
#: A heat contact is only worth marking if it stands out of the ground by this much.
CONTACT_MIN_ABOVE_GROUND_K = 3.0


@dataclass
class HeatContact:
    x: float
    y: float
    temp_c: float
    kind: str
    seen_s: float
    radius_m: float


class ThermalMap:
    def __init__(self, half_extent_m: float = WORLD_HALF_EXTENT_M, cell_m: float = CELL_M) -> None:
        self.cell_m = cell_m
        self.half_extent_m = half_extent_m
        self.span = int(round(2.0 * half_extent_m / cell_m))
        self.temp_c = np.full((self.span, self.span), np.nan, dtype=np.float32)
        self.seen_s = np.full((self.span, self.span), -np.inf, dtype=np.float32)
        self.contacts: list[HeatContact] = []
        self.version = 0  # bumped whenever the mosaic changes, so a view knows to redraw

    def clear(self) -> None:
        self.temp_c.fill(np.nan)
        self.seen_s.fill(-np.inf)
        self.contacts.clear()
        self.version += 1

    def _index(self, x, y):
        return (np.floor((np.asarray(x) + self.half_extent_m) / self.cell_m).astype(np.int64),
                np.floor((np.asarray(y) + self.half_extent_m) / self.cell_m).astype(np.int64))

    @property
    def seen_fraction(self) -> float:
        return float(np.isfinite(self.temp_c).mean())

    def integrate(self, frame: ThermalFrame, now_s: float, entities=(), poses=()) -> int:
        """Write a picture into the mosaic. Returns how many cells it touched.

        Only ground pixels whose footprint is small enough are written, each over the cells it covers, and bodies are
        noted as heat contacts for a moment instead.
        """
        use = frame.ground & np.isfinite(frame.along_m) & (frame.along_m <= MAX_FOOTPRINT_M)
        touched = 0
        if use.any():
            ix, iy = self._index(frame.hx[use], frame.hy[use])
            along, across, temp = frame.along_m[use], frame.across_m[use], frame.temp_c[use]
            radius = np.clip(np.ceil(np.maximum(along, across) / (2.0 * self.cell_m) - 0.5).astype(np.int64), 0, MAX_SPLAT_RADIUS_CELLS)
            for r in np.unique(radius):
                sel = radius == r
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        jx, jy = ix[sel] + dx, iy[sel] + dy
                        ok = (jx >= 0) & (jx < self.span) & (jy >= 0) & (jy < self.span)
                        self.temp_c[jx[ok], jy[ok]] = temp[sel][ok]
                        self.seen_s[jx[ok], jy[ok]] = now_s
                        touched += int(ok.sum())
            self.version += 1
        self._note_contacts(frame, now_s, entities, poses)
        return touched

    def _note_contacts(self, frame: ThermalFrame, now_s: float, entities, poses) -> None:
        ground_level = float(np.median(frame.temp_c[frame.ground])) if frame.ground.any() else None
        for index, (entity, pose) in enumerate(zip(entities, poses, strict=False)):
            mask = frame.entity == index
            if mask.sum() < 3 or ground_level is None:
                continue
            hottest = float(np.percentile(frame.temp_c[mask], 90.0))
            if hottest - ground_level < CONTACT_MIN_ABOVE_GROUND_K and entity.kind != "vehicle":
                continue
            radius = max(entity.width_m, entity.length_m) / 2.0
            self.contacts = [c for c in self.contacts if not (c.kind == entity.kind and np.hypot(c.x - pose[0], c.y - pose[1]) < 2.0 * radius + 1.0)]
            self.contacts.append(HeatContact(pose[0], pose[1], hottest, entity.kind, now_s, radius))
        self.contacts = [c for c in self.contacts if now_s - c.seen_s <= CONTACT_LIFE_S][-CONTACT_CAP:]

    def alpha(self, now_s: float) -> np.ndarray:
        """1 for ground seen within FRESH_S, falling to MIN_STALE_ALPHA over FADE_S, 0 where never seen."""
        age = now_s - self.seen_s
        fade = np.clip((age - FRESH_S) / FADE_S, 0.0, 1.0)
        a = 1.0 - (1.0 - MIN_STALE_ALPHA) * fade
        return np.where(np.isfinite(self.temp_c), a, 0.0).astype(np.float32)

    def observed_range(self) -> tuple[float, float] | None:
        seen = self.temp_c[np.isfinite(self.temp_c)]
        if seen.size == 0:
            return None
        return float(np.percentile(seen, 1.0)), float(np.percentile(seen, 99.0))

    def rgba(self, lo: float, hi: float, palette: str, now_s: float) -> np.ndarray:
        """(span, span, 4) uint8, indexed [x, y]: the palette over what has been seen, transparent elsewhere."""
        temps = np.where(np.isfinite(self.temp_c), self.temp_c, lo)
        rgb = to_rgb(temps, lo, hi, palette)
        out = np.zeros(temps.shape + (4,), dtype=np.uint8)
        out[..., :3] = rgb
        out[..., 3] = (self.alpha(now_s) * 255.0).astype(np.uint8)
        return out

    def contact_colour(self, contact: HeatContact, lo: float, hi: float, palette: str) -> tuple[int, int, int]:
        return colour_of(contact.temp_c, lo, hi, palette)
