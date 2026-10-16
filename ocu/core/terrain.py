"""The ground the rover is actually driving over, and what it is made of.

Two separate things live here and must not be confused:

- **Ground truth.** A procedurally generated world of classified cells:
  open ground, scrub, trees, rock, and a track running through it. This is
  what is really there. The operator never sees it directly.
- **The rover's map.** What the rover has actually observed, accumulated
  from LiDAR returns. It starts entirely unknown and fills in only where
  the sensor has reached. Everything the operator sees comes from this.

Keeping them apart is the whole point. A map that is simply drawn on
screen is a picture; a map that is built from returns, with shadows behind
every rock and nothing at all beyond sensor range, is perception. The
second one is what the vehicle does, and the difference is visible in
about two seconds of watching it drive.

Everything is referenced to the deploy point, which is the origin, with
the y axis along true north. The rover carries its own compass and inertial
solution, so the whole picture is built in that frame rather than needing
an external reference, which is what lets this work with no GPS at all.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from enum import IntEnum


class Terrain(IntEnum):
    """Cell classification. Ordered so higher values take precedence when
    features overlap during generation."""

    UNKNOWN = 0
    GROUND = 1  # open, traversable
    TRACK = 2  # a worn path; the easiest going
    SCRUB = 3  # low vegetation, traversable but slow
    ROCK = 4  # impassable, and it blocks the beam
    TREE = 5  # impassable, and it blocks the beam


#: Classes that stop a LiDAR ray. Everything else is driven over and seen past.
OCCLUDING = frozenset({Terrain.ROCK, Terrain.TREE})

#: Classes a rover cannot drive through.
IMPASSABLE = OCCLUDING

CELL_SIZE_M = 0.5
WORLD_HALF_EXTENT_M = 110.0


@dataclass
class TerrainWorld:
    """Ground truth, on a square grid centred on the deploy point."""

    cell_size_m: float = CELL_SIZE_M
    half_extent_m: float = WORLD_HALF_EXTENT_M
    seed: int = 20261002
    _cells: dict[tuple[int, int], Terrain] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self._cells:
            self._generate()

    # --- frame conversion -------------------------------------------------

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor(x / self.cell_size_m)), int(math.floor(y / self.cell_size_m)))

    def to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        return (cell[0] * self.cell_size_m, cell[1] * self.cell_size_m)

    def classify(self, x: float, y: float) -> Terrain:
        return self._cells.get(self.to_cell(x, y), Terrain.GROUND)

    def at(self, cell: tuple[int, int]) -> Terrain:
        return self._cells.get(cell, Terrain.GROUND)

    def blocks_beam(self, cell: tuple[int, int]) -> bool:
        return self.at(cell) in OCCLUDING

    # --- generation -------------------------------------------------------

    def _generate(self) -> None:
        rng = random.Random(self.seed)
        extent = int(self.half_extent_m / self.cell_size_m)

        self._scatter_clusters(rng, extent, Terrain.SCRUB, patches=90, radius_m=(2.0, 6.0))
        self._scatter_clusters(rng, extent, Terrain.TREE, patches=34, radius_m=(1.5, 4.5))
        self._scatter_clusters(rng, extent, Terrain.ROCK, patches=26, radius_m=(1.0, 3.0))

        # A track is carved last so it cuts through whatever it crosses: a
        # path exists because something has already driven the scrub down.
        self._carve_track(rng, extent)

    def _scatter_clusters(
        self, rng: random.Random, extent: int, kind: Terrain, patches: int, radius_m: tuple[float, float]
    ) -> None:
        for _ in range(patches):
            cx = rng.uniform(-self.half_extent_m, self.half_extent_m)
            cy = rng.uniform(-self.half_extent_m, self.half_extent_m)
            # Keep the deploy point itself clear; a rover does not start
            # inside a tree.
            if math.hypot(cx, cy) < 8.0:
                continue
            radius = rng.uniform(*radius_m)
            self._stamp_blob(rng, cx, cy, radius, kind, extent)

    def _stamp_blob(
        self, rng: random.Random, cx: float, cy: float, radius_m: float, kind: Terrain, extent: int
    ) -> None:
        """An irregular blob, so nothing reads as a perfect circle."""
        cells = int(radius_m / self.cell_size_m) + 1
        centre = self.to_cell(cx, cy)
        wobble = rng.uniform(0.6, 1.0)
        phase = rng.uniform(0, math.tau)
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                distance = math.hypot(dx, dy) * self.cell_size_m
                angle = math.atan2(dy, dx)
                edge = radius_m * (wobble + 0.4 * math.sin(3 * angle + phase))
                if distance > edge:
                    continue
                cell = (centre[0] + dx, centre[1] + dy)
                if abs(cell[0]) > extent or abs(cell[1]) > extent:
                    continue
                if kind >= self._cells.get(cell, Terrain.GROUND):
                    self._cells[cell] = kind

    def _carve_track(self, rng: random.Random, extent: int) -> None:
        """A meandering path from the deploy point outward, two metres wide."""
        x, y = 0.0, 0.0
        heading = rng.uniform(-0.4, 0.4)  # roughly north out of the deploy point
        half_width = int(1.0 / self.cell_size_m)

        for _ in range(900):
            heading += rng.uniform(-0.09, 0.09)
            x += math.sin(heading) * self.cell_size_m
            y += math.cos(heading) * self.cell_size_m
            if abs(x) > self.half_extent_m or abs(y) > self.half_extent_m:
                break
            centre = self.to_cell(x, y)
            for dx in range(-half_width, half_width + 1):
                for dy in range(-half_width, half_width + 1):
                    cell = (centre[0] + dx, centre[1] + dy)
                    if abs(cell[0]) <= extent and abs(cell[1]) <= extent:
                        self._cells[cell] = Terrain.TRACK


@dataclass
class PerceptionMap:
    """What the rover has observed. Empty until something is scanned."""

    cell_size_m: float = CELL_SIZE_M
    _observed: dict[tuple[int, int], Terrain] = field(default_factory=dict)
    _pending: list[tuple[tuple[int, int], Terrain]] = field(default_factory=list)

    def observe(self, cell: tuple[int, int], kind: Terrain) -> bool:
        """Record one cell. Returns True if this is new information."""
        if self._observed.get(cell) == kind:
            return False
        self._observed[cell] = kind
        self._pending.append((cell, kind))
        return True

    def at(self, cell: tuple[int, int]) -> Terrain:
        return self._observed.get(cell, Terrain.UNKNOWN)

    def is_known(self, cell: tuple[int, int]) -> bool:
        return cell in self._observed

    def cells(self) -> dict[tuple[int, int], Terrain]:
        return self._observed

    def coverage_m2(self) -> float:
        return len(self._observed) * self.cell_size_m**2

    def drain_new(self) -> list[tuple[tuple[int, int], Terrain]]:
        """Cells observed since the last drain.

        The renderer keeps a cached image of the discovered map and only
        needs to paint what changed; repainting a hundred thousand cells
        every frame to show a few new ones would be absurd.
        """
        pending, self._pending = self._pending, []
        return pending

    def clear(self) -> None:
        self._observed.clear()
        self._pending.clear()

    def __len__(self) -> int:
        return len(self._observed)
