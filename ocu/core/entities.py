"""People, vehicles and animals on the ground: things a camera can see that no buried-object sensor can.

The buried objects in targets.py are found by the ground suite and, if shallow, by a scout's camera. These are
the other half of what ARES is expected to perceive, threats and non-threats that stand or walk on the
surface: a person (the one the spec's threat_type enum names), a vehicle (also named), and an animal (not
named; see DECISIONS.md, the eighth class is still open).

Like the buried objects they are placed on purpose and positioned relative to the sweep, so that what the
rover and the scouts can see does not depend on which way the sweep happened to be laid out. Nothing here
appears at random. Nothing in this file decides whether anything is seen: that is vision.py.

Sizes, speeds and positions are this build's choices, not the baseline's; DECISIONS.md lists them.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.sweep import LaneSweep, sweep_from_first_scan
from ocu.core.terrain import TerrainWorld

KINDS = ("person", "vehicle", "animal")


@dataclass(frozen=True)
class SurfaceEntity:
    kind: str
    name: str
    height_m: float  # standing height; the detector's curves are indexed by how tall it looks
    width_m: float  # across the direction it faces
    length_m: float  # along the direction it faces
    path: tuple[tuple[float, float], ...]  # world points; one point means it stays put
    speed_ms: float = 0.0

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f"unknown entity kind {self.kind!r}")
        if not self.path:
            raise ValueError("an entity needs at least one position")

    def pose_at(self, t_s: float) -> tuple[float, float, float]:
        """(x, y, heading in degrees clockwise from +y) after t_s seconds, walking its path back and forth."""
        if len(self.path) == 1 or self.speed_ms <= 0.0:
            heading = self._heading(0)
            return self.path[0][0], self.path[0][1], heading
        legs = [math.dist(a, b) for a, b in zip(self.path, self.path[1:], strict=False)]
        total = sum(legs)
        if total <= 0.0:
            return self.path[0][0], self.path[0][1], 0.0
        travelled = (self.speed_ms * max(t_s, 0.0)) % (2.0 * total)
        backwards = travelled > total
        if backwards:
            travelled = 2.0 * total - travelled
        for i, leg in enumerate(legs):
            if travelled <= leg or i == len(legs) - 1:
                f = 0.0 if leg == 0.0 else min(travelled / leg, 1.0)
                (x0, y0), (x1, y1) = self.path[i], self.path[i + 1]
                x, y = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
                dx, dy = (x0 - x1, y0 - y1) if backwards else (x1 - x0, y1 - y0)
                return x, y, math.degrees(math.atan2(dx, dy)) % 360.0
            travelled -= leg
        return self.path[-1][0], self.path[-1][1], 0.0

    def _heading(self, i: int) -> float:
        if len(self.path) < 2:
            return 0.0
        (x0, y0), (x1, y1) = self.path[i], self.path[i + 1]
        return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0


def default_entities(seed: int = 20261016, sweep: LaneSweep | None = None) -> list[SurfaceEntity]:
    """Three entities, each a different lesson, positioned from the sweep.

      - a person walking across the ground ahead of the lanes. Large enough to be seen from the ground
        at tens of metres, and a case where the camera's view from the air is the harder one
      - a parked vehicle off to one side, far enough that the rover's camera barely resolves it
      - an animal moving slowly near the first lane: not a threat, and the thing a detector most often
        invents on rocks, so seeing a real one is the contrast
    """
    sweep = sweep if sweep is not None else sweep_from_first_scan(TerrainWorld(seed=seed))
    return [
        SurfaceEntity("person", "walker", 1.75, 0.5, 0.3, (sweep.at(1, 22.0, 12.0), sweep.at(1, 22.0, -4.0)), speed_ms=1.2),
        SurfaceEntity("vehicle", "parked truck", 1.9, 1.9, 5.0, (sweep.at(3, 26.0, -10.0),)),
        SurfaceEntity("animal", "deer", 1.0, 0.4, 1.5, (sweep.at(0, 10.0, 9.0), sweep.at(0, 16.0, 11.0), sweep.at(0, 12.0, 7.0)), speed_ms=0.5),
    ]
