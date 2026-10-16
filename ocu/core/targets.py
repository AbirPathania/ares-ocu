"""Buried objects: what is actually in the ground for the suite to find.

Until now a detection was an entry in the mission file that fired at a
timestamp. Nothing was buried anywhere; the rover drove past a clock, not
an object. That is fine for rehearsing an interface and useless for
demonstrating a detection system, because every number it produced was
one somebody typed.

A target here is a physical object with the properties the four sensors
actually respond to: how deep it sits, how much metal is in it, whether
that metal is ferrous, and how much vapour it gives off. The sensor models
in `detection.py` read those properties and nothing else, so a confidence
on screen is the output of a geometry calculation rather than a figure
from a script. Move the rover somewhere else and it finds something else,
or nothing.

The mission theme stays a generic hazard sweep, per section 3. These are
buried objects of interest; the classification an operator confirms is a
threat type, and the labels here are deliberately neutral.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
import random

from ocu.core.sweep import LaneSweep, sweep_from_first_scan
from ocu.core.terrain import TerrainWorld
from dataclasses import dataclass, field


@dataclass(frozen=True)
class BuriedTarget:
    """One object in the ground, described by what a sensor can measure."""

    x: float
    y: float
    depth_m: float
    metal_mass_g: float
    ferrous: bool
    voc_emission: float  # 0..1, volatile trace reaching the surface
    truth_class: str  # what it really is; the operator never sees this directly

    def range_to(self, x: float, y: float) -> float:
        """Slant range from a sensor at the surface to the object."""
        return math.hypot(math.hypot(x - self.x, y - self.y), self.depth_m)

    def lateral_offset(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)


@dataclass
class TargetField:
    """Everything buried in the operating area."""

    targets: list[BuriedTarget] = field(default_factory=list)

    def near(self, x: float, y: float, radius_m: float) -> list[BuriedTarget]:
        return [t for t in self.targets if t.lateral_offset(x, y) <= radius_m]

    def __len__(self) -> int:
        return len(self.targets)


def default_field(seed: int = 20261016, sweep: LaneSweep | None = None) -> TargetField:
    """Objects laid along the lane the rover sweeps, plus others dispersed.

    The suite's widest swath is the pulse induction array at 40 cm, so a
    sensor only sees what the vehicle very nearly drives over. That is not
    a limitation of this model, it is what clearance work is: a lane at a
    time. The objects on the lane are therefore placed within centimetres
    of the driven line, and the spread of them is chosen so the sweep
    produces four different outcomes rather than four of the same one.

      - a strong shallow ferrous object, which every sensor sees and which
        escalates
      - a deep low-metal non-ferrous object, the hard case: radar reaches
        it, induction barely, the gradiometer not at all
      - buried scrap, which reads hard on metal and gives off no vapour,
        and which the suite should therefore reject rather than escalate
      - an object just outside the swath, which is missed, because a
        demonstration where everything is found teaches nothing about lane
        width

    They are positioned relative to the sweep rather than in absolute
    coordinates, because the sweep's axis is chosen from the terrain. An
    object buried at a fixed coordinate would end up wherever the lanes
    did not go the moment the ground changed, and the demonstration would
    silently become one where the vehicle sweeps empty dirt.
    """
    rng = random.Random(seed)
    sweep = sweep if sweep is not None else sweep_from_first_scan(TerrainWorld(seed=seed))

    # (lane, metres along it, metres across it, then the object itself)
    placements = [
        # On the lane, dead centre, shallow and ferrous: all four respond.
        (0, 9.0, 0.04, 0.07, 165.0, True, 0.58, "high_metal_shallow"),
        # On the lane but deep and barely metallic, and non-ferrous, so the
        # gradiometer is blind to it and induction is marginal.
        (0, 13.2, -0.09, 0.27, 11.0, False, 0.34, "low_metal_deep"),
        # Scrap: strong metal return, no vapour at all.
        (0, 16.1, 0.11, 0.13, 255.0, True, 0.0, "inert_scrap"),
        # On the second lane, ferrous and deep: the gradiometer leads here
        # because it reaches past the induction array.
        (1, 18.6, 0.05, 0.36, 95.0, True, 0.12, "ferrous_deep"),
        # Between the lanes, clear of every swath including the 2 m the
        # vapour sensor reaches, so the sweep genuinely misses it.
        (0, 6.5, 2.0, 0.06, 140.0, True, 0.5, "outside_swath"),
    ]
    targets = []
    for lane, along, offset, depth, mass, ferrous, voc, truth in placements:
        x, y = sweep.at(lane, along, offset)
        targets.append(BuriedTarget(x, y, depth, mass, ferrous, voc, truth))
    for _ in range(6):
        targets.append(
            BuriedTarget(
                x=rng.uniform(-70, 70),
                y=rng.uniform(-70, 70),
                depth_m=rng.uniform(0.05, 0.35),
                metal_mass_g=rng.uniform(3.0, 200.0),
                ferrous=rng.random() < 0.6,
                voc_emission=rng.uniform(0.0, 0.6),
                truth_class="dispersed",
            )
        )
    return TargetField(targets=targets)
