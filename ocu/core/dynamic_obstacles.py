"""Things that move, as obstacles: what the rover knows about a person or an animal in its way.

A parked vehicle is stamped into the world and the LiDAR maps it like a boulder (terrain.stamp_object).
That cannot be done for something that walks. The perception map keeps every obstacle it has ever mapped,
on purpose (a hole or a rock the LiDAR only resolves close up must not be forgotten the moment the rover
turns away), so a person who crossed the rover's path would leave a permanent wall behind them.
Moving things are kept apart, short-lived, and given to the reactive planner directly.

What the rover knows is what its LiDAR could see: in range, and with nothing that blocks a beam in
between. A contact is held for a few seconds after it was last seen, moved along at the velocity it was
seen to have, because a person who steps behind a boulder has not stopped existing. The planner is given
the contact's disc, grown by a margin for the fact that it can move, at where it is, and again where it
will be a moment from now, so that the rover steers for where the person is going, not where they were.

The radii, margin, horizon and memory are this build's choices, not measurements: DECISIONS.md.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ocu.core.lidar import LIDAR_RANGE_M, has_line_of_sight
from ocu.core.terrain import TerrainWorld

#: Extra room kept round a moving thing, over its own size: it can turn toward the rover faster than the
#: rover can turn away, and the rover's own footprint clearance is added on top of this by the planner.
MOVING_MARGIN_M = 0.6

#: The planner is shown the contact where it is predicted to be this far ahead as well as where it is now.
LEAD_TIMES_S = (0.0, 1.0, 2.0)

#: A contact not seen for this long is forgotten. Short: the thing has moved, and a stale wall helps no one.
MEMORY_S = 4.0

#: How much of a new velocity measurement replaces the old one, so that a flicker of position noise is not
#: read as the person running.
VELOCITY_BLEND = 0.5

#: Spacing of the obstacle points a disc is made of. The planner treats each as an obstacle cell centre.
POINT_SPACING_M = 0.25


@dataclass
class Contact:
    name: str
    radius: float
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_seen_s: float = 0.0

    def predicted(self, now_s: float, ahead_s: float = 0.0) -> tuple[float, float]:
        t = max(now_s - self.last_seen_s, 0.0) + ahead_s
        return self.x + self.vx * t, self.y + self.vy * t


def disc_points(cx: float, cy: float, radius: float, spacing: float = POINT_SPACING_M) -> np.ndarray:
    """Points filling a disc, plus its rim, so the rim is not a ragged inner grid."""
    ticks = np.arange(-radius, radius + spacing / 2.0, spacing)
    gx, gy = np.meshgrid(ticks, ticks)
    inside = gx * gx + gy * gy <= radius * radius
    pts = [np.column_stack([cx + gx[inside], cy + gy[inside]])]
    n = max(12, int(2.0 * math.pi * radius / spacing))
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts.append(np.column_stack([cx + radius * np.cos(a), cy + radius * np.sin(a)]))
    return np.vstack(pts)


class DynamicObstacles:
    """The moving things the rover has seen lately."""

    def __init__(self) -> None:
        self._contacts: dict[str, Contact] = {}

    @property
    def contacts(self) -> list[Contact]:
        return list(self._contacts.values())

    def clear(self) -> None:
        self._contacts.clear()

    def update(self, now_s: float, rover_xy: tuple[float, float], things: list[tuple[str, float, float, float]], world: TerrainWorld) -> None:
        """`things` is (name, x, y, radius) for every moving entity, wherever it is. Only those the LiDAR can see are noticed."""
        rx, ry = rover_xy
        for name, x, y, radius in things:
            if math.hypot(x - rx, y - ry) > LIDAR_RANGE_M or not has_line_of_sight(world, rx, ry, x, y):
                continue
            old = self._contacts.get(name)
            if old is None:
                self._contacts[name] = Contact(name, radius, x, y, 0.0, 0.0, now_s)
                continue
            dt = now_s - old.last_seen_s
            if dt > 1e-6:
                vx, vy = (x - old.x) / dt, (y - old.y) / dt
                old.vx = VELOCITY_BLEND * vx + (1.0 - VELOCITY_BLEND) * old.vx
                old.vy = VELOCITY_BLEND * vy + (1.0 - VELOCITY_BLEND) * old.vy
            old.x, old.y, old.last_seen_s = x, y, now_s
        for name in [n for n, c in self._contacts.items() if now_s - c.last_seen_s > MEMORY_S]:
            del self._contacts[name]

    def points(self, now_s: float) -> np.ndarray:
        """(n, 2) obstacle points for the reactive planner: every live contact, now and a moment ahead."""
        out = []
        for c in self._contacts.values():
            for ahead in LEAD_TIMES_S:
                x, y = c.predicted(now_s, ahead)
                out.append(disc_points(x, y, c.radius + MOVING_MARGIN_M))
        return np.vstack(out) if out else np.zeros((0, 2))

    def nearest(self, x: float, y: float, now_s: float) -> tuple[str, float] | None:
        """(name, distance from the point to the edge) of the nearest contact as it is now, or None."""
        gaps = [(c.name, math.hypot(x - cx, y - cy) - c.radius) for c in self._contacts.values() for cx, cy in [c.predicted(now_s)]]
        return min(gaps, key=lambda g: g[1]) if gaps else None

    def nearest_gap_m(self, x: float, y: float, now_s: float) -> float:
        """Distance from a point to the edge of the nearest contact as it is now. inf if there are none."""
        n = self.nearest(x, y, now_s)
        return n[1] if n else math.inf
