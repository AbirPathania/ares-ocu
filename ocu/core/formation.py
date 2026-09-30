"""Bearing + pattern + spacing -> targets, section 6.4.

One number in (spacing), three coordinates out per aircraft. Rotate each
pattern's local-frame slot offset by the bearing, translate by the rover's
current position, done. This must stay pure and fast: the map redraws this
live on every tick of the bearing dial, section 6.2.

Decision recorded in DECISIONS.md: the spec's pseudocode uses two symbols,
d (forward stagger) and s (lateral offset), fed from a single "Spacing"
operator parameter. Nothing in the spec splits one number into two, so this
build sets d = s = spacing. Revisit if Bhargav specifies otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.constants import FORMATION_PATTERNS, MIN_AIRCRAFT_SEPARATION_M, TETHER_LIMIT_M


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    z: float
    heading: float = 0.0


def clamp_to_tether(
    x: float, y: float, rover_x: float, rover_y: float, limit_m: float = TETHER_LIMIT_M
) -> tuple[float, float]:
    """Pull a point back onto the tether ring if it lies outside it.

    The aircraft never work further than UWB range from the rover (section 2),
    so any commanded position beyond the limit is clamped along the same
    bearing rather than refused: the operator still gets the direction they
    asked for, at the furthest range the system can actually hold.
    """
    dx, dy = x - rover_x, y - rover_y
    distance = math.hypot(dx, dy)
    if distance <= limit_m or distance == 0.0:
        return x, y
    scale = limit_m / distance
    return rover_x + dx * scale, rover_y + dy * scale


def _slot_offsets(pattern: str, d: float, s: float) -> list[tuple[float, float]]:
    if pattern == "vee":
        return [(0.0, 0.0), (-d, +s), (-d, -s)]
    if pattern == "line":
        return [(0.0, -s), (0.0, 0.0), (0.0, +s)]
    if pattern == "column":
        return [(0.0, 0.0), (-d, 0.0), (-2 * d, 0.0)]
    raise ValueError(f"unknown pattern: {pattern!r}")


def clamp_spacing(spacing_m: float) -> float:
    """Enforce the hard minimum separation in code, not only the UI, section 6.2/10."""
    return max(spacing_m, MIN_AIRCRAFT_SEPARATION_M)


def compute_formation_targets(
    rover_x: float,
    rover_y: float,
    bearing_deg: float,
    pattern: str,
    spacing_m: float,
    rally_altitude_m: float,
) -> list[Target]:
    """Compute the three aircraft targets for a guided formation launch.

    bearing_deg is a compass heading, 0 to 360, the direction the formation
    faces relative to the rover, section 6.2. Slot order matches the fixed
    tricopter-1/tricopter-2/tricopter-3 assignment.
    """
    if pattern not in FORMATION_PATTERNS:
        raise ValueError(f"unknown pattern: {pattern!r}")

    spacing_m = clamp_spacing(spacing_m)
    bearing_rad = math.radians(bearing_deg % 360.0)
    cos_b, sin_b = math.cos(bearing_rad), math.sin(bearing_rad)

    targets: list[Target] = []
    for dx, dy in _slot_offsets(pattern, d=spacing_m, s=spacing_m):
        x = rover_x + dx * cos_b - dy * sin_b
        y = rover_y + dx * sin_b + dy * cos_b
        x, y = clamp_to_tether(x, y, rover_x, rover_y)
        # Every aircraft in a formation faces the formation's bearing, so the
        # group presents one sensor direction rather than three.
        targets.append(Target(x=x, y=y, z=rally_altitude_m, heading=bearing_deg % 360.0))
    return targets
