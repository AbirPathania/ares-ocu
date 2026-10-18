"""Bearing + pattern + spacing + standoff -> targets, section 6.4.

Three numbers in, three aircraft positions out, recomputed live as the
operator turns the bearing dial. This is the part of the system an
operator actually commands, so the geometry has to be exact.

Conventions, both load-bearing:

- **Bearing is a compass heading.** 0 is north, 90 is east, measured
  clockwise. The forward unit vector is therefore (sin b, cos b) and the
  right-hand vector is (cos b, -sin b). Getting this wrong rotates every
  formation by 90 degrees from the dial the operator is looking at.
- **The local frame is X forward, Y to the right**, per section 6.4.

Every pattern scouts *ahead* of the rover, at a standoff inside the 30 to
50 m scout band section 2 specifies. The pseudocode in section 6.4 places
slots at (0, 0) and at negative X, which puts one aircraft on top of the
rover and the rest behind it; that reads as a formation trailing its own
vehicle rather than scouting for it, so the offsets here are built around
the purpose each pattern is given in the section 6.2 table instead:

    vee     a forward push      leader probes, wings cover the flanks
    line    a wide sweep        abreast, widest swath across the track
    column  tight terrain       single file in depth, down a trail

Every slot flies the commanded rally altitude, because the pattern already
separates the aircraft on the ground plane; altitude is only layered when a
pattern is squeezed close enough that it does not. The whole formation is
scaled as a rigid shape if it would otherwise breach the tether.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ocu.core.constants import (
    ALTITUDE_DECONFLICTION_M,
    FORMATION_PATTERNS,
    FORMATION_STANDOFF_M,
    MIN_AIRCRAFT_SEPARATION_M,
    TETHER_LIMIT_M,
)


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    z: float
    heading: float = 0.0


def clamp_to_tether(
    x: float, y: float, rover_x: float, rover_y: float, limit_m: float = TETHER_LIMIT_M
) -> tuple[float, float]:
    """Pull a single point back onto the tether ring if it lies outside it."""
    dx, dy = x - rover_x, y - rover_y
    distance = math.hypot(dx, dy)
    if distance <= limit_m or distance == 0.0:
        return x, y
    scale = limit_m / distance
    return rover_x + dx * scale, rover_y + dy * scale


def clamp_spacing(spacing_m: float) -> float:
    """Enforce the hard minimum separation in code, not only the UI, section 6.2/10."""
    return max(spacing_m, MIN_AIRCRAFT_SEPARATION_M)


def _slot_offsets(pattern: str, standoff: float, spacing: float) -> list[tuple[float, float]]:
    """Slot offsets in the local frame: X forward along the bearing, Y right.

    Slot order is the fixed tricopter-1/2/3 assignment, and is chosen so the
    aircraft read left-to-right across the operator's screen in the patterns
    that have a left and a right.
    """
    if pattern == "line":
        # Abreast: level with each other, straddling the track, widest swath.
        return [(standoff, -spacing), (standoff, 0.0), (standoff, +spacing)]

    if pattern == "vee":
        # Leader on the track, wings swept back and out so their sensor cones
        # overlap the leader's rather than duplicating it.
        sweep = spacing * 0.75
        return [(standoff - sweep, -spacing), (standoff, 0.0), (standoff - sweep, +spacing)]

    if pattern == "column":
        # Single file in depth: nothing spreads sideways into the treeline,
        # and the lead aircraft is the furthest down the trail.
        return [(standoff + spacing, 0.0), (standoff, 0.0), (standoff - spacing, 0.0)]

    raise ValueError(f"unknown pattern: {pattern!r}")


def _scale_into_tether(offsets: list[tuple[float, float]], limit_m: float) -> list[tuple[float, float]]:
    """Shrink the formation as one rigid shape if any slot breaches the tether.

    Clamping each slot on its own would flatten the formation against the
    ring and turn a vee into an arc. Scaling the whole thing keeps the shape
    the operator chose and only makes it smaller.
    """
    furthest = max((math.hypot(dx, dy) for dx, dy in offsets), default=0.0)
    if furthest <= limit_m or furthest == 0.0:
        return offsets
    scale = limit_m / furthest
    return [(dx * scale, dy * scale) for dx, dy in offsets]


def _slot_altitudes(
    rally_altitude_m: float, offsets: list[tuple[float, float]]
) -> list[float]:
    """Altitude for each slot.

    Every aircraft flies the altitude the operator asked for. A formation is
    already separated on the ground plane by its own spacing, so stacking it
    in height as well buys nothing and costs the operator the one number they
    actually set: ask for 26 m and all three should read 26 m.

    The exception is a formation whose slots are not far enough apart in plan
    to be safe on their own -- a pattern squeezed by the tether, or a spacing
    set near the minimum. Then, and only then, the slots are layered around
    the commanded altitude so that two aircraft pushed off station cannot
    arrive in the same place.
    """
    count = len(offsets)
    closest = min(
        (
            math.hypot(a[0] - b[0], a[1] - b[1])
            for i, a in enumerate(offsets)
            for b in offsets[i + 1 :]
        ),
        default=math.inf,
    )
    if closest >= MIN_AIRCRAFT_SEPARATION_M:
        return [rally_altitude_m] * count

    middle = (count - 1) / 2.0
    altitudes = [rally_altitude_m + (i - middle) * ALTITUDE_DECONFLICTION_M for i in range(count)]
    floor = min(altitudes)
    if floor < 1.0:
        altitudes = [a + (1.0 - floor) for a in altitudes]
    return altitudes


def compute_formation_targets(
    rover_x: float,
    rover_y: float,
    bearing_deg: float,
    pattern: str,
    spacing_m: float,
    rally_altitude_m: float,
    standoff_m: float = FORMATION_STANDOFF_M,
) -> list[Target]:
    """Compute the three aircraft targets for a guided formation launch.

    bearing_deg is a compass heading, 0 to 360, the direction the formation
    faces relative to the rover, section 6.2. standoff_m is how far ahead of
    the rover the formation sits, defaulting into the section 2 scout band.
    """
    if pattern not in FORMATION_PATTERNS:
        raise ValueError(f"unknown pattern: {pattern!r}")

    spacing_m = clamp_spacing(spacing_m)
    standoff_m = max(standoff_m, MIN_AIRCRAFT_SEPARATION_M)

    offsets = _scale_into_tether(_slot_offsets(pattern, standoff_m, spacing_m), TETHER_LIMIT_M)
    altitudes = _slot_altitudes(rally_altitude_m, offsets)

    bearing = math.radians(bearing_deg % 360.0)
    # Compass frame: forward is (sin, cos), right is 90 degrees clockwise of it.
    forward_x, forward_y = math.sin(bearing), math.cos(bearing)
    right_x, right_y = math.cos(bearing), -math.sin(bearing)

    targets: list[Target] = []
    for (dx, dy), altitude in zip(offsets, altitudes):
        x = rover_x + dx * forward_x + dy * right_x
        y = rover_y + dx * forward_y + dy * right_y
        # Every aircraft faces the formation's bearing, so the group presents
        # one sensor direction rather than three.
        targets.append(Target(x=x, y=y, z=altitude, heading=bearing_deg % 360.0))
    return targets
