"""What a scout sees from the air.

A scout carries an optical and thermal camera, not a ground-penetrating
array. What it can find is what is on the surface or barely under it, and
only inside what the camera actually covers: the field of view, the range at
which something that small is still resolvable, and with nothing in the way.
The frustum here is the same one the rendered feed is drawn with, so what a
scout reports is what is in its picture.

Optical evidence alone is weak by design. It says something is there that
should not be; it cannot say what. It stays below the escalation threshold on
its own and strengthens as the scout closes in, and combined with the
rover's ground-penetrating evidence on the same object it does what the
fusion rule says any two independent sources do.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math

from ocu.core.camera import (
    SCOUT_HFOV_DEG,
    SCOUT_NOMINAL_ASPECT,
    SCOUT_PITCH_DOWN_DEG,
    CameraRig,
)
from ocu.core.targets import BuriedTarget

#: Deeper than this and there is nothing at the surface for a camera to see.
MAX_VISIBLE_BURIAL_M = 0.10

#: What the camera can resolve. A scout's sensor is 1280 pixels across the
#: 84 degree field of view, so a pixel subtends 1.1 milliradians; something
#: has to be about four pixels wide to be told from the ground, and a surface
#: object of interest is about a quarter of a metre. That puts the limit at
#: 0.25 / (4 x 0.00115) = 54 m of slant range, which from the scout's altitude
#: is a little under 50 m across the ground. Detection is graded: the nearer it
#: is, the larger it looks, and the surer the scout is.
SENSOR_WIDTH_PX = 1280
MIN_PIXELS_ON_TARGET = 4
SURFACE_OBJECT_SIZE_M = 0.25
MAX_SLANT_RANGE_M = SURFACE_OBJECT_SIZE_M / (
    MIN_PIXELS_ON_TARGET * math.radians(SCOUT_HFOV_DEG) / SENSOR_WIDTH_PX
)

#: A camera swung faster than this smears small things into the background. A
#: scout turning its camera through a formation change does not see anything it
#: sweeps over.
MAX_YAW_RATE_FOR_SIGHTING_DEG_S = 20.0

#: A camera's evidence that something is out of place, however close it gets.
#: Below the escalation threshold on purpose: it cannot say what it is.
MAX_OPTICAL_CONFIDENCE = 0.62
MIN_OPTICAL_CONFIDENCE = 0.14

#: Vertical field of view implied by the nominal frame shape.
SCOUT_VFOV_DEG = math.degrees(
    2.0 * math.atan(math.tan(math.radians(SCOUT_HFOV_DEG / 2.0)) * SCOUT_NOMINAL_ASPECT)
)


def scout_sighting(
    rig: CameraRig,
    target: BuriedTarget,
    x: float,
    y: float,
    height_above_ground_m: float,
    heading_deg: float,
    yaw_rate_deg_s: float = 0.0,
) -> float:
    """Confidence with which a scout at (x, y) sees `target`, 0 if it does not.

    The scout holds `height_above_ground_m` over the ground beneath it and
    looks along `heading_deg`, tipped down by the camera's pitch.
    """
    if target.depth_m > MAX_VISIBLE_BURIAL_M:
        return 0.0
    if abs(yaw_rate_deg_s) > MAX_YAW_RATE_FOR_SIGHTING_DEG_S:
        return 0.0

    dx, dy = target.x - x, target.y - y
    distance = math.hypot(dx, dy)
    if distance < 1.0:
        return 0.0

    azimuth = (math.degrees(math.atan2(dx, dy)) - heading_deg + 180.0) % 360.0 - 180.0
    if abs(azimuth) > SCOUT_HFOV_DEG / 2.0:
        return 0.0

    camera_z = rig.elevation_at(x, y) + height_above_ground_m
    target_z = rig.elevation_at(target.x, target.y)
    elevation = math.degrees(math.atan2(target_z - camera_z, distance))
    # The optical axis points `pitch` below the horizon; the picture reaches
    # half the vertical field of view either side of it.
    if abs(elevation + SCOUT_PITCH_DOWN_DEG) > SCOUT_VFOV_DEG / 2.0:
        return 0.0

    if not rig.line_of_sight(x, y, camera_z, target.x, target.y, target_z + 0.2):
        return 0.0

    slant = math.hypot(distance, camera_z - target_z)
    if slant > MAX_SLANT_RANGE_M:
        return 0.0
    closeness = 1.0 - slant / MAX_SLANT_RANGE_M
    return MIN_OPTICAL_CONFIDENCE + (MAX_OPTICAL_CONFIDENCE - MIN_OPTICAL_CONFIDENCE) * closeness**0.7
