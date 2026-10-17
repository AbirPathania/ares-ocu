"""Where the rover cannot see, and which way to send scouts to look.

When the rover's sensors have reached the end of what they can resolve it
recommends sending scouts to look beyond. The recommendation needs one
input: which direction ahead is least known. That is read here from the map
the rover has actually built, not from a script. It is a recommendation; the
operator launches.

A LiDAR cannot see through a rock or a stand of trees, so ground behind one
is unknown however close it is. Counting unknown cells in an arc ahead of
the vehicle, within the range the sensor is supposed to cover, therefore
measures exactly occlusion rather than distance: unknown ground *inside*
sensor range is ground something is hiding.

No Qt import belongs in this file.
"""

from __future__ import annotations

import math

from ocu.core.lidar import LIDAR_RANGE_M
from ocu.core.terrain import CELL_SIZE_M, PerceptionMap

#: Only ground the sensor should already have covered counts. Inside this
#: range an unknown cell is hidden, not merely far.
NEAR_M = 6.0
FAR_M = LIDAR_RANGE_M - 2.0

#: How far either side of the direction of travel the rover considers. A
#: scout sent behind the vehicle looks at ground it has just driven over.
HALF_ARC_DEG = 80.0
SECTOR_DEG = 10.0
RADIAL_STEP_M = 1.0


def occlusion_bearing(
    perception: PerceptionMap,
    x: float,
    y: float,
    heading_deg: float,
    cell_size_m: float = CELL_SIZE_M,
) -> tuple[float, float]:
    """The compass bearing ahead with the most hidden ground, and how much.

    Returns (bearing_deg, occluded_fraction) where the fraction is the share
    of sampled cells in that sector that are unknown. With nothing hidden
    anywhere the bearing is simply the direction of travel.
    """
    best_bearing = heading_deg % 360.0
    best_fraction = 0.0
    best_offset = math.inf

    sectors = int(2 * HALF_ARC_DEG / SECTOR_DEG) + 1
    for i in range(sectors):
        offset = -HALF_ARC_DEG + i * SECTOR_DEG
        bearing = math.radians(heading_deg + offset)
        unknown = total = 0
        radius = NEAR_M
        while radius <= FAR_M:
            cell = (
                int(math.floor((x + math.sin(bearing) * radius) / cell_size_m)),
                int(math.floor((y + math.cos(bearing) * radius) / cell_size_m)),
            )
            total += 1
            if not perception.is_known(cell):
                unknown += 1
            radius += RADIAL_STEP_M

        fraction = unknown / total if total else 0.0
        # Ties go to whichever is closest to straight ahead.
        if fraction > best_fraction + 1e-9 or (
            abs(fraction - best_fraction) <= 1e-9 and abs(offset) < best_offset and fraction > 0.0
        ):
            best_fraction, best_offset = fraction, abs(offset)
            best_bearing = (heading_deg + offset) % 360.0

    return best_bearing, best_fraction
