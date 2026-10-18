"""What things are called on screen.

The operator's vocabulary, in one place. These names appear on the roster,
the map, the camera tiles, the threat list and the event log, and an
abbreviation that means one thing in the threat list and another in the log
is a reader's problem rather than a writer's convenience. They lived in
whichever panel happened to need them first, which is how that drifts.
"""

from __future__ import annotations

#: Sensors, as an operator refers to them. Short enough to sit in a column,
#: long enough that nobody has to look them up.
SENSOR_ABBR = {
    "gpr": "GPR",
    "pulse_induction": "PI",
    "magnetic_gradiometer": "MAG",
    "saw_voc": "VOC",
    "thermal": "THRM",
    "optical": "OPT",
    "fused": "FUSE",
}


def short_label(unit_id: str) -> str:
    """A unit's call sign: ROVER, TRI-1."""
    if unit_id.startswith("tricopter-"):
        return "TRI-" + unit_id.split("-")[-1]
    return unit_id.upper()


def sensor_label(sensor: str) -> str:
    """One sensor, named. Anything unrecognised is shown as given rather than
    silently truncated into something that reads like a different sensor."""
    return SENSOR_ABBR.get(sensor, sensor.upper())


def sensor_list(sensors) -> str:
    return " ".join(sensor_label(s) for s in sensors)
