"""What the operator has chosen to be told about as a threat.

Detections of every kind arrive from the sensors and the cameras. Which of them are *threats* is the operator's
choice, made per class and changeable at any time (DECISIONS.md, "The operator chooses what counts as a threat"). A
class that is off is a contact: still on the map, still logged, still avoided by the rover, still shared with the
fleet, but it raises no alert and is not in the threat list. The filter is the operator's own view, never the
fleet's record.

No Qt import belongs in this file.
"""

from __future__ import annotations

#: The classes an operator chooses between, in the order they are shown. Each is a group of the spec's threat types.
GROUPS = ("explosives", "arms", "people", "vehicles", "animals", "other")

LABELS = {
    "explosives": "MINES / IEDs",
    "arms": "ARMS",
    "people": "PEOPLE",
    "vehicles": "VEHICLES",
    "animals": "ANIMALS",
    "other": "OTHER",
}

#: Which threat types (the spec's enum, plus the detector's label for the one class it has no word for) fall in a group.
TYPES_IN = {
    "explosives": ("mine", "ied", "disturbed_soil"),
    "arms": ("weapon",),
    "people": ("person",),
    "vehicles": ("vehicle",),
    "other": ("unknown", "safe_object"),
}

#: Groups that can only be switched off deliberately: silencing them by accident is the dangerous mistake.
GUARDED = frozenset({"explosives"})

#: Groups with no source in this build, and why. They cannot be switched on.
UNAVAILABLE = {
    "arms": "No weapon detector is wired into this build. The one measured so far was tested on web photographs only "
    "and says nothing about a rover's camera at range.",
}

#: A clearance and border role: the explosives, weapons and people it is there for. A choice, not a measurement.
DEFAULT_ENABLED = frozenset({"explosives", "arms", "people"})


def group_of(detection: dict) -> str:
    """The class a detection belongs to. An animal is a threat type of `unknown` with the label kept."""
    threat_type = detection.get("type", "unknown")
    if threat_type == "unknown" and str(detection.get("label", "")).lower() == "animal":
        return "animals"
    for group, types in TYPES_IN.items():
        if threat_type in types:
            return group
    return "other"


class ThreatFilter:
    """The operator's choice of classes, with listeners that are told when it changes."""

    def __init__(self, enabled=DEFAULT_ENABLED) -> None:
        self._enabled = {g for g in enabled if g in GROUPS and g not in UNAVAILABLE}
        self._listeners: list = []

    @property
    def enabled(self) -> frozenset:
        return frozenset(self._enabled)

    def is_enabled(self, group: str) -> bool:
        return group in self._enabled

    def is_threat(self, detection: dict) -> bool:
        return group_of(detection) in self._enabled

    def available(self, group: str) -> bool:
        return group not in UNAVAILABLE

    def set_enabled(self, group: str, on: bool) -> bool:
        """Switch a class. Returns whether anything changed; a class with no source stays off."""
        if group not in GROUPS:
            raise ValueError(f"unknown threat class {group!r}")
        if on and group in UNAVAILABLE:
            return False
        if (group in self._enabled) == on:
            return False
        (self._enabled.add if on else self._enabled.discard)(group)
        for listener in list(self._listeners):
            listener(self)
        return True

    def subscribe(self, listener) -> None:
        self._listeners.append(listener)

    def summary(self) -> str:
        names = [LABELS[g] for g in GROUPS if g in self._enabled]
        return ", ".join(names) if names else "nothing"
