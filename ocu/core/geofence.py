"""Geofence boundary and breach behavior, section 9.3 and section 10.

The boundary itself is operator-configured before a mission starts ("a
geofence boundary if one is configured", section 9.3). Breach behavior is
an explicit rule: "A geofence breach triggers an automatic hold or an
automatic return, whichever the operator selected before the mission
started" (section 10). That choice is not specified anywhere in the ARES
documents, section 14 flags it as open pending confirmation with Abir.
This build defaults to "hold" as the more conservative behavior and makes
it a per-mission setting rather than hardcoding either option. Do not treat
"hold" as a confirmed decision, it is this build's placeholder default.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Geofence:
    vertices: list[tuple[float, float]] = field(default_factory=list)
    breach_action: str = "hold"  # "hold" or "return", operator's pre-mission choice

    def __post_init__(self) -> None:
        if self.breach_action not in ("hold", "return"):
            raise ValueError(f"unknown breach_action: {self.breach_action!r}")

    @property
    def configured(self) -> bool:
        return len(self.vertices) >= 3

    def contains(self, x: float, y: float) -> bool:
        """Ray-casting point-in-polygon test. An unconfigured geofence contains everything."""
        if not self.configured:
            return True

        inside = False
        n = len(self.vertices)
        x1, y1 = self.vertices[-1]
        for x2, y2 in self.vertices:
            crosses = (y1 > y) != (y2 > y)
            if crosses:
                x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < x_at_y:
                    inside = not inside
            x1, y1 = x2, y2
        return inside

    def check_breach(self, x: float, y: float) -> str | None:
        """Returns this geofence's configured action if (x, y) is outside it, else None."""
        if not self.configured:
            return None
        return None if self.contains(x, y) else self.breach_action
