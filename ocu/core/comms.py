"""Mission mode as a communications state, section 4.1.

The modes are not permission levels on the operator's buttons. They
describe what the fleet is radiating, and therefore what reaches the
operator at all:

    STANDARD    Full RF. Live video and telemetry. Real-time override.
    AUTONOMOUS  Minimal RF, periodic status pings only. The mission is
                pre-loaded; the operator monitors but cannot intervene in
                real time. There is no video, and position and state
                arrive on the ping, not continuously.
    GHOST       Zero intentional emissions. No video, no telemetry, no
                MANET traffic. The mission was uploaded once at base and
                the units are executing it silently. The operator sees the
                last picture before the fleet went quiet, and nothing
                after it. Short-range UWB stays up between the vehicles
                for collision avoidance and precision landing, but it
                never reaches the OCU.
    LCA         Lost Communications Autonomous. Entered automatically when
                the rover loses contact with the operator entirely, never
                chosen. The rover continues its pre-loaded rules of
                engagement and returns to the start point on completion.

The important consequence, and the one that makes Ghost worth showing:
the vehicles keep working. The fleet is not paused. The operator simply
stops being told about it. Freezing the display while the simulation runs
underneath is the honest depiction of that, and it is also the thing
every competitor's demo quietly declines to show.

No Qt import belongs in this file.
"""

from __future__ import annotations

from dataclasses import dataclass

STANDARD = "STANDARD"
AUTONOMOUS = "AUTONOMOUS"
GHOST = "GHOST"
LCA = "LCA"

# How often a status ping arrives in Autonomous. Minimal RF means the
# operator's picture is a sequence of snapshots, not a stream.
AUTONOMOUS_PING_INTERVAL_S = 5.0


@dataclass(frozen=True)
class CommsProfile:
    mode: str
    video: bool
    telemetry: bool
    continuous: bool  # telemetry streams, rather than arriving on a ping
    uplink: bool  # the operator can command the fleet in real time
    mesh_traffic: bool  # units share observations with each other
    summary: str

    @property
    def ping_interval_s(self) -> float:
        return 0.0 if self.continuous else AUTONOMOUS_PING_INTERVAL_S


PROFILES = {
    STANDARD: CommsProfile(
        mode=STANDARD,
        video=True,
        telemetry=True,
        continuous=True,
        uplink=True,
        mesh_traffic=True,
        summary="Full RF. Live video and telemetry, real-time override.",
    ),
    AUTONOMOUS: CommsProfile(
        mode=AUTONOMOUS,
        video=False,
        telemetry=True,
        continuous=False,
        uplink=False,
        mesh_traffic=True,
        summary=f"Minimal RF. Status ping every {AUTONOMOUS_PING_INTERVAL_S:.0f}s, no video, no override.",
    ),
    GHOST: CommsProfile(
        mode=GHOST,
        video=False,
        telemetry=False,
        continuous=False,
        uplink=False,
        mesh_traffic=False,
        summary="Emissions silent. No downlink of any kind. UWB between vehicles only.",
    ),
    LCA: CommsProfile(
        mode=LCA,
        video=False,
        telemetry=False,
        continuous=False,
        uplink=False,
        mesh_traffic=True,
        summary="Contact lost. Fleet continuing on pre-loaded rules of engagement.",
    ),
}


def profile_for(mode: str) -> CommsProfile:
    return PROFILES.get(mode, PROFILES[STANDARD])


def operator_selectable() -> list[str]:
    """LCA is a condition the system enters, not an order the operator gives."""
    return [STANDARD, AUTONOMOUS, GHOST]
