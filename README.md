# ARES Operator Control Unit

A desktop application that gives an operator command over one ARES-G rover
and three ARES-A aircraft, running natively on a Mac with no hardware, no
ROS installation, and no physics engine dependency. Built to
`ARES_OCU_Technical_Specification.pdf` (v1, 30 September 2026).

**No hardware exists yet.** The vehicles, the ground and the sensors are
simulated, and that should be stated out loud in any demo of it (section 3
of the spec). The rover navigates itself across a generated landscape from
its own LiDAR map, the aircraft are flown under acceleration limits, what the
ground suite and the scouts' cameras find comes from sensor models over
buried objects, and batteries come from the power budget. Nothing is read
from a script: there is no mission timeline, and nothing happens on a schedule.

## Running it

Double-click **`start.command`** on macOS, or **`start.bat`** on Windows.
The first run builds the virtual environment and installs PySide6, which
takes about a minute; every run after that opens immediately.

To run it by hand instead:

```
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m ocu.main
```

Press **Deploy**. The rover scans, chooses a sweep axis from that scan and
drives it itself, planning around what it finds. **Hold** stops the rover where
it is, with the aircraft keeping station on it; **Resume** carries on. All
three aircraft start docked in the rover's single bay (section 2) — visible as
three color-coded slots on the map, not separate icons — and ride on it until
the operator launches them from the **Launch Aircraft** button / Formation
console. The rover may recommend scouts, from ground it cannot see, and the
dial opens on the bearing it would choose; it never launches anything. Once
launched they are flown, not animated, and hold their slots on the rover as it
drives. **Cut Link** / **Restore Link**
on the map panel is the single most important interaction in the app
(section 8.5): it demonstrates the rover and aircraft maps diverging while
disconnected and reconverging via the CRDT merge on reconnect, with a
delta count showing exactly how much crossed the link.

## The sixty second demo

Section 8.5 calls the cut-link demonstration "the single most important
interaction in the whole application", and says it is the ten seconds
worth rehearsing until it is unmissable. This is that run.

1. **Open it and say the first line out loud.** It is a simulation with no
   hardware behind it. Say so before anyone asks. What is real is the
   navigation, the map merge, the formation geometry, the flight model and
   the state machines; what is simulated is the ground, the sensors and the
   buried objects they find.
2. **DEPLOY.** The rover scans, picks its axis and drives. When it cannot see
   a third of the ground ahead the log says so and recommends a bearing for
   scouts. You launch them: open the Formation console, the dial is already on
   that bearing, pick the pattern and commit. The scouts' feeds look ahead,
   tipped down, and what they spot on the surface is reported from the air.
3. **Watch the threat build.** Four sensors hit the same buried object. They
   merge into one threat, not four rows, with the contributing sensors named
   and a confidence that climbs past the 0.85 escalation threshold.
4. **Cut the link while the fleet is working.** This is the moment. The banner
   over the map shows the two replicas drifting apart by count, and any
   observation only one side holds gets a dashed ring and an R or A badge: the
   rover keeps finding things on its lane, and a scout spots something on the
   surface ahead that the rover's map cannot contain.
5. **Restore the link.** The counts converge, the badges clear, and the log
   names how many observations crossed. Nobody re-sent the whole map.
6. **Then show control.** Open the launch console, pick a pattern, swing the
   bearing dial and watch the formation redraw before you commit. Take
   MANUAL and drive the rover on W/S/A/D. Press RETURN TO BASE and watch the
   tricopters come home with it and wait for it to stop before docking.

The honest line to close on: almost every autonomy pitch claims to work in
a GPS-denied, link-contested environment, and almost none of them show the
failure on purpose and then show the recovery.

## Tests

```
.venv/bin/python -m pytest tests/
```

`tests/test_mesh_state.py` is the one that matters most: it verifies the
CRDT convergence guarantee in section 8.2 directly — the same set of
updates, applied in any order, on any replica, produces a byte-identical
map. That guarantee is the actual patent claim this whole build exists to
demonstrate; everything else is context around it.

## Structure

Follows section 11.2 of the build spec. `ocu/core/` has no PySide import
anywhere in it and is independently testable; `ocu/sim/mission_clock.py`
is the mission clock and nothing else; `ocu/ui/` is presentation only and
holds no equations.

```
ocu/
  main.py
  core/            # pure Python, no Qt import, no magic numbers outside constants.py
  sim/              # the mission clock
  ui/               # PySide6 widgets, dark theme
  data/             # static per-unit specs
tests/
DECISIONS.md         # open items and every number this build had to invent
```

See [DECISIONS.md](DECISIONS.md) before treating any figure not in the
spec's own tables as sourced — in particular the two section 14 items
(the eighth threat class, and the geofence breach default) still need a
real decision from the team, not this build's placeholder.

## What's real and what's simulated (section 13)

| Element | Status |
|---|---|
| Unit positions | Simulated. The rover plans and follows its own route (`core/navigation.py`); aircraft are flown by the flight director under the baseline's speed and acceleration limits, with slots held as offsets from the rover |
| Buried objects and detections | Real. Physical objects in `core/targets.py` with depth, metal mass, ferrous state and vapour; four sensor response models in `core/detection.py` built from the baseline's own reach figures. What is found depends on where the rover drives |
| Battery and endurance | Real, integrated from the power budget in `core/power.py`. Reproduces the baseline: 430 Wh at 143 W gives 3.0 h, ARES-A cruise gives 11 min, a 30 W pad charges in 9 min |
| CRDT merge logic, `core/mesh_state.py` | Real |
| Formation math, `core/formation.py` | Real, computed live from operator input |
| Launch and recovery | Real. A launch is flown (staggered lift-off, climb, transit, station) and its stage is read from the aircraft, not from a timer; the minimum separation is enforced through it. Recovery is the four-layer sequence in section 7 |
| Scout detections | Simulated from the camera's own geometry (`core/aerial.py`): what is on or just under the surface, inside the frame, in range and not hidden. Optical evidence alone stays below the escalation threshold |
| Mission status | Derived from the vehicles' own state (rover mode and sweep progress, each aircraft's flight mode), not announced by a script |
| Camera feeds | Rendered from the simulated world: a forward camera for the rover and, for each tricopter, a forward-looking camera tipped down (18° pitch, 84° field of view; this build's choices, not a published airframe figure). Occlusion, shadows, haze and discrete boulders. Not recorded footage, and not watermarked as simulated on screen; section 9.4 asks for that label, see the open item in DECISIONS.md |
| Terrain ground truth | Generated from a fixed seed. A plausible piece of ground, not a real place |
| The rover's terrain map | Real. Built only from simulated LiDAR returns, with occlusion: unscanned ground stays unknown and obstacles cast shadows that close as the rover drives around them |
| Mission theme | Generic hazard sweep, never labeled minefield |
