# ARES Operator Control Unit

A desktop application that gives an operator command over one ARES-G rover
and three ARES-A aircraft, running natively on a Mac with no hardware, no
ROS installation, and no physics engine dependency. Built to
`ARES_OCU_Technical_Specification.pdf` (v1, 30 September 2026).

**No hardware exists yet.** Every position, detection, and battery
percentage this app displays comes from a scripted mission timeline played
back on a real clock, not a live sensor. That is stated in the app and
should be stated out loud in any demo of it — see section 3 of the spec
and `ocu/sim/mission_timeline.json`.

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

Press **Play** to run the scripted eight-phase mission (deploy through
recharge-and-return, section 5). All three aircraft start docked in the
rover's single bay (section 2) — visible as three color-coded slots on the
map, not separate icons — until they launch, either reactively at the
scripted blind-terrain waypoint or from the **Launch Aircraft** button /
Formation console for a guided formation. **Cut Link** / **Restore Link**
on the map panel is the single most important interaction in the app
(section 8.5): it demonstrates the rover and aircraft maps diverging while
disconnected and reconverging via the CRDT merge on reconnect, with a
delta count showing exactly how much crossed the link.

## The sixty second demo

Section 8.5 calls the cut-link demonstration "the single most important
interaction in the whole application", and says it is the ten seconds
worth rehearsing until it is unmissable. This is that run.

1. **Open it and say the first line out loud.** The top bar reads TRL 2,
   scripted playback, no hardware. Say so before anyone asks. What is real
   is the map merge, the formation geometry, the flight model and the state
   machines; what is scripted is where the vehicles are and what they find.
2. **DEPLOY.** The rover drives and scans. At the blind-terrain waypoint its
   own planner launches two of the three tricopters without being asked, and
   the log says so. The third stays in the bay for you to task.
3. **Watch the threat build.** Four sensors hit the same buried object. They
   merge into one threat, not four rows, with the contributing sensors named
   and a confidence that climbs past the 0.85 escalation threshold.
4. **The link cuts at 00:50.** This is the moment. The banner over the map
   shows the two replicas drifting apart by count, and any observation only
   one side holds gets a dashed ring and an R or A badge. A tricopter finds
   something at (23, 11) that the rover's map cannot contain.
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
anywhere in it and is independently testable; `ocu/sim/playback_engine.py`
is the only thing that reads the scripted mission file and turns it into
Qt signals; `ocu/ui/` is presentation only and holds no equations.

```
ocu/
  main.py
  core/            # pure Python, no Qt import, no magic numbers outside constants.py
  sim/              # QTimer-driven scripted playback
  ui/               # PySide6 widgets, dark theme
  data/             # static per-unit specs
tests/
DECISIONS.md         # open items and every number this build had to invent
```

See [DECISIONS.md](DECISIONS.md) before treating any figure not in the
spec's own tables as sourced — in particular the two section 14 items
(the eighth threat class, and the geofence breach default) still need a
real decision from the team, not this build's placeholder.

## What's real and what's scripted (section 13)

| Element | Status |
|---|---|
| Unit positions | Scripted, from `mission_timeline.json` |
| Buried objects and detections | Real. Physical objects in `core/targets.py` with depth, metal mass, ferrous state and vapour; four sensor response models in `core/detection.py` built from the baseline's own reach figures. What is found depends on where the rover drives |
| Battery and endurance | Real, integrated from the power budget in `core/power.py`. Reproduces the baseline: 430 Wh at 143 W gives 3.0 h, ARES-A cruise gives 11 min, a 30 W pad charges in 9 min |
| CRDT merge logic, `core/mesh_state.py` | Real |
| Formation math, `core/formation.py` | Real, computed live from operator input |
| Launch/recovery state machines | Real, tick on a real clock |
| Camera feeds | Simulated, synthetic HUD panels watermarked "SIMULATED FEED — NO HARDWARE", never real footage (section 9.4's escape hatch, see DECISIONS.md) |
| Terrain ground truth | Generated from a fixed seed. A plausible piece of ground, not a real place |
| The rover's terrain map | Real. Built only from simulated LiDAR returns, with occlusion: unscanned ground stays unknown and obstacles cast shadows that close as the rover drives around them |
| Mission theme | Generic hazard sweep, never labeled minefield |
