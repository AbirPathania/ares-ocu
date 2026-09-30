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
| Unit positions, battery, detections | Scripted, from `mission_timeline.json` |
| CRDT merge logic, `core/mesh_state.py` | Real |
| Formation math, `core/formation.py` | Real, computed live from operator input |
| Launch/recovery state machines | Real, tick on a real clock |
| Camera feeds | Simulated, synthetic HUD panels watermarked "SIMULATED FEED — NO HARDWARE", never real footage (section 9.4's escape hatch, see DECISIONS.md) |
| Map terrain features | Cosmetic, procedurally generated from a fixed seed — not real GIS data |
| Mission theme | Generic hazard sweep, never labeled minefield |
