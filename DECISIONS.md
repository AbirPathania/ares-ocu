# Decisions and open items

Findings get recorded here rather than worked around silently. Anything in
this file is either an explicit open item carried over from section 14 of
`ARES_OCU_Technical_Specification.pdf` (needs a real decision from Abir,
Kapileshwar, or Bhargav) or a number/behavior this build had to invent
because the spec does not give one. Nothing here should be read as a
sourced baseline figure.

## Open items from section 14, unresolved

1. **The eighth threat class.** The technical baseline says the voxel map's
   `threat_type` field has eight classes but only names seven: mine, IED,
   person, vehicle, disturbed soil, safe object, unknown. `THREAT_CLASSES`
   in `ocu/core/constants.py` currently holds only those seven. The scripted
   alert in `mission_timeline.json` at t=55 uses `"type": "TBD"` rather than
   guessing at a class name, matching the spec's own placeholder. Confirm
   the eighth with Bhargav before locking the enum, then update both
   `constants.py` and the mission timeline.

2. **Geofence breach default.** Section 10 says a breach triggers hold or
   return, "whichever the operator selected before the mission started."
   `ocu/core/geofence.py` implements both as a per-`Geofence` setting
   (`breach_action`) and defaults it to `"hold"` as the more conservative
   choice. That default has not been confirmed anywhere in the project
   record. No UI in this build currently lets the operator configure a
   geofence or its breach action before a mission starts; the class
   supports it, the panels do not expose it yet.

## Resolved, applied as given

- ARES-A is called "compound bicopter" throughout this codebase, matching
  the engineering baseline and patent language. "Nano tricopter" is the
  external/deck term and does not belong in this internal build.

## Numbers and behaviors this build invented, not sourced from the spec

The spec is explicit that inventing numbers is the one thing not to do in
the specification text; these are implementation choices below that layer,
mostly animation timing for a playback engine that has to tick against
*something*. Flagged here so nobody mistakes them for baseline figures
later.

- **`MIN_AIRCRAFT_SEPARATION_M = 3.0`** (`core/constants.py`). Section 6.2
  says spacing must have "a hard minimum, enforced in code," but never
  states the number. 3 m is a placeholder. Get a real figure from
  Bhargav before this ships past a demo.

- **The section 6.4 slot offsets were replaced.** The spec's pseudocode
  gives `vee = [(0,0), (-d,+s), (-d,-s)]`, `line = [(0,-s), (0,0), (0,+s)]`
  and `column = [(0,0), (-d,0), (-2d,0)]`. Every one of those puts a slot at
  `(0, 0)`, which is an aircraft hovering on the rover's own position, and
  the rest at negative X, which is behind it. Flown literally, "line abreast
  for a wide sweep" comes out as a trail strung out behind the vehicle,
  which is what it did until this was fixed. The offsets in `formation.py`
  are built from the *purpose* each pattern is given in the section 6.2
  table instead: line is abreast and level for the widest swath, vee is a
  leader on the track with wings swept back and out, column is single file
  in depth. All of them scout ahead of the rover, at a standoff defaulting
  to the near edge of the 30-50 m scout band in section 2. Treat this as a
  deliberate departure from the pseudocode, not a transcription of it.

- **Bearing is a compass heading, and was 90 degrees out.** The original
  rotation used `x = dx*cos - dy*sin`, which makes bearing 0 point east,
  while the map's bearing dial draws 0 as north. The formation therefore
  never went where the operator pointed. Forward is now `(sin b, cos b)`
  and right is `(cos b, -sin b)`, and `tests/test_formation.py` pins all
  four cardinal bearings so this cannot drift back.

- **Formations scale as a rigid shape at the tether.** Clamping each slot
  to the tether ring independently flattens the formation against the ring
  and turns a vee into an arc. The whole set of offsets is scaled by one
  factor instead, so an oversized formation gets smaller rather than
  deformed.

- **Slot altitudes are deconflicted** by ALTITUDE_DECONFLICTION_M, centred
  on the operator's rally altitude so their number stays the mean. Section
  6.2 describes rally altitude as a single figure for all aircraft; three
  aircraft sharing one layer is a collision as soon as one is pushed off
  station, so this is a deliberate addition.

- **A fifth launch parameter, standoff.** Section 6.2 lists four operator
  inputs (bearing, pattern, spacing, rally altitude). Standoff, how far
  ahead the formation scouts, was added because with the corrected geometry
  it is the difference between a close screen and a far one, and the spec
  gives a band for it (30-50 m, section 2) without giving a control.

- **Formation `d` vs `s`.** The section 6.4 pseudocode uses two symbols in
  `slot_offsets` (forward stagger `d`, lateral offset `s`) but the operator
  only sets one "Spacing" parameter. `core/formation.py` sets `d = s =
  spacing`. If the intent was ever two independent numbers, the launch
  console would need a second field.

- **`CLIMB_DURATION_S = 3.0`, `TRANSLATE_DURATION_S = 4.0`**
  (`core/launch_sequencer.py`). The spec describes the climb-then-translate
  sequence qualitatively (section 6.3) but gives no aircraft climb or
  cruise rate anywhere in the baseline. These are this build's own
  animation timing so the state machine has something to tick against.

- **`APPROACH_DURATION_S = 6.0`** (`core/recovery_sequencer.py`). Same
  situation for the four-stage recovery approach, section 7: precision
  figures (50/10/3 cm) are sourced, closing rate is not.

- **`DEFAULT_ALERT_CONFIRM_TIMEOUT_S = 30.0`** (`core/constants.py`).
  Section 8.3 says "a configurable window" and gives no default.

- **Reactive launch formation math.** Section 6.1 says reactive launch
  "trigger[s] the same launch sequencer described in 6.3" with a bearing
  computed toward the scripted occlusion. The spacing/pattern used to
  compute the two scout targets is not specified for the reactive path
  (spacing and pattern are described as operator inputs for the *guided*
  path in 6.2). `sim/playback_engine.py` reuses `formation.py` with
  pattern `"vee"` (the table's own description, "a forward push," fits a
  forward scout best) and spacing `= 2 * MIN_AIRCRAFT_SEPARATION_M`.

## Architecture notes not forced by the spec but adopted for consistency

- **The link-cut demo models the mesh as two sides, "rover" and
  "aircraft"** (`core/mesh_state.py`'s `MeshLink`), not one replica per
  unit. Section 8.5 describes the demo as "the rover's copy" vs. "the
  aircraft's copy," singular on each side, so this matches the prose
  directly rather than generalizing to four independent replicas.

- **Guided formation launch (section 6.2) offers only the tricopters currently
  docked** (`main_window.py`'s `_docked_state`), not a fixed list of three.
  Once the scripted reactive launch takes tricopter-1 and tricopter-2 airborne, the
  console only offers tricopter-3, and shows "NO AIRCRAFT AVAILABLE" if none
  are docked. This was originally "always offer all three" and produced a
  real bug found by running it: launching a guided formation while the
  reactive launch already had tricopter-1/tricopter-2 airborne made two code paths
  fight over the same tricopter's position. Fixed by tracking dock state as
  the single source of truth for what the console can target.

- **The rover carries all three aircraft in one bay, not as separate map
  icons, until each launches** (section 2: "stored folded in a single
  drawer bay on the top deck"). `map_view.py`'s `_draw_rover_bay` renders
  three slots directly above the rover glyph; a docked aircraft is not
  drawn as its own unit on the map at all (`MapView.set_unit_docked`), and
  its camera feed thumbnail shows "NO SIGNAL — DOCKED" until it launches,
  either reactively (`playback_engine.py`'s `unitDockedChanged` signal) or
  from the guided console (`launch_console.py`'s `aircraftUndocked`).
  Clicking a filled bay slot, or the "LAUNCH AIRCRAFT" button on the map
  toolbar, jumps straight to the Formation/Launch console.

- **Camera feeds are back in the build**, reversing section 13's own "left
  out of version one" call, at Abir's explicit request. Section 9.4's own
  escape hatch covers this: "add it later only with SIMULATED FEED clearly
  labeled if a future version needs one." `ocu/ui/camera_feed.py` draws a
  synthetic HUD panel (corner brackets, crosshair, translucent noise bars)
  with "SIMULATED FEED — NO HARDWARE" burned into the panel itself, never
  anything that could be mistaken for a photograph or real sensor output.

- **Terrain features on the tactical map are cosmetic**, a fixed set of
  soft vegetation/rock/contour shapes generated once from a hardcoded seed
  (`map_view.py`'s `TERRAIN_SEED`). There is no real terrain, elevation, or
  GIS data anywhere in this build or in the project record; do not read
  the map's texture as a real location.

- **The alert confirm countdown runs on real wall-clock time**
  (`alert_banner.py`, 100 ms real ticks), not scaled by the playback
  engine's speed multiplier. This is deliberate: the 30 s window is
  standing in for how long a human actually gets to decide, which
  shouldn't compress just because the demo is fast-forwarded. The
  consequence, found by running the app at 2.5x/4x for this build's own
  screenshots: at high playback speeds the scripted mission can finish (or
  move well past mark-and-route) while an alert raised near the end is
  still counting down in real time. At normal 1x speed this doesn't come
  up — the alert at t=55 resolves by t=85, well inside the 90s mission.

- **Per-unit Emergency Return** (`core/unit.py`'s
  `evaluate_emergency_return`) and the **geofence breach check**
  (`main_window.py`, `_on_unit_pose_changed`) are wired into the code path
  but nothing in the current mission timeline actually triggers them — the
  scripted mission never lets a unit's link go past the 15 s Emergency
  Return threshold, and no geofence is configured by default. Exercised by
  unit tests, not by the demo playback.

## Interface decisions, second UI pass

The driving question became: what does an operator in the field actually
need on screen, and what is just the platform describing itself? Anything
in the second category was cut.

- **The detection suite is live state, not an inventory.** It previously
  printed four bullet points naming the sensors the rover carries, which
  an operator already knows. It now shows each sensor as SCANNING, HIT
  (for a few seconds after that sensor contributes a detection), or IDLE.
  The perception stack stays a visibly separate group per section 9.4's
  explicit rule, but compressed to one line, because its job here is to
  prove the two groups are not one list, not to enumerate itself.

- **The right panel follows the selection instead of using tabs.** Section
  9.4 describes this panel by selection ("Rover / An aircraft / The
  formation"); the first cut made the operator find the right tab first,
  which answered no question. Selecting a unit now shows that unit.

- **Threats are keyed by voxel, not by event**, in both the threat list
  and on the map. Four sensors hitting the same buried object is one thing
  on the ground; the first cut drew it four times with four stacked labels
  and listed it as four rows. Merging by voxel with highest-confidence-wins
  is the same rule `core/mesh_state.py` applies, so the operator's view and
  the shared map cannot disagree.

- **The link strips are rolling history, not current state.** A bar
  showing the present link state duplicated the dot already on each roster
  card. Sixty seconds of scrolling history instead shows *when* a link
  dropped and for how long, which is the whole point of the section 8.5
  cut/restore demonstration and is otherwise unavailable anywhere.

- **The fleet roster is cards, not a table.** Columns truncated to
  ellipses at any sane panel width, and a table makes the operator read
  four columns of text to answer "is anything wrong". Battery is a bar,
  link is a colored dot, so the answer arrives before any text is read.

- **A mission phase rail replaces the phase text label**, showing all
  eight phases of section 5 with the current one lit. The phase name says
  what the machine is doing; the rail says how far through the mission
  that is and what is next.

## Bugs found by running the thing

- **Docked tricopter-1/tricopter-2 rendered as separate floating icons before
  launch.** First cut of `mission_timeline.json` gave tricopter-1 and tricopter-2
  their first keyframe at t=30 (already airborne), so before that the
  interpolator clamped to that same position from t=0 — both aircraft
  appeared already in flight, off the rover, from the very start of the
  mission. Symptom was visible directly in a screenshot: two triangle
  icons sitting near the rover square at t=18, before any launch phase.
  Fixed by adding a t=0 keyframe at the rover's start position for both,
  and by adding real docked-state tracking (`unitDockedChanged`,
  `aircraftUndocked`) so a docked aircraft isn't drawn as a standalone map
  unit at all, regardless of what its keyframes say.

- **Guided launch console always targeted all three tricopters**, so opening
  the Formation tab after the scripted reactive launch already had
  tricopter-1/tricopter-2 airborne and then pressing Launch would restage and move
  those two aircraft out from under the running reactive launch. Fixed as
  described above by filtering the console's available tricopters to whatever
  is actually still docked.

- **Two position writers fought over the same aircraft.** Launching a
  guided formation before the scripted reactive launch fired left both the
  launch console and the playback engine writing the same tricopter's pose ten
  times a second. The map drew the fight as large filled fans of zigzag
  trail, which is how it was spotted — it looked like a rendering bug and
  was actually a state-ownership bug. Fixed with `_position_owner` in
  `main_window.py`: every unit has exactly one writer, scripted by default,
  and the guided console takes ownership of what it launches and keeps it.
  The earlier "filter the console to docked tricopters only" fix reduced how
  often this fired but never addressed the cause, since the operator can
  launch all three while the scripted mission still intends to move two.

- **Right-panel tabs and the fleet table columns truncated to ellipses**
  at the panel widths the splitter picked by default — Qt's computed
  `QTabBar` width didn't stretch to the `QTabWidget`'s full width even
  with `setExpanding(True)`, and `QTableWidget` columns need
  `ResizeToContents` on every column, not just the first, to avoid
  reserving almost nothing for the later ones. Fixed with explicit
  `min-width` on `QTabBar::tab` in `style.qss`, explicit initial
  `QSplitter.setSizes(...)`, and abbreviated link-state text
  (`CONNECTED` → `OK`) so the roster fits without a horizontal scrollbar.
