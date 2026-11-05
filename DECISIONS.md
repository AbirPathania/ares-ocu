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

- **Battery moved from scripted to derived**, which changes its section 13
  classification. The mission file used to carry hand-written battery
  curves and they were wrong by about an order of magnitude: the tricopters
  discharged in 73 s against a published 10 to 12 minutes, the rover in 8
  minutes against a published 3.0 hours, and a docked aircraft charged in
  under 2 minutes against a published 8 to 10. Battery is now integrated
  from `core/power.py` using the real motion state, and the published
  figures fall back out of it. Deriving it is more honest than scripting
  it, not less: the numbers now agree with the spec sheet by construction
  rather than by my having typed them correctly.

  What is genuinely this build's model, and not from the baseline: the
  split of the rover's 143 W between a 45 W standing load and a
  speed-dependent term, and the factors by which a rotorcraft's draw
  differs between hover, cruise, climb and descent. The baseline gives
  totals, not breakdowns. The factors are chosen so both published rover
  figures (3.0 h continuous, about 4 h mixed duty) and the published ARES-A
  cruise band fall out of one model.

- **LiDAR range, angular resolution and scan rate** (`core/lidar.py`:
  32 m, one ray per degree, 4 Hz). The technical baseline names LiDAR in
  the rover's perception stack and says phase 2 runs "LiDAR inertial SLAM",
  but specifies no unit. These are ordinary figures for a ground vehicle's
  scanner and are this build's model.

- **The terrain itself is generated, not surveyed.** `core/terrain.py`
  builds a classified world from a fixed seed: open ground, a worn track,
  scrub, tree cover and rock. It is a plausible piece of ground, not a real
  place, and nothing about it should be read as site data. What is real is
  the *process*: the rover's map is built only from what its scanner has
  actually reached, with the ground truth kept strictly separate from the
  rover's belief, so sensor shadows appear and close exactly where
  occlusion says they should.

- **Detections are now simulated, not scripted**, which changes their
  section 13 classification along with battery. The mission file used to
  list detection events that fired at a timestamp; nothing was buried
  anywhere and every confidence on screen was a number somebody typed.
  `core/targets.py` now holds physical objects with a depth, a metal mass,
  whether that metal is ferrous, and a vapour emission, and
  `core/detection.py` holds a response model per sensor. What appears on
  screen is the output of a geometry calculation over what the rover
  actually drove across.

  Every reach figure comes from the baseline: GPR 0-40 cm depth and 30 cm
  scan width, pulse induction 40 cm sweep and 5 g of metal at 15 cm,
  gradiometer fluxgates 50 cm apart and ferrous only, SAW VOC to 2 m
  downwind and confirming only. What is this build's model is the *shape*
  of each response between those limits, since the baseline states ranges
  and not falloff curves. The curves used are the physically expected
  ones: an induced dipole falling as the cube of range for pulse
  induction, a gradient falling as the fourth power for the gradiometer,
  sharp dispersal downwind for vapour.

  Fusion is deliberately not a weighted average, which cannot express the
  thing that matters: two sensors working on *different physics* agreeing
  about one point is far stronger evidence than either being individually
  confident. Section 8.1 singles out co-located GPR and pulse induction as
  the biggest lever on false alarm rate, and that pairing earns the largest
  cross-confirmation bonus.

- **No fused confidence is ever reported above 0.97.** A classifier that
  outputs certainty is either overfitting or lying, and an operator who
  sees 1.00 stops reading the number.

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

## Autonomous navigation and the clearance sweep

**Why the navigation stack is implemented rather than imported.** Section
11.1 of the technical baseline rules out ROS 2 (Tier 3 on macOS), Gazebo
(the Metal threading crash open since 2021) and MuJoCo ("a separate and
much larger undertaking") for this build. Nothing in that decision is
revisited here. What it rules out is also not what was needed: MuJoCo is a
rigid-body contact solver, and the question this demonstrator has to answer
is not how a wheel loads a rock but how a vehicle maps ground it has not
seen, plans across it, and changes its mind when the map changes. That is
the nav2 stack, and its parts are small enough to implement directly and
test: a costmap with obstacle inflation, A* over it, replanning on
invalidation (what D* Lite does more efficiently; at this map size a fresh
A* is cheaper than the bookkeeping to avoid one), and pure pursuit to
follow the result. MuJoCo remains the right tool if rocker-bogie contact
dynamics ever become the question. See `ocu/core/navigation.py`.

**The sweep axis is chosen, not fixed.** A clearance sweep laid across a
rock field is a sweep the vehicle spends its mission detouring around, and
ground that is detoured around is ground nobody looked at. The axis is
scored against the terrain at every fifteen degrees and the clearest taken
(`ocu/core/sweep.py`). Scoring against ground truth is mission planning,
not navigation: a clearance task is planned on prior imagery before
anything drives into it. Everything after that point uses only the map the
rover builds for itself.

**Lane points, not lane ends.** A* minimises traverse cost, which is the
right objective for getting from A to B and the wrong one for clearance:
given scrub on the lane and bare ground two metres off it, the cheapest
route is off the lane. Two corrections, both in the objective rather than
in the output. Lanes carry a point every 3 m, and each leg is planned with
a corridor penalty on distance from the lane (`navigation.Corridor`). The
penalty is a preference, not an override -- a rock on the lane is still a
rock, and the vehicle goes round it and reports the lane points it cannot
reach at all.

**Buried objects are placed relative to the sweep.** Since the axis is
chosen from the terrain, an object buried at a fixed coordinate would end
up wherever the lanes did not go the moment the ground changed, and the
demonstration would silently become one where the vehicle sweeps empty
dirt. `targets.default_field` places them by lane, distance along it and
offset across it.

**Wind bearing is derived from the sweep, and is a scenario choice.** The
SAW VOC sensor only reads when the vehicle is on the plume side of a
source. With the axis chosen from terrain, a fixed wind bearing would put
the plume along or across the lanes by luck of the seed. It is set 25
degrees off the reciprocal of the outbound lane heading, which means the
vapour sensor confirms on outbound lanes and not on return ones. The
sensor model is unchanged; only the environment is chosen.

**Classification reports only the seven named classes.** An earlier
revision returned the literal string "TBD" for metal-plus-vapour, which put
section 14's open item in front of the operator as a threat label. Metal
with explosive vapour is reported as `mine`: telling a mine from an IED
takes more than four sensors, so of the two the suite reports the one it
can support. The eighth class remains unnamed and unguessed.

**The swept band is drawn at its true width.** The pulse induction array is
40 cm across, and the map shows exactly that against the thousands of
square metres the LiDAR maps. The vehicle sees a long way and clears a
hand's breadth; that is the honest shape of clearance work and it is not
flattered here.

## Camera feeds

**Rendered from the world, not drawn over it.** The tiles previously showed
a gradient with random bars on it. They now show views rendered from the
same ground truth the sensors are simulated against (`ocu/core/camera.py`):
a forward camera for the rover, straight-down cameras for the tricopters. A
camera sees the world and not the map, so a tile shows what is actually out
there, including ground the LiDAR has not reached and objects it has not
yet classified.

**Method.** The rover's view is the standard heightfield projection: a ray
per pixel column, ground and standing objects projected by depth. Occlusion
needs no special handling, because a pixel belongs to the nearest sample
whose projected top has reached it. That is a running minimum and an argmax,
so every sample along every ray is resolved in one batch rather than in a
per-step loop. The aircraft view is a nadir projection whose footprint is
set by altitude, so climbing widens what is seen and costs resolution.
Tall things cast shadows away from a fixed sun, because from above a tree is
a green shape and its shadow is what says it is tall.

**Smooth fields, not cells.** The terrain is half-metre cells. Sampled
directly, every slope becomes a staircase and every tree a tower. Ground
colour, elevation, and the height of standing things are built once as smooth
fields and sampled bilinearly; edges of crowns and rocks are thresholded
against fine world-anchored noise so they are ragged the way real ones are,
and texture is value noise rather than a constant per cell, which would show
as squares the moment a cell covers several pixels.

**Cost.** One tile is rendered per 85 ms tick, in turn, so no tick does more
than one frame of work and each tile refreshes about three times a second.
Frames are rendered at roughly 1/1.7 of tile size and scaled up smoothly.

**The link decides, not the camera.** In Autonomous the aircraft's camera is
running and the link is not carrying video; in Ghost nothing is radiated. In
both the tile keeps saying so (`set_downlink`) and no new frames arrive. A
unit in its bay has no picture, and a recalled aircraft's last frame is
cleared rather than left on screen.

**Open: the section 9.4 label.** Section 9.4 of the technical baseline says a
camera feed, if ever added, should be "clearly labeled" as simulated. The
tiles carry no such label on screen, as they did not before this change. The
views are rendered rather than recorded, and nothing on screen says so. That
is a presentation decision, and it is Abir's, not this file's.

**Decided (2026-10-24, Abir): no on-screen label.** The camera tiles stay unlabelled. The README's "What's real and
what's simulated" table says in words that the feeds are rendered from the simulated world and are not recorded footage,
and that is where the disclosure lives.

**Not modelled.** Lens distortion, exposure adaptation, motion blur, rain or
dust on the lens. The aircraft's own shadow and the other aircraft are not
drawn into its frame. The rover is drawn into an aircraft's frame when it is
inside the footprint, because a vehicle is part of the scene an aircraft
looks down on.

## Launch is flown, and the formation is held on the rover

**What was wrong.** A launch was an animation: each aircraft was interpolated
along a fixed line over a fixed number of seconds, aimed at a world position
worked out when the launch was staged. Measured on the real window with the
rover driving, that took an aircraft from rest to 7.5 m/s in one tick (peak
acceleration about 75 m/s², roughly 7 g), and when it finished every aircraft
was 5.1 m off the pattern the operator had chosen, because the rover had
driven on during the launch and `adopt()` then recorded whatever offset each
aircraft happened to be at as its slot.

**What it is now.** The flight director flies the launch (`FlightDirector.launch`):
each aircraft waits in the bay riding the rover, lifts off on the staggered
schedule, climbs straight up over the rover to its own altitude, crosses to its
slot and holds it. Slots are offsets *from the rover*, so the formation is a
shape held on a moving vehicle. Everything goes through the same acceleration
limits as the rest of the fleet, and the rover's velocity is fed forward so a
slot is held rather than trailed (0.28 m at 2 m/s, against 0.80 m without it).
`LaunchSequencer` no longer moves anything; it records the plan and takes its
stage from the aircraft, so FORMED means every aircraft is on its slot, and
the launch takes as long as the physics takes (about 19 s, not 7).

**Separation.** Three aircraft leave one spot within seconds of each other.
Measured, the airborne minimum was 1.9 m against the 3.0 m hard limit. A
climbing aircraft now holds its altitude rather than climb into one above it
that is still launching. Aircraft parked on the pad are not counted as flying.
The guard only looks at aircraft that are themselves still launching, so it
cannot deadlock on one that is simply holding its slot.

**A defect in the shared motion code, found by measuring it.** The terminal
settle ramped the velocity down and then zeroed it in the same tick, so the
last step could be twice the acceleration limit. Invisible on screen, but it
broke the limit that is claimed. Settling is now judged on the velocity going
into the tick, in both the vertical and horizontal paths.

**Facing.** Moving to its slot an aircraft faces where it is going; holding it,
the formation bearing. Every aircraft in a formation faces the same way, so the
group presents one sensor direction. The launch dial now starts on the rover's
heading rather than north.

## Reactive launch is the rover's own decision

Section 6.1's reactive launch used to start from a marker in the mission file
and move the aircraft along scripted keyframes written for a rover that no
longer follows that route. The aircraft were therefore flying somewhere
unrelated to the vehicle they belong to. The script no longer places any
aircraft (`_on_unit_pose_changed` ignores tricopters; in the bay they ride the
rover, afterwards the director flies them).

The marker is kept as the mission plan's schedule, since "the aerial-recon phase
begins now" is a fact about the plan. What the rover does at that point is its own:
`core/occlusion.py` reads its map, counts unknown cells inside sensor range in
each sector ahead (unknown *inside* range is hidden ground, because a LiDAR
cannot see through a rock), and sends two scouts toward the most hidden sector.
The event log says what it saw ("36% of the ground ahead is hidden, committing 2
scouts on 280°").

## The sweep axis is chosen from the first scan, and nothing is drawn ahead of the rover

The four dashed lane lines used to be drawn on the map before the rover had
moved, which made the path look scripted. They are gone. The map shows the
planned route as the planner makes it and the ground actually swept.

The axis was also chosen by scoring the ground truth. It is now chosen by the
rover from its own first scan, with ground it cannot see scoring half clear:
neither assumed drivable nor assumed blocked. The scenario's buried objects are
placed relative to that sweep, so the same choice is made ahead of time by
scanning from the same pose (`sweep_from_first_scan`); a scan from the same
place marks the same cells, so the two cannot differ, and a test holds that for
several seeds. That placement is scenario authoring and is a demonstration
arrangement: objects are planted where the rover will sweep.

## Rocks are boulders

Rock used to be every cell above a slope threshold, which made whole hillsides
solid bands. It is now a population of individual boulders (`Terrain._scatter_boulders`):
lognormal in radius, rarely round, partly buried, sparse on flat ground, thick
on slopes, heaped into outcrops where it is steepest. Each is a half-ellipsoid
dome, so the world holds a height field; the ROCK class is where a boulder
stands over 30 cm, and the cameras draw the domes themselves rather than
inferring a shape from a mask. On the map, rock is a cold pale grey that nothing
else is (it used to share the track's tan). A LiDAR returns the face it hits and
nothing behind it, so a boulder is drawn as that face with its shadow behind.

## Scout cameras look forward

A nano-class scout carries a gimballed forward-looking camera, not a straight-down
one, because what it is for is seeing ahead of the formation. The tricopter tiles
now use `CameraRig.scout_view`: the same renderer as the rover's, from the
aircraft, tipped down. **Pitch (18°) and field of view (84°) are this build's
choices, not published figures of any airframe.** Scouts hold height over the
ground beneath them, not over a datum. The nadir view is kept for a camera that
really does look down.

The oblique views are resolved in angle space: every pixel has a true ray, and a
pixel belongs to the nearest sample whose top has reached its elevation. That is
exact for a pitched camera, where the usual shortcut of treating image columns as
constant-azimuth rays is only right with the camera level. A test checks the
horizon against f·tan(pitch) rather than trusting that it looks right.

## Nothing flies, cuts or announces itself

**Aircraft launch only when the operator launches them.** The reactive launch
from section 6.1, where the rover's planner commits scouts itself, is not
implemented, on the operator's instruction: aircraft must not come out unless
asked. This departs from the baseline, which describes the rover committing
them. The rover keeps the part that is a judgement about its own map: when a
third of the ground ahead is hidden (`SCOUT_RECOMMEND_HIDDEN_FRACTION`) it says
so once in the log, with the bearing it would choose, and the launch dial opens
on that bearing. Launching is still the operator's press.

**The mission timeline is gone.** `mission_timeline.json` and the playback
engine that played it back are deleted. They carried phase markers ("alert",
"fusion", "recharge and return") that fired at fixed times whether or not they
were true, a link cut and restore nobody had asked for, and keyframes for a
rover route that no longer exists. What remains is a mission clock
(`sim/mission_clock.py`). The top bar and the roster are derived from the
vehicles' own state (`_refresh_status`): the rover's mode and where it is in its
sweep, each aircraft's flight mode.

**The speed readout was wrong, not the rover.** Measured, the rover's top speed
over a full sweep is 2.17 m/s (7.8 km/h), well inside the 15 km/h limit. The
readout divided how far the position had moved by how much mission clock had
elapsed, but the position moves on the flight director's timer and the clock on
another, so two moves landed in one interval some of the time and none in the
next, and a clamp at the top speed hid the doubling (six samples sat on the clamp).
It now shows the vehicle's own speed.

**Hold holds.** Pressing Hold used to pause the clock while the rover kept
driving. It now stops the rover where it is; aircraft keep station on it.

**Scouts see things.** With the timeline gone the aircraft had nothing to find,
so a cut link diverged on nothing: a demo run reported "0 observation(s) crossed
to reconcile". Scouts now detect what their cameras can see (`core/aerial.py`):
objects on or within 10 cm of the surface, inside the field of view and pitch the
picture is rendered with, in range, with a clear line of sight over the same
heights the picture uses. Optical evidence tops out at 0.62, below the 0.85
escalation threshold, because a camera can say something is out of place and not
what it is; it adds to the rover's evidence under the same fusion rule if both
see it. Scout evidence is held separately from the rover's, since they are
different replicas and cannot share it over a cut link. Two surface-laid objects
sit on the lanes the sweep has not reached, for scouts looking along the sweep.

**Reports are throttled to news.** A reading that creeps up by a thousandth as
the vehicle moves was logged each time (seven identical lines in two seconds).
A reading is now reported when it rises by 0.05, and always when it crosses the
escalation threshold.

**Reset forgets.** The per-object evidence and the set of already-escalated
objects were never cleared on reset, so after one the rover would stay silent
about everything it had found and no alert could fire again.

**The geofence is checked on real positions.** It was only ever checked against
scripted poses, so real motion was never tested against it. It is unconfigured by
default, so nothing changes visibly, but the check is on the right data.

## Pace, and what the operator's numbers mean

**The rover sweeps at a survey speed.** It was driving lanes at up to 2.17 m/s
(7.8 km/h). The platform's 15 km/h is a transit figure: how fast it can move,
not how fast it can clear ground. A detection pass is limited by the sensors.
The pulse induction array is 40 cm wide, and ground-penetrating radar needs
dense trace spacing to form a subsurface picture, which is why survey GPR is
walked rather than driven. `ROVER_SURVEY_SPEED_MS = 0.8` (2.9 km/h, a slow
walk) is this build's figure, not the baseline's. Driving home is transit and
is not held to it: the crawl is the sensors' doing, and when nothing is being
cleared there is nothing to crawl for. The full lane sweep now takes about
2:10 of mission time instead of 1:07.

**Playback speed is a real control, and it was half-wired.** The mission clock
had a speed multiplier that scaled the clock readout and the battery
integration but not the vehicles, so a fast-forwarded run drove at normal
speed under a fast clock. One mission `dt` now feeds the director, the power
model and the scan schedule, and the top bar carries a ½×/1×/2×/4× control.
Playback speed decides when you watch, not how fast anything drives.

**A long tick is split before it is integrated.** At 4× a tick covers 0.4 s of
mission time, and a follower that re-aims once per step tracks its path more
loosely the longer the step gets. `MAX_INTEGRATION_STEP_S` splits anything
longer, so one long tick is the same motion as that much mission time taken in
short ones. Measured: at every playback speed the rover is within 0.19 m after
40 s of mission time. The residual is the control loop above the integrator
(replanning, taking the next lane point) running once per tick like a real
one, not the motion model.

**Rally altitude means rally altitude.** Ask for 26 m and all three aircraft
read 26 m. They used to be layered to 23/26/29 by an automatic ±3 m
deconfliction, which took the one number the operator actually sets. A
formation is already separated on the ground plane by its own spacing, so the
layering bought nothing; it is now applied only when the slots are closer
together in plan than the minimum separation, which happens when the tether
squeezes a pattern or the spacing is set near its floor.

## Holes in the ground

**A pothole is impassable but not occluding**, which is the first thing in
this model that is one and not the other: `OCCLUDING` used to equal
`IMPASSABLE` and a test asserted it. A hole stops the wheels and lets the beam
straight over, so the sets are now properly nested and the test says so.

**Negative obstacles have their own, much shorter range.** A scanning LiDAR
sweeping a horizontal plane cannot see a hole: it passes over and returns off
the ground beyond, reporting open ground perfectly honestly. Finding a hole
means looking *down*, which is the drive stereo pair's job and only works near
the vehicle (`NEGATIVE_OBSTACLE_RANGE_M = 8 m`). Short detection range is
exactly what makes negative obstacles dangerous, and it is why the rover meets
these and replans rather than routing around them from the start.

**A mapped hole is not unlearned.** Found by driving it: the rover resolved a
washout at 8 m, drove on, and the far-field reading of the same cells -- honest
open ground, because the beam passes over -- overwrote the hazard. The map
forgot every hole the moment it moved away. A reading of open ground no longer
supersedes a mapped hole; everything else still replaces what came before.

**The washouts on the lanes are scenario content.** Potholes are generated
across the terrain, thicker along the track where water runs, but the sweep is
laid on the clearest axis the first scan can find, so whether any fall on it is
luck of the seed. Measured across three densities, none did: the planner keeps
finding a clean corridor, which is its job. Two washouts are therefore cut onto
the lanes deliberately (`sweep.LANE_WASHOUTS`), past the range at which a
negative obstacle can be resolved, exactly as the buried objects in targets.py
are placed. This is a demonstration arrangement and is called one.

**Boulders run larger.** The size distribution was capped at 2.4 m and is now
3.6 m, with a wider spread, so an outcrop reads as something to route around
rather than gravel.

## Resolution

The camera tiles were rendered at 1/1.7 of their own size and scaled up, which
is the one thing on this display that cannot be defended as what the sensor
saw. They now render at the tile's own device pixels (about 2.9x the pixels,
31 ms per camera tick against an 85 ms budget). The map's accumulated point
cloud went from 8 to 14 px/m, finer than the half-metre terrain grid, so two
returns from neighbouring cells land on different pixels instead of stacking on
one and flattening the cloud into fill.

**The terrain grid stays at 0.5 m.** Measured, 0.25 m takes 9.0 s to generate
against 2.3 s and quadruples the costmap and planner work for ground detail
finer than the rover is wide (1.8 m). The resolution that matters for
navigation is already there; what was missing was pixels where the operator
actually looks.

## The display runs at its own rate

**Drawing was tied to the simulation.** The simulation advances ten times a
second and the map was drawn when it did, so an aircraft crossing the map
moved in five-pixel steps. The display now runs at sixty and eases each icon
toward wherever the simulation last put it (`POSE_SMOOTHING_S`). The
simulation is untouched: what moves is only the position an icon is drawn at,
and anything that asks where a unit *is* gets the simulation's answer. A
docking or a reset snaps instead, because easing across a teleport draws a
unit crossing ground it never crossed.

**A frame cost 33 ms, which is thirty hertz at best.** Three things were being
redone every frame that had not changed:

- The newest scan's returns, drawn point by point: 23 ms of the 33. They do
  not move once measured, so they are rasterised once into a layer and the
  decay is applied to the layer.
- The accumulated cloud, held at 14 px/m and scaled down to the viewport
  every frame. Scaled once per scan now.
- The grid, a few hundred lines and labels that only move when the view does.

A frame is now 6.0 ms, about a third of a core at sixty hertz.

**Camera frames go where the operator is looking.** A tile costs about 20 ms,
so four live feeds cannot all be current. The selected tile takes every other
turn (about 8 Hz) and the rest share the remainder, because an even split
would make the one feed being watched as slow as the three that are not.

**Occlusion resolves by binary search.** Each pixel was tested against every
sample along its ray, which is steps x pixels per frame and scales with tile
area, so making the tiles bigger made it worse. The running maximum of the
sample tops only ever rises, so the owning sample can be found by
searchsorted instead: rays are offset into one ascending array and the whole
frame resolves in one call. 31 ms to 19 ms at the larger tile size.

That offset has to be done in double precision, which the tests caught. In
single precision the offset for a ray far across the frame is thousands of
times larger than the angles being compared, and the gap between
representable values swallows them: four tests failed with a handful of
pixels leaking through a ridge.

## Layout

**A four pixel baseline** (`theme.UNIT`), so the rhythm of the layout is
decided once rather than argued out again in each panel. Panels that each
chose their own padding are what make an interface look assembled rather than
designed.

**The camera tiles take the column's slack.** The roster below them is a fixed
stack of four cards, so a trailing stretch left a third of the column as empty
background. The tiles went from 104 px to 149 px and render at their own
device pixels.

**The log reads like a log.** A confidence climbing on one buried object was
five lines of it, which pushes everything else off the visible log -- the one
place a missed alert can still be found. An entry now carries the subject it
is about, and a fresh report on the subject already at the bottom rewrites
that line. Lines carry a severity, which is the only thing that colours them,
and the list is bounded because the display is meant to run for a shift.

**One table of names.** `ui/labels.py`. The sensor abbreviations lived in the
threat panel and the call signs in the launch console, which is how an
abbreviation comes to mean one thing in one place and another elsewhere: the
log was cutting raw sensor names to three letters and showing PUL for the
pulse induction array.

## Arriving and leaving

**Motion is continuity, not decoration** (`ui/motion.py`). A panel that
appears instantly asks the operator to re-find the layout and work out what
changed; one that eases in keeps the eye where it already was. A banner that
blinks out between two glances leaves a doubt about whether it was answered
or missed, so it stays visible for the whole of its exit and hides only once
it has gone. Reversing mid-flight is scaled to the distance left, so a banner
dismissed while still arriving leaves at the speed it was arriving at.

Durations are 110 ms for a change to something already on screen and 180 ms
for arriving or leaving. An operator who has decided to press something
should never wait for an animation first, so nothing that would delay a
decision is animated at all, and a reset takes everything away with no frames
in between.

The alert banner opens downward from the edge it belongs to, which is the top
of the map it is reporting about. It is a composite of child widgets rather
than something painted, so it fades through an opacity effect and slides by
its own height rather than through a painter.

## The threat list reads as a register

A card carried a full border, a filled background, a ring around its glyph
and four type sizes: four enclosures for one row, and a dozen of them stacked
up meant reading boxes rather than contacts. What is left is the status
spine, the glyph, the line naming the contact, the confidence against its
threshold, and a hairline to the next row. One title size and one small size.

Removing the card's background fill put Qt's default **white** behind every
row, which the full-size screenshot made look merely dim. The row owns its
background explicitly now. Found by sampling the rendered pixels rather than
by looking: every one came back `#efefef`.

## The link strip shows the exception

It painted ninety-five percent of its area solid green to say nothing had
happened, spending all its contrast on the usual case and leaving none for
the one thing worth seeing. A connected link is now a dim rule and only a
degraded or lost one is given colour and height, with a -60s/-30s/now scale
so "how long was it out" can be read rather than guessed, and a live dot at
the right edge for the current state.

## Two mistakes worth recording

**My render scripts never loaded the application stylesheet.** `ocu/main.py`
applies `ui/style.qss` to the QApplication; the scripts I was taking
screenshots with created the window directly, so every visual judgement I
made from them was of an unstyled interface -- buttons in Qt's default white,
and none of the panel styling. Fixed in the scripts; worth remembering that a
screenshot of a widget is not a screenshot of the app.

**I overwrote `tests/test_motion.py`.** It already existed and tested the
flight model's `KinematicBody`; I wrote the interface's animation tests over
it and destroyed fourteen physics tests. The suite still reported a healthy
number because the count happened to match what I had added. Restored from
git, and the interface tests now live in `test_ui_motion.py`.

## The map and the cameras show the same ground

They had drifted a long way apart. Measured, the two agreed about *what* was
where (96 to 98 percent of cells classified TREE, ROCK or SCRUB were drawn as
such by the camera), so the geometry was never the problem. The colours were:
open ground was blue on the map (hue 209) and tan in the camera (hue 37), rock
was cold blue-grey against warm grey, and scrub was a vivid green against
olive. An operator looking from the map to a feed was being asked to match a
green patch to a different green and a tan hillside to a blue one.

The map's classification colours are now derived from the colours the cameras
render the same classes in, so the two cannot drift again. Hue and saturation
come from the camera; only lightness is the map's own, and it ranks by what
has to be found rather than by what the ground looks like: open ground is most
of the view and stays quiet, a thing that stops the vehicle is the brightest
mark on the display. Lifting every class equally was truer to the camera and
turned the map into an even wash with no hierarchy in it, which is why ground
sits well below scrub, and scrub below tree and rock.

One class is deliberately not natural. A hole is a hazard rather than terrain
and is coloured to be found rather than recognised; a test holds it well away
from every other hue.

## A formation belongs to the rover, not to the compass

Slots were offsets on the map. That already held the formation on the rover as
it drove, but not as it *turned*: measured, a rover that formed up facing north
and then turned east left its aircraft at the same compass bearings, which put
them off its left-hand side. A screen flown for a vehicle belongs ahead of that
vehicle, and a vehicle that turns takes its screen with it.

A slot is now a place on the rover -- so far forward of its nose, so far across
it -- and the world position is worked out from the rover's current heading
every tick (`to_body_frame` / `to_map_frame`). The aircraft's facing is held the
same way, so the group's sensors keep pointing where the vehicle is going.

**The dial changed meaning, so it changed label.** It reads "Off nose" and
ranges from -180 to +180, where zero is straight ahead. An absolute compass
bearing would have been correct at the moment it was set and wrong a lane
later, which is a worse kind of wrong than being relative. The formation
preview on the map turns live as the rover turns, because a vee drawn ahead of
a rover that has since turned is showing the operator somewhere the aircraft
will not be.

Three existing tests had been passing for the wrong reason: they only ever
drove a rover at a heading of zero, where the vehicle's frame and the map's
are identical. They now turn the rover.

## Staging is a place on the rover, not a place on the map

The body-frame change above still left one hole: staging computed the
formation from the rover's pose *at the moment of staging*, and launch read it
back at the moment of launch. Stage facing 127 degrees, let the rover reach the
end of its lane and turn to 247, press Launch: the aircraft finished 146
degrees off the nose, behind the vehicle, with the dial reading -26.

The console now works the formation out with the rover at the origin facing
+y (`_body_targets`), so a staged slot is "this far ahead, this far across"
and nothing about the world is baked in. Launch hands the flight director the
dial's bearing relative to the nose. A regression test stages, turns the rover
by 120 degrees, launches, and checks the aircraft finish ahead of the vehicle;
it fails if the staged bearing is wrong.

## The rover's pace is half its published top speed

Operator decision: the demonstrator drives at `ROVER_PACE_FACTOR = 0.5` of the
baseline top speed (operating 2.08 m/s, sweeping 0.4 m/s), because the map
moved too fast to read on a laptop screen. The baseline figure is unchanged
and still what the platform is rated for; only the picture's pace changed.
The readout shows the operating ceiling, not the rated one.

Two things that were quietly tied to the old pace had to follow it. The
"rover is moving" threshold for scout advice was 0.5 m/s, above the new
sweep speed, so the rover stopped recommending anything; it is now half the
survey pace. And tests that waited a fixed number of seconds for a detection
now wait for the event.

## Small screens

The window opens at 92% of the screen up to 1600x960 and cannot be sized below
the sum of its columns (left 176, map 380, right 322 px). The right column's
formation panel scrolls rather than being cut off; the threat list keeps its
share. Fleet cards and threat cards derive their layout from their width and
elide text instead of overprinting it. The alert banner keeps both decisions
whole by reflowing to two rows below 600 px.

## The operator sets the rover's speed

The half-pace decision above is replaced by a control. A slider in the top bar
sets `commanded_speed_ms` on the flight director, a ceiling on every way the
rover drives (sweep, route to a destination, the way home, the sticks), from
0.1 m/s to the published 15 km/h top speed. It starts at the 0.4 m/s survey
crawl and keeps the operator's setting through a reset. The scale is
quadratic because the speeds that matter for clearance are all in its first
third.

The playback-speed dropdown (½× to 4×) is gone from the top bar; the mission
clock still has a speed, it just is not offered. Sweeping faster than the
survey crawl means the 40 cm induction swath passes over objects faster than the
sensor model resolves them, which is the sensor behaving, not a bug.

## Going somewhere on purpose

GO TO on the map, then a click, sends the rover to a point. It is the existing
navigation, not a second one: the destination becomes a one-point waypoint
list in place of the lanes, with no lane corridor on the planner, and the
sweep's progress is parked only for a destination set before deployment. Refused when the
rover is under manual control, outside the operating area, or outside a
configured geofence. Aircraft keep station on the rover as they always do, so
the whole formation arrives together.

Going cross-country exposed four faults that lane sweeps on clear ground never
could, all fixed and each with a test:

- **The route checked was not the route driven.** Validity was tested against
  the cells the search crossed; the rover drives the smoothed line pulled taut
  off them, which cuts across cells the search went round. A tree mapped beside
  the search path but on the driven line was driven into. The driven line is
  now checked.
- **Late-seen obstacles.** A tree behind another tree is not seen until the
  rover is close, so it can find itself inside an obstacle's clearance. The
  planner now lets a vehicle in that position route out of it, only nearby and
  at a heavy cost, and the follower aims close while there, because a long
  lookahead chords across the corner the route was bent round.
- **A last line of defence.** Whatever the planner does, the rover will not move
  onto a cell it has mapped as an obstacle; it stops at the edge and replans.
- **Long routes failed outright.** The search ceiling ran out on a leg of a
  hundred metres. The distance estimate is weighted 1.5 and the ceiling raised;
  routes can be a few percent longer than optimal, which cannot be seen.

The map follows the rover when it nears an edge (and leaves the view alone for
four seconds after the operator drags it), and a destination off the edge of
the view is marked on that edge with its distance.

## Fewer contacts

The buried-object field is five objects, down from thirteen: the shallow
ferrous one, the deep low-metal one, the scrap, the one just outside the swath
and one on the surface for a scout to find. Nothing is scattered at random. A
dozen contacts buried the one the operator had to decide about.

The classification key and scanner statistics were taken off the map.

## The rover only knows the way over ground it has seen

The first go-to drew a complete route to the destination the instant it was set,
across ground the rover had never looked at. The planner treated unseen cells as
open, which draws a path to anywhere; it is a guess presented as navigation, and
it made the vehicle look as though it knew the way. That was wrong, and it was
wrong for lane sweeps too.

Every autonomous route is now planned by `plan_toward` over **seen ground only**,
within a **planning horizon of 22 m** of the rover (the scanner reports out to
32 m, but the far end of that is thin and about to be revised). If the goal is
inside what has been seen, the route is complete. If it is not, the route
is as long as what is known and ends at the **frontier**, a seen, drivable cell
beside an unseen one, that costs least to reach plus 1.5 times the distance
still to go. The rover drives there, scans, and chooses again. A frontier it has
already stood at and found still unresolved is a shadow it cannot see into from
there, and is not gone to twice.

What is drawn follows: the line stops where the knowledge stops, with the
frontier marked, and the destination is marked separately. The rover reports
EXPLORING TO DESTINATION until it has seen all the way, logs once when it has,
and then GOING TO DESTINATION. The old optimistic planner is still there
(`explore=False`) for a caller that hands it a complete map; nothing in the
application uses it.

**Sensor ranges are honest about what they can resolve.** A rock half a metre
across is a return or two from a one-degree scan at thirty metres, which is not
something to plan around, so rocks are resolved at **16 m** (`ROCK_DETECTION_RANGE_M`),
potholes at 8 m, trees at the scanner's full 32 m. Beyond its range a rock reads
as ground and does not cast a shadow, so the rover plans a route over it and
reroutes when it is close enough to see: the same way holes were already
handled. A rock or a hole that has been mapped is not forgotten when the
rover turns away and reads ground beyond it.

Known limit: a destination inside or behind dense forest can take the rover a long
time to approach, because it explores the forest edge one frontier at a time.
Measured on this seed, one such point was still unreached after 400 s at the
survey crawl; the other seven drives tried arrived, none entered an obstacle.

## Clearing a destination holds position

Clearing a destination used to resume the lane sweep, which drove the rover
back across the ground to the lane point it had left. Clearing is the operator
withdrawing an instruction, not giving a new one, so nothing is resumed now:
the rover stops where it is (HOLDING POSITION) until it is given a new
destination or sent home. The one exception is a destination set and cleared
before the mission was deployed, where the sweep is still the plan.

## The map is drawn at half its old scale, and zooms

The rover looked too fast on the map. Changing the vehicle's speed would have
changed the physics; slowing only the drawn icon would have put it somewhere
other than where it is. The honest way to make a picture slower is to show more
ground in the same window, so the map is now 4.5 px per metre where it was 9: a
rover at 4 m/s crosses the screen at the speed a 2 m/s rover used to. Position,
speed readout and everything else are unchanged.

The scroll wheel and a trackpad pinch zoom between 2.5 and 20 px per metre about
the pointer, so the operator can come in on a detail without losing the wide
picture. The scale bar picks a round length that stays readable at any zoom.

## What a scout's camera points at, and how far it can see

A scout reported objects 35 to 55 m *behind* the formation while the whole
formation faced and moved away from them. Two causes, both fixed.

**Facing.** An aircraft following its slot faced its direction of travel. When
the rover turns, every slot swings, and an outer scout follows its slot
sideways at several metres a second; the "face where you are going" rule then
yawed it round. Measured: 8 to 122 degrees in two seconds through a 13 degree
turn of the rover, with the camera sweeping across ground behind the formation.
Course-facing now applies only while joining the formation. On station an
aircraft faces the formation bearing, as a camera platform should, and a
multirotor flying sideways does not have to point where it is going.

**Range.** The sighting range was a round 90 m. It is now what the sensor can
resolve: a 1280 pixel sensor across the 84 degree lens, four pixels to tell a
quarter-metre object from the ground, which is 55 m of slant range and a little
under 50 m across the ground from scout altitude (`aerial.MAX_SLANT_RANGE_M`,
derived, not typed). The sensor and object sizes are assumptions, written down
where they are used. A camera yawing faster than 20 degrees a second also sees
nothing it sweeps over.

## The world is made of objects, and everything works from the same ones

The rover was reported driving through rocks and bushes. Measured, the cause was
that three things each had their own idea of what was out there: the camera drew
smoothed height fields, the LiDAR map was cells, and a boulder only counted as an
obstacle where its dome stood over 30 cm, so a rover driving up the low flank of a
big rock was, by the map's rules, on open ground; and bushes were drivable scrub.

The world now generates **individual objects**: boulders (an ellipse, a height),
trees (a trunk position, height and crown) and bushes (a patch). Their footprints
*are* the class grid, so one list feeds all three:

- **Boulders** are obstacles across their whole footprint, grown by 0.3 m so a cell
  they only touch is on the map. Stones under 20 cm are pebbles, drawn and driven
  over.
- **Trees** stand at least 1.9 m inside the stands the grid already marks, with
  crowns capped at 2.3 m, so every tree on a camera is inside ground the map
  calls impassable.
- **Bushes** are a new impassable class (`Terrain.BUSH`): too low for the
  horizontal beam to strike, resolved at close range like a rock (16 m), remembered
  once mapped.
- A track no longer paves over a rock or a tree.

Tests hold these together: every boulder footprint is ROCK on the map, every tree
is inside TREE ground, every bush is BUSH, and the same seed gives the same world.

## Avoiding things in real time

A route plus a hard stop was not obstacle avoidance. The rover knew about one rock
for fifty seconds, drove a route that hugged it, and froze against the last-line
stop when its own heading drifted into the margin. There is now a reactive layer
(`core/local_planner.py`) under the route planner, run every tick:

- It lays a fan of straight tracks out from the vehicle, finds how far each can be
  driven before the footprint (1.0 m from an obstacle cell's centre, against the
  planner's 1.1 m) would touch something mapped, and picks the one that gets
  nearest where the route wants to go while leaving room. It keeps to one side of
  an obstacle rather than swinging between sides.
- Tracks are examined as far as the vehicle needs to stop from its present speed,
  plus a margin, and the speed allowed follows from how far the chosen track is
  clear. It slows through a tight gap.
- Starting inside the margin (an obstacle seen only when it was near) it leaves the
  way that opens clearance instead of freezing.
- The route planner's clearance rose from 0.9 m to 1.1 m: the vehicle is 1.15 by
  0.85 m and sweeps a 0.72 m circle turning on the spot, and a map cell is a
  quarter of a metre from its own edge.
- The last-line stop is still there behind it, and is now never what stops the rover.
- A destination whose surroundings are walled in is declared unreachable after 45 s
  without getting 1.5 m closer, rather than circled forever.

Measured across nine drives flat out (4.2 m/s) and a sweep, the rover's centre
stayed at least 0.83 m from the real surface of every object. That is now a test.

**The formation flies the course, not the wobble.** A rover weaving round
obstacles yaws ten degrees a second, which swings a slot thirty metres ahead of it
sideways at five metres a second; no formation flies that. Slots and facing are held
against the rover's heading smoothed over 2.5 s.

## The camera feeds are a 3D scene on the GPU

The feeds were drawn by a numpy ray-marcher over blurred fields: flat-shaded
blobs, a few frames a second. They are now a real-time scene (`ocu/render/`, on
`moderngl`): the terrain as a mesh with a sun, a 4096 px shadow map of the whole
world, hemisphere sky light, aerial perspective, a sky with clouds, and filmic
tone mapping; boulders as cleavage-cut convex solids with weathering, lichen and
cracks in the shader; trees built from whorls of boughs (conifers) and lobed crowns
with leaf facets and a broken outline (broadleaf), swaying in wind; bushes as
clumps of lobes; thousands of grass tufts near the camera; pebbles; the rover as a
model when a scout looks at it. Everything is generated from numbers, with no
asset files, from the same objects the map is made from.

It runs on a worker thread, so a frame never makes the interface wait, and every
live tile is redrawn about 25 times a second rather than one tile per tick. A frame
costs about 8 to 11 ms on an Apple M1. Without a GPU the application falls back to
the numpy renderer. Using ready-made photoscanned models and textures (Poly Haven,
Quaternius) would look better still; they are downloads, so that is left for the
team to decide.

## Poly Haven models and textures

Approved by the team and fetched by `scripts/fetch_assets.py` (about 66 MB, CC0, from
the Poly Haven API with checksums verified; `assets/polyhaven/` is not committed):

- **Rocks:** five Namaqualand boulders become the world's boulders. Each boulder takes
  the model whose proportions fit its footprint best and is stretched to it, so a camera
  shows the same rock the map has. Small rocks and stones are scattered as pebbles.
- **Shrubs:** the Othonna shrub is every bush.
- **Grass:** two grass models, about 50,000 clumps, thickest in scrub.
- **Ground:** five photographic textures (dry ground, grassy rock, gravelly sand, dry mud,
  rocky terrain) with normal maps, blended by what the map says the ground is, sampled
  twice at unrelated scales so the repeat does not show, and graded toward this scene.

The files are tens of thousands of triangles each, so they are read as untrusted data
(accessors range-checked, buffers kept inside the asset folder) and simplified once, with
their normals and texture coordinates weighed in, to a near mesh of 500 to 5,500 triangles
and a far one. A boulder that would not simplify (`boulder_01`, unwrapped into many small
islands) was dropped.

**Trees stay procedural.** Poly Haven's trees are 20 to 960 MB each, far too heavy to draw
three hundred of. They are the one part of the scene that does not look photographic.

Without the downloads the renderer draws its procedural rocks, bushes, grass and ground
as before, and the application is none the worse for it. Costs: about 3 s to load, and a
frame is 10 to 12 ms on an Apple M1.

## Low battery: the vehicles act without being told

- **An aircraft below 8%** goes home to dock by itself (`TRICOPTER_AUTO_DOCK_BATTERY_PCT`).
  It is recalled, and if it was on the sticks the sticks are taken back. It cannot dock onto a
  moving rover, so a rover that is navigating stands still until nothing is left to recover,
  then carries on with its mission.
- **The rover below 12%** returns to base at once (`ROVER_AUTO_RETURN_BATTERY_PCT`). It gives up
  the sticks, drops its destination, and every aircraft still out is recalled to come with it.
  Higher than the aircraft's, because the way home is the long one and the rover is the one
  vehicle that cannot be recovered.
- **An aircraft is not launched below 15%.** It would be recalled as soon as it lifted off, and
  the console says which one was left in the bay and why.
- **The operator is told in two places:** a red line in the event log, and a notice over the map
  that stays until it has been read. Each vehicle is acted on once; an aircraft that goes back on
  charge is armed again.
- The way home was a straight line that ignored everything in it. It is now driven like any other
  route, over ground that has been seen and steered round obstacles, and if there is no clear way
  home the rover stops and says so instead of driving at the first rock.
- The thresholds are the team's numbers, not the baseline's; they are named constants.

- **A flat rover stops where it is.** At 0% it halts in whatever mode it was in and nothing moves it
  again: manual control and new destinations are refused, and Return To Base can only bring the
  aircraft down onto it where it stands. It is logged once, shown over the map for 30 s, and the top
  bar and roster read BATTERY DEPLETED. A reset restores power.

## Cameras start when they have something to see

- **The scene is not drawn twice.** The tiles used to show the numpy renderer's picture for the first
  few seconds of a run and then swap to the GPU one. They now say CAMERA STARTING until the scene is
  built, and draw nothing in the meantime. Startup is about 1.5 s once the simplified meshes are
  cached (3.6 s the first time).
- **An aircraft's camera comes on at 3 m** (`CAMERA_ON_ALTITUDE_M`), not at lift-off. Just off the
  deck it sees the rover's own body filling the bottom of the frame; the tile reads CLEARING ROVER
  until it is above that, and goes dark again at the same height coming down to dock. A frame that
  arrives late for an aircraft that has gone back below it is dropped.

## People, vehicles and animals: a camera detector built on measured curves

The spec names `person` and `vehicle` as threat types and nothing in the simulation could ever produce one:
every contact was a buried object. This adds the other half of what ARES is meant to perceive, things that
stand or walk on the surface, and a camera detector for the rover and the scouts. **The detector is simulated.
No neural network runs inside this application.** What it does is behave the way a particular trained model
was measured to behave, which is the same rule the rest of this build follows for its sensors: the shape of the
response comes from somewhere checkable, and where this build has to invent a number the number is listed here.

**Where the measured part comes from.** `ocu/data/detector_curves.json` is built by the `ares-perception`
repository (`tools/build_ocu_curves.py`, decisions 37 to 43 there) from a trained RT-DETRv2 r18's saved
detections on images it never trained on: 500 camera-trap images from 39 unseen locations (animals), COCO
photographs (people, vehicles), and one drone flight (people from the air; evaluation only, the dataset's
licence is unconfirmed and only aggregate recall numbers are used, no images or labels). It records, per
apparent size (the object's height as a share of the frame height, which is what a detector that squashes any
frame to a square actually sees), how often the model found the object, with a 95 percent interval and a count,
and how confident it was when it did. It also records how often the model invents a person or an animal on
empty rover ground (1 frame in 72, from 2 invented detections in 144 frames) and how confident those are.
The file carries its model hash and its caveats, and `tests/test_vision.py` checks it is sane.

**What the measurements say, and the model keeps.**
- Finding something is far from certain at small apparent sizes (animals: 21 percent when 7 to 11 percent of the
  frame height, 61 percent at 17 to 26 percent, 90 percent above 60 percent), and each frame is sampled, so a
  camera sometimes loses what it had a moment ago.
- **From a scout, a person at the sizes its camera usually produces was not found at all** (0 of 505 boxes at 3
  to 5 percent of frame height). The aerial person curve is that. Above 6 percent of the frame the ground curve
  is used and is **untested from the air**; the file says so.
- The model invents people and animals now and then. Here a false alarm lasts one frame, which is an
  assumption: real ones can sit on one rock for many frames, and persistence was not measured.
- There is no vehicle false-alarm rate. The vehicle labels in the source data have gaps, any figure would be
  wrong, so none is invented and vehicles are never falsely detected here.

**What is this build's choice, not a measurement** (all in `core/vision.py` and `core/entities.py`):
- Detector rate: 2 frames per second of mission time (`PERCEPTION_HZ`). The real rate depends on hardware that
  has not been chosen.
- Confirmation gate: a track is reported after 3 hits in 5 frames within 3.5 m of its last position
  (`CONFIRM_HITS`, `CONFIRM_WINDOW`, `GATE_RADIUS_M`), and forgotten after 10 frames unseen
  (`TRACK_MEMORY_FRAMES`). A detector that flickers is not shown to an operator as a
  threat until it persists. In the tests, this removes nearly all invented reports (at most 5 percent of raw
  false alarms survive) at the cost of at least a second before a real one is announced (three frames at 2 per second).
- Confidence: the measured median for that size, plus or minus 0.06 (invented spread), never below the model's
  operating threshold (0.40) and never above 0.97 (the rule detection.py already uses).
- Position: where the bottom of the box meets flat ground, with 2 pixels of noise, which grows with the square of
  range. A distant detection is placed loosely.
- The three entities and their sizes, speeds and positions (`default_entities`): a person 1.75 m tall walking
  1.2 m/s across the ground ahead of the lanes, a parked vehicle (now the 2.2 m outline of the pickup that is drawn) 21 m along lane 3 and 14 m off
  it on the far side, clear of the destinations the operator and the tests send the rover to, a deer-sized animal 1.0 m tall wandering at 0.5 m/s near the first lane. Placed on purpose relative to the
  sweep, like the buried objects; nothing appears at random except what the detector invents.

**The eighth threat class is still open, and this does not close it.** An animal is not in the spec's enum
(`mine`, `ied`, `person`, `vehicle`, `disturbed_soil`, `safe_object`, `unknown`). Reporting it as `unknown`
and carrying the word in a separate `label` keeps the data model inside the enum and lets the panels say
ANIMAL. "animal" is the obvious candidate for the missing eighth value, and that is a call for Bhargav and
Abir; nothing here guesses it. An animal is reported but never raised as a threat alert (`informational`): a
deer at 0.9 confidence is a contact, and an alert for it would train the operator to stop reading alerts.

**How it reaches the operator.** `_tick_vision` runs a detector frame for each running camera, draws every
current track over that camera's feed (dashed until it has persisted, solid after), and sends a confirmed track
through the same `_on_detection` as everything else: threat panel, map marker, event log, the shared map
replica (rover or aircraft, so a cut link leaves each side with what it saw), and an alert for a person or
vehicle at 0.85 or above. A track is reported once, where it was first confirmed, and a later better reading
updates that report instead of adding another. Tracks of one kind that confirm within 8 m of an existing contact
(`MERGE_RADIUS_M`) are that contact: monocular ground ranging at 30 m from a camera 1.6 m up is good to metres,
and without this a single parked vehicle was reported four times in 200 s and the threat panel reached 12
contacts, which `test_there_are_few_contacts_to_deal_with` rightly refused. That test now counts buried and
camera contacts separately (at most 5 buried; at most one per entity plus 2 camera ones). A person who walks
further than 8 m from where they were first reported becomes a second report, because the map marker does not
move. The first report writes a line to the event log saying camera
detections are simulated, so it cannot be mistaken for a live model.

**What is drawn.** Low-poly figures (`render/meshes.py`) in the GPU renderer, in both the threaded and the
direct paths. The numpy fallback renderer does not draw them. A test renders each figure with the real
renderer and checks that its drawn height, centre and feet agree with the box the detector model projects for
it, so an overlay cannot drift from the picture. The figures are not photographs and a model trained on
photographs is not claimed to find them: what the detector reports about them comes from the curves.

**Not covered.** Thermal and night cameras (a terrain fine-tune nearly erased thermal people in the source
repository, so the numbers here would be wrong for them). Tiled or ensembled detection (not adopted there for
the driving camera). Entities that hide, cross each other, or change size. The scouts' detections on the
real renders, which no model was measured on.

## Things on the ground are obstacles, and look like what they are

The first version of the people, vehicles and animals was drawn and detected and nothing more: the rover drove
straight through a parked truck, and they were boxes with limbs. Both are fixed.

**A parked vehicle is part of the ground.** `TerrainWorld.stamp_object` marks the cells its outline covers (grown
by 0.3 m, as a boulder's are) as ROCK: impassable, and a surface a LiDAR ray stops at. The LiDAR maps it, the route
planner and the reactive layer drive round it, and the costmap's last-line check refuses a step onto it, exactly as
for a boulder. The sensor cannot tell a truck from a rock and the map says ROCK; what it is comes from the camera.
Run with the stamp switched off, the rover's centre ends up 0.0 m from the truck (it drives through it); with it,
never closer than half a metre, over the whole run (`tests/test_obstacles_window.py`).

**Things that walk are obstacles the rover is told about, briefly.** The perception map keeps every obstacle it has
ever mapped, on purpose, so stamping a person who crosses the lane would leave a wall behind them for the rest of
the mission. `core/dynamic_obstacles.py` keeps them apart: a person or animal is noticed only when the LiDAR could
see them (within 32 m, nothing that blocks a beam between), is followed at the velocity it was seen to have, is
remembered for 4 s after it was last seen, and is given to the reactive planner as obstacle points: its disc, grown
by 0.6 m because it can move, now and again 1 s and 2 s ahead, so the rover steers for where it is going. The route
planner does not see them. A last line, like the one for mapped obstacles, refuses any step that would bring the
rover's centre within 1.0 m of the edge of one, but only a step that closes the gap, so the rover can always back
away. The radii (0.4 m for a person, 0.8 m for an animal), the margin, the lead times, the memory and the 1.0 m are
this build's choices.

**They stop for the rover.** Nothing walks into a rover that is driving round it or waiting for it. A person or an
animal within 2.5 m of the rover stops until it has passed (`YIELD_GAP_M`): without that, a deer wandered into a
waiting rover and the rover's centre ended up inside it (a gap of minus 0.58 m in the default run). That stand-off
has a second half: a walker that has stopped beside a waypoint can hold the rover there for as long as both wait,
so a rover that has stood still for 8 s (`STALL_GIVE_UP_S`) with one beside it skips that point, the same decision
the planner takes for a point blocked by a rock, and says so in the log ("... point blocked by the deer, skipping
to the next"). Both numbers are choices.

**Placement.** Parked and walking things are placed on ground they can honestly stand or walk on: a vehicle needs
level ground (0.3 m of relief or less under its footprint) with a metre of clear ground all round it, a walker needs
every point of its path clear with 0.4 m either side, nothing is within 7 m of the deploy point or off the edge of
the map, and the nearest place that works is used (`settle_entities`). The scenario raises if there is none within
24 m, rather than putting a truck in a stand of trees. Sizes are those of what is drawn (a roof rack and a pack
are part of the outline), so the detector's box, the obstacle footprint and the picture agree.

**What they look like.** There are no photographed models of these in the free, licence-clean sources this build uses
(Poly Haven has rocks, plants and props, not a pickup, a person or a deer), so they are built, in
`render/figures.py`, from three primitives (a superellipsoid, a lofted tube, a surface of revolution) with the
proportions and colours of the real thing: a long-wheelbase utility pickup with a glass greenhouse, roof rack,
snorkel, bull bar, tyres with tread blocks and an open bed with a spare wheel and a jerrycan in it (about 39 000
triangles; a person is about 8 500 and a deer about 7 800); a person in field clothes with a rucksack, sleeping mat and wide-brimmed hat; a doe with a countershaded
coat, white rump and tail, large ears and a lateral-sequence walk. Each vertex says what it is made of (paint,
glass, rubber, cloth, skin, hide, bright metal, lamp, hair), and the shader treats each properly: clear-coated
paint under a layer of dust that rises from the ground, glass that darkens and shows the sky, cloth with a weave and
folds, hide with a fine grain and soft light, skin that glows when the sun is behind it. People and deer come as a
cycle of 12 poses, chosen from how far they have walked, so the legs move as fast as the ground goes by and stop
when the figure stops. They cast shadows: each frame starts from a copy of the still scene's shadow map and adds
them. Frame time with and without the three figures is the same to within noise on this machine (about 10 ms for a
640 by 360 frame). A test renders each figure with the real renderer and checks its drawn height, centre and feet
against the detector model's box for it, on flat open ground.

**Not covered.** The numpy fallback renderer draws none of them (and shows an old-style dome where a truck is
stamped). The figures are made, not scanned, and are plainly of a lower class than the photographed rocks they stand
among; the colours were chosen by eye against the scene. They are not given the ground's slope: a vehicle rests on
the average of the ground under its four wheels, so on ground more than a few degrees out of level a wheel can sink or
hover, which is why the level-ground rule exists. Moving things do not appear in the LiDAR cloud. A walker that
leaves its first marker by more than 8 m is a second report. Nothing about what a real operator does with any of
this has been tested.

## The scanner sees people and animals; and the deer is a deer

**Returns off moving bodies.** The LiDAR used to see only the ground truth's classes, so a person or an animal in
front of it was invisible to it (though not to the camera, and not to the planner, which was told about them
separately). `lidar.entity_returns` now casts the scanner's 360 rays against the outlines a horizontal beam cuts through
each walker (`entities.LIDAR_SHAPES`: an ellipse for a person's jacket and pack, one for a deer's barrel), takes the
first outline a ray reaches inside the 32 m range, and keeps it only if nothing that blocks a beam stands between the
scanner and that point (the same test the dynamic-obstacle tracker uses, so what is drawn and what the planner knows
agree). Range error is 2 cm, brightness falls with range as everywhere else, and a person's clothing and a deer's hide
return less than rock (0.55 and 0.48 against 0.95). Because the scanner has a degree between rays, a person at 10 m
is struck by about three of them and a broadside deer at 20 m by about three: that is how a real unit shows something
that size, and it is why the clusters are small. A parked vehicle is not in this list: it is part of the ground
(stamp_object), so it is mapped, drawn and remembered like a boulder.

These returns are **not written into the map or the cloud.** A body that moves is not ground; written in, a person who
crossed the lane would leave a wall along their whole path. They go to the map view's own short-lived layer
(`add_entity_returns`): warm orange points for a person, violet for an animal, held for 2.2 s and faded, at most 600
at once, cleared on reset. Held that long, a walker leaves the short trail a real display shows behind something that
moves. Tests: the returns lie on the outline and come off the near face only; fewer rays strike a body the further it
is; a body is struck by more rays side-on than head-on; nothing beyond range or behind a wall returns; a seed gives the
same returns; they never touch the permanent cloud's pixels; they are released on time and capped; and on the real
window a walker in view appears in the picture while the rover's own map holds no rock where they stand.

**A buck, and more of a deer.** The animal is now a buck (the doe is still available as the plain "animal" drawing):
antlers with a curved main beam, a brow tine and three more tines each side on burrs at the skull; a heavier neck
with a darker mane; a face with a rounded forehead, pale rings round dark eyes, nostrils, a mouth line, a
pale chin and throat patch, and ears with pale insides and a tuft that twitch as it walks; a darker line down the
back and a paler band along the flank over the countershading; knees, the point of the hock, dew claws and a split
hoof on every leg; a tail dark above and white beneath that flicks; a belly and chest paler than the flank. Its
body is closed with a rounded taper at both ends (an earlier version ended in a flat cut-off cylinder, which a
close-up showed and the previews did not). About 17 000 triangles, in 12 walking poses. Bone has its own material
(ridged, polished at the tips). The buck's outline is taller than the doe's (1.86 m with antlers against 1.47), so
the detector's box, the entity's size and the renderer-versus-model test all use the buck's.

**Not covered.** Walkers' returns come from one ellipse each, so a person has no separate arms or legs in the scan and a
deer's neck and head are not in it (the beam is at body height). Moving bodies do not occlude the beam for the
ground behind them. The antlers and the face are fine at a few metres and invisible in the rover's feed, where a deer
is a few dozen pixels tall; they are there for the day the camera is closer. The deer is made, not scanned.

## The operator chooses what counts as a threat

**What was wrong.** Every detection was dressed as a threat: a parked truck raised an alert that wanted a decision, a
deer sat in the threat list with a red marker, and a person walking on the lane did the same. An operator clearing
ground for mines does not want to be asked about vehicles and animals, and one watching a border does. What counts as
a threat is the operator's call, not the software's.

**What is built.** A strip of chips at the top of the threat list, **TREAT AS A THREAT**: Mines and IEDs, Arms, People,
Vehicles, Animals, Other. A class that is on behaves as before. A class that is off becomes a **contact**:

- it raises no alert and asks for no decision, and is not in the threat list;
- it is still drawn on the map, as a small quiet marker with a dim label (no red, no pulse, no ring), and on the camera
  feed as a thin muted box;
- it is still logged, at the quietest level, as a contact, and a count of contacts sits in the strip;
- it is **still avoided**: the rover keeps clear of a vehicle or an animal exactly as before, because what the operator
  chooses to be told about and what the rover drives into are separate questions;
- it is **still shared**: the map between rover and scouts holds every observation, because the filter is the
  operator's own view, not the fleet's record.

Changing a chip re-draws everything at once: markers become contacts or threats, cards leave or enter the list, and a
line in the log records the new choice. A contact turned into a threat is listed as PENDING; an alert is raised only for
a detection that arrives after the change, not retroactively.

**Defaults.** Mines and IEDs, Arms and People on; Vehicles, Animals and Other off. This is a choice for a clearance and
border role (the one the operator in the request named), not a measurement.

**Mines and IEDs are guarded.** Turning them off takes the same two-second hold that an irreversible command does
(section 10): a chip clicked off by accident would silence the most dangerous class in the product. Turning them on is
a plain click. Nothing guards the other classes.

**Arms has no source.** No weapon detector is wired into this build. The perception work measured one on web photographs
(perception decisions 47 and 48), and that says nothing about a rover's camera at range, so there is no honest simulated
detection rate to give it. The chip is shown, disabled, and says why; the plumbing treats an `arms` class like any other
the day a source exists.

**The animal alert rule changes.** Until now an animal was reported but never raised as an alert, whatever the
operator wanted. It is now an ordinary class: off by default, and if the operator turns Animals on, a confident animal
raises an alert like a person does.

**Tests.** The classification of every detection type into a class; the defaults; that a contact raises no alert and a
threat does; that a contact is not in the threat list but is on the map and in the log; that it is still shared into the
mesh and still an obstacle to the rover; that turning a class on and off re-draws; the two-second hold on Mines and IEDs
and the single click on the others; and that Arms cannot be switched on.

## Thermal imagery: a physical model, not a palette

**What was asked.** An infrared view on the map: the thermal signature of everything, low to high, as in real life.
That is a request for physics, so what follows is written before it is built, with the checks it will be held to.

**The principle that decides the design.** Thermal imagery is not a recolouring of the visible scene. A surface's
infrared brightness depends on its temperature, its emissivity and what it reflects, and the temperature is the result
of an energy balance that depends on the time of day. The same ground is hot at 14:00 and nearly invisible at dawn; a
rock that is cooler than sand at noon is warmer at night; a buried object shows only when the soil above it heats or
cools at a different rate, and it vanishes twice a day when the two cross. The model therefore has a clock, a sun, a sky
and a ground that stores heat, and the picture is what a camera would record of that.

**Where it comes from (and what is NOT claimed).** This is a **simulation of physics with textbook parameters**, checked
against analytic solutions and against well known qualitative behaviour. It is not validated against radiometric
measurements of a real site, and the material properties are typical published values, not values measured here.

### The model

- **Ground temperature.** A one-dimensional heat-conduction column under every half-metre cell, six layers (1, 2, 4, 8,
  16 and 32 cm) with an implicit solver, forced at the surface by an energy balance: absorbed sunlight
  `(1 - albedo) x (beam x cos(incidence) x not-in-shadow + sky diffuse)`, absorbed sky long-wave
  (Swinbank clear sky, `5.31e-13 T^6`), emitted long-wave `eps sigma T^4`, convection `h (T - T_air)` with
  `h = 5.7 + 3.8 x wind`, and a latent term for vegetation (a fixed share of the absorbed sunlight). Slope and aspect
  change the sunlight each cell receives; rock, trees and bushes cast shadows computed from the same height field the
  cameras draw, at the sun's real position. Two days are run to settle, then one day is stored every ten minutes.
- **Sun and sky.** Solar position from latitude and day of year; clear-sky beam and diffuse sunlight from the Hottel
  model with an altitude correction. Site: **34 degrees north, 3 500 m, 20 October** (the visible camera's fixed sun, 42
  degrees up and 150 degrees round, is what that date gives at about 10:40), clear sky, air temperature 6 degrees C mean
  with a 9 K swing peaking at 15:00, wind 3 m/s. These are choices for a high-altitude border region, not data.
- **Materials** (textbook values; albedo, emissivity, conductivity, volumetric heat capacity): dry soil 0.30, 0.95, 0.30,
  1.3e6; compacted track 0.28, 0.95, 0.60, 1.5e6; low scrub 0.20, 0.98, 0.20, 0.9e6 with a small latent share; rock
  0.25, 0.93, 2.8, 2.2e6; trees and bushes as a thin canopy with a larger latent share; pothole as sheltered bare soil.
  Thermal inertia `sqrt(k rho c)` is therefore about 620 for soil and 2 480 for rock.
- **Buried objects.** A mine or IED is a column of its own, in the same sun and shadow as the ground round it, with
  loosened fill above it (lower conductivity) and, for a metal case, a slab of high conductivity at its depth. The
  surface difference it produces falls away with depth the way the daily temperature wave does (the wave's damping
  depth in this soil is about 10 cm), so a device 30 cm down is invisible to a thermal camera and one at 3 cm is
  faintly visible near dawn and dusk. A one-dimensional column ignores sideways heat flow, which flatters a small
  metal object; that is stated, not corrected.
- **Things that stand on the ground.** A person, a deer and a truck are drawn from their own surfaces: skin, clothing and
  hair; coat, face and legs; painted sheet metal and glass. A body's temperature follows the air and the sun (a coat is
  warmer than the air, a truck's roof is hotter than the ground at noon and colder than the sky-facing ground at night,
  an engine that has been off for hours is not hot). Ground-truth temperatures are set from the air temperature and the
  sunlight at that moment.
- **The camera.** Radiance in the 8 to 14 micron band from Planck's law, emissivity and reflected sky (the clear sky is
  far colder than the air in that band), atmospheric transmission with distance, a small optical blur, and noise equal to
  the sensor's noise-equivalent temperature difference. Rover: the mast camera of the sensor suite, 320 by 256, 50 degrees
  across, 50 mK (CANDIDATE, from `ares-perception`'s sensor record). Scouts: 160 by 120, 57 degrees across, 50 mK (also
  CANDIDATE). The image is resolved by the same rays and occlusion as the visible feed, so a rock hides what is behind it
  in infrared as in light.
- **Display.** A thermal camera has no colour. Brightness is the apparent temperature between two levels set by an
  automatic gain control (the 1st and 99th percentile of the frame) or by the operator, shown through one of three
  palettes (white-hot, black-hot, iron), with a scale in degrees Celsius beside it. The operator picks the local time of
  day.

### What the map shows

The map's infrared layer is a mosaic of what the thermal cameras have actually seen, exactly as the LiDAR cloud is a
mosaic of what the scanner has seen: unseen ground is dark, and each cell keeps the temperature it had when it was last
imaged, fading as it goes stale (the ground changes; the picture does not until it is seen again). The rover's camera
sits 1.5 m off the ground, on the mast of the visible camera, and is read 2 x 2 binned for mapping, so it writes ground
only out to about 23 m (beyond that one pixel covers more than 2 m along the line of sight); a scout looking down sees
detail close to the ground it flies over. People, animals and
vehicles are drawn as live heat blobs for the moment they are in a frame, never accumulated into the mosaic (a walker
would leave a streak). The camera feeds can be switched to infrared.

### Checks it will be held to (written before the code)

| # | Check | Passes if |
|---|---|---|
| V1 | Heat is conserved by the solver (an insulated column with no forcing) | total heat changes by less than 1e-9 relative over a day |
| V2 | Conduction against the analytic answer: a medium driven at the surface by a daily temperature wave | the wave's amplitude at one damping depth is within 5 percent of `A e^-1`, and its phase lags by `z/delta` within 0.1 rad |
| V3 | Clear-sky sunlight at 3 500 m | global horizontal 800 to 1 000 W/m2 with the sun 50 degrees up; zero at night; beam falls as the sun drops |
| V4 | Dry ground, 20 October, 34 degrees north | peak surface temperature 25 to 45 degrees C between 12:00 and 14:30; minimum 3 to 9 K below the air minimum before dawn |
| V5 | Rock against soil | rock's peak is lower by at least 3 K; rock is warmer by at least 2 K between 02:00 and 05:00; the two cross twice a day |
| V6 | Shadow | a cell in a boulder's shadow is at least 5 K cooler than a lit one beside it in the late morning |
| V7 | A disturbed-soil patch 3 cm down | largest difference from undisturbed ground between 0.5 and 4 K, changes sign twice a day; at 30 cm the largest difference is under 0.15 K |
| V8 | A person at 6 degrees C air | exposed skin 24 to 34 degrees C, clothing 8 to 20; the person is warmer than the ground at night and not at midday |
| V9 | Planck | the band radiance table inverts to within 1 mK; integrated over all wavelengths it gives `sigma T^4 / pi` within 0.5 percent |
| V10 | Camera noise | the standard deviation of a uniform scene's apparent temperature is the NETD within 10 percent |
| V11 | Atmosphere | transmission at 100 m is above 0.97 |

### What this cannot say

It cannot say what a particular real minefield looks like: real soil moisture, vegetation, recent weather and how long a
device has been buried change a thermal signature by more than the model's own error, and that is why mine detection by
infrared is done with ground-penetrating radar and metal detection and not by thermal alone. It does not model rain,
snow, wind-driven cooling gusts, fog, smoke, or a sun that moves in the visible camera (the visible feeds keep their fixed
late-morning light whatever time of day is chosen for infrared: at night the picture in the visible feed is wrong and
is labelled so in the panel). Animals and people are two or three part models, not anatomical ones.

### Results of the thermal build: what came back

All eleven registered checks were run. Measured values, then what did not hold.

| # | Check | Measured | Verdict |
|---|---|---|---|
| V1 | Heat conserved by the solver | largest relative drift over a day, 2.8e-14 | HELD |
| V2 | A daily wave against the analytic answer | damping depth 8.0 cm; amplitude at one depth 3.775 against 3.780 (0.1 percent); lag 0.975 against 0.973 rad | HELD |
| V3 | Clear-sky sunlight at 3 500 m | 834 W/m2 global horizontal at 50 degrees (beam 1 015, diffuse 57); zero at night | HELD |
| V4 | Dry ground, 20 October | peak 27.6 C at 13:00; 3.1 K below the air minimum before dawn | HELD, **after a model change**, see below |
| V5 | Rock against soil | rock's peak 5.6 K lower; 5.7 K warmer at night; crosses soil twice | HELD |
| V6 | Shadow | late morning, lit ground 22.4 C and shaded 6.7 C: 15.8 K | HELD |
| V7 | A disturbed patch 3 cm down | largest difference 0.75 K, two sign changes a day | HELD |
| V7 | A device 30 cm down, under 0.15 K | **0.80 K** | **REFUTED** |
| V8 | A person at 6 C air: skin 24 to 34, clothing 8 to 20 | face 28 C, clothing 12 C and legs 11 C | HELD |
| V8 | Warmer than the ground at night | by more than 10 K | HELD |
| V8 | Not warmer than the ground at midday | warmer by about 5 K at this site | **REFUTED** |
| V9 | Planck | table inverts exactly; the all-wavelength integral equals the Stefan-Boltzmann value | HELD |
| V10 | Camera noise | the standard deviation matches the noise-equivalent temperature difference within 10 percent | HELD |
| V11 | Atmosphere | transmission 0.980 at 100 m, 0.819 at 1 km | HELD |

**V4 failed first, and the model was changed afterwards.** With a steady 3 m/s wind all night, the ground cooled only 2.2 K
below the air before dawn, short of the registered 3 K. Clear nights are calm, and a wind that eases at night and peaks
mid-afternoon is a real effect, so the wind became `2.4 m/s x (1 + 0.5 cos(2 pi (h - 14) / 24))` (1.2 m/s at 02:00, 3.6 at
14:00) and the check was rerun: 3.1 K. That is inside the range but on its lower edge, and the change was made because the
check failed. A reader should treat the night cooling as the model's least certain result.

**V7 at 30 cm was refuted, and the prediction was the mistake.** The check expected a device 30 cm down to be invisible. But
a hole dug to bury something at 30 cm is backfilled with loosened soil all the way to the surface, and the surface only
feels the top few centimetres, so a fresh burial is as visible at 30 cm as at 3. What hides a deep device from a thermal
camera is time (rain compacts the fill), not depth. The model has no weathering, so every buried object here is a fresh
one, and the sentence in the model description above that a device 30 cm down is invisible was wrong for fresh ground.
All five objects in the default field show about 0.75 K at their largest. A thermal camera with 50 mK of noise sees a
difference that size only where the pixel is small, which is why it shows from a scout looking down and barely from the
rover.

**V8 at midday was refuted, and the prediction was a hot-desert one.** At 3 500 m on a cool October day the ground reaches
26 C while a person's sunlit side reaches 30 C or more, so the person stays warmer than the ground all day here. In a hot
desert, where the ground is 50 C, the expectation would hold.

### What was built into the console

- **Layer.** A button on the map cycles LIDAR, IR and LIDAR + IR. In IR the map shows the mosaic with a colour scale in
  degrees C, the sensor's band, how many thermal cameras are running and what share of the ground has been seen. A row of
  controls appears: palette (iron, white-hot, black-hot), gain (automatic, which follows the ground seen with a one-kelvin
  hysteresis so the legend does not flicker, or hold), and the time of day in one-hour steps. Setting the time of day does
  not change what was imaged before; those cells keep the temperatures they had then and fade as they age.
- **Tiles.** Every camera tile has an IR chip in its corner. Switched to infrared, the tile is drawn by the thermal camera
  at its own resolution (320 x 256 for the rover, 160 x 120 for a scout), with the ground end of the frame kept when it is
  cropped, a scale in degrees C, and no detector boxes: the detector's claims are about the visible picture, and a
  thermal detector's curve is not measured.
- **Power.** The thermal cameras run only when infrared is wanted (the layer or a tile). Otherwise nothing is imaged and the
  mosaic does not grow. A cut link stops the mosaic, as it stops every other shared product.
- **Cost.** One thermal frame is 47 ms for the rover's mast camera at full size (about 20 ms binned) and 15 ms for a scout.
  The mosaic is written once a second of mission time. The day is solved in 11.9 s the first time, in a process of its own
  (a thread inside the console took several minutes, because the interface and the renderers hold the interpreter), and
  stored beside the user's other caches; from then on it loads in 0.2 s.

### Bounds that stand

The visible feeds keep their fixed late-morning light whatever time of day infrared is set to, so at night the visible
picture is wrong; this is stated here and not hidden. The sky is a clear one, always. There is no weathering of buried
objects, no rain, snow or fog, and no wind gusts. The engine of the parked truck is cold. The scale of the signature of a
buried object is a model's, not a measurement's. The tests in `test_thermal_physics.py`, `test_thermal_scene.py`,
`test_thermal_map.py`, `test_thermal_system.py` and `test_thermal_window.py` hold all of the above.

## The animals: antlers that read as antlers, an ibex, and a glyph

**What was wrong.** At working distance the buck's antlers were two tall thin spikes, which read as a gazelle's horns, and
the console drew every animal with the question mark of an unclassified contact, because the spec's enum has no animal
class. There was also one animal in a country whose animal is not a deer.

**What changed.**

- **The buck's antlers** spread into a V and branch: a thicker main beam (3.6 cm at the burr), five tines each side, the
  brow tine forward and the crown at the top. The buck is now 1.91 m tall and 0.72 m across the antlers (it was 0.38 m
  across), and its recorded outline follows, so the detector's box and the picture still agree.
- **An ibex**, the Siberian ibex that lives in this kind of ground: a stocky, short-bodied animal (1.64 m long against the
  deer's 2.14, 0.52 m across the barrel and horns), a pale grey-brown coat with a lighter belly and darker legs, short
  ears, a dark short tail, a beard, and heavy flattened horns that sweep up and far back in a scimitar with a transverse
  ridge every few centimetres. It is built on the deer's skeleton and gait, then made shorter in the leg and body and
  wider across, with its own horns and beard added afterwards so that they are not squashed. About 15 000 triangles, in
  12 walking poses. It walks a slow path on the far side of the lanes.
- **A glyph**: a deer in profile for the map marker, the threat card and the alert badge, in place of the question mark.
  The label still says ANIMAL.
- **The thermal model** takes an animal's body width as at most half a metre, so a buck's antlers are not drawn as warm
  body.

**Tests.** The renderer-versus-model box test now runs for the buck and the ibex as well as the person and the truck (the
drawn animal is where the detector model says its box is); the ibex is shorter-bodied, lower and wider than the buck and its
horns sweep behind its head; every style in the default field has a figure with the right number of poses; and the window
starts with one person, one vehicle and two animals.

**Not covered.** The ibex does not climb: it walks the same flat ground as the deer. Its horns are two tubes with a ridge
profile, not a keratin sheath with growth rings and a worn tip. A herd of one is not a herd.


## Arms is a working class: a weapon stage on a person the camera has already found

**What was wrong.** The Arms chip was shown disabled, because no weapon detector was wired into this build. That was
honest but it left a dead control on the strip, and the perception work has since measured a gun model (decisions 47
and 48 of `ares-perception`). The question was how to use it without claiming something it does not support.

**Why it cannot simply run on the frame.** The gun model was measured on photographs, where a gun fills a good share of
the picture. The rover's camera at 20 m sees a rifle as a few hundredths of a percent of its frame, forty times smaller
in area than the smallest band anything was measured in. Running it on a whole frame and reading a number off the end
of the curve would be inventing one.

**What is built instead, and it is what a real system does.** A second stage on a person:

1. the person detector finds somebody (the curves already in this build);
2. the frame is cropped to their box and enlarged;
3. the weapon model runs on that crop.

In the crop a carried rifle is a large share of the picture, which is the regime that **was** measured. So the curve is
indexed by the weapon's area as a share of the crop, exactly as decision 48 measured it, and the console computes that
share against the person's box. `tools/build_ocu_weapon_curves.py` in the perception repository writes the curve from
the arm B model's own detections on the author-disjoint test images at its registered threshold; nothing is re-run or
re-tuned, and `ocu/data/weapon_curves.json` is a copy of that manifest, which a test checks has not drifted.

**What that gives, from the measurements and not from a wish:**

| | share of the person's crop | measured recall |
|---|---|---|
| a rifle carried in the open | 0.086 | about 0.65 per frame |
| a pistol | 0.016 | **below anything measured; the console does not claim to find one** |

- **Range.** Enlarging cannot invent detail, so the stage runs only while the weapon is at least 12 pixels across in the
  original frame, which on the rover's camera is about 27 m. An allocation, not a measurement.
- **Persistence.** A weapon call needs two hits in the last five frames of a confirmed person track, for the same reason
  a person needs three: one frame of a classifier is not a report.
- **False calls.** Someone carrying nothing is called armed at **0.024 per crop**, the rate decision 48 measured on
  photographs of people holding a bat, a racket, a phone, a camera, a tripod or an instrument. Every one of those was
  looked at by eye and none was a gun, so that rate is the model's own.
- **One contact, not two.** An armed person is reported as `weapon`, labelled "armed person . long gun", with the
  person's own position and confidence. The weapon stage describes a contact the camera already has, so an armed call
  replaces a plain `person` on the same voxel whatever the confidences are (`threat_filter.replaces`).
- **The scene** now holds an armed walker crossing the lanes ahead, drawn with a rifle held across the body (about
  11 000 triangles, 12 poses) and a map glyph of a figure with a slung rifle.

**What is still not claimed.** The recalls are from web photographs; a crop from a rover camera at 20 m, enlarged, is
blurrier than a photograph, so they are an **upper bound** and the caveat travels with the curve into the console.
Handgun rests on a few dozen boxes. Decision 47's random-box control still scored 0.26 to 0.44 AP50 on this data, so some
of the apparent skill is recognising a gun photograph rather than locating the gun. Nothing was measured at night, in
thermal, from the air, or on a weapon that is not being carried in the open, and the console does not pretend otherwise:
the thermal camera has no weapon stage at all.

## The speed control goes past the published figure, and says so

The operator's speed slider ran to the platform's published 15 km/h (4.2 m/s). Crossing the map at a demonstration took
real time, so the control now runs to **8 m/s** (`ROVER_COMMAND_CEILING_MS`), with the published figure still recorded
as `ROVER_TOP_SPEED_MS` and sitting at about three quarters of the slider's travel.

Above the published figure the speed readout turns amber and its tooltip says that it is past 15 km/h, a console
ceiling and not a vehicle figure. The power model stays calibrated to 15 km/h whatever is commanded, so the endurance
beside it is not a prediction at those speeds, and the rover panel's reference now reads "/ 15.0 km/h published".

## Saying what the controls do

The thermal row read `IRON | GAIN AUTO | -1 H | LOCAL 10:41 | +1 H`, which is thermal-camera jargon in a line of bare
words. Each control now sits under a caption, as the panels on the right do, and says what it changes:

| was | now | what it is |
|---|---|---|
| `LAYER IR` | `MAP  THERMAL` | which picture of the ground the map draws: LIDAR, THERMAL, or both |
| `IRON` | `COLOUR` `IRON` | the colours warm and cold are drawn in |
| `GAIN AUTO` / `GAIN HOLD` | `BRIGHTNESS` `AUTO` / `LOCKED` | which two temperatures are drawn darkest and brightest |
| `- 1 H` `LOCAL 10:41` `+ 1 H` | `TIME OF DAY` `< 10:41 >` | local solar time at the site, in one-hour steps |

Every one carries a tooltip of two or three lines saying what it is for, and every chip on the threat strip now names
what finds that class. The map's own furniture (the compass, the mesh line, the colour bar) is placed under the
controls rather than at fixed heights, so a wider toolbar never covers it. Marker labels on the map are one word
("ARMED") where the card has room for the full name, because a label long enough to run across the rover is worse than
a short one.

**Also removed:** `_draw_terrain` and `_generate_terrain`, the decorative contour layer. It had not been called since
the map began drawing the real scan, and it named a colour it never imported, so it would have raised a `NameError` the
moment anything called it.

## The merge was not commutative on a full tie (a real defect, now fixed)

**The defect.** `merge()` in `core/mesh_state.py` decided on confidence, then on timestamp, and stopped there. When two
observations of the same voxel tied on both and differed in `threat_type` or `source_id`, it fell through to argument
order: `merge(a, b)` returned `a` and `merge(b, a)` returned `b`. Two replicas that received the same two observations in
opposite orders therefore held different maps, and nothing would ever correct that, because each side kept the one it
had. This is the property the shared map is claimed to guarantee (any order of the same updates gives the same map), and
the claim is part of what we describe as patentable. The module docstring and the unit tests both said the merge was
commutative. For this one class of input it was not. `MeshLink`, `reconcile()` and `MeshState.apply()` all inherited the
defect, since they all call `merge()`.

**How it was found.** Abir read `merge()` while preparing the two-device demo, where two real replicas would each record
an observation of the same voxel, and reproduced it by hand with
`a = VoxelObservation((1,2,0), 1000, 0.9, "mine", "rover")` and
`b = VoxelObservation((1,2,0), 1000, 0.9, "ied", "aircraft_1")`. The existing convergence tests did not catch it because
every tie in them was on confidence only, with different timestamps. None built a tie on confidence and timestamp both.

**The fix.** One total order over observations of a voxel: `(confidence, timestamp_us, source_id, threat_type)`, compared
left to right, exposed as `VoxelObservation.order_key()`. `merge()` returns the maximum under that order. A maximum under
a total order is commutative, associative and idempotent by construction. The earlier rule is unchanged where it decided
anything: higher confidence still wins, then the newer timestamp. `source_id` and `threat_type` only break ties that were
previously decided by argument order. The rule is now longer than the one in the build specification's section 12.2,
which names only confidence and timestamp. That is a deliberate addition, and the specification should be updated to
match.

Two related defects closed with it. `VoxelObservation` now rejects a NaN confidence and any confidence outside 0.0 to
1.0 at construction, because NaN compares false against everything and breaks the order. And it stores `-0.0` as `0.0`:
the two compare equal but serialize differently, so two replicas could otherwise hold equal observations with unequal
bytes. Callers were checked first. The only production caller, `_on_detection` in `ui/main_window.py`, receives
confidences from `fuse()` and the camera tracker, both capped at 0.97, and the full test suite builds none outside 0 to 1.

**The tests** (`tests/test_mesh_state.py`). The exact reproduction above. A tie on everything except `threat_type`. A
seeded property test, 5,000 cases drawn from a small value pool so that ties are common, checking commutativity,
associativity and idempotence, and that `repr` of the two merge results is identical. A convergence test that applies 200
observations plus 100 duplicates in 40 different shuffled orders to 40 replicas and asserts a byte-identical canonical
serialization on every one. A pairwise `reconcile()` of the tied pair. Rejection of NaN, infinity and out-of-range
confidence, and the `-0.0` normalization. With the fix reverted, six of these fail; with it applied they pass. No
existing test encoded the old behavior and none was edited.

## Thermal detection, and where the demonstration puts its threats

**Thermal detection.** The infrared tiles used to be pictures only. `core/thermal_detect.py` now reads the imager's
temperatures the way a detector does and knows nothing of what is in the world. The background is the median of each
row of the picture (a warm body cannot raise its own background, which a window mean lets it do). A blob is anything
beyond 3.5 robust standard deviations and at least a degree; blobs are joined across small gaps, and a strong blob is
split again at half its own peak, because a body on sunlit ground is one blob at the clutter's threshold and two at its
own. A person or animal must be at least 3 K warmer than the picture, 2.5 K warmer than the ring round it, fill 40% of
its box, have 40 pixels and about five across its smaller side (the recognition criterion), and be the size and shape
of one. Positions come from the nearest surface a pixel sees: the imager's ground hit for a body is the ground behind
it, which put a person tens of metres out, and is no longer used for bodies.

Measured on 120 random looks at 07:30 (`/tmp` probe, not shipped): people seen by the rover are found about three
times in four, animals about two in five (the ibex is often partly hidden), and a handful of false person or animal
calls arise, mostly warm patches on the parked truck. The pipeline therefore adds three things: a confirmation gate
(3 looks in 20), a clearance (5 m) round any vehicle, and a rule that a thermal call does not add a second contact
beside a body of the other kind. Thermal vehicles are classed by the detector but not reported: a stand of trees makes
the same shape, and the visible camera reports vehicles. Scouts at 20 m resolve a person only within about 25 m.

**The start hour is 07:30.** At 10:40 the ground in the sun is warmer than a clothed person, so an infrared sweep finds
nothing it should. The visible sun is the thermal site's sun at the start hour (azimuth 112, elevation 12), as the
earlier test already required of the two, so the picture and the physics agree. The operator can still step the time of
day.

**Sites.** Everything buried used to be within about 50 m of the start, so the demonstration read as one cluster. The
sweep block keeps its targets; three further sites (B, C, D) are placed on open gentle ground 35 to 90 m out, with a
crossing, a pattern row, and a deep heavy object with scrap, and with a person, an animal and the truck spread over
them. `core/scenario.py` finds the ground; nothing is positioned by hand.

**Display.** The thermal gain never stretches a picture narrower than 12 K (14 K for the map), and the default palette
is white-hot: a sun-lit slope at a low sun differs from its shade by several kelvin, and stretching 4 K over a colour
ramp made that read as noise.

## Reference footage: the world is made to look like the ground it was filmed on

Seven iPhone 15 clips of a dry deciduous foothill forest (about 30.87 N, 76.74 E, 420 to 480 m, early November) are
the reference for how the simulated ground looks. The raw video (5.3 GB) is not in the repository (`reference_footage/`
is ignored); what is derived from it is, about 4 MB: `assets/footage/ground/` and `ocu/data/footage_palette.json`.

**Reading the clips** (`scripts/footage_ingest.py`). The video is 10-bit HLG, which a plain decode shows grey and flat,
so it is converted to sRGB (HLG inverse OETF, BT.2020 to BT.709, a gentle highlight shoulder). Three clips carry a
rotation flag and two were filmed partly upside down (IMG_5109 for 44 s, IMG_5128 for 61 s) with no flag at all.
Orientation is decided from where the sky is: upside down after two consecutive samples with the sky clearly at the
bottom, upright again on one clear sample at the top, unchanged between. The first version used a vote per frame and
turned 34 upright frames of IMG_5108 over because shaded ground and a sign read as blue; the state machine is the fix.

**Colours** (`scripts/footage_palette.py`). Medians over every sharp frame, by colour and position only: lit trail 199/167/130,
shaded 94/79/62, lit straw 191/171/115, lit foliage 106/117/76, and a sky profile above the horizon from the hilltop
clip (178/188/195 at 1 degree, 142/185/218 at 10). Measurements stop at 14 degrees; the sky above that is continued
to a deeper blue by assumption (`palette.ABOVE_DEG`), which the footage does not show. The renderer finishes in an exposure,
a filmic curve and a gamma, so each measured colour is run backwards through that ending (`render/palette.py`) to find the
scene colour that comes out as the measured one. After the change, the rendered sky at the top of a level view is
150/187/218 against the footage's 151/194/232, lit sand 198/178/147 against 199/167/130 and lit grass 181/163/106
against 191/171/115. Shade is still lighter in the simulation than in the footage (132/114/90 against 94/79/62): the
footage's shade is dappled under canopy, and the simulation has fewer and softer shadows.

**Ground** (`scripts/footage_textures.py`). Five tiles (track, ground, scrub, dark, slope), each cut at full video resolution from a
patch chosen by eye, un-lit by dividing out its slow brightness, made seamless by blending with itself shifted by half
a tile (variance-preserving, so the join keeps its grain), and given a normal map from the brightness. The normal maps
are a stand-in for a scan: there is no depth in the footage. A tile covers about 1.35 m (3.3 m for the second sample), not
the 2.6 and 6.1 of the photographs they replace. Open ground takes 45% of the straw tile, because the footage's ground
is under dry grass and leaves almost everywhere it is not trail.

**Plants and stone.** The conifers and the round "lollipop" trees are gone. Trees are now three forms built from tapering
forked limbs and clouds of small leaf sprays: an acacia with an open umbrella crown, a rounder broadleaf, and a tree standing
leafless as many are in the dry season (`terrain.py` chooses between them by position). Shrubs are open and ragged with a
share of dead leaves, and grass is a kans tussock, narrow arching blades olive at the root and straw at the tip with a few
flowering stems. The fynbos shrub and green grass clumps from Poly Haven are no longer used (their downloads remain
in `fetch_assets.py` and are simply unreferenced). The downloaded boulders are lichen-covered granite, orange, which
nothing in the footage is; their colour is replaced with a grey-tan and the photograph's grain kept. Grass grows in
patches and clumps by a slow noise, not evenly. These are choices of look, made from the footage by eye; no count of
trees or grass was measured from it.

**What this does not do.** The terrain's shape is still the simulation's own rolling ground. The footage has gullies, cut
banks and steps that the height model does not generate, and nothing was reconstructed from the video in three
dimensions (photogrammetry would only cover the strip that was walked and would break what the rover's planner and the
LiDAR rely on).

**The horizon.** Past the edge of the simulated ground the terrain used to be a flat plain for five kilometres. It is now
rolling hills that rise from the edge (a smooth rise over 170 m, then about 40 m of relief at the scale of a few hundred
metres, from two octaves of value noise), shaded as forest canopy by the terrain shader (dark olive in the folds, dry
patches on the open slopes, lumpy at the scale of a crown) and left to the haze, as in the hilltop clip. The ring of
vertices is 29 rings out to 5 km: the first version, with 15, shaded in visible streaks across its large triangles.
It is scenery only: nothing in the planner, the LiDAR, the thermal model or the detector reads it, and the shadow map
still covers only the simulated ground.

## Reconstructing the trail from the footage (in progress)

Procedural plants and tiled textures can get near the footage but not be it, so the next step reconstructs the real trail.
Two tools were downloaded with the user's approval: pycolmap 4.2.1 (PyPI), in its own environment under
`reference_footage/recon/venv`, and Brush v0.3.0 (`brush-app-aarch64-apple-darwin.tar.xz`, 41.4 MB, from the project's GitHub
releases; its SHA-256 was checked against the published one before unpacking). Both live under `reference_footage/`, which is
git-ignored; the scripts that drive them are tracked (`scripts/recon_frames.py`, `scripts/recon_sfm.py`).

First result, IMG_5129 (the 41 s climb up a trail between walls of dry grass): 207 sharp frames chosen by `recon_frames.py`
(sharpest of each window at about 5 per second), all 207 registered by COLMAP into one model of 91 189 points with a mean
reprojection error of 0.63 px (a fit well inside what the footage can support). The first run also produced a spurious
two-image fragment, ignored. The reconstructed camera path folds back on itself near the end, which is not what was
walked; it has not yet been diagnosed. The model has no real-world scale yet (the path is 21.5 model units; the
walk is estimated at tens of metres), which will come from the GPS tags and known sizes (the trail's width, a person's
height) before it is placed in the simulation.

Not done yet: training the splat. The training tool is a downloaded binary, and running it was refused by the environment's
permission check, so it has not been run; the dataset is prepared in `reference_footage/recon/IMG_5129/dataset`.

**First splat, and what it showed.** A 1 500-step training of IMG_5129 (155 000 splats, a few minutes) already redraws the
real frames from their own camera poses with the right trail, shadows, grass walls and sky, only soft. That is the test that
matters for a reconstruction (`scripts/splat_check.py` draws frame and splats one above the other), and it passed at the first
attempt. `render/splats.py` is the renderer (GPU, one float texture plus an index buffer, sorted on the CPU), and
`render/trail.py` finds the route in metres. The camera the clip was filmed on has a field of view of about 90 degrees
(the COLMAP estimate, f = 643 px at 1280): it is the phone's ultra-wide lens.

**Scale is the weakest number.** A reconstruction has none. Anchored to a camera height of 1.45 m the route comes out 87 m
long for a 41 s clip, 2.1 m/s, which is not a walk up a hill. Anchored to a walking pace of 1.25 m/s it is 51.5 m long, rises 5.4 m
(about a 10% grade) and implies the phone was held 0.92 m above the ground, which is low but within what the footage looks like.
The pace is used; the height is reported as a check. Both are assumptions, good to a quarter either way. One real measurement
(the trail's width at a spot, or the distance walked from a fitness app) replaces them: `TrailRoute.rescale_to`.

**The trail, trained.** IMG_5129 trained for 12 000 steps with 500 000 splats (the cap was reached by about step 6 000), 15.4 mean
absolute error per pixel (0..255) against the real frames at their own poses. It is the real trail: the gravel, the tyre ruts,
the leaf litter, the shadow pattern and the sky are in the right places. It is not as sharp as the video. The grass walls
streak into soft blades, because grass is thousands of fine strands and 500 000 splats are too few for it (reconstructions of
this kind use millions); the first 10 m of the walk, seen by few frames, is poor and has floaters near the camera when the
view is lowered to a mast's height. The replay in the rover's tile (the TRAIL chip) therefore starts at 10 m. Training took
about 1 h 40 min on the M1; the cap is set by 9 GB of shared memory.

## A world built round the trail (groundwork)

`python -m ocu.main --trail [NAME]` (or `OCU_TRAIL=NAME`) builds the world on a reconstructed trail from `assets/footage/trail`
(`core/trail_scenario.py`). Without the flag nothing changes. With it:

  * the real centreline is put into the world with the deploy point on it, 10 m into the walk (the first stretch is the poorest
    reconstructed), running north; the world is right-handed with z up, east being to the right of travel;
  * the terrain is carved to it: elevation follows the walk's own ground along the line (blending back to the procedural
    ground over 7 m), a 3.2 m strip becomes TRACK, 2.5 m of verge either side becomes SCRUB, and the boulders, trees,
    bushes, pebbles and holes that were there are removed so that LiDAR and planner meet what the camera shows;
  * the sweep is one lane along the line (waypoints every 3 m). `TrailSweep.at` places the scenario's buried objects and people
    relative to the line, with a lane worth 0.7 m across and an offset worth half, so the code written for four straight lanes four
    metres apart lays out a footpath instead; vehicles are left out of a footpath's content, and the far sites are not built;
  * the rover's tile is drawn from the real place: where the rover is across and along the trail, and which way it faces, select the
    camera on the walk (`camera_state`). Within 1.0 m of the line and 30 degrees of the walk's direction it is drawn from the
    splats; outside that, which the walk never saw, the simulated world is drawn as everywhere else.
  * figures (people, animals) are drawn first, with depth, and the splats are tested against it (`SplatRenderer.render(depth=...)`), so a
    clump of grass in front of someone hides them, and they stand on the carved ground. The sky and the shadows of the figures
    are the simulation's.

Not done: the scouts' cameras still draw the simulated world with the corridor cut into it, not the splats; a second trail cannot be
chained to the first; the thermal model and the detectors have not been tuned for this world.

**Chaining the trails.** One trail is 30 to 45 m of patrol, which is a minute of driving, so `--trail` with no name chains every packed trail
into one patrol (`TrailWorld`): each leg's deploy point is placed at the end of the one before it, leaving in the direction it arrived,
except that a leg may turn up to 35 degrees at the join (80 when the path is more than 70 m from the middle of the map, which is steering
back inside the world; the cut between two real places hides the corner). With legs of about 40 m the patrol stays inside +-80 m however
many there are (checked to 20 legs, 800 m). Only the patrol part of each leg is carved and driven, from its deploy point 10 m into the
walk to 9 m short of its end, so the poorly reconstructed head and the filmer's shadow at the tail are never on the path. The carved path
is one path, with the ground carried across each join at the same height; what changes at a join is only which real place the rover's
camera is drawn from (a cut, as between shots). The scene keeps the current leg's splat and the next one's on the GPU and reads the next from
disk when the rover is within 12 m of the end of the current one. Every leg gets buried objects and a person and an animal of its own.

**Reconstructing everything** (`scripts/recon_scheduler.py`): eleven jobs, IMG_5108 in six 40 s segments and the other five clips whole
(IMG_5117 is someone standing at a viewpoint, with no walk to reconstruct from). Frames, structure from motion (CPU) and training (GPU) are
separate stages with marker files, so the scheduler can be stopped and started and the CPU stages of the next trail overlap the GPU stage of
this one. It tidies as it goes (feature database, frames and intermediate checkpoints are deleted once the stage after them has what it needs),
because nine more trails would otherwise want more disk than there is. Each trail is trained for 9 000 steps with up to 600 000 splats,
against 14 000 and 800 000 for the first.

**One bad lens estimate.** IMG_5108 segment A reconstructed all 200 frames but with focal lengths of 1404 and 1073 px, which cannot be (square pixels), while
segments of the other clips agreed within 0.5%. A side-by-side on 67 frames of segment B showed the lens model was not the cause (RADIAL and
OPENCV both fit at 0.455 px with a focal length of 1391 px, OPENCV's x and y within 0.3%): segment A had found a bad solution of its own. So for the
IMG_5108 segments (one camera, one setting) the lens is pinned at B's values (`LENS_5108` in `scripts/recon_scheduler.py`; `recon_sfm.py`
takes the parameters and does not refine them), and A was re-reconstructed. A lens that a reconstruction is free to get wrong stretches the
whole world it builds; pinning it removes that, and is faster.
