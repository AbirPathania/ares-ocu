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
