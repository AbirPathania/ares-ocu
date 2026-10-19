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
sweep's progress is parked so CLEAR DESTINATION resumes it. Refused when the
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
