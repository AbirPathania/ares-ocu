# ARES Operator Control Unit: the operator's guide

Everything on the screen, what it means, and where every number on it comes from.

This guide is for somebody sitting in front of the console for the first time. It assumes nothing about thermal
cameras, autonomy or the ARES programme. Where a figure is measured, it says what measured it; where a figure is a
choice, it says that it is a choice.

**Read this first.** No ARES hardware exists. The vehicles, the ground, the weather and the sensors are simulated. What
is real in this application is the navigation, the shared-map merge, the formation geometry, the flight model, the
power model, the physics of the heat picture and the measured detection curves. What is simulated is the ground, the
objects buried in it and the camera that looks at it. Say so out loud before anybody asks.

---

## Contents

1. [Starting it](#1-starting-it)
2. [The screen in one page](#2-the-screen-in-one-page)
3. [The top bar](#3-the-top-bar)
4. [Camera feeds](#4-camera-feeds)
5. [The fleet roster](#5-the-fleet-roster)
6. [The map](#6-the-map)
7. [Infrared](#7-infrared)
8. [Deciding what counts as a threat](#8-deciding-what-counts-as-a-threat)
9. [The threat list](#9-the-threat-list)
10. [The alert](#10-the-alert)
11. [The rover panel](#11-the-rover-panel)
12. [Flying the scouts](#12-flying-the-scouts)
13. [The link](#13-the-link)
14. [The event log](#14-the-event-log)
15. [Keyboard and mouse](#15-keyboard-and-mouse)
16. [Where every number comes from](#16-where-every-number-comes-from)
17. [A sixty second demonstration](#17-a-sixty-second-demonstration)
18. [When something looks wrong](#18-when-something-looks-wrong)

---

## 1. Starting it

Double-click **`start.command`** on macOS or **`start.bat`** on Windows. The first run builds a virtual environment and
installs PySide6, which takes about a minute. Every run after that opens immediately.

The window opens at 1600 by 960, or smaller if the screen is smaller. Nothing is running yet: the rover is stationary
at the deploy point, the three aircraft are folded in its bay, and the mission clock reads 00:00.

Press **DEPLOY** in the top right to begin.

---

## 2. The screen in one page

```
┌──────────────────────────────────────────────────────────────────────────────────────────────┐
│ CLOCK   WHAT THE ROVER IS DOING        LINK MODE   LINK STATE  BATTERY   SPEED   HOLD  RESET  │
├───────────────┬──────────────────────────────────────────────┬───────────────────────────────┤
│ CAMERA FEEDS  │  THREAT ALERT (only when one is waiting)     │  TREAT AS A THREAT            │
│  ROVER        │                                              │   the six class chips          │
│  TRI-1        │  toolbar: LAUNCH  CUT LINK  GO TO  MAP       │                               │
│  TRI-2        │  thermal: COLOUR  BRIGHTNESS  TIME OF DAY    │  THREATS                      │
│  TRI-3        │                                              │   one card per contact         │
│               │                                              │                               │
├───────────────┤                  THE MAP                     ├───────────────────────────────┤
│ FLEET         │                                              │  ROVER or AIRCRAFT panel      │
│  four rows    │                                              │   or FORMATION LAUNCH         │
├───────────────┴──────────────────────────────────────────────┴───────────────────────────────┤
│ EVENT LOG                                                     │  LINK over the last 60 s      │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
```

The left column is **what the fleet sees**. The middle is **where everything is**. The right is **what to do about it**.
The bottom is **what has happened**.

---

## 3. The top bar

| Control | What it does |
|---|---|
| **Clock** | Mission time, counting from DEPLOY. Everything in the log is stamped with it. |
| **Status** | What the rover is doing right now, in words: SWEEPING LANE 3, EXPLORING TO DESTINATION, HOLDING, RETURNING TO BASE. Derived from the vehicle's own state, never announced by a script. |
| **Link mode** | STANDARD, AUTONOMOUS, GHOST. See [The link](#13-the-link). |
| **Link state** | CONNECTED, DEGRADED or LOST, and whether the maps are diverging. |
| **BATT** | The rover's battery, integrated from the power model. |
| **Speed slider** | The most the rover will drive at, in any mode. See below. |
| **Speed value** | What the slider is set to, in metres per second. |
| **HOLD / RESUME** | Pause and resume the mission clock. Everything stops together. |
| **RESET** | Start the mission again from zero. Clears the map, the threats, the log and the heat picture. |

### The speed slider

The scale is not linear. The useful speeds for a clearance lane are all in the first few tenths of a metre per second,
and a linear slider would bury them in the first few pixels, so the scale is squared: the left half is the crawl, the
right half is transit.

- **Default** 0.4 m/s, the survey crawl. At that speed an object stays inside the 40 cm pulse-induction swath for about
  a second, which is what the detection pass needs.
- **Published platform figure** 15 km/h, or 4.2 m/s, at about three quarters of the slider's travel.
- **Top of the control** 8 m/s, which is 28.8 km/h.

**Above 15 km/h the speed figure turns amber.** That is deliberate. The vehicle is not claimed to do it. The control
goes there so that crossing the map at a demonstration does not take real time, and the power model stays calibrated to
15 km/h whatever is commanded, so the endurance figure beside it is not a prediction at those speeds.

---

## 4. Camera feeds

One tile per unit: the rover's forward camera and each aircraft's forward-looking camera, tipped down 18 degrees.

The pictures are **rendered from the same ground truth the sensors are simulated against**. A camera sees what is out
there, including ground the LiDAR has not reached. It is not recorded footage, and it is not a photograph of anything.

**What a tile can say instead of a picture:**

| Caption | Why |
|---|---|
| IN BAY | the aircraft is folded in the rover's drawer and its camera is off |
| CLEARING ROVER, CAMERA ON AT 4 m | it has lifted off but would be looking at the rover's own body |
| CAMERA STARTING | the 3D scene is still being built, which takes a second |
| NO VIDEO, STATUS LINK ONLY | AUTONOMOUS: the camera is running, the link is not carrying video |
| EMISSIONS SILENT, NO DOWNLINK | GHOST: nothing is being radiated at all |

**Detector boxes.** A box drawn over the picture is the camera detector's claim about something in view.

- **dashed** the detector has seen it once or twice and it has not persisted yet
- **solid** it has persisted and has been reported
- **thin and grey** the class is switched off in TREAT AS A THREAT, so it is a contact rather than a threat
- colours: red for a person, blue for a vehicle, violet for an animal, grey for a contact

**The IR chip.** Every tile has a small **IR** button in its top right corner. Click it and the tile is drawn by that
unit's thermal camera instead of its visible one, at the sensor's own resolution (320 by 256 for the rover, 160 by 120
for an aircraft), with a temperature scale down the right-hand side. Click it again to go back.

A thermal tile carries **no detector boxes**. The detector's measurements are about the visible picture, and no thermal
detection rate has been measured, so the console does not draw one.

---

## 5. The fleet roster

Four rows: the rover and the three aircraft. Each row carries the unit's name, whether it is under AUTO or MANUAL
control, a battery bar, what it is doing, and how long its battery will last at the current draw.

Click a row to select that unit. The panel on the right follows the selection, and the map marks it.

---

## 6. The map

The map is drawn north-up at 4.5 pixels per metre by default. Scroll or pinch to zoom, drag to pan.

### The toolbar

| Button | What it does |
|---|---|
| **LAUNCH n READY** | opens the formation console for the aircraft still in the bay |
| **CUT LINK** | severs the link on purpose, so the two map replicas drift apart. **RESTORE LINK** puts it back |
| **GO TO** | arms the map: the next click sets a destination. Becomes **CLEAR DESTINATION** once one is set |
| **MAP** | which picture of the ground is drawn: **LIDAR**, **THERMAL**, or **LIDAR + THERMAL** |

### What is drawn on it

- **The LiDAR cloud.** Ground the scanner has actually swept, coloured by what it was classified as. Unseen ground is
  left dark, so the shape of the cloud is the shape of what has been looked at. Obstacles cast shadows that close as
  the rover drives round them, and a mapped obstacle is never un-mapped.
- **The rover and the aircraft**, each with a trail behind it.
- **The swept lanes**, as runs rather than one continuous line, so a gap where the suite was off stays a gap.
- **The planned route**, ending where the rover's knowledge ends. It plans only over ground it has seen, out to a 22 m
  horizon; where it has not seen the way, it drives to the best place to look from, scans, and decides again.
- **The tether ring** at 50 m, which the aircraft slots are clamped to.
- **Contacts and threats** (next section).
- **Live heat blobs**, when the thermal layer is on: people, animals and vehicles the thermal cameras can see right
  now, held for a couple of seconds and let go.
- **The mesh line**, centred under the toolbar: how many voxels each replica holds and how many have not crossed the
  link yet.

### Markers

| Shape | Meaning |
|---|---|
| **diamond** | confirmed by the operator: an exclusion zone |
| **quatrefoil** (four lobes) | an unconfirmed contact of a class being treated as a threat |
| **square, grey-blue** | a contact of a class that is switched off: drawn, logged, shared and avoided, but quietly |
| **dashed ring with R or A** | only one replica holds this observation; it has not crossed the link |

Inside each marker is a glyph for its class: a mine, an IED, a person, a figure with a slung rifle for an armed person,
a vehicle, a deer, scattered dots for disturbed soil, a tick for something inert, a question mark for unclassified.

Marker labels on the map are one word. The full name is on the card and in the log.

---

## 7. Infrared

Press **MAP** until it reads **THERMAL**. A row of thermal controls appears under the toolbar.

The first time, the console spends about twelve seconds building the day's heat model and says so. It is kept
afterwards in `~/.cache/ares-ocu`, so every later run loads it in a fraction of a second.

### What the thermal map is

It is a **mosaic of what the thermal cameras have actually seen**, exactly as the LiDAR cloud is a mosaic of what the
scanner has seen. Ground no camera has covered is dark. A cell keeps the temperature it had when it was last imaged
and fades as it ages, because the ground changes and the picture does not until it is looked at again.

People, animals and vehicles are never written into the mosaic. A walker would leave a streak of false heat along their
whole path. They are drawn as live blobs for the moment they are in a frame.

The rover's camera sits 1.5 m off the ground, so it maps ground usefully only out to about 23 m; beyond that one pixel
covers more than two metres along the line of sight and is not written. An aircraft looking down maps a wide strip at
once, which is why the mosaic grows quickly once the scouts are up.

### The thermal controls

| Caption | Values | What it changes |
|---|---|---|
| **COLOUR** | IRON, WHITE-HOT, BLACK-HOT | A thermal camera has no colour of its own. IRON is the usual black-red-yellow-white scale; the other two are plain grey, hot as white or hot as black. |
| **BRIGHTNESS** | AUTO, LOCKED | Which two temperatures are drawn darkest and brightest. AUTO follows the ground that has been imaged, so the picture always uses the whole scale. LOCKED keeps the present two, so a hot thing entering the view does not darken everything else. Both are always printed beside the colour bar. |
| **TIME OF DAY** | `‹ 10:41 ›` | Local solar time at the site, in one-hour steps. |

The colour bar down the right of the map is the scale, in degrees Celsius, with the sensor's band and the share of the
ground that has been imaged printed under it.

### Why the time of day matters more than anything else

Thermal imagery is not the visible picture recoloured. What a surface looks like in infrared depends on its
temperature, and its temperature is the result of a day's heating and cooling. The console models that: a
heat-conduction column under every half-metre cell, driven by the sun at its real position for 34 degrees north on 20
October, the clear sky, the wind and the air temperature, with the shadows of rocks and trees.

So at **10:41** the ground is warm where the sun has reached it and cool in shadow; at **13:00** it is hot and the rocks
are cooler than the soil around them; at **02:00** the ground is colder than the air and the rocks glow, because rock
stores heat and dry soil does not.

Changing the time of day does **not** rewrite ground already imaged. Those cells keep the temperatures they had when
they were imaged, which is what a real mosaic does.

### What it will not do

- The visible camera feeds keep their fixed late-morning light whatever time you set for infrared. At night the visible
  picture is wrong, and nothing about it is claimed.
- The sky is always clear. There is no rain, snow, fog or gusting wind.
- The parked truck's engine is cold. It has been off for hours.
- A buried object shows as a faint patch that changes sign twice a day, and its size is a model's figure, not a
  measurement.
- **None of it is validated against a real site.** It is checked against analytic solutions and against known
  behaviour; the material properties are published typical values. See `DECISIONS.md` for the eleven checks, the two
  predictions that turned out wrong, and the one change made after a check failed.

---

## 8. Deciding what counts as a threat

The strip at the top of the right-hand column, **TREAT AS A THREAT**, has a chip for each class.

| Chip | Found by |
|---|---|
| **MINES / IEDs** | ground-penetrating radar, pulse induction, magnetic gradiometer and vapour, fused |
| **ARMS** | a weapon model run on the crop around a person the camera has already found |
| **PEOPLE** | the camera detector |
| **VEHICLES** | the camera detector |
| **ANIMALS** | the camera detector |
| **OTHER** | anything a sensor reports that none of the above classifies |

**Lit** means that class raises alerts and is listed as a threat.

**Unlit** does **not** mean ignored. A class that is off becomes a **contact**:

- it raises no alert and demands no decision, and it is not in the threat list;
- it is still drawn on the map, small and grey-blue, with a quiet label;
- it is still boxed on the camera feed, thin and muted;
- it is still written to the log, at the quietest level, as a contact;
- it is still **avoided**: the rover keeps clear of a vehicle or an animal exactly as before;
- it is still **shared** with the fleet over the mesh.

What the operator chooses to be told about and what the rover drives into are separate questions, and so are what the
operator sees and what the fleet records.

A count of what is being held back sits under the chips: `CONTACTS 2 animals, 1 vehicle`.

**Defaults** are MINES / IEDs, ARMS and PEOPLE. That is a clearance and border role, and it is a choice, not a
measurement. Turn VEHICLES on at a checkpoint; turn PEOPLE off where a work party is expected.

**Switching MINES / IEDs off takes a two-second hold**, the same deliberate second confirmation any irreversible command
takes. Silencing the most dangerous class in the product by a stray click is the one mistake this strip could make.
Switching it back on is a single click, and nothing else on the strip is guarded.

**Changing a chip re-draws everything at once.** Markers become contacts or threats, cards leave or enter the list, and
the log records the new choice. A contact that becomes a threat is listed as PENDING; no alert is raised for it
retrospectively, only for detections that arrive afterwards. A threat the operator has already confirmed stays in the
list whatever the chips say, because a decision that has been made stands.

### Arms, and what it is honestly worth

The weapon model does not run on the whole frame. At 20 m a rifle is a few hundredths of a percent of a rover frame,
far smaller than anything the model was measured on, so a number read off the end of that curve would be invented.

It runs as a **second stage on a person**, which is how a real system deploys it: the person detector finds somebody,
the frame is cropped to their box and enlarged, and the weapon model runs on the crop. In the crop a carried rifle is a
large share of the picture, which is the size regime that was measured.

What that gives, from the measurements:

- **a rifle carried in the open** fills about 8.6 percent of a person's crop, and is found in roughly two frames in
  three while the person is within about 27 m;
- **a pistol** fills about 1.6 percent, which is below anything that was measured, so **the console does not claim to
  find one**;
- **somebody carrying nothing** is called armed about once in forty crops, the rate measured on photographs of people
  holding a bat, a racket, a phone, a camera, a tripod or an instrument;
- a weapon call needs **two hits in the last five frames** of a confirmed person track before it is reported, for the
  same reason a person needs three.

An armed person is **one contact**, reported as ARMED PERSON, not a person and a weapon side by side.

**The caveat travels with it.** Those recalls come from web photographs. A crop from a rover camera at 20 m, enlarged,
is blurrier than a photograph, so they are an upper bound and not a field figure. Nothing was measured at night, in
thermal, from the air, or on a weapon that is not being carried in the open, and the thermal camera has no weapon stage
at all.

---

## 9. The threat list

One card per contact, keyed by voxel rather than by event: four sensors hitting the same buried object is one threat
with four contributing sensors and a rising confidence, not four rows to reconcile by eye. It is the same merge the
shared map performs, shown where the operator can act on it.

Each card carries the classification and its glyph, the map coordinate, which sensors contributed, a confidence bar
with the 0.85 escalation line marked on it, and a status.

| Status | Meaning |
|---|---|
| **PENDING** | nobody has decided yet. CONFIRM and CLEAR are offered |
| **CONFIRMED** | the operator called it a threat. The map marks it as an exclusion zone |
| **CLEARED** | the operator called it inert |

Click a card to centre the map on it.

---

## 10. The alert

When a contact of a class being treated as a threat passes **0.85** confidence, the banner opens over the map. It
carries the classification, the confidence against the threshold, the coordinate, which unit saw it, and a countdown.

**CONFIRM THREAT** marks it an exclusion zone. **REJECT** clears it as inert. If the countdown runs out the log records
that it timed out unresolved, and nothing is decided on the operator's behalf.

A contact whose own classification reads inert still escalates, but the banner says CONTACT, READS INERT rather than
THREAT ALERT, because calling it a threat would argue with the evidence underneath it.

---

## 11. The rover panel

With the rover selected the right-hand panel shows speed (against the published 15 km/h), heading, power draw,
endurance at the current draw, and the four ground sensors with their scanning state and a flash when one contributes
to a hit.

**GO TO.** Press GO TO on the map toolbar and click anywhere. The rover plans only over ground it has seen. Where it
has not seen the way, it drives to the best place to look from, scans, and decides again, so the line on the map ends
where its knowledge ends rather than running optimistically to the destination. CLEAR DESTINATION withdraws it.

**RETURN TO BASE** is a hold-to-confirm button. It recalls any aircraft still out; they travel home with the rover and
dock once it is stationary, because the funnel pad and the fiducial are both fixed to the deck.

**TAKE MANUAL CONTROL** hands the rover to the keyboard: **W** and **S** to drive, **A** and **D** to turn. Manual
driving is still guarded: the rover will refuse a step that closes on a mapped obstacle or on somebody walking, and it
can always reverse out. **RELEASE MANUAL** gives it back to autonomy.

---

## 12. Flying the scouts

Press **LAUNCH** on the map toolbar, or click a filled slot in the rover's bay. The formation console opens.

| Control | What it sets |
|---|---|
| **Aircraft** | which of the three to launch |
| **Off nose** | the bearing of the formation relative to the rover's nose. The dial on the map follows it, and the preview redraws as it moves |
| **Pattern** | VEE, LINE ABREAST, COLUMN, and so on |
| **Spacing** | between aircraft |
| **Standoff** | how far ahead of the rover the formation sits |
| **Rally altitude** | what they climb to |

**STAGE** works the slots out and draws the preview. **LAUNCH** commits. Aircraft lift off one at a time about 1.5
seconds apart, climb, transit and take station; the minimum separation of 3 m is enforced in code and not only in the
interface. The stage the console reports is read from the aircraft themselves, not from a timer.

Slots are clamped to the 50 m tether ring. Formation geometry is computed live from what the operator sets.

---

## 13. The link

| Mode | What it carries |
|---|---|
| **STANDARD** | full RF: live video, telemetry and real-time override |
| **AUTONOMOUS** | minimal RF, a status ping every five seconds. The mission continues; the feeds say they have no video |
| **GHOST** | zero intentional emissions. No video, no telemetry, no downlink. The vehicles go on working |
| **LCA** | entered automatically on a loss of communications. Each unit begins its own emergency return after fifteen seconds |

**CUT LINK** is the demonstration the whole shared-map design exists for. While it is cut, the rover and the aircraft
keep finding things and writing them into their own replicas. The banner over the map shows the two counts drifting
apart, and any observation only one side holds gets a dashed ring and an R or A badge.

**RESTORE LINK** merges them. The counts converge, the badges clear, and the log names how many observations crossed.
Nobody re-sent the whole map: the merge is by voxel and the result does not depend on the order the updates arrive in.

While the link is down, detections are found and logged on the vehicle and **not reported to the console**. They appear
when the fleet is allowed to talk again.

---

## 14. The event log

Along the bottom. Every line is stamped with mission time and coloured by weight: grey for routine, amber for something
that needs attention, red for a threat, green for something resolved.

Repeated events about the same subject collapse rather than scrolling the useful lines away.

To the right of it, the **LINK** strip shows the last sixty seconds for each unit, so a dropout is visible as a gap
rather than as a line of text that has already scrolled past.

---

## 15. Keyboard and mouse

| Input | What it does |
|---|---|
| **W / S** | drive forward and back, while holding manual control |
| **A / D** | turn, while holding manual control |
| **R / F** | climb and descend, while holding manual control of an aircraft |
| **F5**, or **Ctrl+R** / **Cmd+R** | reset the mission |
| **Scroll** or **pinch** | zoom the map |
| **Drag** | pan the map |
| **Click a unit** | select it; the right-hand panel follows |
| **Click a threat card** | centre the map on it |
| **Drag the compass rose** | swing the formation bearing |

---

## 16. Where every number comes from

| What | Status |
|---|---|
| Rover and aircraft positions | **Simulated.** The rover plans and follows its own route; the aircraft are flown by the flight director under the baseline's speed and acceleration limits |
| The rover's terrain map | **Real.** Built only from simulated LiDAR returns, with occlusion. Unscanned ground stays unknown |
| Negative obstacles | **Real.** A hole stops the wheels and not the beam, so it is found by the drive stereo pair within 8 m, and once mapped it is not unlearned |
| Buried objects and what finds them | **Real models.** Physical objects with depth, metal mass, ferrous state and vapour; four sensor response models built from the baseline's reach figures. What is found depends on where the rover drives |
| Battery and endurance | **Real.** Integrated from the power budget: 430 Wh at 143 W gives 3.0 h, an aircraft gets 11 min of cruise, a 30 W pad charges it in 9 min |
| The shared-map merge | **Real.** The same set of updates in any order on any replica gives a byte-identical map |
| Formation geometry | **Real**, computed live from what the operator sets |
| Launch and recovery | **Real.** A launch is flown and its stage read from the aircraft; recovery is the four-layer sequence |
| Camera feeds | **Rendered** from the simulated world, with occlusion, shadows and haze. Not recorded footage |
| Camera detections | **Simulated from measured curves.** No neural network runs in this application. The curves come from a real model's detections on data it never trained on, and they are unflattering on purpose |
| The weapon stage | **Simulated from measured curves**, from a model measured on web photographs. An upper bound, not a field figure |
| Infrared | **Simulated physics** with published material values, checked against analytic solutions. Not validated against a real site |
| Terrain | **Generated** from a fixed seed. A plausible piece of ground, not a real place |
| Guns on the thermal camera | **Not modelled at all** |
| Mission theme | A generic hazard sweep. Never labelled a minefield |

The detector's measurements are kept unflattering deliberately. A person seen from an aircraft at the usual distances
was **not found at all** in the data that was measured, small things are mostly missed, and the detector invents a
person or an animal about once in seventy frames, which the persistence gate normally catches.

---

## 17. A sixty second demonstration

1. **Say the first line out loud.** It is a simulation and no hardware exists. Say it before anybody asks.
2. **DEPLOY.** The rover scans, picks its axis and starts sweeping. When it cannot see a third of the ground ahead it
   says so and recommends a bearing for the scouts.
3. **Launch them.** The dial is already on that bearing. Pick a pattern, STAGE, LAUNCH.
4. **Watch a threat build.** Four sensors hit one buried object. They merge into one card with the sensors named and a
   confidence climbing past 0.85.
5. **Switch the map to THERMAL.** The ground the cameras have covered appears as a heat picture. Step the time of day
   back to 02:00: the ground goes cold and the rocks glow, because rock holds the day's heat and dry soil does not.
6. **Cut the link while the fleet is working.** This is the moment. The two replicas drift apart by count, and any
   observation only one side holds gets a dashed ring and a badge.
7. **Restore it.** The counts converge, the badges clear, the log names how many crossed. Nobody re-sent the whole map.
8. **Show the threat strip.** Turn ANIMALS on and off and watch the deer move between the threat list and a quiet
   contact, while the rover goes on driving round it either way.

The honest line to close on: almost every autonomy pitch claims to work in a GPS-denied, link-contested environment,
and almost none of them show the failure on purpose and then show the recovery.

---

## 18. When something looks wrong

| What you see | What it is |
|---|---|
| The map is mostly dark | The LiDAR cloud only holds ground that has actually been swept. Drive, or launch the scouts |
| The thermal layer says it is building | The day's heat model is being solved, once, in about twelve seconds. It is kept afterwards |
| The thermal map is nearly empty | The thermal cameras only run while infrared is wanted, and the rover maps only about 23 m either side of its track. Launch the scouts and it fills quickly |
| A tile says NO VIDEO | The link mode is AUTONOMOUS or GHOST. The camera is fine |
| A contact appeared and vanished | The detector invents something about once in seventy frames. The persistence gate drops it before it is reported. That is the gate working |
| A person at a distance is not found | The measured curves say so. Small things are mostly missed and this console does not flatter them |
| The rover stopped in front of something | It refuses to close on a mapped obstacle or on somebody walking. It gives up a waypoint it cannot reach after eight seconds and goes on |
| The endurance figure looks optimistic at speed | The power model is calibrated to 15 km/h. Above that the speed figure is amber and the endurance is not a prediction |
| Nothing happens on the map after DEPLOY | The rover scans first, then plans its lanes. Give it a few seconds |

---

Anything this guide states as measured is recorded in `DECISIONS.md` with the run that produced it, including the
predictions that failed. If the two ever disagree, `DECISIONS.md` is the record and this guide is the mistake.
