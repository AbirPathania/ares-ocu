# Demonstrating ARES

## Live, in front of people (the two and a half minutes that show everything)

**Launch before you are on.** `.venv/bin/python -m ocu.main --trail`. It is about 17 seconds to the first picture in the tiles and
about 25 to the thermal field; the tiles say so while they wait. Launch it, let it settle, then go up. Do not launch on stage.

Measured on an M1, with all four tiles, three scouts, both detectors and the thermal system running:

| | |
|---|---|
| Rover tile | about 10 frames a second (the photographic one; 800 000 splats a frame) |
| Scout thumbnails | about 2 a second, staggered (one downlink, three aircraft and the vehicle) |
| Mission clock | about 0.73x real time while everything is drawing: 01:49 on screen is 2.5 minutes in the room |
| Memory | flat at about 1.8 GB after the first half-minute |

**The run.** Set the pace slider to about half (2.1 m/s; the slider is the rover's speed ceiling, not a time multiplier — at the
default 0.8 m/s the patrol takes six minutes and will not fit your slot). Press play, then LAUNCH for the formation.
What then happens, with the seconds measured on the clock in the room:

| At | What |
|---|---|
| 0:03 | first animal (thermal) and first person (camera) raised on the threat panel |
| 0:07 | first buried object: MINE, from the gradiometer and the vapour sensor together |
| 1:09 | an armed person — the Arms class, camera only |
| 1:21 | a vehicle |
| 2:30 | SWEEP COMPLETE: the rover has driven all ten legs, 215 m |

Along the way: switch a tile to IR with its chip (the boxes there are the thermal detector's, not the camera's), switch the map
between LIDAR, IR and BOTH, and use GO TO to send the rover somewhere. TAKE MANUAL and the WASD keys drive it by hand; it will
refuse to drive into what it knows is there, and slides along it rather than stopping dead.

**If something goes wrong.** RESET (the button, or F5, or ctrl-R) puts the fleet, the map and the mission back to the start in under
a second without rebuilding the world, and the camera worker survives it. Every control has been exercised three laps in a running
mission without raising. If a tile goes blank it is waiting for a frame, not broken.

## What to say, and what not to claim

The rover's tile is a photographic reconstruction of real trails, from phone footage: ten legs chained into one 215 m patrol
(3D Gaussian splats, `DECISIONS.md`). It is **softer than the video it came from** — about a third of its sharpness, measured —
and it streaks in thick vegetation. Say that before someone else notices it.

Three things are not what a quick look suggests, and all three are the sort of thing a good investor asks about:

- **The scouts' views and the thermal view are rendered, not photographic.** The scouts fly too high for the splats to hold up
  (they are trained from a phone at 1.2 m). The ground textures in them are from the footage; the rest is built.
- **The people, animals and vehicles are models, not real figures.** They are graded and given contact shadows so they sit in the
  photograph, and they are still obviously computer graphics beside it.
- **The camera "AI" is a model of a trained detector, not a network running on these pixels**: real geometry for what is in frame,
  recall curves measured from one real model's detections on images it never trained on for whether it finds it, and a
  confirm-over-several-frames gate. The application says this in its own event log the first time it reports anything. What is real
  about it is the camera: the detector looks through the lens of the footage that drew the picture, and can only find a figure the
  photograph actually shows — stand one behind a real bush and it does not.

The buried-object sensors are physics against the object's own properties, not a script: drive a different route and the suite
finds different things. Driven straight over all 54 objects in the world, it finds all 54, and calls scrap scrap.

## Recording the demonstration

Start: `.venv/bin/python -m ocu.main`. The first launch with a new scenario takes a few minutes to solve the day's
thermal field in the background (cached afterwards in `~/.cache/ares-ocu`); wait until the infrared tiles stop showing
the building message before recording the thermal part.

The mission starts at 07:30 local solar time, when people and animals are warmer than the ground.

## A run that shows everything (about four minutes of mission time)

1. Press play, set the speed slider to about 45, launch the formation. The rover sweeps lane by lane and the scouts
   fly out ahead of it.
2. Around 1:30 of mission time the first person is raised (camera, then thermal on the same threat card). Confirm or
   reject it from the alert banner.
3. Switch a tile to IR with its IR chip: boxes drawn there are the thermal detector's. Switch the map layer to
   THERMAL for the mosaic. Colour and time of day are on the bar above it.
4. Buried objects appear from about 0:30 along the sweep block. The farther sites (35 to 90 m out) fill in as the
   scouts and rover reach them; use GO TO on the map to send the rover to one.

## Things to avoid

- Do not step the thermal time of day to midday on camera: bodies are then colder than sunlit ground and are rightly
  not found.
- The thermal detector reports people and animals only; vehicles come from the visible camera.

## The real trail

`.venv/bin/python -m ocu.main --trail` builds the world on the real trails reconstructed from the footage (assets/footage/trail; see
DECISIONS.md, "Reconstructing the trail from the footage"): the rover drives a carved path along them and its camera tile shows the real
place, with the people and animals standing on it. `--trail NAME` (or `a,b,c`) chooses which. Without the flag the world is the
simulated one, and the TRAIL chip on the rover's tile (next to IR) shows a replay of the first trail in it.

The splats are not in the repository (tens of megabytes each, made from footage that is not either): `scripts/recon_scheduler.py` rebuilds
them from reference_footage/, in about an hour a trail on an M1.
