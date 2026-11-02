# Recording the demonstration

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
