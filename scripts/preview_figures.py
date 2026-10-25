"""Render the people, vehicles and animals from a few distances and angles into one picture, to look at.

    .venv/bin/python scripts/preview_figures.py out.png
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ocu.core.terrain import TerrainWorld  # noqa: E402
from ocu.render import gpu_scene  # noqa: E402


def main(out: str) -> None:
    world = TerrainWorld()
    scene = gpu_scene.GpuScene.create(world)
    if scene is None:
        raise SystemExit("no GL context")
    time.monotonic = lambda: 50.0  # still wind
    # the flattest open ground within 40 m of the deploy point
    best = None
    for cx in range(-40, 41, 2):
        for cy in range(-40, 41, 2):
            if np.hypot(cx, cy) < 12:
                continue
            zs = [world.elevation_at(world.to_cell(cx + dx, cy + dy)) for dx in (-6, 0, 6) for dy in (-6, 0, 6)]
            ok = all(world.at(world.to_cell(cx + dx, cy + dy)).name in ("GROUND", "TRACK", "SCRUB") for dx in range(-8, 9, 2) for dy in range(-8, 9, 2))
            if ok and (best is None or max(zs) - min(zs) < best[0]):
                best = (max(zs) - min(zs), cx, cy)
    _, ox, oy = best
    print("figures stand at", ox, oy, "relief", round(best[0], 2), "m")
    gz = float(scene._ground(np.array([ox]), np.array([oy]))[0])
    tiles = []
    # (label, kind, camera distance, camera height, azimuth of the camera round the figure, phase)
    shots = [("truck 3/4 near", "vehicle", 11.0, 1.6, 35), ("truck side", "vehicle", 11.0, 1.4, 90), ("truck rear", "vehicle", 10.0, 1.8, 180),
             ("truck far", "vehicle", 32.0, 1.6, 40), ("person near", "person", 4.5, 1.5, 30), ("person side", "person", 4.5, 1.5, 90),
             ("person walking", "person", 5.0, 1.4, 70), ("person far", "person", 22.0, 1.6, 20), ("buck near", "animal", 5.0, 1.5, 40),
             ("buck head", "animal", 3.2, 1.5, 25), ("doe walking", "doe", 5.5, 1.4, 60), ("buck far", "animal", 22.0, 1.6, 20),
             ("ibex near", "ibex", 5.0, 1.4, 40), ("ibex side", "ibex", 5.5, 1.3, 90), ("ibex head", "ibex", 3.0, 1.5, 25), ("ibex walking", "ibex", 6.0, 1.4, 60),
             ("armed near", "armed", 4.5, 1.5, 35), ("armed side", "armed", 4.5, 1.5, 90), ("armed walking", "armed", 5.0, 1.4, 70), ("armed at 20 m", "armed", 20.0, 1.6, 55)]
    for i, (label, kind, dist, cam_h, az) in enumerate(shots):
        phase = 0.3 if "walking" in label else 0.0
        style = {"animal": "buck", "ibex": "ibex", "armed": "armed"}.get(kind, "")
        if kind in ("doe", "ibex"):
            kind = "animal"
        if kind == "armed":
            kind = "person"
        a = math.radians(az)
        cam = (ox + dist * math.sin(a + math.pi), oy + dist * math.cos(a + math.pi))
        eye = (cam[0], cam[1], float(scene._ground(np.array([cam[0]]), np.array([cam[1]]))[0]) + cam_h)
        heading = math.degrees(math.atan2(ox - cam[0], oy - cam[1]))
        with scene.ctx:
            frame = scene.render(eye=eye, heading_deg=heading, pitch_down_deg=2.0, hfov_deg=50.0, width=480, height=360, entities=[(kind, ox, oy, 0.0, phase, style)])
        tiles.append(frame)
    cols = 4
    rows = [np.concatenate(tiles[r * cols:(r + 1) * cols], axis=1) for r in range(len(tiles) // cols)]
    Image.fromarray(np.concatenate(rows, axis=0)).save(out)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/figures_preview.png")
