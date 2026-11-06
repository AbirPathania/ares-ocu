"""Package a trained splat and its route for the renderer.

    .venv/bin/python scripts/trail_pack.py IMG_5129 out/splat_12000.ply [metres_per_unit]

Copies the splat to assets/footage/trail/<clip>.ply and writes the route (poses, scale, up) beside it as <clip>.npz. The
folder is git-ignored: a splat is tens of megabytes and is made from footage that is not in the repository, so it is
rebuilt by the scripts in this directory (see DECISIONS.md, "Reconstructing the trail"). The renderer uses it when it is
there and draws the procedural world when it is not. A third argument replaces the estimated scale with a measured one.
"""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ocu.render import splats, trail  # noqa: E402


def main() -> None:
    clip, ply = sys.argv[1], sys.argv[2]
    folder = ROOT / "reference_footage" / "recon" / clip
    sparse = folder / "dataset" / "sparse" / "0"
    k, poses = splats.read_colmap_text(sparse)
    points, rgb = trail.read_points(sparse / "points3D.txt")
    route, report = trail.build_route(poses, points, rgb, seconds_per_frame=float(os.environ.get("PACK_SECONDS_PER_FRAME", 0.2)))
    if len(sys.argv) > 3:
        route.rescale_to(float(sys.argv[3]))
        print("scale replaced by", sys.argv[3], "m per unit")
    trail.TRAIL_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(folder / ply, trail.TRAIL_DIR / f"{clip}.ply")
    route.save(trail.TRAIL_DIR / f"{clip}.npz", k)
    print(report)
    print("wrote", trail.TRAIL_DIR / f"{clip}.ply", "and", trail.TRAIL_DIR / f"{clip}.npz")


if __name__ == "__main__":
    main()
