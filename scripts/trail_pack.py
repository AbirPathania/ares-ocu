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

import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ocu.render import splats, trail  # noqa: E402


def render_sharpness(ply: Path, route, k, width: int = 480, every: int = 3, laterals=(-0.6, 0.0, 0.6)) -> np.ndarray:
    """How sharp the splats draw the route's own view at each of its cameras: the least, over the camera and a little to either side of it, of the variance of the picture's Laplacian.
    Computed every `every` cameras and interpolated between. Returns None if there is no GL to draw with."""
    try:
        import moderngl

        ctx = moderngl.create_standalone_context()
    except Exception:
        return None
    from scipy import ndimage

    ks = k.scaled(width)
    renderer = splats.SplatRenderer(ctx, splats.load_ply(ply))
    fbo = ctx.simple_framebuffer((ks.width, ks.height), components=4)
    fbo.use()
    n = len(route.arc_m)
    indices = list(range(0, n, every)) + ([n - 1] if (n - 1) % every else [])
    values = []
    for i in indices:
        worst = np.inf
        for lateral in laterals:
            fbo.clear(0.0, 0.0, 0.0, 1.0)
            renderer.render(route.pose_at(float(route.arc_m[i]), lateral_m=lateral), ks)
            image = np.frombuffer(fbo.read(components=3), dtype=np.uint8).reshape(ks.height, ks.width, 3)[::-1].astype(np.float32)
            worst = min(worst, float(ndimage.laplace(image @ np.array([0.299, 0.587, 0.114], np.float32)).var()))
        values.append(worst)
    renderer.release()
    return np.interp(np.arange(n), indices, values)


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
    route.sharpness = render_sharpness(folder / ply, route, k)
    trail.TRAIL_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(folder / ply, trail.TRAIL_DIR / f"{clip}.ply")
    route.save(trail.TRAIL_DIR / f"{clip}.npz", k)
    print(report)
    print("wrote", trail.TRAIL_DIR / f"{clip}.ply", "and", trail.TRAIL_DIR / f"{clip}.npz")


if __name__ == "__main__":
    main()
