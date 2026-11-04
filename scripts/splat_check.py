"""Draw a splat scene from the camera poses of the frames it was trained on and set each beside the real frame.

    .venv/bin/python scripts/splat_check.py IMG_5129 out/splat_20000.ply [every=25]

This is the honest test of a reconstruction: if it is right, the picture drawn from a training pose is the frame.
It writes reference_footage/recon/<clip>/check.jpg (frame on top, splats below) and prints the mean absolute colour error.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ocu.render import splats  # noqa: E402


def main() -> None:
    import moderngl

    clip = sys.argv[1]
    ply = ROOT / "reference_footage" / "recon" / clip / sys.argv[2]
    every = int(sys.argv[3]) if len(sys.argv) > 3 else 25
    folder = ROOT / "reference_footage" / "recon" / clip
    k, poses = splats.read_colmap_text(folder / "dataset" / "sparse" / "0")
    cloud = splats.load_ply(ply)
    print(f"{len(cloud):,} splats; camera {k.width}x{k.height} f={k.fx:.1f}")
    ctx = moderngl.create_standalone_context()
    renderer = splats.SplatRenderer(ctx, cloud)
    w = 640
    ks = k.scaled(w)
    fbo = ctx.simple_framebuffer((ks.width, ks.height), components=4)
    fbo.use()
    rows, errors = [], []
    for pose in poses[::every]:
        fbo.clear(0.0, 0.0, 0.0, 1.0)
        renderer.render(pose, ks)
        image = np.frombuffer(fbo.read(components=3), dtype=np.uint8).reshape(ks.height, ks.width, 3)[::-1]
        real = np.array(Image.open(folder / "images" / pose.name).resize((ks.width, ks.height)))
        errors.append(np.abs(image.astype(int) - real.astype(int)).mean())
        rows.append(np.vstack([real, image]))
    Image.fromarray(np.hstack(rows[:4])).save(folder / "check.jpg", quality=92)
    print("mean absolute error per pixel (0..255):", round(float(np.mean(errors)), 1), "over", len(errors), "views; splats drawn in the last view:", renderer._last_count)


if __name__ == "__main__":
    main()
