"""Fly a camera along the reconstructed trail at walking pace and write a video.

    .venv/bin/python scripts/splat_flythrough.py IMG_5129 out/splat_20000.ply flythrough.mp4 [seconds=0] [fps=30] [height_m=0]

`seconds` of 0 means the whole route. `height_m` of 0 keeps the height the phone had; a number puts the camera that many
metres above the ground under it (a rover's mast, say). The camera is the one the clip was filmed on, so the field of view
is the phone's.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ocu.render import splats, trail  # noqa: E402


def main() -> None:
    import moderngl

    clip, ply, out = sys.argv[1], sys.argv[2], sys.argv[3]
    seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    fps = int(sys.argv[5]) if len(sys.argv) > 5 else 30
    height = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    folder = ROOT / "reference_footage" / "recon" / clip
    sparse = folder / "dataset" / "sparse" / "0"
    k, poses = splats.read_colmap_text(sparse)
    points, rgb = trail.read_points(sparse / "points3D.txt")
    route, report = trail.build_route(poses, points, rgb, seconds_per_frame=0.2)
    print({key: (round(v, 2) if isinstance(v, float) else v) for key, v in report.items()})
    cloud = splats.load_ply(folder / ply)
    print(f"{len(cloud):,} splats")
    ctx = moderngl.create_standalone_context()
    renderer = splats.SplatRenderer(ctx, cloud)
    ks = k.scaled(1280)
    fbo = ctx.simple_framebuffer((ks.width, ks.height), components=4)
    fbo.use()
    total = route.length_m / trail.WALK_PACE_MS
    if seconds:
        total = min(total, seconds)
    frames = int(total * fps)
    cmd = ["ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{ks.width}x{ks.height}", "-r", str(fps), "-i", "-",
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "17", "-preset", "medium", out]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    for i in range(frames):
        s = i / fps * trail.WALK_PACE_MS
        pose = route.pose_at(s, height_m=height or None)
        fbo.clear(0.0, 0.0, 0.0, 1.0)
        renderer.render(pose, ks)
        proc.stdin.write(np.frombuffer(fbo.read(components=3), dtype=np.uint8).reshape(ks.height, ks.width, 3)[::-1].tobytes())
        if i % 60 == 0:
            print(f"frame {i}/{frames}", flush=True)
    proc.stdin.close()
    proc.wait()
    print("wrote", out)


if __name__ == "__main__":
    main()
