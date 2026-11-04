"""Pick the frames of a clip that a 3D reconstruction should use.

    .venv/bin/python scripts/recon_frames.py IMG_5129 [fps_candidates=10] [keep_per_second=4]

A walk filmed on a phone is hundreds of near-identical frames, some blurred by a step. Reconstruction wants a few
sharp ones with about three-quarters overlap, so the clip is decoded at `fps_candidates`, tone-mapped from HLG, and the
sharpest frame in each window is kept. Frames land in reference_footage/recon/<clip>/images/, which is git-ignored.

Only clips that were held the right way up throughout are handled here (IMG_5108, 5114, 5117, 5129, 5132); the two
that were turned over part of the time need the per-frame orientation from footage_ingest first.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import footage_ingest as fi  # noqa: E402

RECON = fi.FOOTAGE / "recon"


def pick_frames(clip_name: str, fps_candidates: int = 10, keep_per_second: int = 4, width: int = 1280) -> list[Path]:
    clip = fi.probe(fi.FOOTAGE / f"{clip_name}.MOV")
    height = int(round(width * clip.height / clip.width / 2)) * 2
    out_dir = RECON / clip_name / "images"
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-v", "error", "-hwaccel", "videotoolbox", "-i", str(clip.path), "-vf", f"fps={fps_candidates},scale={width}:{height}:flags=lanczos",
           "-pix_fmt", "rgb48le", "-f", "rawvideo", "-"]
    size = width * height * 6
    group = max(fps_candidates // keep_per_second, 1)
    kept: list[Path] = []
    best: tuple[float, np.ndarray] | None = None
    index = 0
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    assert proc.stdout is not None
    while True:
        raw = proc.stdout.read(size)
        if len(raw) < size:
            break
        signal = np.frombuffer(raw, dtype=np.uint16).reshape(height, width, 3).astype(np.float32) / 65535.0
        rgb = fi.linear_to_srgb8(fi.hlg_to_linear(signal))
        score = fi.sharpness(rgb)
        if best is None or score > best[0]:
            best = (score, rgb)
        index += 1
        if index % group == 0:
            path = out_dir / f"{clip_name}_{len(kept):04d}.jpg"
            Image.fromarray(best[1]).save(path, quality=95)
            kept.append(path)
            best = None
    proc.wait()
    return kept


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "IMG_5129"
    fps = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    keep = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    frames = pick_frames(name, fps, keep)
    print(f"{name}: kept {len(frames)} frames in {frames[0].parent}")
