"""Pick the frames of a clip that a 3D reconstruction should use.

    .venv/bin/python scripts/recon_frames.py IMG_5129 [name=clip] [width=1280] [keep_per_second=5] [start_s=0] [end_s=clip end]

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


def pick_frames(clip_name: str, name: str | None = None, width: int = 1280, keep_per_second: int = 5, start_s: float = 0.0,
                end_s: float | None = None, candidate_fps: int = 10) -> list[Path]:
    """Sharpest frame in each window of `candidate_fps / keep_per_second` candidates, of the clip's `start_s` to `end_s`."""
    clip = fi.probe(fi.FOOTAGE / f"{clip_name}.MOV")
    name = name or clip_name
    height = int(round(width * clip.height / clip.width / 2)) * 2
    out_dir = RECON / name / "images"
    out_dir.mkdir(parents=True, exist_ok=True)
    end_s = clip.duration_s if end_s is None else min(end_s, clip.duration_s)
    cmd = ["ffmpeg", "-v", "error", "-hwaccel", "videotoolbox", "-ss", f"{start_s:.2f}", "-t", f"{end_s - start_s:.2f}", "-i", str(clip.path),
           "-vf", f"fps={candidate_fps},scale={width}:{height}:flags=lanczos", "-pix_fmt", "rgb48le", "-f", "rawvideo", "-"]
    size = width * height * 6
    group = max(candidate_fps // keep_per_second, 1)
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
            path = out_dir / f"{name}_{len(kept):04d}.jpg"
            Image.fromarray(best[1]).save(path, quality=95)
            kept.append(path)
            best = None
    proc.wait()
    return kept


if __name__ == "__main__":
    args = sys.argv[1:]
    clip = args[0] if args else "IMG_5129"
    name = args[1] if len(args) > 1 and args[1] != "-" else None
    width = int(args[2]) if len(args) > 2 else 1280
    keep = int(args[3]) if len(args) > 3 else 5
    start = float(args[4]) if len(args) > 4 else 0.0
    end = float(args[5]) if len(args) > 5 else None
    frames = pick_frames(clip, name, width, keep, start, end)
    print(f"{name or clip}: kept {len(frames)} frames in {frames[0].parent}")
