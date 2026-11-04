"""Read the reference footage: tone-map it, turn it the right way up, and catalogue what is sharp enough to use.

    .venv/bin/python scripts/footage_ingest.py catalogue   # one frame a second from every clip -> reference_footage/derived/

The clips are iPhone video in HLG (BT.2100) at 10 bits, some of it filmed with the phone held upside down. Neither
shows in a plain decode: the HLG looks flat and grey, and the rotation flag is only set on some clips. This module
does what a player does for the first, and decides for itself, from where the sky is, what the second is.

Nothing here runs at simulation time. The raw video stays out of the repository (it is several gigabytes); what is
derived from it (textures, colour measurements) is small and is what the renderer reads. See DECISIONS.md,
"Reference footage".
"""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

ROOT = Path(__file__).resolve().parent.parent
FOOTAGE = ROOT / "reference_footage"
DERIVED = FOOTAGE / "derived"

# --- colour ----------------------------------------------------------------------------------------------------------

_A, _B, _C = 0.17883277, 0.28466892, 0.55991073  # BT.2100 HLG constants
_BT2020_TO_BT709 = np.array([[1.6605, -0.5876, -0.0728], [-0.1246, 1.1329, -0.0083], [-0.0182, -0.1006, 1.1187]], dtype=np.float32)


def hlg_to_linear(signal: np.ndarray) -> np.ndarray:
    """Scene-linear BT.709 light from HLG-encoded BT.2020 (float 0..1, any shape ending in 3)."""
    e = np.clip(signal, 0.0, 1.0)
    scene = np.where(e <= 0.5, e * e / 3.0, (np.exp((e - _C) / _A) + _B) / 12.0).astype(np.float32)
    return np.clip(scene @ _BT2020_TO_BT709.T, 0.0, None)


def linear_to_srgb8(linear: np.ndarray) -> np.ndarray:
    """A gentle shoulder, so that sunlit sand does not clip, then the sRGB curve."""
    shouldered = linear / (1.0 + 0.25 * linear) * 1.25
    srgb = np.where(shouldered <= 0.0031308, 12.92 * shouldered, 1.055 * np.power(np.clip(shouldered, 1e-9, None), 1 / 2.4) - 0.055)
    return (np.clip(srgb, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


# --- reading a clip --------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Clip:
    name: str
    path: Path
    width: int  # as displayed, after the container's own rotation
    height: int
    duration_s: float
    latitude: float | None
    longitude: float | None
    altitude_m: float | None


def probe(path: Path) -> Clip:
    info = json.loads(subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
        "stream=width,height:stream_side_data=rotation:format=duration:format_tags=com.apple.quicktime.location.ISO6709",
        "-of", "json", str(path)]))
    stream = info["streams"][0]
    width, height = int(stream["width"]), int(stream["height"])
    rotation = abs(int(round(stream.get("side_data_list", [{}])[0].get("rotation", 0)))) if stream.get("side_data_list") else 0
    if rotation in (90, 270):
        width, height = height, width
    lat = lon = alt = None
    iso = info.get("format", {}).get("tags", {}).get("com.apple.quicktime.location.ISO6709")
    if iso:
        # +30.8728+076.7454+424.426/
        parts = iso.rstrip("/").replace("-", "+-").split("+")[1:]
        if len(parts) >= 2:
            lat, lon = float(parts[0]), float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 else None
    return Clip(path.stem, path, width, height, float(info["format"]["duration"]), lat, lon, alt)


def read_frame(clip: Clip, t: float, width: int | None = None, crop: tuple[float, float, float, float] | None = None) -> np.ndarray:
    """One frame at time t as sRGB uint8, as the container says it is turned (not yet corrected for a held-upside-down phone).

    `crop` is (x0, y0, x1, y1) in fractions of the displayed frame, taken before scaling, so a texture can be cut out at
    the clip's full resolution."""
    w, h = clip.width, clip.height
    x0, y0, x1, y1 = crop or (0.0, 0.0, 1.0, 1.0)
    cw, ch = int(round((x1 - x0) * w)) // 2 * 2, int(round((y1 - y0) * h)) // 2 * 2
    ox, oy = int(round(x0 * w)), int(round(y0 * h))
    out_w = width or cw
    out_h = int(round(out_w * ch / cw / 2)) * 2
    vf = f"crop={cw}:{ch}:{ox}:{oy},scale={out_w}:{out_h}:flags=lanczos"
    cmd = ["ffmpeg", "-v", "error", "-hwaccel", "videotoolbox", "-ss", f"{t:.3f}", "-i", str(clip.path), "-frames:v", "1",
           "-vf", vf, "-pix_fmt", "rgb48le", "-f", "rawvideo", "-"]
    raw = subprocess.check_output(cmd)
    signal = np.frombuffer(raw, dtype=np.uint16).reshape(out_h, out_w, 3).astype(np.float32) / 65535.0
    return linear_to_srgb8(hlg_to_linear(signal))


# --- which way up ----------------------------------------------------------------------------------------------------


def sky_score(rgb: np.ndarray) -> float:
    """How much of the top rows against the bottom rows is open sky: positive if the sky is at the top.

    Sky is bright and blue (blue above red, blue above green) where foliage is green-brown and sand is red-yellow, so
    the share of 'blue and bright' pixels in a band is a fair sky meter."""
    f = rgb.astype(np.float32)
    sky = (f[..., 2] > f[..., 0] + 12) & (f[..., 2] > f[..., 1] + 4) & (f[..., 2] > 110)
    band = max(rgb.shape[0] // 6, 1)
    return float(sky[:band].mean() - sky[-band:].mean())


def uprightness(scores: list[float], inverted_below: float = -0.20, upright_above: float = 0.10) -> list[bool]:
    """For each sample, whether the phone was the right way up.

    The phone is turned over for tens of seconds at a time, so this is a state that changes rarely, not a vote per frame.
    It goes upside down when two samples in a row have the sky clearly at the bottom, and comes back when one has it
    clearly at the top; a frame with no clear sky keeps whatever the last one decided. (A single weak reading is
    not enough either way: blue shade, a sign or a puddle of sky reads as sky in the wrong place.)"""
    out: list[bool] = []
    upright = True
    for i, score in enumerate(scores):
        nxt = scores[i + 1] if i + 1 < len(scores) else 0.0
        prev = scores[i - 1] if i else 0.0
        if upright and score < inverted_below and (nxt < inverted_below or prev < inverted_below):
            upright = False
        elif not upright and score > upright_above:
            upright = True
        out.append(upright)
    return out


def turn_upright(rgb: np.ndarray, upright: bool) -> np.ndarray:
    return rgb if upright else rgb[::-1, ::-1].copy()


# --- how sharp --------------------------------------------------------------------------------------------------------


def sharpness(rgb: np.ndarray) -> float:
    """Variance of the Laplacian of the luminance, per pixel at a fixed 640-wide scale: higher is crisper. A frame taken
    while walking, or into the sun, scores low."""
    grey = rgb.astype(np.float32) @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
    if grey.shape[1] != 640:
        grey = ndimage.zoom(grey, 640.0 / grey.shape[1], order=1)
    return float(ndimage.laplace(grey).var())


# --- the catalogue ----------------------------------------------------------------------------------------------------


def catalogue(step_s: float = 1.0, width: int = 960) -> list[dict]:
    DERIVED.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    for path in sorted(FOOTAGE.glob("IMG_*.MOV")):
        clip = probe(path)
        times = np.arange(0.5, clip.duration_s - 0.5, step_s)
        frames = [read_frame(clip, float(t), width) for t in times]
        up = uprightness([sky_score(f) for f in frames])
        print(f"{clip.name}: {len(frames)} frames, {sum(not u for u in up)} upside down", flush=True)
        folder = DERIVED / "frames" / clip.name
        folder.mkdir(parents=True, exist_ok=True)
        for t, frame, is_up in zip(times, frames, up):
            frame = turn_upright(frame, is_up)
            name = f"{clip.name}_{int(round(t * 1000)):06d}.jpg"
            Image.fromarray(frame).save(folder / name, quality=90)
            rows.append({"clip": clip.name, "t": float(t), "upright": bool(is_up), "sharpness": sharpness(frame), "file": f"frames/{clip.name}/{name}"})
    (DERIVED / "catalogue.json").write_text(json.dumps({"clips": [{**asdict(probe(p)), "path": str(p)} for p in sorted(FOOTAGE.glob("IMG_*.MOV"))], "frames": rows}, indent=1))
    return rows


def main(argv: list[str]) -> int:
    command = argv[1] if len(argv) > 1 else "catalogue"
    if command == "catalogue":
        catalogue()
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
