"""Remove the splats that are not the scene: floaters near where the camera goes, and splats too big to be anything.

    .venv/bin/python scripts/splat_clean.py IMG_5129_hq out/splat_12000.ply out/splat_clean.ply

Training explains a frame with whatever splats make it match, and some of them end up as a soft blob hanging in the air just
in front of the camera that the frames passed through. They are harmless from the poses it was trained at and a smear from
a pose a little to one side or lower, which a rover's camera is. Two things are removed: splats within `NEAR_M` of the camera's path
(at the height the phone had, and at the height of a rover's mast), and splats with an axis longer than `TOO_BIG_M`, which are
sky and haze modelled as something solid. Both distances are in metres, converted by the route's scale.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from ocu.render import splats, trail  # noqa: E402

NEAR_M = 0.30
TOO_BIG_M = 1.2
MAST_HEIGHT_M = 0.6


def read_raw(path: Path) -> tuple[list[str], np.ndarray, bytes]:
    with open(path, "rb") as handle:
        header = []
        while True:
            line = handle.readline()
            header.append(line)
            if line.strip() == b"end_header":
                break
    names = [h.decode().split()[-1] for h in header if h.startswith(b"property")]
    count = int([h.decode().split()[2] for h in header if h.startswith(b"element vertex")][0])
    data = np.fromfile(path, dtype=np.dtype([(n, "<f4") for n in names]), count=count, offset=sum(len(h) for h in header))
    return names, data, b"".join(header)


def main() -> None:
    name, source, target = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    folder = ROOT / "reference_footage" / "recon" / name
    sparse = folder / "dataset" / "sparse" / "0"
    _, poses = splats.read_colmap_text(sparse)
    points, rgb = trail.read_points(sparse / "points3D.txt")
    route, _ = trail.build_route(poses, points, rgb, seconds_per_frame=0.2)
    names, data, header = read_raw(folder / source)
    position = np.stack([data["x"], data["y"], data["z"]], axis=1)
    # the camera's two paths: where the phone was, and where a mast camera would be (lowered along `up`)
    centres = np.array([p.centre for p in poses])
    lowered = centres - route.up * (route.ground_height.mean() - MAST_HEIGHT_M / route.scale)
    near = np.zeros(len(data), dtype=bool)
    radius = NEAR_M / route.scale
    for path in (centres, lowered):
        for c in path:
            near |= np.linalg.norm(position - c, axis=1) < radius
    largest = np.exp(np.maximum.reduce([data["scale_0"], data["scale_1"], data["scale_2"]])) * route.scale
    big = largest > TOO_BIG_M
    keep = ~near & ~big
    print(f"{len(data):,} splats: {int(near.sum()):,} near the camera path, {int(big.sum()):,} too big, {int(keep.sum()):,} kept")
    kept = data[keep]
    header = header.replace(f"element vertex {len(data)}".encode(), f"element vertex {len(kept)}".encode())
    with open(folder / target, "wb") as handle:
        handle.write(header)
        kept.tofile(handle)
    print("wrote", folder / target)


if __name__ == "__main__":
    main()
