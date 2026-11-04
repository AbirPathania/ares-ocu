"""Recover where the camera was, frame by frame, and a sparse cloud of the scene (structure from motion).

    reference_footage/recon/venv/bin/python scripts/recon_sfm.py IMG_5129

Runs COLMAP (through pycolmap) on the frames that scripts/recon_frames.py chose: SIFT features, matches between
frames that are close together in the walk (the clip is a path, so only neighbours need matching), and an incremental
reconstruction. The output is a COLMAP model (cameras, images, points) in reference_footage/recon/<clip>/sparse/0, which is
what the splat trainer reads. Run it with the reconstruction environment, not the project's.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import pycolmap

RECON = Path(__file__).resolve().parent.parent / "reference_footage" / "recon"


def main(clip: str) -> None:
    work = RECON / clip
    images, database, sparse = work / "images", work / "database.db", work / "sparse"
    sparse.mkdir(exist_ok=True)
    if database.exists():
        database.unlink()
    t0 = time.time()
    print("extracting features", flush=True)
    pycolmap.extract_features(database, images, camera_mode=pycolmap.CameraMode.SINGLE)
    print(f"  {time.time() - t0:.0f} s; matching neighbours", flush=True)
    pycolmap.match_sequential(database)
    print(f"  {time.time() - t0:.0f} s; reconstructing", flush=True)
    models = pycolmap.incremental_mapping(database, images, sparse)
    print(f"  {time.time() - t0:.0f} s; {len(models)} model(s)", flush=True)
    best = max(models.values(), key=lambda m: m.num_reg_images())
    # Brush reads the classic layout (cameras, images, points3D as text); pycolmap's own binary layout is newer and differs.
    dataset = work / "dataset"
    (dataset / "sparse" / "0").mkdir(parents=True, exist_ok=True)
    best.write_text(dataset / "sparse" / "0")
    if not (dataset / "images").exists():
        (dataset / "images").symlink_to("../images")
    for index, model in models.items():
        print(f"model {index}: {model.num_reg_images()} of {len(list(images.glob('*.jpg')))} images registered, {model.num_points3D()} points, "
              f"mean reprojection error {model.compute_mean_reprojection_error():.2f} px", flush=True)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "IMG_5129")
