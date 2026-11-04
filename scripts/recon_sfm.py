"""Recover where the camera was, frame by frame, and a sparse cloud of the scene (structure from motion).

    reference_footage/recon/venv/bin/python scripts/recon_sfm.py <name> [camera_model=OPENCV]

Runs COLMAP (through pycolmap) on the frames that scripts/recon_frames.py chose: SIFT features, matches between
frames that are close together in the walk (the clip is a path, so only neighbours need matching), and an incremental
reconstruction. The output is a COLMAP model (cameras, images, points) in reference_footage/recon/<clip>/sparse/0, which is
what the splat trainer reads. Run it with the reconstruction environment, not the project's.
"""

from __future__ import annotations

import shutil
import sys
import time
from pathlib import Path

import pycolmap

RECON = Path(__file__).resolve().parent.parent / "reference_footage" / "recon"


def main(name: str, camera_model: str = "OPENCV") -> None:
    work = RECON / name
    images, database, sparse = work / "images", work / "database.db", work / "sparse"
    sparse.mkdir(exist_ok=True)
    if database.exists():
        database.unlink()
    t0 = time.time()
    print("extracting features", flush=True)
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = camera_model
    pycolmap.extract_features(database, images, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader)
    print(f"  {time.time() - t0:.0f} s; matching neighbours", flush=True)
    pycolmap.match_sequential(database)
    print(f"  {time.time() - t0:.0f} s; reconstructing", flush=True)
    models = pycolmap.incremental_mapping(database, images, sparse)
    print(f"  {time.time() - t0:.0f} s; {len(models)} model(s)", flush=True)
    for index, model in models.items():
        print(f"model {index}: {model.num_reg_images()} of {len(list(images.glob('*.jpg')))} images registered, {model.num_points3D()} points, "
              f"mean reprojection error {model.compute_mean_reprojection_error():.2f} px", flush=True)
    best_index = max(models, key=lambda i: models[i].num_reg_images())
    # Train on undistorted pictures with a plain pinhole camera: a lens model the trainer does not know is a lens error it cannot remove.
    undistorted = work / "dataset"
    if undistorted.exists():
        shutil.rmtree(undistorted)
    pycolmap.undistort_images(undistorted, sparse / str(best_index), images)
    cleaned = pycolmap.Reconstruction(undistorted / "sparse")
    (undistorted / "sparse" / "0").mkdir(exist_ok=True)
    cleaned.write_text(undistorted / "sparse" / "0")
    shutil.rmtree(undistorted / "stereo", ignore_errors=True)
    print(f"  {time.time() - t0:.0f} s; undistorted dataset in {undistorted}", flush=True)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "IMG_5129", sys.argv[2] if len(sys.argv) > 2 else "OPENCV")
