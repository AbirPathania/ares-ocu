"""Reconstruct every trail in the footage, unattended, keeping the CPU and the GPU both busy.

    .venv/bin/python scripts/recon_scheduler.py            (from the repository root; writes reference_footage/recon/scheduler.log)

Each trail is a job with four stages, each leaving a marker file in reference_footage/recon/<name>/ when it is done, so the
scheduler can be stopped and started again and carries on where it was:

    frames.ok   the sharpest frames of the clip's segment, 1600 px wide     (scripts/recon_frames.py, one at a time)
    sfm.ok      camera poses, a sparse cloud and the undistorted dataset      (scripts/recon_sfm.py, one at a time, CPU)
    trained.ok  the splat                                                     (Brush, one at a time, GPU)
    packed.ok   the splat and its route in assets/footage/trail               (scripts/trail_pack.py)

The CPU stages of the next trails run while the GPU trains the current one. Training is budgeted (`SPLATS`, `STEPS`) for
9 GB of memory shared with the GPU: a trail of about 40 s is a good size for it.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RECON = ROOT / "reference_footage" / "recon"
PY = ROOT / ".venv" / "bin" / "python"
SFM_PY = RECON / "venv" / "bin" / "python"
BRUSH = RECON / "tools" / "brush-app-aarch64-apple-darwin" / "brush_app"
LOG = RECON / "scheduler.log"

STEPS = 9000
SPLATS = 600_000
RESOLUTION = 1100


@dataclass(frozen=True)
class Job:
    name: str
    clip: str
    start_s: float = 0.0
    end_s: float | None = None
    steps: int = STEPS
    splats: int = SPLATS
    camera_model: str = "OPENCV"
    camera_params: str | None = None  # a pinned lens (see recon_sfm.py)
    keep_per_second: int = 5  # frames kept per second of the clip (recon_frames.py); 10 keeps every frame it decodes
    sfm_overlap: int | None = None  # frames each is matched against (recon_sfm.py's SFM_OVERLAP); None for the default


#: IMG_5108's camera (the 1x lens, 1600 px wide frames) as calibrated on segment B, where the geometry pins it down: f, cx, cy, k1, k2.
LENS_5108 = {"camera_model": "RADIAL", "camera_params": "1391.22,800,450,0.08,-0.13"}

#: Every trail in the footage that can be walked: IMG_5117 is someone standing at a viewpoint (no walk to reconstruct from).
JOBS = [
    Job("IMG_5129_hq", "IMG_5129", steps=14000, splats=800_000),
    Job("IMG_5132_hq", "IMG_5132"),
    Job("IMG_5108_A", "IMG_5108", 160, 200, **LENS_5108),
    Job("IMG_5108_B", "IMG_5108", 40, 80),
    Job("IMG_5114", "IMG_5114"),
    Job("IMG_5108_C", "IMG_5108", 0, 40, **LENS_5108),
    Job("IMG_5128", "IMG_5128"),
    Job("IMG_5109", "IMG_5109"),
    Job("IMG_5108_D", "IMG_5108", 80, 120, **LENS_5108),
    Job("IMG_5108_E", "IMG_5108", 120, 160, **LENS_5108),
    Job("IMG_5108_F", "IMG_5108", 200, 240, **LENS_5108),
    # The same stretches again, with twice the frames and wider matching: B, C and D had runs of cameras put metres off the path where the filmer swung round (D at 87 s: from a chain-link
    # fence to the trail in under two seconds).
    Job("IMG_5108_D2", "IMG_5108", 80, 120, keep_per_second=10, sfm_overlap=12, **LENS_5108),
    Job("IMG_5108_B2", "IMG_5108", 40, 80, keep_per_second=10, sfm_overlap=12, **LENS_5108),
    Job("IMG_5108_C2", "IMG_5108", 0, 40, keep_per_second=10, sfm_overlap=12, **LENS_5108),
]


def say(text: str) -> None:
    with open(LOG, "a") as handle:
        handle.write(f"{time.strftime('%H:%M:%S')} {text}\n")


def done(job: Job, stage: str) -> bool:
    return (RECON / job.name / f"{stage}.ok").exists()


def mark(job: Job, stage: str) -> None:
    (RECON / job.name).mkdir(parents=True, exist_ok=True)
    (RECON / job.name / f"{stage}.ok").touch()


def running(pattern: str) -> bool:
    return subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0


def final_splat(job: Job) -> Path:
    return RECON / job.name / "out" / f"splat_{job.steps}.ply"  # Brush pads the iteration to the width of the total, so a 9000-step run writes splat_9000.ply


def tidy(job: Job) -> None:
    """Give back the disk a finished stage no longer needs: nine trails would otherwise want more than there is."""
    import shutil

    folder = RECON / job.name
    if done(job, "sfm"):
        for name in ("database.db", "database.db-wal", "database.db-shm"):
            (folder / name).unlink(missing_ok=True)
        shutil.rmtree(folder / "sparse", ignore_errors=True)  # the undistorted dataset has its own copy of the model
        shutil.rmtree(folder / "images", ignore_errors=True)  # and its own, undistorted, pictures
    if done(job, "trained"):
        for path in (folder / "out").glob("splat_*.ply"):
            if path != final_splat(job):
                path.unlink(missing_ok=True)


def settle_markers() -> None:
    """Notice work that was finished before the scheduler existed (or by an earlier run)."""
    for job in JOBS:
        folder = RECON / job.name
        if not done(job, "frames") and len(list((folder / "images").glob("*.jpg"))) >= 150 and not running(f"recon_frames.py {job.clip} {job.name}"):
            mark(job, "frames")
        if not done(job, "sfm") and (folder / "dataset" / "sparse" / "0" / "cameras.txt").exists() and not running(f"recon_sfm.py {job.name}"):
            mark(job, "sfm")
        if not done(job, "trained") and final_splat(job).exists() and not running(f"brush_app {folder}"):
            mark(job, "trained")
        if not done(job, "packed") and (ROOT / "assets" / "footage" / "trail" / f"{job.name}.npz").exists():
            mark(job, "packed")


def main() -> None:
    say("scheduler started")
    children: dict[str, tuple[Job, subprocess.Popen]] = {}
    while True:
        settle_markers()
        for job in JOBS:
            tidy(job)
        # reap
        for key in list(children):
            job, proc = children[key]
            if proc.poll() is None:
                continue
            del children[key]
            stage = key.split(":")[0]
            ok = {"frames": len(list((RECON / job.name / "images").glob("*.jpg"))) >= 100,
                  "sfm": (RECON / job.name / "dataset" / "sparse" / "0" / "cameras.txt").exists(),
                  "train": final_splat(job).exists(), "pack": (ROOT / "assets" / "footage" / "trail" / f"{job.name}.npz").exists()}[stage]
            say(f"{stage} {job.name}: {'done' if ok else 'FAILED'} (exit {proc.returncode})")
            mark(job, {"train": "trained", "pack": "packed"}.get(stage, stage)) if ok else mark(job, f"{stage}.failed")
        busy = {k.split(":")[0] for k in children}
        # pack what has trained
        for job in JOBS:
            if done(job, "trained") and not done(job, "packed") and not done(job, "pack.failed") and "pack" not in busy:
                say(f"pack {job.name}")
                children[f"pack:{job.name}"] = (job, subprocess.Popen([str(PY), str(ROOT / "scripts" / "trail_pack.py"), job.name, f"out/splat_{job.steps}.ply"],
                                                                      stdout=open(RECON / job.name / "pack.log", "w"), stderr=subprocess.STDOUT,
                                                                      env={**os.environ, "PACK_SECONDS_PER_FRAME": str(1.0 / (10 if job.keep_per_second >= 10 else job.keep_per_second))}))
                busy.add("pack")
        # GPU: train the first reconstructed trail that has not been
        if "train" not in busy and not running("brush_app"):
            for job in JOBS:
                if done(job, "sfm") and not done(job, "trained") and not done(job, "train.failed"):
                    say(f"train {job.name} ({job.steps} steps, up to {job.splats:,} splats)")
                    (RECON / job.name / "out").mkdir(parents=True, exist_ok=True)
                    env = {**os.environ, "RUST_LOG": "info"}
                    children[f"train:{job.name}"] = (job, subprocess.Popen(
                        [str(BRUSH), str(RECON / job.name / "dataset"), "--total-steps", str(job.steps), "--max-splats", str(job.splats), "--sh-degree", "2",
                         "--max-resolution", str(RESOLUTION), "--export-every", "3000", "--export-path", str(RECON / job.name / "out"),
                         "--export-name", "splat_{iter}.ply"],
                        env=env, stdout=open(RECON / job.name / "train.log", "w"), stderr=subprocess.STDOUT))
                    busy.add("train")
                    break
        # CPU: one structure-from-motion at a time, then one frame extraction at a time
        if "sfm" not in busy and not running("recon_sfm.py"):
            for job in JOBS:
                if done(job, "frames") and not done(job, "sfm") and not done(job, "sfm.failed"):
                    say(f"sfm {job.name}")
                    children[f"sfm:{job.name}"] = (job, subprocess.Popen([str(SFM_PY), str(ROOT / "scripts" / "recon_sfm.py"), job.name, job.camera_model] + ([job.camera_params] if job.camera_params else []),
                                                                         stdout=open(RECON / f"{job.name}.sfm.log", "w"), stderr=subprocess.STDOUT,
                                                                         env={**os.environ, **({"SFM_OVERLAP": str(job.sfm_overlap)} if job.sfm_overlap else {})}))
                    busy.add("sfm")
                    break
        if "frames" not in busy and not running("recon_frames.py"):
            for job in JOBS:
                if not done(job, "frames") and not done(job, "frames.failed"):
                    say(f"frames {job.name}")
                    args = [str(PY), str(ROOT / "scripts" / "recon_frames.py"), job.clip, job.name, "1600", str(job.keep_per_second), str(job.start_s)]
                    if job.end_s is not None:
                        args.append(str(job.end_s))
                    children[f"frames:{job.name}"] = (job, subprocess.Popen(args, stdout=open(RECON / f"{job.name}.frames.log", "w"), stderr=subprocess.STDOUT))
                    busy.add("frames")
                    break
        if all(done(job, "packed") or any(done(job, f"{s}.failed") for s in ("frames", "sfm", "train", "pack")) for job in JOBS) and not children:
            say("all trails done")
            return
        time.sleep(30)


if __name__ == "__main__":
    sys.exit(main())
