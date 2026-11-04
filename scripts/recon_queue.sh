#!/bin/sh
# Run the rest of the reconstruction work unattended, one GPU job at a time.
#
#   sh scripts/recon_queue.sh            (from the repository root; logs go to reference_footage/recon/queue.log)
#
# Order: frames and structure-from-motion for the two segments of IMG_5108 (CPU work, which can overlap a training), then
# training for IMG_5132 and each segment, each waiting until no other Brush is running (the GPU has room for one).
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
R="$ROOT/reference_footage/recon"
PY="$ROOT/.venv/bin/python"
SFM="$R/venv/bin/python"
BRUSH="$R/tools/brush-app-aarch64-apple-darwin/brush_app"
LOG="$R/queue.log"

say() { echo "$(date +%H:%M:%S) $*" >> "$LOG"; }
wait_free() { while pgrep -f brush_app > /dev/null; do sleep 30; done; }
wait_sfm()  { while pgrep -f recon_sfm > /dev/null; do sleep 30; done; }

train() {  # name, total steps, splat cap
    wait_free
    say "training $1 ($2 steps, cap $3)"
    mkdir -p "$R/$1/out"
    RUST_LOG=info "$BRUSH" "$R/$1/dataset" --total-steps "$2" --max-splats "$3" --sh-degree 2 --max-resolution 1100 \
        --export-every 2000 --export-path "$R/$1/out" --export-name "splat_{iter}.ply" > "$R/$1/train.log" 2>&1
    say "trained $1"
}

say "queue started"
"$PY" "$ROOT/scripts/recon_frames.py" IMG_5108 IMG_5108_A 1600 5 160 200 >> "$LOG" 2>&1
"$PY" "$ROOT/scripts/recon_frames.py" IMG_5108 IMG_5108_B 1600 5 40 80 >> "$LOG" 2>&1
wait_sfm
say "sfm IMG_5108_A"; "$SFM" "$ROOT/scripts/recon_sfm.py" IMG_5108_A OPENCV > "$R/IMG_5108_A.sfm.log" 2>&1
say "sfm IMG_5108_B"; "$SFM" "$ROOT/scripts/recon_sfm.py" IMG_5108_B OPENCV > "$R/IMG_5108_B.sfm.log" 2>&1
wait_free
train IMG_5132_hq 10000 650000
train IMG_5108_A 10000 650000
train IMG_5108_B 10000 650000
say "queue finished"
