#!/usr/bin/env bash
# Verify that model weights and sample data are present.
# Nothing large is stored in git, so this script tells you exactly what to fetch
# and where to put it. It never downloads silently.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

MODEL="models/yolo11n.onnx"

echo "== checking model weights =="
if [[ -f "$MODEL" ]]; then
  echo "found $MODEL ($(du -h "$MODEL" | cut -f1))"
else
  cat <<EOF
missing: $MODEL

Two ways to get it:
  a) export it locally (needs the training deps, ~1 min):
       pip install -r requirements-train.txt
       python3 scripts/export_onnx.py
  b) download the release asset:
       https://github.com/wuuu854-boop/uav-yolo-offboard-tracking/releases
       put the file at $MODEL
EOF
fi

echo "== verifying checksums (if models/md5sums.txt exists) =="
if [[ -f models/md5sums.txt ]]; then
  (cd models && md5sum -c md5sums.txt)
else
  echo "no models/md5sums.txt - skipping (create one with: cd models && md5sum *.onnx > md5sums.txt)"
fi

echo "== sample bag =="
if compgen -G "*.bag" >/dev/null || compgen -G "*.db3" >/dev/null; then
  echo "sample bag present"
else
  echo "no sample bag in the repo root (bags are git-ignored; record one with scripts/record_demo.sh)"
fi
