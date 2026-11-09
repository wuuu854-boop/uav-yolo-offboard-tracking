#!/usr/bin/env bash
# Record a short rosbag for the demo and offline comparison.
# Bags are git-ignored; upload a trimmed copy as a Release asset instead.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
mkdir -p bags

DURATION=30
[[ $# -gt 0 ]] && DURATION="$1"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="bags/demo-${STAMP}"

echo "recording ${DURATION}s of /camera/image_raw and /detections -> ${OUT}"
timeout "${DURATION}" ros2 bag record -o "$OUT" /camera/image_raw /detections || true

echo "recorded. size: $(du -sh "$OUT" | cut -f1)"
cat <<EOF

To turn a screen recording into a README GIF (<= 8 MB, 1280x720, 15 fps):
  ffmpeg -i demo.mp4 -vf "fps=15,scale=1280:-1" -loop 0 assets/01-demo.gif
EOF
