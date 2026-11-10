#!/usr/bin/env bash
# Apply the PX4 v1.15 magnetometer patch that SITL needs, then rebuild.
#
# PX4 v1.15's gz bridge subscribes to the IMU, the barometer and GPS but has no
# magnetometer callback, so EKF2 never gets a compass and the vehicle refuses to
# arm. The patch in patches/ adds it plus the airframe that spawns the nadir
# camera model.
#
#   bash scripts/apply_px4_patch.sh [PX4_DIR]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${1:-${PX4_DIR:-$HOME/PX4-Autopilot}}"
PATCH="$REPO_ROOT/patches/px4-v1.15-gz-magnetometer.patch"

if [[ ! -d "$PX4_DIR/.git" ]]; then
  echo "ERROR: no PX4 checkout at $PX4_DIR" >&2
  echo "  bash scripts/sitl_setup.sh     # clones v1.15.0" >&2
  exit 1
fi

tag="$(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo unknown)"
if [[ "$tag" != v1.15* ]]; then
  echo "ERROR: this patch targets PX4 v1.15.x, found '$tag'" >&2
  echo "  the gz bridge changes between releases; port it deliberately" >&2
  exit 1
fi

if git -C "$PX4_DIR" apply --check --reverse "$PATCH" >/dev/null 2>&1; then
  echo "patch already applied to $PX4_DIR ($tag)"
else
  echo "applying $PATCH to $PX4_DIR ($tag)"
  git -C "$PX4_DIR" apply --3way "$PATCH"
fi

echo "rebuilding px4_sitl_default (incremental)"
make -C "$PX4_DIR" px4_sitl_default -j"$(nproc)" 2>&1 | tail -n 3
echo "done: $(stat -c %s "$PX4_DIR/build/px4_sitl_default/bin/px4" 2>/dev/null || echo '?') bytes"
