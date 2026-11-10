#!/usr/bin/env bash
# Run N closed-loop trials, restarting the simulation between them.
#
# The nine trials behind the README were run by hand nine times. This is that
# procedure as a script: same pose, same world, one row in results/ per trial.
#
# Prerequisites: scripts/sitl_setup.sh has run, the PX4 patch is applied
# (make patch-px4), the world is installed (make install-world).
#
#   bash scripts/run_trials.sh --count 3 --pose -3.5,0,0.05 --spawn -3.5,0
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WORLD="${WORLD:-target_tracking}"
MODEL="${MODEL:-gz_x500_mono_cam_down}"
POSE="${POSE:--3.5,0,0.05}"
SPAWN="${SPAWN:--3.5,0}"
COUNT=1
DURATION=180
HEADLESS=1

usage() {
  cat <<'EOF'
Usage: run_trials.sh [--count N] [--pose x,y,z] [--spawn east,north] [--duration S]

  --count     how many trials to run (default 1)
  --pose      PX4_GZ_MODEL_POSE for the spawn (default -3.5,0,0.05)
  --spawn     world-ENU spawn point, passed to the node as UAV_SPAWN_ENU
  --duration  seconds to wait for a landing row per trial (default 120)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count) COUNT="${2:?}"; shift 2 ;;
    --pose) POSE="${2:?}"; shift 2 ;;
    --spawn) SPAWN="${2:?}"; shift 2 ;;
    --duration) DURATION="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
CAMERA_TOPIC="/world/$WORLD/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"

command -v gz >/dev/null || { echo "gz not found; run scripts/sitl_setup.sh" >&2; exit 1; }
[[ -x "$PX4_BUILD/bin/px4" ]] || { echo "PX4 not built at $PX4_BUILD" >&2; exit 1; }
[[ -d "$HOME/.simulation-gazebo/worlds" ]] || { echo "world not installed; make install-world" >&2; exit 1; }

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/ros_ws/install/setup.bash" 2>/dev/null || true
set -u

stop_sim() {
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'bin/px4' 2>/dev/null || true
  pkill -9 -f gz_camera_bridge 2>/dev/null || true
  pkill -9 -f capture_pose_frames 2>/dev/null || true
}

pkill -9 -f MicroXRCEAgent 2>/dev/null || true
setsid nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 < /dev/null &
sleep 2

for i in $(seq 1 "$COUNT"); do
  echo "===== trial $i/$COUNT  pose=$POSE  spawn=$SPAWN ====="
  stop_sim
  sleep 3

  (
    cd "$HOME/.simulation-gazebo" || exit 1
    GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$HOME/.simulation-gazebo/worlds" \
      setsid nohup gz sim -s -r --headless-rendering "worlds/$WORLD.sdf" \
      > "/tmp/gz_t$i.log" 2>&1 < /dev/null &
  )
  sleep 8

  (
    cd "$PX4_BUILD" || exit 1
    setsid nohup env PX4_SIM_MODEL="$MODEL" PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_POSE="$POSE" \
      ./bin/px4 -d -s etc/init.d-posix/rcS > "/tmp/px4_t$i.log" 2>&1 < /dev/null &
  )
  sleep 30

  (
    cd "$REPO_ROOT" || exit 1
    setsid nohup python3 scripts/gz_camera_bridge.py \
      --gz-topic "$CAMERA_TOPIC" --ros-topic /camera/image_raw \
      > "/tmp/bridge_t$i.log" 2>&1 < /dev/null &
  )
  sleep 6

  if [[ -n "${CAPTURE_DIR:-}" ]]; then
    (
      cd "$REPO_ROOT" || exit 1
      setsid nohup python3 scripts/capture_pose_frames.py --out "$CAPTURE_DIR" \
        > "/tmp/capture_t$i.log" 2>&1 < /dev/null &
    )
    sleep 3
  fi

  ( cd "$REPO_ROOT" && UAV_SPAWN_ENU="$SPAWN" timeout $((DURATION + 30)) \
      bash scripts/run_e2e.sh --duration "$DURATION" ) || echo "trial $i: no landing row"
  tail -n 1 "$REPO_ROOT/results/landing_error.csv"
done

stop_sim
echo "===== summary ====="
tail -n "$COUNT" "$REPO_ROOT/results/landing_error.csv"
