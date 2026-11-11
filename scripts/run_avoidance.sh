#!/usr/bin/env bash
# One depth-camera avoidance flight, start to finish, with the evidence written
# to results/ and checked afterwards.
#
# Prerequisites: a patched PX4 build, `make install-world` (this script calls it
# through `make avoid`), ROS 2 Humble sourced by this script.
#
#   make avoid
#   # or: bash scripts/run_avoidance.sh --duration 120
#   # phase 2:
#   MISSION=config/mission_detour.yaml PREFIX=detour MODE=detour \
#     bash scripts/run_avoidance.sh
#
# The world and the wall are in sim/worlds/target_tracking_obstacle.sdf; the
# mission is config/mission_avoidance.yaml. The run is deliberately headless:
# the depth image is recorded to results/avoidance_depth_frame.png, and a GUI
# would only add load to the renderer that produces it.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 1

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
WORLD="${WORLD:-target_tracking_obstacle}"
POSITION="${POSITION:--7.5,0,0.05,0,0,1.5708}"
DURATION=120
# One script runs both behaviours: `hold` (phase 1) and `detour` (phase 2). The
# prefix keeps their CSVs, traces and frames separate so a later run cannot
# overwrite the evidence a release quotes.
MISSION="${MISSION:-config/mission_avoidance.yaml}"
PREFIX="${PREFIX:-avoidance}"
MODE="${MODE:-hold}"
RESULTS="results/${PREFIX}_landing.csv"
RECORD="results/${PREFIX}_run.csv"
FRAME="results/${PREFIX}_depth_frame.png"
TRACE="results/${PREFIX}_trace.csv"
SUMMARY="results/${PREFIX}_summary.json"
PLOT="results/${PREFIX}_run.png"
DEPTH_CONFIG="config/depth.yaml"

usage() {
  cat <<'EOF'
Usage: run_avoidance.sh [--duration SECONDS] [--pose x,y,z]

  --duration  seconds to wait for the landing row (default 120)
  --pose      PX4_GZ_MODEL_POSE for the spawn (default -7.5,0,0.05,0,0,1.5708,
              i.e. facing east like the mission's heading)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="${2:?}"; shift 2 ;;
    --pose) POSITION="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

command -v gz >/dev/null || { echo "gz not found; run scripts/sitl_setup.sh" >&2; exit 1; }
command -v MicroXRCEAgent >/dev/null || { echo "MicroXRCEAgent not found" >&2; exit 1; }
[[ -x "$PX4_BUILD/bin/px4" ]] || { echo "PX4 not built at $PX4_BUILD" >&2; exit 1; }
[[ -f "$HOME/.simulation-gazebo/worlds/$WORLD.sdf" ]] || {
  echo "world $WORLD not installed; make install-world" >&2
  exit 1
}

mkdir -p log results
# Each avoidance run is one piece of evidence, not a trial series: stale rows
# from a previous attempt would make the trace and the landing row ambiguous.
rm -f "$RESULTS" "$TRACE"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/ros_ws/install/setup.bash" 2>/dev/null || true
set -u
export PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
touch "$RESULTS"

DEPTH_BRIDGE_PID=""
DEPTH_NODE_PID=""
RECORDER_PID=""
OFFBOARD_PID=""

stop_nodes() {
  local pid
  for pid in "$OFFBOARD_PID" "$RECORDER_PID" "$DEPTH_NODE_PID" "$DEPTH_BRIDGE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$OFFBOARD_PID" "$RECORDER_PID" "$DEPTH_NODE_PID" "$DEPTH_BRIDGE_PID"; do
    if [[ -n "$pid" ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}

stop_sim() {
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'bin/px4' 2>/dev/null || true
}

cleanup() {
  stop_nodes
  stop_sim
  pkill -9 -f MicroXRCEAgent 2>/dev/null || true
}
trap cleanup EXIT

echo "== starting simulation =="
stop_sim
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
sleep 3

setsid nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent_avoid.log 2>&1 < /dev/null &
sleep 2

(
  cd "$HOME/.simulation-gazebo" || exit 1
  GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$HOME/.simulation-gazebo/worlds" \
    setsid nohup gz sim -s -r --headless-rendering "worlds/$WORLD.sdf" \
    > /tmp/gz_avoid.log 2>&1 < /dev/null &
)
sleep 10

(
  cd "$PX4_BUILD" || exit 1
  setsid nohup env PX4_SIM_MODEL=gz_x500_mono_cam_down PX4_GZ_WORLD="$WORLD" \
    PX4_GZ_MODEL_POSE="$POSITION" ./bin/px4 -d -s etc/init.d-posix/rcS \
    > /tmp/px4_avoid.log 2>&1 < /dev/null &
)
sleep 32

if ! gz topic -l 2>/dev/null | grep -qx "/depth_camera"; then
  echo "ERROR: the vehicle depth camera did not appear. gz topics:" >&2
  gz topic -l 2>/dev/null | grep -E 'depth|model/x500' >&2 || true
  echo "PX4 log tail:" >&2
  tail -20 /tmp/px4_avoid.log >&2
  exit 1
fi

CONTACT_TOPIC="$(gz topic -l 2>/dev/null | grep -E '/obstacle_wall/.*/contact/contact$' | head -1)"
echo "contact topic: ${CONTACT_TOPIC:-<none; contact count will stay 0>}"

echo "== starting nodes =="
python3 scripts/gz_depth_bridge.py --gz-topic /depth_camera --ros-topic /camera/depth/image_raw \
  > log/depth_bridge.log 2>&1 &
DEPTH_BRIDGE_PID=$!
sleep 4

if ! timeout 10 ros2 topic list 2>/dev/null | grep -qx "/camera/depth/image_raw"; then
  echo "ERROR: /camera/depth/image_raw was not bridged. log/depth_bridge.log:" >&2
  tail -20 log/depth_bridge.log >&2
  exit 1
fi

python3 src/uav_tracking/perception/depth_node.py --config "$DEPTH_CONFIG" \
  > log/depth_node.log 2>&1 &
DEPTH_NODE_PID=$!
python3 scripts/record_avoidance.py --out "$RECORD" --frame-out "$FRAME" \
  --contact-topic "$CONTACT_TOPIC" \
  > log/avoidance_recorder.log 2>&1 &
RECORDER_PID=$!
sleep 3

python3 src/uav_tracking/control/offboard_node.py --mission "$MISSION" \
  --results "$RESULTS" --trace "$TRACE" \
  > log/offboard_avoidance.log 2>&1 &
OFFBOARD_PID=$!
sleep 2

if ! kill -0 "$OFFBOARD_PID" 2>/dev/null; then
  echo "ERROR: offboard node exited at startup. log/offboard_avoidance.log:" >&2
  tail -30 log/offboard_avoidance.log >&2
  exit 1
fi

ROWS_BEFORE=$(($(wc -l < "$RESULTS") - 1))
if ((ROWS_BEFORE < 0)); then ROWS_BEFORE=0; fi

echo "== waiting for the landing row (timeout ${DURATION}s) =="
DEADLINE=$((SECONDS + DURATION))
RECORDED=0
while ((SECONDS < DEADLINE)); do
  ROWS_NOW=$(($(wc -l < "$RESULTS") - 1))
  if ((ROWS_NOW > ROWS_BEFORE)); then
    RECORDED=1
    break
  fi
  if ! kill -0 "$OFFBOARD_PID" 2>/dev/null; then
    echo "ERROR: offboard node exited. Last lines:" >&2
    tail -30 log/offboard_avoidance.log >&2
    exit 1
  fi
  sleep 2
done

if ((RECORDED == 0)); then
  echo "ERROR: no landing row after ${DURATION}s. Last lines:" >&2
  tail -30 log/offboard_avoidance.log >&2
  exit 1
fi

echo "== landed; recording the descent for a few more seconds =="
sleep 8
stop_nodes
OFFBOARD_PID=""
RECORDER_PID=""
DEPTH_NODE_PID=""

echo "== stop trace =="
grep -E 'avoidance|detour|STOP|CLEAR|trial ' log/offboard_avoidance.log log/depth_node.log | tail -20 || true

cleanup
trap - EXIT

echo "== report =="
python3 scripts/avoidance_report.py --mode "$MODE" --csv "$RECORD" \
  --summary "$SUMMARY" --plot "$PLOT" --trace "$TRACE" \
  --obstacle config/obstacle.yaml --mission "$MISSION"
REPORT_RC=$?

echo "== landing row (this run only) =="
tail -n 2 "$RESULTS"

exit "$REPORT_RC"
