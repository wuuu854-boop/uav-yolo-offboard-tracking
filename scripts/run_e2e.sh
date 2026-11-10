#!/usr/bin/env bash
# Run one closed-loop trial against an already running simulation, or check the
# environment without one.
#
# Prerequisites (terminal A):  make sim
# This script only starts the ROS 2 nodes and waits for one landing row to be
# appended to results/landing_error.csv.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# shellcheck disable=SC1091
if [[ -f .env ]]; then source .env; fi
# shellcheck disable=SC1091
if [[ -f .venv/bin/activate ]]; then source .venv/bin/activate; fi

MODE="run"
DURATION_S=180
RESULTS="results/landing_error.csv"

usage() {
  cat <<'EOF'
Usage: run_e2e.sh [--dry-run] [--duration SECONDS]

  --dry-run    check configuration, imports and writable directories, print the
               commands that would run, then exit. Needs no simulator, so run it
               before make sim.
  --duration   seconds to wait for one landing row (default 180)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run|--preflight) MODE="dry"; shift ;;
    --duration)
      if [[ $# -lt 2 ]]; then echo "--duration needs a value" >&2; exit 2; fi
      DURATION_S="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

# Everything that can be wrong before a simulation is running. Each failure is
# printed with the command that fixes it, because the alternative is debugging
# it mid-flight.
preflight() {
  local failed=0
  local file

  echo "== configuration =="
  for file in config/detector.yaml config/camera.yaml config/mission.yaml config/targets.yaml; do
    if [[ -f "$file" ]]; then
      echo "  ok   $file"
    else
      echo "  MISS $file"
      failed=1
    fi
  done

  echo "== python =="
  if python3 -c "import rclpy" >/dev/null 2>&1; then
    echo "  ok   rclpy (a ROS environment is sourced)"
  else
    echo "  MISS rclpy: source /opt/ros/humble/setup.bash"
    failed=1
  fi
  if PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -c "import uav_tracking.control.offboard_node" >/dev/null 2>&1; then
    echo "  ok   uav_tracking imports"
  else
    echo "  MISS uav_tracking: pip install -e . (or add src/ to PYTHONPATH)"
    failed=1
  fi

  echo "== directories =="
  if mkdir -p results log && touch results/.write_test 2>/dev/null; then
    rm -f results/.write_test
    echo "  ok   results/ and log/ are writable"
  else
    echo "  MISS results/ or log/ is not writable"
    failed=1
  fi

  return "$failed"
}

if ! preflight; then
  echo "preflight failed: fix the MISS lines above before flying" >&2
  exit 1
fi

if [[ "$MODE" == "dry" ]]; then
  cat <<EOF
== dry run: nothing was started ==

would run:
  python3 src/uav_tracking/perception/detector_node.py --config config/detector.yaml
  python3 src/uav_tracking/control/offboard_node.py
and wait up to ${DURATION_S}s for a new row in ${RESULTS}.
EOF
  exit 0
fi

mkdir -p log
touch "$RESULTS"

require_topic() {
  # Discovery right after a daemon restart is not instant: retry for ~10 s
  # before declaring the simulation missing.
  local topic="$1" tries=0
  while (( tries < 5 )); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 2
    tries=$((tries + 1))
  done

  echo "ERROR: topic $topic not found after 10 s." >&2
  echo "  Is the simulation running?  make sim" >&2
  echo "  Visible topics:" >&2
  ros2 topic list 2>/dev/null | sed 's/^/    /' | head -20 >&2
  exit 1
}

echo "== checking topics =="
# A stale ROS daemon makes `ros2 topic list` print nothing even though the
# topics exist, which reads as "the simulation is not running". Restart it.
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
require_topic "/camera/image_raw"
require_topic "/fmu/out/vehicle_odometry"

ROWS_BEFORE=$(($(wc -l < "$RESULTS") - 1))
if ((ROWS_BEFORE < 0)); then ROWS_BEFORE=0; fi

echo "== starting nodes (logs in log/) =="
python3 src/uav_tracking/perception/detector_node.py --config config/detector.yaml \
  > log/detector.log 2>&1 &
DETECTOR_PID=$!
python3 src/uav_tracking/control/offboard_node.py > log/offboard.log 2>&1 &
OFFBOARD_PID=$!

cleanup() {
  kill -INT "$DETECTOR_PID" "$OFFBOARD_PID" 2>/dev/null || true
  wait "$DETECTOR_PID" "$OFFBOARD_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "== waiting for a landing row (timeout ${DURATION_S}s) =="
DEADLINE=$((SECONDS + DURATION_S))
while ((SECONDS < DEADLINE)); do
  ROWS_NOW=$(($(wc -l < "$RESULTS") - 1))
  if ((ROWS_NOW > ROWS_BEFORE)); then
    echo "== trial recorded =="
    tail -n 1 "$RESULTS"
    exit 0
  fi
  if ! kill -0 "$OFFBOARD_PID" 2>/dev/null; then
    echo "ERROR: offboard node exited. Last lines:" >&2
    tail -n 20 log/offboard.log >&2
    exit 1
  fi
  sleep 2
done

echo "ERROR: no landing row after ${DURATION_S}s. Last lines of log/offboard.log:" >&2
tail -n 20 log/offboard.log >&2
exit 1
