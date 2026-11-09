#!/usr/bin/env bash
# Run one closed-loop trial against an already running simulation.
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

DURATION_S=180
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION_S="$2"; shift 2 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

RESULTS="results/landing_error.csv"
mkdir -p log results
touch "$RESULTS"

require_topic() {
  if ! ros2 topic list 2>/dev/null | grep -qx "$1"; then
    echo "ERROR: topic $1 not found."
    echo "  Is the simulation running?  make sim"
    echo "  Visible topics:"; ros2 topic list 2>/dev/null | sed 's/^/    /' | head -20
    exit 1
  fi
}

echo "== checking topics =="
require_topic "/camera/image_raw"
require_topic "/fmu/out/vehicle_odometry"

ROWS_BEFORE=$(($(wc -l < "$RESULTS") - 1))
if ((ROWS_BEFORE < 0)); then ROWS_BEFORE=0; fi

echo "== starting nodes (logs in log/) =="
python3 src/uav_tracking/perception/detector_node.py --config config/detector.yaml > log/detector.log 2>&1 &
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
    echo "ERROR: offboard node exited. Last lines:"
    tail -n 20 log/offboard.log
    exit 1
  fi
  sleep 2
done

echo "ERROR: no landing row after ${DURATION_S}s. Last lines of log/offboard.log:"
tail -n 20 log/offboard.log
exit 1
