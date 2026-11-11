#!/usr/bin/env bash
# Ten repeats of the retained gate-target1 (margin) mission.
set -u
cd "$(dirname "$0")/.." || exit 1
RUNS="${1:-10}"
mkdir -p results/repeat
for index in $(seq 1 "$RUNS"); do
  prefix="repeat_gate-target1-margin_${index}"
  echo "== gate-target1-margin ${index}/${RUNS} =="
  timeout 200s env MISSION=config/mission_gate_target1.yaml \
    WORLD=target_tracking_gate_target1 PREFIX="$prefix" MODE=global \
    OBSTACLE=config/gate_target1_obstacle.yaml TARGET_ID=1 \
    bash scripts/run_avoidance.sh > "log/${prefix}.log" 2>&1
  rc=$?
  cp "results/${prefix}_summary.json" "results/repeat/gate-target1-margin_${index}_summary.json" 2>/dev/null || true
  cp "results/${prefix}_landing.csv" "results/repeat/gate-target1-margin_${index}_landing.csv" 2>/dev/null || true
  cp "log/${prefix}.log" "results/repeat/gate-target1-margin_${index}.log" 2>/dev/null || true
  echo "rc=$rc"
done
python3 scripts/repeat_report.py --dir results/repeat --out results/repeat_report
