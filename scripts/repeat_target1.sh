#!/usr/bin/env bash
# Repeat the retained gate-target1 mission (1.0 m / 0.05 m, 100 s budget).
set -u
cd "$(dirname "$0")/.." || exit 1
RUNS="${1:-10}"
mkdir -p results/repeat
for index in $(seq 1 "$RUNS"); do
  prefix="repeat_gate-target1-time_${index}"
  echo "== gate-target1-time ${index}/${RUNS} =="
  timeout 220s env MISSION=config/mission_gate_target1.yaml \
    WORLD=target_tracking_gate_target1 PREFIX="$prefix" MODE=global \
    OBSTACLE=config/gate_target1_obstacle.yaml TARGET_ID=1 \
    bash scripts/run_avoidance.sh > "log/${prefix}.log" 2>&1
  rc=$?
  for suffix in summary.json landing.csv trace.csv run.csv; do
    cp "results/${prefix}_${suffix}" "results/repeat/gate-target1-time_${index}_${suffix}" 2>/dev/null || true
  done
  cp "log/${prefix}.log" "results/repeat/gate-target1-time_${index}.log" 2>/dev/null || true
  echo "rc=$rc"
done
python3 scripts/repeat_report.py --dir results/repeat --out results/repeat_report
