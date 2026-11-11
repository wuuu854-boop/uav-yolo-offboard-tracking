#!/usr/bin/env bash
# Repeat the three recent SITL scenarios and aggregate the evidence.
#
#   bash scripts/repeat_scenarios.sh [runs]
#
# Each run gets its own prefix (repeat_<scenario>_<n>) so the per-run outputs are
# preserved under results/repeat/; the simulator is restarted for every run.

set -u
cd "$(dirname "$0")/.." || exit 1
RUNS="${1:-3}"
OUT="results/repeat"
mkdir -p "$OUT"
make install-world >/dev/null || exit 1

run_one() {
  local name="$1" mission="$2" world="$3" obstacle="$4" target_id="$5" index="$6"
  local prefix="repeat_${name}_${index}"
  echo "== ${name} ${index}/${RUNS} (${prefix}) =="
  MISSION="$mission" WORLD="$world" PREFIX="$prefix" MODE=global \
    OBSTACLE="$obstacle" TARGET_ID="$target_id" \
    bash scripts/run_avoidance.sh > "log/${prefix}.log" 2>&1
  local rc=$?
  cp "results/${prefix}_summary.json" "$OUT/${name}_${index}_summary.json" 2>/dev/null || true
  cp "results/${prefix}_landing.csv" "$OUT/${name}_${index}_landing.csv" 2>/dev/null || true
  cp "log/${prefix}.log" "$OUT/${name}_${index}.log" 2>/dev/null || true
  echo "rc=$rc"
}

for index in $(seq 1 "$RUNS"); do
  run_one gate config/mission_gate.yaml target_tracking_gate config/gate_obstacle.yaml 0 "$index"
done
for index in $(seq 1 "$RUNS"); do
  run_one gate-target1 config/mission_gate_target1.yaml target_tracking_gate_target1 \
    config/gate_target1_obstacle.yaml 1 "$index"
done
for index in $(seq 1 "$RUNS"); do
  run_one gate-double config/mission_gate_double.yaml target_tracking_gate_double \
    config/gate_double_obstacle.yaml 1 "$index"
done

python3 scripts/repeat_report.py --dir "$OUT" --out results/repeat_report
