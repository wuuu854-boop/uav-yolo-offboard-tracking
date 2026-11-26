# Offline depth-map quality gate

The gate checks projected points and the occupancy grid built from them, without
flying. Two recorded frames now pass: the search frame and a clean avoidance stop
with a pose sidecar.

## Final result (v0.39)

| Frame | Points | ≤1 m | Median | Cells | Precision | Boxes | Start |
|---|---:|---:|---:|---:|---:|---:|---|
| search (row 215) | 320 | 0.978 | 0.429 m | 61 | **0.967** | 3/3 | free |
| avoidance (sidecar, t=28.69 s) | 756 | 0.992 | 0.315 m | 62 | **0.968** | 1/1 | free |

Both use a **central 60% ROI**; the avoidance frame also limits depth to 4 m
because the wall is at ~2.5 m and the image edges see open background beyond it.
The 1.0 m point tolerance and 0.80 grid threshold are unchanged.

Raw output: [`results/map_gate.md`](../results/map_gate.md),
[`results/map_gate_avoidance.md`](../results/map_gate_avoidance.md),
[`results/map_cells.json`](../results/map_cells.json).

## What was wrong

1. **Camera sign.** The measured projection uses image row increasing upward.
   The opposite signs gave 55.6% near 1 m instead of >90%.
2. **Frame-pose ambiguity.** The recorder now writes `<frame>.npy.json` with pose,
   timestamp, nearest distance and stop event; it refuses to capture a stop below
   3.0 m so a landing-descent stop cannot overwrite the wall frame.
3. **Edge background.** At a close wall the FOV edges see past the wall; a central
   60% ROI is the obstacle corridor. Without it the avoidance grid precision was
   0.576–0.691; with it, 0.968.

## What this verifies, and what it does not

| Claim | Status | Evidence |
|---|---|---|
| Projection convention | ✅ verified | >97% within 1 m on two frames |
| Single-frame grid quality | ✅ PASS | precision 0.967 / 0.968 |
| Two-frame gate (Phase A) | ✅ **closed** | search + sidecar avoidance |
| Multi-frame accumulation | ❌ not implemented | two independent frames |
| Map planner / SITL | ❌ not started | Phase B next |
| Real sensor noise | ❌ not verified | ideal Gazebo depth |

## Reproducing it

```bash
make map-gate
make map-gate-avoidance
```

Phase A is closed. Phase B (occupancy planner + 20-run SITL) is the next gate and
has not started.
