# results/

Raw experiment data. These files are the evidence behind every number in the
README and in the resume, so they are written by scripts, never by hand.

| File | Written by | Content |
|---|---|---|
| `landing_error.csv` | `src/uav_tracking/control/recorder.py` during `make e2e` | one row per trial: nearest target id, landing error, latency percentiles, success |
| `latency_raw.csv` | same recorder, same run | every latency sample, so the run's p95 is computed by pooling rather than by averaging per-trial percentiles |
| `state_trace.csv` | `src/uav_tracking/control/trace.py` during `make e2e` | one row per state change: when the mission armed, hovered, searched, landed, and why |
| `v02_validation.csv` | three runs with the v0.2.0 safety constraints | 0.94 / 1.01 / 1.49 m, all within tolerance; the nine-row table above stays the v0.1.0 measurement |
| `v03_yolo_trials.csv` | four runs with the YOLO backend and hard negatives | one within tolerance (0.52 m), three at 2.2–2.4 m; kept as the evidence for `docs/model.md` |
| `backend_comparison.txt` | both backends on the same 30 recorded frames and poses | mean difference (+0.49, +0.35) m — the per-frame projection is close; the trial error is closed-loop |
| `yolo_close_range.txt` | the same comparison on a YOLO flight, 30 frames | (+0.52, +0.39) m — the bias is stable across both backends' flights |
| `yolo_failure_trial2.txt`, `yolo_failure_trial3.txt` | replay of the two failing YOLO trials | both backends' projections swing ~1 m near the target while their difference stays ~0.6 m: the swing is the flat-nadir projection under pitch, not the detector |
| `v04_attitude.csv` | three trials per backend after the projection learned roll/pitch | baseline 0.86/1.00/0.66 m, YOLO 0.64/1.57/1.49 m — the 2.2 m mode is gone |
| `latency.csv` | `scripts/benchmark.py` (via `make bench`) | aggregate error/latency statistics for the README table |
| `projection_error.csv` | W2 static test | pixel-to-ground error against the three known target coordinates |
| `detection_bench.csv` | `scripts/verify_model.py` + manual runs | detection rate and latency per backend |

An empty file with only a header row is the correct state before the MVP is
finished. Do not paste placeholder numbers into these files.

Why `latency_raw.csv` exists: a percentile of percentiles is not the percentile.
With raw samples the whole run has a real p50 and p95; without them
`benchmark.py` says so in the table instead of producing a number that looks
precise and is not.
