# Repeatability of the recent doorway tasks

A single SITL pass is not evidence of robustness. The retained suite repeats the
doorway missions, restarting the simulator between runs: **five runs each** for
gate and gate-double, and **ten runs** for the fixed gate-target1 configuration
(the five from `repeat5` plus five additional identical repeats). The original
gate-target1 configuration is kept at **n=3** because those three failures are the
evidence for the fix.

The rendered table is [`results/repeat_report.md`](../results/repeat_report.md).

## Method

`make repeat5` runs `scripts/repeat_scenarios.sh 5`:

- `gate` 5× (aligned 2.4 m doorway, target_0),
- `gate-double` 5× (two offset doorways, target_1),
- `gate-target1` 10× with the fixed 0.8 m / 0.1 m configuration (offset doorway,
  target_1): the five from `repeat5` plus five additional repeats of the same
  command.

Each run gets its own prefix (`repeat_<scenario>_<n>`), its own simulator start,
and copies its summary/landing into `results/repeat/`. The aggregation is
`scripts/repeat_report.py`; no numbers are typed in by hand. The three original
gate-target1 failures (`gate-target1_1..3`) remain in the same directory, so the
table shows both the broken original and the fixed configuration.

## Results

| Scenario | Runs | PASS | Target error min/mean/max (m) | Clearance min/max (m) | Contacts | Landings |
|---|---:|---:|---|---|---:|---:|
| gate | 5 | **5/5** | 0.133 / 0.489 / 0.783 | 0.810 / 1.176 | 0 | 5/5 |
| gate-double | 5 | **5/5** | 0.167 / 0.260 / 0.350 | 0.544 / 0.708 | 0 | 5/5 |
| gate-target1, original 0.5 m / 0.2 m | 3 | **0/3** | 0.364 / 0.393 / 0.408 | 0.018 / 0.384 | **4** | 3/3 |
| gate-target1, fixed 0.8 m / 0.1 m | 10 | **9/10** | 0.043 / 0.287 / 0.612 | 0.667 / 0.776 | **1** | 10/10 |

The single v0.23 pass at gate-target1 was **not repeatable**. With the original
0.5 m inflation and 0.2 m waypoint tolerance, three repeats clipped a gate
segment: 4 contacts in total, minimum clearance 0.018 m. The 0.8 m / 0.1 m fix
improved this to **9/10**, but one of ten fixed runs still had a single gate
contact (run 8) with 0.704 m reported clearance. The reported wall clearance is
therefore **not** the full contact envelope; the configuration is improved but not
proven robust.

The expanded sample also caught the tail of gate's target error: 0.783 m at n=5,
still inside the 1.5 m check but wider than the first single run. gate-double is
the tightest route: 0.544 m minimum clearance at n=5.

## What is verified, and what is not

| Claim | Status | Evidence |
|---|---|---|
| gate 5/5 repeats | ✅ verified | 0 contacts, 0.810 m minimum |
| gate-double 5/5 repeats | ✅ verified | 0 contacts, 0.544 m minimum |
| gate-target1 fixed 9/10 repeats | ⚠️ **improved, not robust** | 1 contact, 0.667 m reported minimum |
| gate-target1 original config | ❌ **not robust** | 0/3, 4 contacts |
| Larger sample / confidence interval | ❌ **not done** | n = 10 fixed target1, n = 5 others |
| Hardware / real sensor | ❌ **not done** | SITL, ideal depth |

## Evidence

- `results/repeat_report.md` and `results/repeat_report.json` — the table above.
- `results/repeat/<scenario>_<n>_summary.json` — 23 per-run reports (5 + 5 + 3 +
  10).
- `results/repeat/<scenario>_<n>_landing.csv` — per-run landing row.
- `results/repeat/*.log` — per-run offboard/depth log (kept locally; not all
  copied into git).

## Reproducing it

```bash
make repeat5
```

This produces five runs per scenario. The target1 10-run sample is the five
`repeat5` runs plus five more invocations of the same gate-target1 command; the
per-run summaries are merged under `results/repeat/`. It takes roughly 15 minutes
on the same machine. The result is a table, not a single best run; a scenario with
any contact is visibly not 10/10.
