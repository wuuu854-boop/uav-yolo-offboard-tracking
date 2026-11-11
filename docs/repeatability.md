# Repeatability of the recent doorway tasks

A single SITL pass is not evidence of robustness. This suite repeats the three
most recent missions **three times each**, restarting the simulator between runs,
and aggregates the per-run summaries. The rendered table is
[`results/repeat_report.md`](../results/repeat_report.md).

## Method

`make repeat` runs `scripts/repeat_scenarios.sh 3`:

- `gate` 3× (aligned 2.4 m doorway, target_0),
- `gate-target1` 3× (offset doorway, target_1),
- `gate-double` 3× (two offset doorways, target_1).

Each run gets its own prefix (`repeat_<scenario>_<n>`), its own simulator start,
and copies its summary/landing into `results/repeat/`. The aggregation is
`scripts/repeat_report.py`; no numbers are typed in by hand.

## Results

| Scenario | Runs | PASS | Target error min/mean/max (m) | Clearance min/max (m) | Contacts | Landings |
|---|---:|---:|---|---|---:|---:|
| gate | 3 | **3/3** | 0.133 / 0.365 / 0.783 | 1.040 / 1.176 | 0 | 3/3 |
| gate-double | 3 | **3/3** | 0.207 / 0.264 / 0.350 | 0.570 / 0.708 | 0 | 3/3 |
| gate-target1, original 0.5 m / 0.2 m | 3 | **0/3** | 0.364 / 0.393 / 0.408 | 0.018 / 0.384 | **4** | 3/3 |
| gate-target1, fixed 0.8 m / 0.1 m | 3 | **3/3** | 0.043 / 0.285 / 0.612 | 0.684 / 0.736 | 0 | 3/3 |

The single v0.23 pass at gate-target1 was **not repeatable**. With the original
0.5 m inflation and 0.2 m waypoint tolerance, all three repeats clipped a gate
segment: 4 contacts in total, minimum clearance 0.018 m. The fix is the same one
that made gate-double robust: **0.8 m inflation, 0.1 m waypoint tolerance**. The
fixed configuration passes 3/3 with 0.684 m minimum clearance.

This is exactly why the repeat suite exists: the original single run looked clean,
but one repeat is not a reliability claim.

## What is verified, and what is not

| Claim | Status | Evidence |
|---|---|---|
| gate 3/3 repeats | ✅ verified | 0 contacts, 1.040 m minimum |
| gate-double 3/3 repeats | ✅ verified | 0 contacts, 0.570 m minimum |
| gate-target1 fixed 3/3 repeats | ✅ verified | 0 contacts, 0.684 m minimum |
| gate-target1 original config | ❌ **not robust** | 0/3, 4 contacts |
| Larger sample / confidence interval | ❌ **not done** | n = 3 per scenario |
| Hardware / real sensor | ❌ **not done** | SITL, ideal depth |

## Evidence

- `results/repeat_report.md` and `results/repeat_report.json` — the table above.
- `results/repeat/<scenario>_<n>_summary.json` — per-run report.
- `results/repeat/<scenario>_<n>_landing.csv` — per-run landing row.
- `results/repeat/*.log` — per-run offboard/depth log (kept locally; not all
  copied into git).

## Reproducing it

```bash
make repeat
```

It takes roughly 10 minutes on the same machine. The result is a table, not a
single best run; a scenario with any contact is visibly not 3/3.
