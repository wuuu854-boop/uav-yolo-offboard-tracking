# Repeatability of the recent doorway tasks

A single SITL pass is not evidence of robustness. The retained suite repeats the
three doorway missions **five times each**, restarting the simulator between runs,
and aggregates the per-run summaries. The original gate-target1 configuration is
kept at **n=3** because those three failures are the evidence for the fix.

The rendered table is [`results/repeat_report.md`](../results/repeat_report.md).

## Method

`make repeat5` runs `scripts/repeat_scenarios.sh 5`:

- `gate` 5× (aligned 2.4 m doorway, target_0),
- `gate-target1` 5× with the fixed 0.8 m / 0.1 m configuration (offset doorway,
  target_1),
- `gate-double` 5× (two offset doorways, target_1).

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
| gate-target1, fixed 0.8 m / 0.1 m | 5 | **5/5** | 0.043 / 0.247 / 0.612 | 0.684 / 0.776 | 0 | 5/5 |

The single v0.23 pass at gate-target1 was **not repeatable**. With the original
0.5 m inflation and 0.2 m waypoint tolerance, three repeats clipped a gate
segment: 4 contacts in total, minimum clearance 0.018 m. The fix is the same one
that made gate-double robust: **0.8 m inflation, 0.1 m waypoint tolerance**. The
fixed configuration passes 5/5 with 0.684 m minimum clearance.

The expanded sample also caught the tail of gate's target error: 0.783 m at n=5,
still inside the 1.5 m check but wider than the first single run. gate-double is
the tightest route: 0.544 m minimum clearance at n=5.

## What is verified, and what is not

| Claim | Status | Evidence |
|---|---|---|
| gate 5/5 repeats | ✅ verified | 0 contacts, 0.810 m minimum |
| gate-double 5/5 repeats | ✅ verified | 0 contacts, 0.544 m minimum |
| gate-target1 fixed 5/5 repeats | ✅ verified | 0 contacts, 0.684 m minimum |
| gate-target1 original config | ❌ **not robust** | 0/3, 4 contacts |
| Larger sample / confidence interval | ❌ **not done** | n = 5 per fixed scenario |
| Hardware / real sensor | ❌ **not done** | SITL, ideal depth |

## Evidence

- `results/repeat_report.md` and `results/repeat_report.json` — the table above.
- `results/repeat/<scenario>_<n>_summary.json` — 18 per-run reports (5 + 5 + 3 + 5).
- `results/repeat/<scenario>_<n>_landing.csv` — per-run landing row.
- `results/repeat/*.log` — per-run offboard/depth log (kept locally; not all
  copied into git).

## Reproducing it

```bash
make repeat5
```

It takes roughly 15 minutes on the same machine. The result is a table, not a
single best run; a scenario with any contact is visibly not 5/5.
