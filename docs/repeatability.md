# Repeatability: the 5/5 claims were small-sample illusions

The final suite repeats the three doorway missions **20 times each**. This
overturned the earlier 5/5 results: gate and gate-double are **not robust** at
n=20, and even the target1 100 s sample has one final-row drift.

The rendered table is [`results/repeat_report.md`](../results/repeat_report.md).

## Final n=20 table

| Scenario | PASS | Target error min/mean/max (m) | Clearance min/max (m) | Contacts | Landings |
|---|---:|---|---|---:|---:|
| gate | **17/20** | 0.133 / 0.551 / 1.265 | 0.307 / 1.176 | **59** | 20/20 |
| gate-double | **15/20** | 0.112 / 0.612 / 3.246 | 0.265 / 0.797 | **5** | 18/20 |
| gate-target1, 100 s | **20/20 landings / 19/20 final-row** | 0.114 / 0.461 / 2.149 | 0.756 / 1.136 | **0** | 20/20 |

Historical samples that led to the false confidence:

| Scenario | Runs | Result |
|---|---:|---|
| gate, first sample | 5 | 5/5 |
| gate-double, first sample | 5 | 5/5 |
| gate-target1 original 0.5/0.2 | 3 | 0/3 |
| gate-target1 0.8/0.1 | 10 | 9/10 |
| gate-target1 margin 1.0/0.05, 70 s | 20 | 16/20 (3 timeouts, 186 contacts) |
| gate-target1 margin, 100 s | 20 | 20/20 landings / 19/20 final-row |

## What this means

- **gate 17/20**: the first 5/5 was not predictive. The 20-run sample has 3
  failures and **59 contacts**; all 20 landed, but 3 were unsafe.
- **gate-double 15/20**: 5 failures, 5 contacts, 18/20 landings. The 5/5 was also
  a small-sample illusion.
- **gate-target1 100 s**: 20/20 landings, 0 contacts, one final-row drift
  (run 16 landed at 0.285 m, final row 2.149 m). This is the best safety result
  in the suite, but final-row is 19/20.
- The only previous claim that survives in improved form is target1 with the
  **100 s** budget; gate and gate-double need redesign before they can be called
  robust.

No new tuning is claimed in this release. The failures are preserved; the
earlier 3/5, 9/10, 10/10, 16/20 and 19/20 samples are not hidden.

## What is verified, and what is not

| Claim | Status | Evidence |
|---|---|---|
| gate robust | ❌ **17/20** | 3 failures, 59 contacts |
| gate-double robust | ❌ **15/20** | 5 failures, 5 contacts |
| gate-target1 100 s landing | ✅ **20/20** | 0 contacts, landing rows all success |
| gate-target1 100 s final-row | ⚠️ **19/20** | one 2.149 m post-landing drift |
| Any task at 20/20 final-row with 0 contacts | ❌ **not achieved** | target1 is 19/20 final-row |
| Hardware / real sensor | ❌ **not done** | SITL, ideal depth |

## Evidence

- `results/repeat_report.md` / `.json` — the table above.
- `results/repeat/<scenario>_<n>_summary.json` — 93 per-run reports
  (20 + 20 + 3 + 10 + 20 + 20).
- `results/repeat/<scenario>_<n>_landing.csv` — per-run landing row.
- `results/repeat/*.log` — per-run offboard/depth logs (kept locally; not all
  copied into git).

## Reproducing it

```bash
make repeat5             # the old 5-run quick sample (not an acceptance run)
make repeat-target1-20   # twenty target1 runs at 100 s
```

The gate and gate-double 20-run samples are the historical
`repeat_scenarios.sh` runs; the per-run summaries are committed. Do not use a
5-run sample to claim robustness: this suite is the counterexample.
