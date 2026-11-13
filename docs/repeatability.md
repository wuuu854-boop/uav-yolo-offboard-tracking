# Repeatability: small samples lied, then the safe configurations passed 20/20

The final suite repeats all three doorway missions **20 times each**. The first
5/5 samples for gate and gate-double were small-sample illusions; after changing
the geometry/margin and re-running, both safe configurations pass **20/20 with 0
contacts**.

The rendered table is [`results/repeat_report.md`](../results/repeat_report.md).

## Final 20-run table

| Scenario | PASS | Target error min/mean/max (m) | Clearance min/max (m) | Contacts | Landings |
|---|---:|---|---|---:|---:|
| **gate-safe** (stand-off, 0.8/0.05) | **20/20** | 0.028 / 0.331 / 0.751 | 0.787 / 1.196 | **0** | 20/20 |
| **double-safe** (1.0/0.05, 100 s) | **20/20** | 0.094 / 0.226 / 0.351 | 0.781 / 0.950 | **0** | 20/20 |
| gate (old 0.5/0.2, target_0) | **17/20** | 0.133 / 0.551 / 1.265 | 0.307 / 1.176 | **59** | 20/20 |
| gate-double (old 0.8/0.1) | **15/20** | 0.112 / 0.612 / 3.246 | 0.265 / 0.797 | **5** | 18/20 |
| gate-target1 100 s | **20/20 landings / 19/20 final-row** | 0.114 / 0.461 / 2.149 | 0.756 / 1.136 | **0** | 20/20 |

## What changed

- **gate-safe**: target_0 is only 0.8 m from the gate wall; landing exactly on it
  leaves no propeller margin. The retained goal is an east **stand-off at local
  8.2 m** (target_0 error ≤0.751 m), with **0.8 m inflation** and **0.05 m
  waypoint tolerance**. Result: **20/20, 0 contacts**, clearance min 0.787 m.
- **double-safe**: the old 0.8/0.1 configuration was 15/20 with 5 contacts. The
  retained **1.0 m inflation / 0.05 m tolerance / 100 s** configuration is
  **20/20, 0 contacts**, clearance min 0.781 m.
- **gate-target1** remains 20/20 landings / 19/20 final-row at 100 s; run 16
  landed at 0.285 m and the post-landing final CSV row drifted to 2.149 m.

The old gate 17/20 and gate-double 15/20 evidence is preserved in
`results/repeat/`; nothing was overwritten.

## What is verified, and what is not

| Claim | Status | Evidence |
|---|---|---|
| gate-safe 20/20, 0 contacts | ✅ verified | stand-off goal, clearance 0.787 m |
| double-safe 20/20, 0 contacts | ✅ verified | 1.0/0.05, 100 s, clearance 0.781 m |
| gate-target1 20/20 landings | ✅ verified | 0 contacts, one final-row drift |
| gate-target1 20/20 final-row | ❌ **not achieved** | 19/20; run 16 post-landing drift |
| old gate/gate-double 5/5 | ❌ **small-sample illusion** | 17/20 and 15/20 |
| Hardware / real sensor | ❌ **not done** | SITL, ideal depth |

## Evidence

- `results/repeat_report.md` / `.json`.
- `results/repeat/<scenario>_<n>_summary.json` — 133 per-run reports
  (20 + 20 + 20 + 20 + 3 + 10 + 20 + 20).
- `results/repeat/<scenario>_<n>_landing.csv`.
- `results/repeat/*.log` for the safe runs.

## Reproducing it

```bash
make gate            # gate-safe: stand-off goal, 0.8/0.05
make gate-double     # double-safe: 1.0/0.05, 100 s
make repeat-target1-20
```

`make repeat5` is only a quick smoke sample; the acceptance claim is the n=20
table. The gate task now lands on an east stand-off, not on target_0 itself,
because target_0 is inside the wall/propeller envelope.
