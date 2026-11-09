# Skill 07 — Turn a run into evidence

| | |
|---|---|
| **When** | A number is about to appear in a README, a report or a résumé |
| **Time** | 30 minutes the first time, seconds afterwards |
| **Needs** | The run itself |
| **From** | `docs/experiment-protocol.md`, `src/uav_tracking/control/recorder.py` |

## Symptom

The README says "median landing error 0.9 m". Someone asks which run that was.
Nobody knows. Three months later the number is still there and nobody can defend
it.

## Why

Numbers computed by hand rot. They are copied between documents, rounded twice,
and detached from the run that produced them. The fix is not discipline, it is
plumbing: the code that performs the run should be the code that writes the
number.

## Procedure

1. Make the run write a row, not a print. Here the control node appends to
   `results/landing_error.csv` on every landing: trial, target, error, latency
   percentiles, success, notes.

   ```python
   row = recorder.finish_trial((self.east, self.north), notes=decision.note)
   ```

2. Aggregate with one script, so the definition of the metric lives in one place:

   ```bash
   make bench          # -> trials, success, error median/p95, latency median
   ```

3. Quote from the script's output only. If the table and the CSV disagree, the
   table is wrong.

4. Keep the failures. A trial that missed is a row with `success=false`, not a
   deleted line — a 7/9 that hides the two misses is worse than no number.

## Verify

```bash
python3 scripts/benchmark.py --csv results/landing_error.csv --markdown
```

Open the CSV and find the row the table was computed from. If you cannot point at
it, the number does not go in the README.

## What this buys

- Reviewers can re-run the aggregate without trusting you.
- A regression shows up as a changed table, not a changed sentence.
- The honest state "no trials yet" is representable — which is why this
  repository's results table is empty rather than estimated.
