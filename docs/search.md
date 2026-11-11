# Side-stepping after a blocked stop

Phase 7 sized the retreat. Phase 8 adds the other local degree of freedom: when a
stop is still blocked, **shift sideways** toward the freer side and re-read the
strips. It is bounded by `search_max_attempts`; if no side appears, it holds.

![The blocked frame with the side-strip overlay](../results/search_depth_frame.png)

*The frame the recorder captured when the label became `blocked`: `L 2.61`,
`R 1.98`, both inside the 4.0 m floor. It is from **during** the search, not the
first stopped frame; see the timing caveat below.*

## The sequence

1. The gate stops. A `blocked` label arrives during the settle; the controller
   latches it and starts the side search.
2. The search side is the one with the larger measured clearance. The planner
   sets a waypoint `search_step_m` (2 m) sideways and re-reads on arrival.
3. If a side is free, the normal lateral leg starts from the searched position.
   If not, another step is taken up to `search_max_attempts`; then it holds.
4. The lateral leg, forward leg and target resume are unchanged.

## The scenario

`make search` uses `target_tracking_search.sdf`: the phase-1 wall plus two
0.8 × 0.8 × 6.0 m pillars at `(-1.5, -1.0)` and `(-1.5, +1.5)`, with their own
contact sensors, and a 4.0 m side floor. The mission is
[`config/mission_search.yaml`](../config/mission_search.yaml):
`search_step_m: 2.0`, `search_max_attempts: 2`, no back-off, forward leg 7.5 m.

The `-1.0` pillar matters: at `-1.5` it sat outside the left strip at the stop
(the first phase-8 attempt read `L 9.47 / R 2.05`). Moving it to `-1.0` puts it
inside the strip, and the blocked frame then reads `L 2.61 / R 1.98`.

## What the run measured

One headless SITL flight, `make search`, 2026-11-11. Raw data:
[`results/search_run.csv`](../results/search_run.csv) (787 rows at 10 Hz),
[`results/search_summary.json`](../results/search_summary.json),
[`results/search_trace.csv`](../results/search_trace.csv),
[`results/search_landing.csv`](../results/search_landing.csv),
[`results/search_depth_frame.npy`](../results/search_depth_frame.npy).

| Metric | Value | How it is measured |
|---|---|---|
| Stop engaged | **yes**, at t = 17.6 s | first `stop=1` row |
| Stop-frame clearances | left 7.66 m, right 2.47 m at t = 18.0 | `left_m`, `right_m` |
| Blocked frame | **L 2.61 m, R 1.98 m**, captured at the `blocked` label | `search_depth_frame.png` |
| Search | **`side-search left`** → `search found a side: lateral leg` | trace |
| Search step | **1.95 m** (y −0.21 → 1.74) | trace positions |
| Lateral leg after it | **3.31 m** (y 1.74 → 5.05) | trace |
| Max lateral excursion | **5.063 m** | path |
| Forward leg | **7.5 m** | mission config |
| Gate released | **yes** | report |
| Final target error | **0.381 m** | final CSV row |
| Landing row | **0.506 m**, `success=True`, `patrol complete` | recorder |
| Minimum wall clearance | **0.836 m** | vehicle centre to the wall AABB |
| Contacts (wall + pillars) | **0** | both contact topics |
| Depth frames | 1181 (≈15 Hz) | recorder |
| Flight duration | 78.6 s | CSV |

Report: `scripts/avoidance_report.py --mode detour` → eight checks, all PASS.

## What this does and does not prove

| Claim | Status | Evidence |
|---|---|---|
| The side search executes and re-reads | ✅ verified | trace `side-search left` → `search found a side` |
| It recovers to target_0 after the search | ✅ verified | 0.381 m final error, 0.506 m landing, 8/8 checks |
| It does not touch the wall or pillars | ✅ verified | 0 contacts, 0.836 m clearance |
| Both strips can read below the floor | ✅ verified | blocked frame L 2.61 / R 1.98 during the search |
| **Both strips were below the floor on the first stopped frame** | ❌ **not proven** | the stop rows are left 7.66 / right 2.47; the both-blocked frame is later, during the search |
| A stable both-blocked stop from the start | ❌ **not verified** | the search trigger is still the latched `blocked` label during the settle |
| Alternating the search direction / a 2-D map | ❌ **not implemented** | one side, fixed step, bounded attempts |
| Multiple obstacles / dynamic / global planning | ❌ **not verified** | one static wall + two pillars |

**The caveat is narrower than v0.16.0, not gone.** The run now contains a frame
with both strips inside the floor and a successful recovery from it, but that
frame is captured **during** the search. The first stopped frame still has one
side open, so the search starts from the latched `blocked` label, not from a
stable both-blocked stop. That last step still needs a geometry where both strips
are blocked for several frames **at** the stop.

## Reproducing it

```bash
make install-world
make search
```

The target uses `config/mission_search.yaml`, world
`target_tracking_search`, prefix `search`, `config/depth_search.yaml` and the
two-pillar world. It fails if the stop does not happen, the target is not
reached, the clearance is below 0.4 m, or any contact sensor fires.

## Known artifacts

- The `blocked` frame is saved whenever the depth label becomes `blocked`, so it
  can be later than the first stop row. The CSV timestamps and the frame's
  `L`/`R` values are the evidence for the timing.
- The search side was `left`; the frame's left strip is the one that clears after
  the step. The side choice uses the latest `/avoidance/side_clearance` values,
  not the stop frame.
- The first stop event in `depth_node.log` (nearest 0.25 m) is the ground during
  takeoff; the controller is in `TAKEOFF` and ignores it.
