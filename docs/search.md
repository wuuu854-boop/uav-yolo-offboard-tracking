# Side-stepping after a both-blocked stop

Phase 7 sized the retreat. Phase 8 adds the other local degree of freedom: when
the stop is blocked on both sides, **shift sideways** toward the freer side and
re-read the strips. It is bounded by `search_max_attempts`; if no side appears,
it holds.

![The blocked frame with the side-strip overlay](../results/search_depth_frame.png)

*The recorder's `blocked` frame: `L 3.96 m`, `R 3.97 m`, both inside the 4.0 m
floor. The first stopped CSV rows are even closer: `L 2.75 / R 1.91`.*

## The sequence

1. The gate stops. Both strips read below the 4.0 m floor; the depth label is
   `blocked` on every stopped row, not just a transient.
2. The controller starts a side search toward the larger measured clearance. The
   planner sets a waypoint `search_step_m` (2 m) sideways and re-reads on arrival.
3. A side is free at the searched point; the normal lateral leg starts there.
4. The lateral leg, forward leg and target resume are unchanged.

## The scenario

`make search` uses `target_tracking_search.sdf`: the phase-1 wall plus two
0.8 × 0.8 × 6.0 m pillars at `(-1.5, -0.8)` and `(-1.5, +1.8)`, with their own
contact sensors, and a 4.0 m side floor. The asymmetric positions put both
pillars at about ±27° from the stop camera, inside the two side strips. The
mission is [`config/mission_search.yaml`](../config/mission_search.yaml):
`search_step_m: 2.0`, `search_max_attempts: 2`, no back-off, forward leg 7.5 m.

The previous v0.16.0/v0.16.1 attempts had the right pillar at `-1.5` (outside the
left strip at the stop, `L 9.47`). Moving it to `-0.8` is what makes the stop
genuinely both-blocked.

## What the run measured

One headless SITL flight, `make search`, 2026-11-11. Raw data:
[`results/search_run.csv`](../results/search_run.csv) (785 rows at 10 Hz),
[`results/search_summary.json`](../results/search_summary.json),
[`results/search_trace.csv`](../results/search_trace.csv),
[`results/search_landing.csv`](../results/search_landing.csv),
[`results/search_depth_frame.npy`](../results/search_depth_frame.npy).

| Metric | Value | How it is measured |
|---|---|---|
| Stop engaged | **yes**, at t = 17.9 s | first `stop=1` row |
| Stop rows | **`blocked` from 17.8 to 18.6 s**, L 2.75 → 2.70, R 1.91 → 1.88 | `clear_side`, `left_m`, `right_m` |
| Blocked frame | **L 3.96 m, R 3.97 m** | `search_depth_frame.png` |
| Search | **`side-search left`** → `search found a side: lateral leg` | trace |
| Search step | **1.93 m** (y −0.69 → 1.24) | trace positions |
| Lateral leg after it | **3.36 m** (y 1.24 → 4.60) | trace |
| Max lateral excursion | **4.658 m** | path |
| Forward leg | **7.5 m** | mission config |
| Gate released | **yes** | report |
| Final target error | **0.669 m** | final CSV row |
| Landing row | **0.663 m**, `success=True`, `patrol complete` | recorder |
| Minimum wall clearance | **0.869 m** | vehicle centre to the wall AABB |
| Contacts (wall + pillars) | **0** | both contact topics |
| Depth frames | 1177 (≈15 Hz) | recorder |
| Flight duration | 78.4 s | CSV |

Report: `scripts/avoidance_report.py --mode detour` → eight checks, all PASS.

## What is verified

| Claim | Status | Evidence |
|---|---|---|
| Both strips are below the floor at the stop | ✅ verified | every stopped row is `blocked`; L 2.75 / R 1.91 |
| The side search executes and re-reads | ✅ verified | trace `side-search left` → `search found a side` |
| It recovers to target_0 after the search | ✅ verified | 0.669 m final error, 0.663 m landing, 8/8 checks |
| It does not touch the wall or pillars | ✅ verified | 0 contacts, 0.869 m clearance |
| Alternating the search direction / a 2-D map | ❌ **not implemented** | one side, fixed step, bounded attempts |
| Multiple obstacles / dynamic / global planning | ❌ **not verified** | one static wall + two pillars |
| A real depth sensor | ❌ **not verified** | ideal Gazebo float32 depth |

## Reproducing it

```bash
make install-world
make search
```

The target uses `config/mission_search.yaml`, world
`target_tracking_search`, prefix `search`, `config/depth_search.yaml` and the
two-pillar world. It fails if the stop does not happen, either strip is not
blocked on the stop rows, the target is not reached, the clearance is below
0.4 m, or any contact sensor fires.

## Known artifacts

- The frame saved when the `blocked` label arrives can be a later frame than the
  first stop row (here L 3.96/R 3.97 versus L 2.75/R 1.91). Both are inside the
  4.0 m floor; the stopped rows are the stronger evidence because they are
  sampled every 0.1 s.
- The first stop event in `depth_node.log` (nearest 0.25 m) is the ground during
  takeoff; the controller is in `TAKEOFF` and ignores it.
