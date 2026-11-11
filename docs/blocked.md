# When both sides are blocked: refuse to guess

Phase 4 is a safety behaviour, not a mission behaviour. If the depth node sees
both side strips inside the clearance floor, it publishes `blocked`; the
controller **holds** instead of detouring toward the less-bad side. The mission
then times out and lands from the stop point. That landing row is a failure to
reach target_0, and that is the expected outcome.

## What is different from phase 3

| Layer | Phase 3 | Phase 4 |
|---|---|---|
| Side label | `left` / `right` / `unknown` | adds `blocked` |
| Decision | pick the freer side | if both are inside the floor, pick nothing |
| Controller | detour | hold; latch `blocked` for the rest of the stop |
| Scenario | one side pillar | a 4 m barrier across the path |

Two details were needed to make the refusal reliable in SITL:

1. **A start delay.** The first stopped frame had the side label `left`; the
   label became `blocked` 0.6 s later as the vehicle pitched to a halt. Starting
   on the first frame chose the wrong answer. The blocked mission sets
   `detour_start_delay_s: 1.0`, so the controller holds through the transient
   before committing.
2. **A latch.** Once `blocked` has been seen during a stop, it stays blocked for
   the rest of that stop even if a later frame briefly says `left`/`right`. The
   label flickers at the floor; a one-frame "left" between two "blocked" frames
   must not start a detour.

The floor is `min_clear_side_m` in [`config/depth_blocked.yaml`](../config/depth_blocked.yaml)
(3.0 m for this scenario; the normal detour keeps 2.5 m in `config/depth.yaml`).

## The scenario

`sim/worlds/target_tracking_blocked.sdf` is `target_tracking_obstacle.sdf` plus a
0.2 × 4.0 × 6.0 m barrier at world `(-1.2, 0, 3.0)`, 0.2 m in front of
target_0's wall. A barrier is used instead of two side pillars because the
vehicle's lateral drift at the stop decides which pillar is inside a side strip;
a 4 m barrier is blocked on both sides regardless of that drift. The barrier has
its own contact sensor, so "0 contacts" covers the wall and the barrier.

## What the run measured

One headless SITL flight, `make blocked`, 2026-11-11. Spawn `(-7.5, 0, 0.05)`
facing east; approach 0.6 m/s. Raw data:
[`results/blocked_run.csv`](../results/blocked_run.csv) (547 rows at 10 Hz),
[`results/blocked_summary.json`](../results/blocked_summary.json),
[`results/blocked_trace.csv`](../results/blocked_trace.csv),
[`results/blocked_landing.csv`](../results/blocked_landing.csv).

| Metric | Value | How it is measured |
|---|---|---|
| Stop engaged | **yes**, at t = 19.6 s | first `stop=1` row of the wall stop |
| Side response | `left` on the first stopped frame, **`blocked` 0.6 s later** | `clear_side` column |
| Controller response | **`GOTO,blocked: holding`**; no detour | trace |
| Hold before timeout | **35.0 s** | stop run length |
| Settled drift | **0.129 m** (limit 0.5 m for this long hover) | 10 s after the stop |
| Movement after stop | +0.417 m forward, −0.139 m back | `stop_advance_m`, `stop_retreat_m` |
| Minimum clearance to target wall | **2.68 m** | vehicle centre to the wall AABB |
| Contacts (wall + barrier) | **0** | both `gz.msgs.Contacts` topics |
| Depth frames | 821 (≈15 Hz) | recorder |
| Landing row | 4.032 m from target_0, `success=False`, `mission timeout` | expected: it refused to move |

Report: `scripts/avoidance_report.py --mode hold` → all seven checks PASS. The
landing row is a failure **on purpose**: this run tests the refusal, not the
delivery. The `clear_side` value on the *first* stopped row is `left`; the
`blocked` value arrives 0.6 s later and is what the controller latches. The
recorded CSV's `clear_side_at_stop` is therefore `left`; the trace and the
`blocked` run of rows are the evidence that the latch did its job.

## Reproducing it

```bash
make install-world
make blocked
```

The target uses `config/mission_blocked.yaml`, world `target_tracking_blocked`,
prefix `blocked`, and `config/depth_blocked.yaml` for the 3.0 m floor. It fails if
the stop does not hold, if it drifts more than 0.5 m, if it is too close to the
target wall, or if either contact sensor fires.

## What is verified, and what is not

| Claim | Status | Evidence |
|---|---|---|
| Both-side-blocked is detected | ✅ verified | `clear_side=blocked` in the stop rows |
| The controller refuses to guess | ✅ verified | trace has no detour; it holds 35.3 s |
| It does not touch the wall or barrier | ✅ verified | 0 contacts, 2.758 m to the wall |
| The transient label cannot start a detour | ✅ verified | 1 s start delay + blocked latch; `tests/test_detour.py` |
| Backing off / searching for another gap | ✅ **back off phases 5–6** | one 2 m step, or a bounded 0.2 m sweep; no 2-D search |
| A path when both sides are blocked | ❌ **not implemented** | no local search, no global planner |
| Dynamic obstacles, multiple obstacles | ❌ **not verified** | one static barrier + wall |
| A real depth sensor | ❌ **not verified** | ideal Gazebo float32 depth |

## Known artifacts

- The first stop event in `depth_node.log` (nearest 0.25 m) is the ground inside
  the near clip during takeoff; the controller is in `TAKEOFF` and ignores it.
- The side label at the wall stop is `left` for the first stopped rows before `blocked`.
  The latch and the 1 s start delay are there because of exactly this; the trace
  shows `blocked: holding` 0.6 s after the stop.
- The hold drift (0.129 m) is PX4 position hold over 35 s; the scenario states its
  own 0.5 m limit instead of borrowing phase 1's tighter 0.3 m.
