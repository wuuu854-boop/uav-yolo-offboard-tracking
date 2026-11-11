# Experiment protocol

Anything that goes into the README results table must have been produced by this
protocol. It is deliberately boring: the point is that someone else can repeat it.

## Fixed conditions (record once, never change mid-series)

| Item | Value | Where to check |
|---|---|---|
| CPU model | e.g. Intel i5-1240P | `lscpu \| grep "Model name"` |
| RAM | e.g. 16 GB | `free -h` |
| PX4 version | `v1.15.x` | `git -C $PX4_DIR describe --tags` |
| Gazebo version | gz-sim 8 | `gz sim --versions` |
| ROS 2 | Humble | `echo $ROS_DISTRO` |
| Altitude | 5.0 m | `config/mission.yaml` |
| Speed limit | 1.0 m/s | `config/mission.yaml` |
| Detection rate | 5 Hz | `config/detector.yaml` |
| Target poses | 3 fixed cubes, 20 m apart | `config/targets.yaml` |

If anything in this table changes, start a new series and say so in the README.

## Spawn pose per trial

The nadir camera sees 11.9 × 8.9 m at 5 m, so one trial starts 3.5 m west of the
target under test and no other target is in view:

| Trial target | Command |
|---|---|
| `target_0` | `make sim` (default `POSE=-3.5,0,0.05`) |
| `target_1` | `make sim POSE="16.5,0,0.05"` |
| `target_2` | `make sim POSE="0,16.5,0.05"` |

The depth-camera avoidance run is a separate protocol: world
`target_tracking_obstacle`, pose `-7.5,0,0.05,0,0,1.5708` (facing east), mission
`config/mission_avoidance.yaml`, one run via `make avoid`. Its landing row
(`results/avoidance_landing.csv`) is evidence of where the vehicle stopped, **not**
a tracking-trial row; keep it out of the nine-trial table. Details:
[`avoidance.md`](avoidance.md).

The phase-2 detour uses the same world and pose with
`config/mission_detour.yaml` and `make detour`. Its landing row
(`results/detour_landing.csv`) **is** a success — the L reaches target_0 — but it
is still a separate scenario, not one of the nine tracking trials. Details:
[`detour.md`](detour.md).

The phase-3 side-choice run uses `sim/worlds/target_tracking_detour.sdf`,
`config/mission_detour_auto.yaml` and `make auto-detour`; its landing row is
`results/autod_landing.csv`. Details: [`side-selection.md`](side-selection.md).

The phase-4 both-sides-blocked run uses
`sim/worlds/target_tracking_blocked.sdf`, `config/mission_blocked.yaml` and
`make blocked`; its landing row is `results/blocked_landing.csv` and is
**expected to fail** (`mission timeout`), because the vehicle holds rather than
reaching target_0. Details: [`blocked.md`](blocked.md).

The phase-5 back-off run uses the same world and a 4.0 m side floor,
`config/mission_backoff.yaml` and `make backoff`; its landing row is
`results/backoff_landing.csv` and is a success (0.684 m). Details:
[`backoff.md`](backoff.md).

The phase-6 repeated back-off uses the same world with a 6.0 m floor and 0.2 m
steps, `config/mission_backoff_sweep.yaml` and `make backoff-sweep`; its landing
row is `results/sweep_landing.csv` and is a success (0.808 m). Details:
[`backoff-sweep.md`](backoff-sweep.md).

The phase-7 measured back-off uses the same world, `config/depth_adaptive.yaml`
and `config/mission_adaptive.yaml`; its landing row is
`results/adaptive_landing.csv` and is a success (0.572 m). Details:
[`adaptive.md`](adaptive.md).

The phase-8 side-step search uses `target_tracking_search.sdf` with two side
pillars, `config/mission_search.yaml` and `make search`; its landing row is
`results/search_landing.csv` and is a success (0.663 m). Every stopped row is
`blocked` (L 2.75 / R 1.91), so this is the genuine both-blocked start. Details:
[`search.md`](search.md).

The phase-9 known-map global run uses `target_tracking_obstacle.sdf`,
`config/mission_global.yaml` and `make global`; its landing row is
`results/global_landing.csv` and is a success (0.216 m). The map is hand-entered
and the reactive stop is disabled. Details: [`global.md`](global.md).

The multi-obstacle variant uses `target_tracking_search.sdf`,
`config/mission_global_multi.yaml` and `make global-multi`; its landing row is
`results/global_multi_landing.csv` and is a success (1.001 m). The map has three
hand-entered boxes and the reactive stop is disabled.

The one-shot perception variant uses `target_tracking_obstacle.sdf`,
`config/mission_global_perception.yaml` and `make global-perception`; its landing
row is `results/global_perception_landing.csv` and is a success (1.13 m). The
initial map is empty; one depth stop becomes one box, then the planner replans.

The rotated-wall variant uses `target_tracking_rotated.sdf`,
`config/mission_global_rotated.yaml` and `make global-rotated`; its landing row is
`results/global_rotated_landing.csv` and is a success (0.391 m). The wall is
rotated 30° about z and the report uses the exact rotated distance.

The two-rotated-obstacle variant uses `target_tracking_rotated_multi.sdf`,
`config/mission_global_rotated_multi.yaml` and `make global-rotated-multi`; its
landing row is `results/global_rotated_multi_landing.csv` and is a success
(0.439 m). The wall is 30° and the pillar 45°; both are hand-entered.

The doorway task uses `target_tracking_gate.sdf`,
`config/mission_gate.yaml` and `make gate`; its landing row is
`results/gate_landing.csv` and is a success (0.275 m). The wall is split into two
segments with a 2.4 m gap and the goal is target_0 itself.

The offset-doorway task uses `target_tracking_gate_target1.sdf`,
`config/mission_gate_target1.yaml` and `make gate-target1`; its landing row is
`results/gate_target1_landing.csv` and is a success (0.425 m target 1). The gate
is at x = 9.2 with the gap at y = 2.4 and the target is target_1 at (20, 0); the
report uses `--target-id 1`.

The double-doorway task uses `target_tracking_gate_double.sdf`,
`config/mission_gate_double.yaml` and `make gate-double`; its landing row is
`results/gate_double_landing.csv` and is a success (0.404 m target 1). Gate 1 is
north, gate 2 is south, and the target is target_1; the report uses
`--target-id 1`.

Repeatability uses `make repeat5` (`scripts/repeat_scenarios.sh 5`) for gate,
gate-target1 and gate-double five times each, plus `make repeat-target1-10` for
ten runs of the retained gate-target1 margin mission. The aggregate is
`results/repeat_report.md`. The original gate-target1 0.5/0.2 was **0/3**; the
0.8/0.1 fix was **9/10**; the retained 1.0/0.05 margin is **10/10**.

Restart the simulation between targets; do not teleport a running vehicle.

## Trial procedure

0. `make preflight` — checks configs, imports and writable directories without a
   simulator. It is the cheapest way to avoid discovering a typo after the
   vehicle has taken off.
1. Restart the simulation (a fresh Gazebo instance, not a reused one).
2. `make e2e` — this starts the nodes and waits for one landing row.
3. `make trials N=3` — the same thing three times, restarting the simulator
   between runs so every trial starts from the same pose.
4. Repeat for the other targets (nine rows total: 3 targets × 3 runs); the
   per-target pose and spawn offset are in the table above.
5. Do **not** delete failed trials. A failure is a result.
6. Run `make bench` to turn the rows into the table — never compute the numbers
   by hand, and never copy a number that the script did not print.

## Metric definitions

| Metric | Definition | Unit |
|---|---|---|
| Landing error | horizontal distance between the final vehicle position and the nearest ground-truth target | m |
| Success | landing error ≤ 1.5 m | bool |
| End-to-end latency | image header stamp → setpoint publish, measured in `offboard_node.on_timer` | ms |
| Detection rate | frames with a detection whose centre is within the target box, divided by frames with the target in view | 0–1 |
| Detection latency | `time.perf_counter()` around `session.run` only | ms |

`results/state_trace.csv` records one row per state change (state, note, position,
altitude). It is not a metric, it is the explanation: when a trial fails, this is
where you find out whether it hovered, searched or never armed.

Latency is reported as p50 **and** p95. A mean alone hides the jitter that makes
a controller unstable, and that jitter is the interesting part.

The p95 for a series is computed by **pooling the raw samples** from
`results/latency_raw.csv`, not by taking the median of per-trial medians: a
percentile of percentiles is not a percentile. When the raw file is missing,
`make bench` prints `per-trial medians (no raw file)` next to the table instead
of a p95 it cannot justify.

## Reporting rules

- Report the median and the count of successes (`7/9`), not a percentage alone.
- Every table cell links to the CSV row it came from.
- Round to a precision you can defend: 0.1 m, 1 ms, 0.01 for a rate.
- If a number came from a single run, write "n=1" next to it.
