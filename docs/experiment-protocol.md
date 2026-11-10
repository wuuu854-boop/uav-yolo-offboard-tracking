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
