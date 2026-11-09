# Adopting this pipeline for your own target

This repository is a working example, not a framework. What follows is the
shortest path from "it runs with coloured cubes" to "it runs with my target, my
camera, my scene", with the check that tells you each step worked.

## What you change, and what you do not

| You want | Touch this | Verify with |
|---|---|---|
| A different target colour | `config/detector.yaml` → `baseline_hsv_*`, `baseline_min_area_px` | `make detect`, look at `/detections` |
| A trained model | `config/detector.yaml` → `backend: yolo`, `model_path`, `classes` | `python3 scripts/verify_model.py` |
| A different camera | `config/camera.yaml` → intrinsics, `image_up_is_forward`, `mount_offset_xyz` | one known target, see `skills/04` |
| A different world / target layout | `sim/worlds/*.sdf` **and** `config/targets.yaml` | `python3 scripts/check_world_targets.py` |
| A different flight profile | `config/mission.yaml` | `make demo`, then one SITL trial |
| Search behaviour after a loss | `config/mission.yaml` → `search_after_s`, `search_area_m`, `search_spacing_m` | `python3 -c` on `lawnmower(...)`, then one SITL trial |
| Your own recorded frames | `scripts/detect_images.py --images <dir>` | the CSV it writes |

Everything under `src/uav_tracking/perception/projection.py`, `src/uav_tracking/control/state_machine.py`
and `src/uav_tracking/control/frames.py` should stay as it is. Those three files hold the
frame conventions and the safety logic, and they are the parts with tests.

## Scenario A — a different target

1. If the target has a distinctive colour, change the HSV bounds and run
   `make detect`. Watch the node's log: it prints the blob area it accepted.
2. If it needs a model, export it (`scripts/export_onnx.py`) and point
   `model_path` at it. `classes` are COCO indices unless you trained your own
   single-class model, in which case use `[0]`.
3. Decide how the mission picks between detections:
   `target_selection` in `config/mission.yaml`.

## Scenario B — a different camera

Derive the intrinsics from the model file, not from a tutorial (`skills/04`), then
set `mount_offset_xyz` to where the camera sits relative to the vehicle origin in
body FLU metres. After that, verify with a target directly below the vehicle: the
detection should land at the image centre, not at an edge.

## Scenario C — a different scene

Keep the two-place rule: the SDF is what the simulator renders, the YAML is what
the test believes, and they must agree. `scripts/check_world_targets.py` fails if
they do not, and also if the world loses a system plugin the launcher does not
provide — a world without Sensors renders no camera images (see `skills/03`).

## Scenario D — a different mission

Change `takeoff_altitude_m`, `max_speed_mps`, `reach_tolerance_m` and
`hover_duration_s`. Then re-run `make demo`: the offline loop uses the same
`MissionStateMachine`, so a state that never terminates shows up in seconds
instead of after a simulator run.

## Scenario E — your own images

```bash
python3 scripts/detect_images.py --images ~/flight_2026_11 --backend baseline
# -> results/detect_images.csv: one row per image, detections + latency
```

This is the cheapest way to find out whether the decoding path works on real
frames before any simulation is involved. It needs no ROS and, with the baseline
backend, no model.

## The validation ladder

Run the rungs in order; each one is cheaper than the next.

1. `make check` — lint, links, doc claims, the unit suite.
2. `make demo` — the control loop on a synthetic camera.
3. `python3 scripts/detect_images.py ...` — real frames, real detector.
4. `make sim` + one `make e2e` trial — the stack end to end.
5. Nine trials → `make bench` → the numbers that go in the README.

Skipping rungs 1–3 does not save time. It moves the failure to the most expensive
place to debug it.
