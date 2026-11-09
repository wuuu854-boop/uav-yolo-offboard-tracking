# Simulation assets

## 1. Install the world

`Tools/simulation/gz` inside PX4 is a **git submodule** (it points at
`PX4/PX4-gazebo-models`). A non-recursive clone leaves it empty, which is why
`doctor.py` checks it explicitly.

```bash
# if $PX4_DIR/Tools/simulation/gz/worlds is empty:
git -C "$PX4_DIR" submodule update --init --recursive

cp sim/worlds/target_tracking.sdf "$PX4_DIR/Tools/simulation/gz/worlds/"
```

## 2. Start it

```bash
make sim                              # target_0, spawn 3.5 m west
make sim POSE="16.5,0,0.05"           # target_1
make sim POSE="0,16.5,0.05"           # target_2

# equivalent manual command:
cd "$PX4_DIR" && PX4_GZ_WORLD=target_tracking \
  PX4_GZ_MODEL_POSE=-3.5,0,0.05 make px4_sitl gz_x500_mono_cam_down
```

If `make px4_sitl gz_x500_mono_cam_down` does not exist in your PX4 version:

```bash
cd "$PX4_DIR" && make px4_sitl list_vmd_make_targets | grep gz_
```

## 3. Use the downward camera, not the forward one

| Model | Camera | Usable for ground projection? |
|---|---|---|
| `x500_mono_cam` | forward, pose `.12 .03 .242`, no rotation | ❌ sees the horizon |
| **`x500_mono_cam_down`** | nadir, +90° pitch, 0.10 m above `base_link` | ✅ |

The intrinsics in `config/camera.yaml` come from `models/mono_cam/model.sdf`
(1280×960, HFOV 1.74 rad, fx = 538.4) and are quoted in `docs/scenario.md`.

## 4. Find the camera topic

The Gazebo topic name contains world, spawned model and sensor:

```bash
gz topic -l | grep -i image
```

Copy the result into `sim/launch/sim.launch.py` (`WORLD`, `MODEL`, `SENSOR`,
`LINK`). Then:

```bash
ros2 launch sim/launch/sim.launch.py
ros2 topic hz /camera/image_raw     # expect ~30 Hz, 1280x960
```

## 5. Ground truth

Target poses live in two places on purpose:

| File | Role |
|---|---|
| `sim/worlds/target_tracking.sdf` | what the simulator renders |
| `config/targets.yaml` | what the acceptance test believes |

`scripts/check_world_targets.py` fails CI when they disagree. Verify by hand with:

```bash
gz model -m target_0 --pose
```

The three targets are 20 m apart so that exactly one is inside the 11.9 × 8.9 m
nadir footprint per trial (see `docs/scenario.md`).

## 6. Why the world declares system plugins

PX4 v1.15's launcher (`Tools/simulation/gz/simulation-gazebo`) runs
`gz sim -r <world>.sdf` and passes no config, so nothing loads the systems on the
world's behalf. Without `gz-sim-sensors-system` the camera sensor is never
instantiated: the vehicle flies, the IMU publishes, and `/camera/image_raw` stays
empty. The world therefore declares Physics, UserCommands, SceneBroadcaster,
Sensors and Imu itself, and `scripts/check_world_targets.py` fails if any of the
first four go missing.

If you launch Gazebo with a config that already loads them
(`gz sim --config ...`), remove the block — one loader, not two. See
`docs/troubleshooting.md` #9.

## 7. Why the default world uses coloured cubes

A COCO pre-trained YOLO does not detect synthetic cubes zero-shot. The default
`backend: baseline` (HSV threshold) therefore drives the closed-loop demo, while
YOLO runs on the same frames for the latency comparison. To get YOLO in the loop
immediately, replace the cubes with textured objects (e.g. a Fuel model of a
person) and switch `backend: yolo` in `config/detector.yaml`. Fine-tuning on
Gazebo-rendered frames is the v0.2.0 task.
