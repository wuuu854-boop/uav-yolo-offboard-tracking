# Simulation assets

## 1. Install the world

`Tools/simulation/gz` inside PX4 is a **git submodule** (it points at
`PX4/PX4-gazebo-models`). A non-recursive clone leaves it empty, which is why
`doctor.py` checks it explicitly.

```bash
# if $PX4_DIR/Tools/simulation/gz/worlds is empty:
git -C "$PX4_DIR" submodule update --init --recursive

make install-world   # both worlds + the depth-camera vehicle model
```

`make install-world` writes to the model store PX4 actually reads
(`~/.simulation-gazebo`), not just the source tree, and overwrites the installed
vehicle model. That overwrite is deliberate: since v0.9.0 the model carries the
forward depth camera, and keeping an older copy removes `/depth_camera` without
an error.

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
| **`x500_mono_cam_down`** | nadir, +90° pitch, 0.10 m above `base_link`, **plus a 30° down forward depth camera** | ✅ |

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

The **depth** camera is different: its topic is not scoped under the model. It is
`/depth_camera` (and `/depth_camera/points`), because the SDF names the sensor
topic directly. `scripts/gz_depth_bridge.py` turns it into
`/camera/depth/image_raw`:

```bash
python3 scripts/gz_depth_bridge.py --gz-topic /depth_camera \
  --ros-topic /camera/depth/image_raw
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

## 8. The obstacle worlds

`target_tracking_obstacle.sdf` is `target_tracking.sdf` plus one 0.2 × 3.0 × 6.0 m
static wall at world `(-0.8, 0, 3.0)` and a contact sensor on it. It exists so the
nine tracking trials keep flying the unobstructed world: a wall on the approach
path there would invalidate every one of them.

`target_tracking_detour.sdf` is that world plus a 0.8 × 0.8 × 6.0 m pillar at
`(-1.8, -0.8)`, clear of the wall by 0.5 m, with its own contact sensor. It makes
the right depth strip more blocked than the left, so `make auto-detour` can prove
the side choice. The pillar is kept out of the wall on purpose: overlapping
static bodies are not what "0 contacts" should be measuring.

`target_tracking_blocked.sdf` is `target_tracking_obstacle.sdf` plus a
0.2 × 4.0 × 6.0 m barrier at `(-1.2, 0)`, with its own contact sensor. A barrier,
not two side pillars, because the vehicle's lateral drift at the stop decides
which pillar falls inside a side strip; a 4 m barrier is in both strips
regardless. `make blocked` uses it with the 3.0 m side floor in
`config/depth_blocked.yaml`; the controller holds and the mission times out.
`make backoff` uses the same world with the 4.0 m floor in
`config/depth_backoff.yaml`: the vehicle backs 2 m, the barrier's edges leave the
strips, and the L goes around its north end. `make backoff-sweep` uses a 6.0 m
floor and 0.2 m steps in `config/depth_backoff_sweep.yaml`: the barrier stays
blocked for four small steps, then clears. `make adaptive` uses the 6.0 m floor
in `config/depth_adaptive.yaml` and sizes the step from the two strip distances.

`target_tracking_search.sdf` is `target_tracking_obstacle.sdf` plus two 6 m
pillars at `(-1.5, -0.8)` and `(-1.5, +1.8)` with contact sensors, for the
phase-8 side-step search. Both sit inside the side strips at the stop, so every
stopped row is `blocked` (L 2.75 / R 1.91) and the search has a genuine
both-blocked start.

`make global` uses `target_tracking_obstacle.sdf` itself: the wall at local
`(6.7, 0)` is passed to the known-map planner as a box. The map is hand-entered,
not read from the world file, and the reactive stop is disabled for that run.

The wall's pose and size are duplicated in `config/obstacle.yaml`, the pillar's
in `config/side_obstacle.yaml`; `check_world_targets.py` fails when a world and
its config disagree, because the clearance metric is computed from them. Both
obstacle worlds load `gz-sim-contact-system` — without it the contact sensor's
topic is never created and "0 contacts" would be produced by a system that was
never loaded. The positive check for that sensor is in
[`docs/avoidance.md`](../docs/avoidance.md).
