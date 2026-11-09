# Simulation assets

## 1. Install the world into PX4

PX4 only looks for worlds inside its own tree, so copy (do not symlink, PX4's
build copies the folder) the world file:

```bash
cp sim/worlds/target_tracking.sdf "$PX4_DIR/Tools/simulation/gz/worlds/"
```

Then any of these starts the simulation:

```bash
make sim                                        # from this repo
# equivalent manual command:
cd "$PX4_DIR" && PX4_GZ_WORLD=target_tracking make px4_sitl gz_x500_mono_cam
```

If `gz_x500_mono_cam` does not exist in your PX4 version, run
`make px4_sitl gz_x500` instead and add a camera sensor to the model — the
camera target name differs between PX4 releases.

## 2. Find the camera topic

The Gazebo topic name contains the world, the spawned model and the sensor:

```bash
gz topic -l | grep -i image
```

Copy the result into `sim/launch/sim.launch.py` (`WORLD`, `MODEL`, `SENSOR`,
`LINK`). Then:

```bash
ros2 launch sim/launch/sim.launch.py
ros2 topic hz /camera/image_raw     # expect >= 15 Hz
```

## 3. Ground truth

Target poses live in two places on purpose:

| File | Role |
|---|---|
| `sim/worlds/target_tracking.sdf` | what the simulator actually renders |
| `config/targets.yaml` | what the acceptance test believes |

They must agree. Verify with:

```bash
gz model -m target_0 --pose
```

## 4. Why the default world uses coloured cubes

A COCO pre-trained YOLO does not detect synthetic cubes zero-shot. The default
`backend: baseline` (HSV threshold) therefore drives the closed-loop demo, while
YOLO runs on the same frames for the latency comparison. To get YOLO in the loop
immediately, replace the cubes with textured objects (e.g. a Fuel model of a
person) and switch `backend: yolo` in `config/detector.yaml`. Fine-tuning on
Gazebo-rendered frames is the v0.2.0 task.
