# Skill 02 — Tell a nadir camera from a forward one

| | |
|---|---|
| **When** | Detection works but the projected ground position is nonsense |
| **Time** | 10 minutes |
| **Needs** | The Gazebo model directory |
| **From** | `docs/verification-log.md` row 2 |

## Symptom

The detector finds the target, the maths runs, and the estimate lands hundreds of
metres away. Or the vehicle flies off in a straight line. Nothing crashes.

## Why

Two PX4 models share a name prefix and behave completely differently:

| Model | Camera | Mount |
|---|---|---|
| `x500_mono_cam` | **forward** | pose `.12 .03 .242`, no rotation |
| `x500_mono_cam_down` | **nadir** | +90° pitch about `y`, 0.10 m above `base_link` |

A ground projection assumes the optical axis points down. With the forward model
the image is a horizon view, so every pixel maps to a place that is not on the
ground under the vehicle.

## Procedure

1. Read the model, not the documentation:

   ```bash
   sed -n '1,40p' "$PX4_DIR/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf"
   ```

   Look for the `<include>` pose and the fixed joint. A rotation of `0 1.5707 0`
   is the +90° pitch that makes the camera look down.

2. Confirm which model is actually spawned:

   ```bash
   gz model -m x500_mono_cam_down_0 --pose
   ```

   If the name has no `_down`, you are flying the forward camera.

3. Check the camera frame the bridge is publishing:

   ```bash
   ros2 topic echo /camera/camera_info --once | grep frame_id
   ```

## Verify

Put one target exactly below the vehicle and hover. The detection centre should
land at the image centre, `(cx, cy)` from `config/camera.yaml`. If it lands at the
top edge or not at all, the mount is wrong.

With the forward camera you can also simply look at the Gazebo window: a nadir
view shows the ground filling the frame and no horizon.

## What we got wrong first

The default was `x500_mono_cam` because the name looked right. The projection was
"working" — it returned numbers — which is the dangerous part. `docs/scenario.md`
now records which file every intrinsic came from.
