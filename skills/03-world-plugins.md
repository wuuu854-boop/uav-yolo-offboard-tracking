# 03 · A world that loads its systems

**Symptom** Detection node reports zero frames, `/camera/image_raw` never appears,
yet the vehicle flies and `/fmu/out/vehicle_odometry` is fine.

**Cause** PX4 v1.15 launches Gazebo through
`Tools/simulation/gz/simulation-gazebo`, which runs

```
gz sim -r <store>/worlds/<world>.sdf
```

and passes no config file. Nothing loads the system plugins on the world's
behalf. A world without a Sensors system still simulates physics and still
publishes the IMU (some systems are created lazily), so it looks healthy — but
the camera sensor is never instantiated and no image topic exists.

**Fix** Declare the systems in the world:

```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
```

**Verify** the image topic exists, not merely that the simulation runs:

```bash
gz topic -l | grep sensor/camera
# /world/target_tracking/model/.../link/camera_link/sensor/camera/image
```

`make doctor` and `scripts/check_world_targets.py` both fail when the Sensors
system is missing, so this cannot regress silently.

**Limit of this advice** It is specific to the launcher. If you start Gazebo
with a config that already loads these systems (`gz sim --config server.config`),
the same block double-loads them and the simulation misbehaves. One loader, not
two.

**Time** 5 minutes once you know to look for the topic rather than the process.

**What made this hard** Every symptom pointed at the perception code: zero
frames, empty detections. The actually useful check was `gz topic -l | grep
camera` — five seconds, and it localised the fault to the simulator, not the
node.
