# Skill 03 — Keep a PX4 Gazebo world valid

| | |
|---|---|
| **When** | A custom world jitters, double-loads systems, or never spawns the vehicle |
| **Time** | 10 minutes |
| **Needs** | The world file, `tools/check_world_targets.py` |
| **From** | `docs/verification-log.md` row 4 |

## Symptom

Any of these, usually after copying a world from a tutorial:

- physics feels doubled: contacts are wrong, the vehicle vibrates;
- a system is loaded twice in the Gazebo console;
- the model never appears and `gz_bridge` reports a spawn timeout.

## Why

PX4 ships a `server.config` that loads Physics, UserCommands, SceneBroadcaster,
Sensors, Imu, NavSat and Magnetometer into **every** world. PX4's own worlds
therefore contain no `<plugin>` tags. Copy a world from elsewhere that declares
them and you run two physics engines.

## Procedure

1. Count the plugin tags — the answer must be zero:

   ```bash
   grep -c "<plugin" sim/worlds/target_tracking.sdf
   ```

2. Match PX4's physics block:

   ```xml
   <physics type="ode">
     <max_step_size>0.004</max_step_size>
     <real_time_update_rate>250</real_time_update_rate>
   </physics>
   ```

3. Keep the parts a camera and NavSat need: a `<light>`, and
   `<spherical_coordinates>` with an ENU world frame.

4. Run the checker instead of eyeballing it:

   ```bash
   python3 scripts/check_world_targets.py
   ```

## Verify

The checker prints:

```
OK: 3 targets agree, world structure matches PX4 conventions
```

It fails on a `<plugin>` tag, a wrong `max_step_size`, a missing light, a missing
spherical origin, or ground-truth coordinates that drifted from the world.

## Why this is a script and not a paragraph

A world that is subtly wrong still runs — it just produces numbers you cannot
trust. The check runs in CI (`.github/workflows/ci.yml`), so the failure surfaces
before a 20-minute simulation does.
