# Skill 01 — Find a silent ROS 2 topic

| | |
|---|---|
| **When** | A node starts, prints no error, and its callback never fires |
| **Time** | 5 minutes |
| **Needs** | `ros2` CLI, both ends running |
| **From** | `CHANGELOG.md` → subscriptions moved to `SensorDataQoS` |

## Symptom

The node is up. `ros2 node info` shows the subscription. The publisher is clearly
publishing — `ros2 topic hz` prints a rate. The callback still never runs, and
nothing in the log says why.

## Why

ROS 2 matches topics by **type and QoS**, not by name alone. PX4 publishes
`/fmu/out/*` as *best effort*. A subscriber that asks for *reliable* is
incompatible: the middleware silently declines to connect them, and no error is
raised at either end.

## Procedure

1. Look at both ends at once:

   ```bash
   ros2 topic info /fmu/out/vehicle_odometry -v
   ```

   The output lists each endpoint with its `Reliability` and `Durability`. If the
   publisher says `BEST_EFFORT` and your subscription says `RELIABLE`, stop here —
   that is the bug.

2. Check what your code actually asked for:

   ```bash
   grep -n "create_subscription" -A2 src/control/offboard_node.py
   ```

   A bare `10` is the default *reliable* profile.

3. Fix it in the subscriber, not the publisher:

   ```python
   from rclpy.qos import qos_profile_sensor_data
   self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                            self.on_odometry, qos_profile_sensor_data)
   ```

4. Rebuild, restart both sides, and confirm the callback fires.

## Verify

```bash
ros2 topic info /fmu/out/vehicle_odometry -v | grep -i reliability
```

Both endpoints should now read `BEST_EFFORT`. A quick behavioural check: the node
logs its first odometry sample within a second of starting.

## Rule of thumb

If a ROS 2 subscription is silent and the topic exists, suspect QoS **before**
suspecting your logic. Reliable-subscriber-vs-best-effort-publisher is the most
common version of this, and it never produces a diagnostic.
