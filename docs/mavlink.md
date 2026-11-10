# The MAVLink mirror

The control path is uXRCE-DDS: PX4 publishes uORB topics, the micro-ROS agent
bridges them to ROS 2, and both nodes live in that world. Field tools do not.
A ground station, a log collector, a companion computer or a customer's existing
tooling expects MAVLink.

`scripts/mavlink_bridge.py` is a **read-only** mirror: it subscribes to the
topics the controller already uses and emits three messages at a fixed rate.

| Message | What it carries |
|---|---|
| `HEARTBEAT` | armed state, PX4 autopilot id, offboard custom mode |
| `ATTITUDE` | roll, pitch, yaw in radians, NED |
| `LOCAL_POSITION_NED` | north, east, down and the velocities |

Local position rather than a global position on purpose: SITL has no GPS fix
worth trusting, and the controller does not use one. A ground station showing
local position is showing the same thing the mission is flying.

## Why read-only

A second link that can command the vehicle is how a demonstration becomes an
incident. This module has no arm, mode, takeoff or setpoint message — not a
disabled code path, an absent one. If commanding over MAVLink is ever wanted, it
deserves its own design, its own test, and its own paragraph here.

## Running it

```bash
# point a ground station at udp:127.0.0.1:14550
python3 scripts/mavlink_bridge.py --target udpout:127.0.0.1:14550 --rate 2
```

`pymavlink` is a runtime dependency of this script only; nothing in the control
loop imports it.

## What is tested

`tests/test_mavlink.py` encodes a snapshot, decodes the bytes with `pymavlink`,
and checks the three messages, the armed bit, the NED semantics and the attitude
values; one test sends the stream through a real UDP socket and parses it back.
That is the encoding and the transport. The ROS wiring is exercised by running
the node against a live simulation — it is not covered by a unit test, and this
page says so rather than implying otherwise.
