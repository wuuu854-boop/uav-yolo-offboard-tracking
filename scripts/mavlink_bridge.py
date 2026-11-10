#!/usr/bin/env python3
"""Mirror the vehicle's state over MAVLink, read-only.

The control path is uXRCE-DDS. This node exists because the field tools — a
ground station, a log collector, a companion computer — speak MAVLink, and the
honest way to support them is a one-way mirror: it subscribes to the same topics
the controller uses and sends HEARTBEAT, ATTITUDE and LOCAL_POSITION_NED at a
fixed rate.

Nothing here can command the vehicle. There is no way to arm, take off, change a
mode or move a setpoint, on purpose.

    python3 scripts/mavlink_bridge.py --target udpout:127.0.0.1:14550 --rate 2
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import rclpy
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.control.frames import (  # noqa: E402
    quaternion_to_heading_ned,
    quaternion_to_roll_pitch,
)
from uav_tracking.control.mavlink import Telemetry, encode  # noqa: E402


class MavlinkMirror(Node):
    def __init__(self, target: str, rate_hz: float) -> None:
        super().__init__("mavlink_mirror")
        from pymavlink import mavutil  # noqa: PLC0415 - optional dependency

        self.link = mavutil.mavlink_connection(target)
        self.target = target
        self.seq = 0
        self.sent = 0
        self.received = 0
        self.armed = False
        self.pose: tuple[float, float, float, float, float, float, float] | None = None
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.on_status, qos_profile_sensor_data
        )
        self.create_timer(1.0 / max(rate_hz, 0.1), self.on_timer)
        self.get_logger().info(f"mirroring to {target} at {rate_hz:.1f} Hz (read-only)")

    def on_odometry(self, msg: VehicleOdometry) -> None:
        # PX4 odometry is already NED, which is what MAVLink wants.
        north, east, down = (float(value) for value in msg.position[:3])
        vn, ve, vd = (float(value) for value in msg.velocity[:3])
        yaw = quaternion_to_heading_ned(msg.q)  # radians, NED
        roll, pitch = quaternion_to_roll_pitch(msg.q)
        self.pose = (north, east, down, vn, ve, vd, roll, pitch, yaw)
        if self.received == 0:
            self.get_logger().info(f"first odometry: n={north:.2f} e={east:.2f} d={down:.2f}")
        self.received += 1

    def on_status(self, msg: VehicleStatus) -> None:
        self.armed = int(msg.arming_state) == 2

    def on_timer(self) -> None:
        if self.pose is None:
            return
        if self.sent == 0:
            self.get_logger().info(f"sending to {self.target}")
        self.sent += 1
        north, east, down, vn, ve, vd, roll, pitch, yaw = self.pose
        snapshot = Telemetry(
            armed=self.armed,
            north_m=north,
            east_m=east,
            down_m=down,
            vn_mps=vn,
            ve_mps=ve,
            vd_mps=vd,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            heading_deg=math.degrees(yaw) % 360.0,
        )
        for payload in encode(snapshot, seq=self.seq):
            self.link.write(payload)
        self.seq = (self.seq + 1) % 256


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="udpout:127.0.0.1:14550")
    parser.add_argument("--rate", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = MavlinkMirror(args.target, args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
