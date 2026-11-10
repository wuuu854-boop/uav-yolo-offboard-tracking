#!/usr/bin/env python3
"""Record frames together with the vehicle pose, for offline backend comparison.

During one trial this saves every Nth camera frame and the odometry at that
moment, so two detectors can be compared on *the same image with the same pose*
instead of on two different flights. That is the only way to tell a systematic
projection offset from flight-to-flight noise.

    python3 scripts/capture_pose_frames.py --out /root/capture --every 5
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.control.frames import (  # noqa: E402
    heading_ned_to_yaw_enu,
    ned_position_to_enu,
    quaternion_to_heading_ned,
    quaternion_to_roll_pitch,
)


class PoseFrameRecorder(Node):
    def __init__(self, out_dir: pathlib.Path, every: int, max_frames: int) -> None:
        super().__init__("pose_frame_recorder")
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.pose_file = (out_dir / "poses.jsonl").open("w", encoding="utf-8")
        self.every = max(1, every)
        self.max_frames = max_frames
        self.seen = 0
        self.saved = 0
        self.bridge = CvBridge()
        self.pose: tuple[float, float, float, float] | None = None

        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)

    def on_odometry(self, msg: VehicleOdometry) -> None:
        east, north, up = ned_position_to_enu(msg.position[:3])
        heading = quaternion_to_heading_ned(msg.q)
        roll, pitch = quaternion_to_roll_pitch(msg.q)
        self.pose = (east, north, up, heading_ned_to_yaw_enu(heading), roll, pitch)

    def on_image(self, msg: Image) -> None:
        self.seen += 1
        if self.pose is None or self.seen % self.every or self.saved >= self.max_frames:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        name = f"frame_{self.saved:04d}.jpg"
        cv2.imwrite(str(self.out_dir / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        east, north, up, yaw, roll, pitch = self.pose
        self.pose_file.write(
            json.dumps(
                {
                    "file": name,
                    "east": east,
                    "north": north,
                    "altitude": up,
                    "yaw": yaw,
                    "roll": roll,
                    "pitch": pitch,
                    "wall_time": time.time(),
                }
            )
            + "\n"
        )
        self.pose_file.flush()
        self.saved += 1
        if self.saved % 20 == 0:
            self.get_logger().info(f"{self.saved} frames captured")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="/root/capture")
    parser.add_argument("--every", type=int, default=5)
    parser.add_argument("--max-frames", type=int, default=120)
    args = parser.parse_args()

    rclpy.init()
    node = PoseFrameRecorder(pathlib.Path(args.out), args.every, args.max_frames)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pose_file.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
