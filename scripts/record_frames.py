#!/usr/bin/env python3
"""Save camera frames from a running trial, for a demo GIF.

python3 scripts/record_frames.py --out assets/frames --every 2
python3 scripts/make_gif.py assets/frames assets/01-demo.gif
"""

from __future__ import annotations

import argparse
import pathlib

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameRecorder(Node):
    def __init__(self, out_dir: pathlib.Path, every: int) -> None:
        super().__init__("frame_recorder")
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.every = max(1, every)
        self.seen = 0
        self.saved = 0
        self.bridge = CvBridge()
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)

    def on_image(self, msg) -> None:  # noqa: ANN001 - ROS message
        self.seen += 1
        if self.seen % self.every:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        path = self.out_dir / f"frame_{self.saved:04d}.jpg"
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        self.saved += 1
        if self.saved % 20 == 0:
            self.get_logger().info(f"{self.saved} frames saved")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="assets/frames")
    parser.add_argument("--every", type=int, default=2)
    args = parser.parse_args()

    rclpy.init()
    node = FrameRecorder(pathlib.Path(args.out), args.every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"done: {node.saved} frames")
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
