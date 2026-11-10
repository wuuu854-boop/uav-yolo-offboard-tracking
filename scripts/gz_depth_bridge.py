#!/usr/bin/env python3
"""Bridge a Gazebo depth image to a ROS 2 ``32FC1`` Image, without ros_gz.

`ros_gz_bridge` on Humble is built against Fortress-era gz-msgs and receives
nothing from Harmonic; that is why this repository already has a hand-written
image bridge. The depth image is simpler than the RGB one: it is already
float32 metres, so there is no colour conversion, only a re-wrap.

    python3 scripts/gz_depth_bridge.py \
        --gz-topic /depth_camera \
        --ros-topic /camera/depth/image_raw

The Gazebo topic is deliberately global (the model sets `<topic>depth_camera</topic>`)
so the same bridge works for either world. One vehicle per simulation.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import gz.msgs10.image_pb2 as gz_image
import gz.transport13 as gz_transport

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from uav_tracking.perception.depth import depth_from_bytes  # noqa: E402


class GzDepthBridge(Node):
    def __init__(self, gz_topic: str, ros_topic: str, frame_id: str) -> None:
        super().__init__("gz_depth_bridge")
        self.publisher = self.create_publisher(Image, ros_topic, qos_profile_sensor_data)
        self.frame_id = frame_id
        self.received = 0
        self.dropped = 0

        self.gz_node = gz_transport.Node()
        if not self.gz_node.subscribe(gz_image.Image, gz_topic, self.on_image):
            raise RuntimeError(f"failed to subscribe to {gz_topic}")
        self.get_logger().info(f"bridging depth {gz_topic} -> {ros_topic}")

    def on_image(self, msg) -> None:  # noqa: ANN001 - protobuf message
        width, height = int(msg.width), int(msg.height)
        try:
            # Decode once to validate the byte count before publishing. A
            # truncated frame passed through as-is would crash the depth node
            # with a reshape error, one node further from the cause.
            depth_from_bytes(msg.data, width, height)
        except ValueError as error:
            self.dropped += 1
            if self.dropped % 30 == 1:
                self.get_logger().warn(str(error))
            return

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.height = height
        message.width = width
        message.encoding = "32FC1"
        message.is_bigendian = 0
        message.step = width * 4
        message.data = bytes(msg.data)
        self.publisher.publish(message)

        self.received += 1
        if self.received % 100 == 0:
            self.get_logger().info(f"{self.received} depth frames bridged")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gz-topic", default="/depth_camera")
    parser.add_argument("--ros-topic", default="/camera/depth/image_raw")
    parser.add_argument("--frame-id", default="depth_cam_link")
    args = parser.parse_args()

    rclpy.init()
    node = GzDepthBridge(args.gz_topic, args.ros_topic, args.frame_id)
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
