#!/usr/bin/env python3
"""Bridge a Gazebo camera topic to a ROS 2 Image topic, without ros_gz.

On Humble, `ros-humble-ros-gz-bridge` is built against Fortress-era gz-msgs, so
it silently receives nothing from a Harmonic (gz-sim 8) simulation: the bridge
is created, the topic appears, and no message ever arrives. The Python bindings
(`gz.transport13` + `gz.msgs10`) do match the running simulator, so this node
does the one conversion we need.

    python3 scripts/gz_camera_bridge.py \
        --gz-topic /world/target_tracking/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image \
        --ros-topic /camera/image_raw
"""

from __future__ import annotations

import argparse

import gz.msgs10.image_pb2 as gz_image
import gz.transport13 as gz_transport
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class GzCameraBridge(Node):
    def __init__(self, gz_topic: str, ros_topic: str) -> None:
        super().__init__("gz_camera_bridge")
        self.publisher = self.create_publisher(Image, ros_topic, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.received = 0
        self.dropped = 0

        self.gz_node = gz_transport.Node()
        if not self.gz_node.subscribe(gz_image.Image, gz_topic, self.on_image):
            raise RuntimeError(f"failed to subscribe to {gz_topic}")
        self.get_logger().info(f"bridging {gz_topic} -> {ros_topic}")

    def on_image(self, msg) -> None:  # noqa: ANN001 - protobuf message
        width, height = int(msg.width), int(msg.height)
        if width <= 0 or height <= 0:
            self.dropped += 1
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        pixels = width * height
        # PX4's camera publishes 3 channels; tolerate 4 by dropping alpha.
        if raw.size == pixels * 3:
            frame = raw.reshape(height, width, 3)
            encoding = "rgb8"
        elif raw.size == pixels * 4:
            frame = raw.reshape(height, width, 4)[:, :, :3]
            encoding = "rgb8"
        elif raw.size == pixels:
            frame = raw.reshape(height, width)
            encoding = "mono8"
        else:
            self.dropped += 1
            if self.dropped % 30 == 1:
                self.get_logger().warn(
                    f"unexpected image size {raw.size} for {width}x{height}; dropping"
                )
            return

        message = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(frame), encoding=encoding)
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_link"
        self.publisher.publish(message)

        self.received += 1
        if self.received % 100 == 0:
            self.get_logger().info(f"{self.received} frames bridged")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gz-topic", required=True)
    parser.add_argument("--ros-topic", default="/camera/image_raw")
    args = parser.parse_args()

    rclpy.init()
    node = GzCameraBridge(args.gz_topic, args.ros_topic)
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
