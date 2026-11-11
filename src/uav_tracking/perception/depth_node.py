#!/usr/bin/env python3
"""Turn a Gazebo depth image into a ROS 2 stop decision.

Subscribes
    sensor_msgs/Image   /camera/depth/image_raw    (32FC1, metres)
Publishes
    std_msgs/Bool       /avoidance/stop
    std_msgs/Float32    /avoidance/nearest_m       (NaN when nothing is valid)

The arithmetic lives in ``uav_tracking.perception.depth`` and is unit-tested
without ROS. This file is the wiring: one subscription, one timer, one decision
per tick, and a log line when the decision changes.

    python3 src/uav_tracking/perception/depth_node.py --config config/depth.yaml
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

# Running as a script: put the package root (src/) on the path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

from uav_tracking.perception.depth import (  # noqa: E402
    StopGate,
    analyse_depth,
    choose_side,
    depth_from_bytes,
    side_clearance,
)


class DepthNode(Node):
    def __init__(self, config: dict) -> None:
        super().__init__("uav_depth")
        self.gate = StopGate(
            stop_distance_m=float(config.get("stop_distance_m", 2.0)),
            clear_distance_m=float(config.get("clear_distance_m", 2.6)),
        )
        self.roi = {
            "width_fraction": float(config.get("roi_width_fraction", 0.6)),
            "height_fraction": float(config.get("roi_height_fraction", 0.5)),
            "center_x": float(config.get("roi_center_x", 0.5)),
            "center_y": float(config.get("roi_center_y", 0.5)),
        }
        self.min_valid_m = float(config.get("min_valid_m", 0.25))
        self.max_valid_m = float(config.get("max_valid_m", 12.0))
        self.stale_timeout_s = float(config.get("stale_timeout_s", 0.5))
        self.fail_safe_on_stale = bool(config.get("fail_safe_on_stale", True))
        self.side_width_fraction = float(config.get("side_width_fraction", 0.2))
        self.side_margin_m = float(config.get("side_margin_m", 0.5))
        self.min_clear_side_m = float(config.get("min_clear_side_m", 2.5))

        self.depth = None
        self.depth_s: float | None = None
        self.frames = 0
        self.dropped = 0
        self._last_stop: bool | None = None
        self._last_side: str | None = None
        self._stale_logged = False

        self.stop_pub = self.create_publisher(Bool, "/avoidance/stop", 10)
        self.nearest_pub = self.create_publisher(Float32, "/avoidance/nearest_m", 10)
        self.side_pub = self.create_publisher(String, "/avoidance/clear_side", 10)
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data
        )

        rate = float(config.get("publish_rate_hz", 10.0))
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError(f"publish_rate_hz must be positive, got {rate!r}")
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f"depth node ready: stop <= {self.gate.stop_distance_m:.2f} m, "
            f"clear >= {self.gate.clear_distance_m:.2f} m"
        )

    # ------------------------------------------------------------------ inputs
    def on_depth(self, msg: Image) -> None:
        if msg.encoding != "32FC1":
            self.dropped += 1
            if self.dropped % 30 == 1:
                self.get_logger().warn(
                    f"depth image encoding is {msg.encoding!r}, expected 32FC1; dropping"
                )
            return

        try:
            self.depth = depth_from_bytes(msg.data, msg.width, msg.height)
        except ValueError as error:
            self.dropped += 1
            if self.dropped % 30 == 1:
                self.get_logger().warn(str(error))
            return

        self.depth_s = self.get_clock().now().nanoseconds / 1e9
        self.frames += 1

    # ---------------------------------------------------------------- decision
    def on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        nearest: float | None = None
        side = "unknown"
        stale = self.depth_s is None or (now_s - self.depth_s) > self.stale_timeout_s

        if stale and self.fail_safe_on_stale:
            # No depth frames: a dead bridge must not look like clear sky. This
            # is published from startup too; the controller ignores a stop until
            # it is in a horizontal cruise, so takeoff is unaffected.
            stop = True
            if not self._stale_logged:
                self.get_logger().warn("no fresh depth frames: publishing stop (fail safe)")
                self._stale_logged = True
        else:
            self._stale_logged = False
            if stale:
                nearest = None
            else:
                stats = analyse_depth(
                    self.depth,
                    min_distance_m=self.min_valid_m,
                    max_distance_m=self.max_valid_m,
                    **self.roi,
                )
                nearest = stats.nearest_m
                clearance = side_clearance(
                    self.depth,
                    width_fraction=self.side_width_fraction,
                    height_fraction=self.roi["height_fraction"],
                    min_distance_m=self.min_valid_m,
                    max_distance_m=self.max_valid_m,
                )
                if clearance.left_m is None and clearance.right_m is None:
                    side = "unknown"
                else:
                    side = choose_side(
                        clearance,
                        fallback="right",
                        margin_m=self.side_margin_m,
                        min_clearance_m=self.min_clear_side_m,
                    )
            stop = self.gate.update(nearest)

        self.stop_pub.publish(Bool(data=bool(stop)))
        self.nearest_pub.publish(Float32(data=math.nan if nearest is None else float(nearest)))
        self.side_pub.publish(String(data=side))
        if side != self._last_side:
            self.get_logger().info(f"clear side: {side}")
            self._last_side = side

        if stop != self._last_stop:
            if stop:
                self.get_logger().warn(
                    f"STOP: nearest {nearest:.2f} m <= {self.gate.stop_distance_m:.2f} m"
                    if nearest is not None
                    else "STOP: no usable depth"
                )
            else:
                self.get_logger().info("CLEAR: no obstacle inside the stop distance")
            self._last_stop = stop


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/depth.yaml")
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    rclpy.init()
    node = DepthNode(config)
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
