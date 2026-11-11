#!/usr/bin/env python3
"""Record one avoidance flight: pose, depth distance, stop decision and contacts.

One row at 10 Hz plus a colourised depth frame captured when the gate first
stops the vehicle. The CSV is the evidence behind results/avoidance_summary.json;
the frame is what a human looks at to confirm the camera saw a wall rather than
the sky.

    python3 scripts/record_avoidance.py \
        --out results/avoidance_run.csv \
        --frame-out results/avoidance_depth_frame.png \
        --contact-topic /world/.../obstacle_wall/.../contact,/world/.../side_block/.../contact

The contact topics are passed in because Gazebo scopes a contact sensor under
the model even when the SDF names the topic; scripts/run_avoidance.sh discovers
them from `gz topic -l` instead of hard-coding a path that a renamed model
breaks. A comma-separated list records the wall and the side block together.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import threading
from pathlib import Path

import cv2
import gz.msgs10.contacts_pb2 as gz_contacts
import gz.transport13 as gz_transport
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import rclpy  # noqa: E402
from px4_msgs.msg import VehicleOdometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Bool, Float32, Float32MultiArray, String  # noqa: E402

from uav_tracking.control.frames import (  # noqa: E402
    heading_ned_to_yaw_enu,
    ned_position_to_enu,
    quaternion_to_heading_ned,
)
from uav_tracking.perception.depth import (  # noqa: E402
    analyse_depth,
    depth_from_bytes,
    roi_bounds,
)

HEADER = [
    "t_s",
    "east_m",
    "north_m",
    "altitude_m",
    "yaw_deg",
    "nearest_m",
    "stop",
    "clear_side",
    "left_m",
    "right_m",
    "stop_events",
    "contacts_total",
    "depth_frames",
]


class AvoidanceRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("avoidance_recorder")
        self.args = args
        self.start_s: float | None = None
        self.east = 0.0
        self.north = 0.0
        self.altitude = 0.0
        self.yaw = 0.0
        self.have_odometry = False
        self.nearest: float | None = None
        self.stop = False
        self.clear_side = "unknown"
        self.left_m = math.nan
        self.right_m = math.nan
        self.stop_events = 0
        self.depth_frames = 0
        self.contacts_total = 0
        self.contact_events = 0
        self._last_stop_value = False
        self._save_pending = False
        self._lock = threading.Lock()

        self.csv_handle = open(args.out, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_handle)
        self.writer.writerow(HEADER)
        self.csv_handle.flush()

        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/avoidance/stop", self.on_stop, 10)
        self.create_subscription(Float32, "/avoidance/nearest_m", self.on_nearest, 10)
        self.create_subscription(String, "/avoidance/clear_side", self.on_clear_side, 10)
        self.create_subscription(
            Float32MultiArray, "/avoidance/side_clearance", self.on_side_clearance, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data
        )

        self.gz_node = gz_transport.Node()
        contact_topics = [item.strip() for item in args.contact_topic.split(",") if item.strip()]
        if contact_topics:
            for topic in contact_topics:
                if not self.gz_node.subscribe(gz_contacts.Contacts, topic, self.on_contact):
                    self.get_logger().warn(f"could not subscribe to contact topic {topic}")
        else:
            self.get_logger().warn("no contact topic given; contact count will stay 0")

        self.create_timer(1.0 / float(args.rate), self.on_timer)
        self.get_logger().info(f"recording to {args.out}")

    # ------------------------------------------------------------------ inputs
    def on_odometry(self, msg: VehicleOdometry) -> None:
        self.east, self.north, self.altitude = ned_position_to_enu(msg.position)
        self.yaw = heading_ned_to_yaw_enu(quaternion_to_heading_ned(msg.q))
        self.have_odometry = True

    def on_stop(self, msg: Bool) -> None:
        value = bool(msg.data)
        if value and not self._last_stop_value:
            # Save the frame for *every* stop engagement, overwriting the file.
            # The takeoff phase can briefly stop on the ground below the camera;
            # the last engagement is the wall, which is the frame worth keeping.
            self.stop_events += 1
            self._save_pending = True
        self._last_stop_value = value
        self.stop = value

    def on_nearest(self, msg: Float32) -> None:
        value = float(msg.data)
        self.nearest = None if math.isnan(value) else value

    def on_clear_side(self, msg: String) -> None:
        side = str(msg.data).strip().lower()
        if side in ("left", "right", "blocked", "unknown"):
            self.clear_side = side
            if side == "blocked" and self.args.frame_out:
                # The both-strips-blocked frame is the evidence for the search
                # trigger; save it as well as the stop frame.
                self._save_pending = True

    def on_side_clearance(self, msg: Float32MultiArray) -> None:
        values = [float(value) for value in msg.data]
        if len(values) >= 2:
            self.left_m, self.right_m = values[0], values[1]

    def on_contact(self, msg) -> None:  # noqa: ANN001 - protobuf message
        with self._lock:
            self.contact_events += 1
            # gz.msgs.Contacts uses the singular repeated field name `contact`.
            self.contacts_total += len(msg.contact)

    def on_depth(self, msg: Image) -> None:
        self.depth_frames += 1
        if not self._save_pending or not self.args.frame_out:
            return
        try:
            depth = depth_from_bytes(msg.data, msg.width, msg.height)
        except ValueError:
            return
        self._save_pending = False
        stats = analyse_depth(
            depth,
            min_distance_m=float(self.args.min_valid_m),
            max_distance_m=float(self.args.max_valid_m),
        )
        self.save_frame(depth, stats.nearest_m)

    # ------------------------------------------------------------------ output
    def on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.start_s is None:
            self.start_s = now_ns / 1e9
        if not self.have_odometry:
            return

        row = [
            round(now_ns / 1e9 - self.start_s, 3),
            round(self.east, 3),
            round(self.north, 3),
            round(self.altitude, 3),
            round(math.degrees(self.yaw), 2),
            "" if self.nearest is None else round(self.nearest, 3),
            int(self.stop),
            self.clear_side,
            self.left_m,
            self.right_m,
            self.stop_events,
            self.contacts_total,
            self.depth_frames,
        ]
        self.writer.writerow(row)
        self.csv_handle.flush()

    def save_frame(self, depth: np.ndarray, nearest: float | None) -> None:
        """Colourise the frame that caused the stop, with the ROI drawn on it."""
        near, far = 0.2, float(self.args.max_valid_m)
        scaled = np.clip(depth, near, far)
        normalised = ((scaled - near) / max(far - near, 1e-6) * 255.0).astype(np.uint8)
        colour = cv2.applyColorMap(255 - normalised, cv2.COLORMAP_TURBO)

        roi = roi_bounds(
            depth.shape[1],
            depth.shape[0],
            width_fraction=float(self.args.roi_width_fraction),
            height_fraction=float(self.args.roi_height_fraction),
        )
        cv2.rectangle(
            colour,
            (roi.left, roi.top),
            (roi.left + roi.width - 1, roi.top + roi.height - 1),
            (255, 255, 255),
            2,
        )
        label = "nearest: n/a" if nearest is None else f"nearest: {nearest:.2f} m"
        cv2.putText(
            colour, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA
        )
        band = roi_bounds(
            depth.shape[1],
            depth.shape[0],
            width_fraction=1.0,
            height_fraction=float(self.args.roi_height_fraction),
        )
        strip = max(1, int(round(depth.shape[1] * 0.2)))
        for x0, x1, text in ((0, strip, "L"), (depth.shape[1] - strip, depth.shape[1], "R")):
            cv2.rectangle(
                colour,
                (x0, band.top),
                (x1 - 1, band.top + band.height - 1),
                (255, 0, 255),
                2,
            )
            cv2.putText(
                colour,
                text,
                (x0 + 4, band.top + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
        sides = f"L {self.left_m:.2f}  R {self.right_m:.2f}"
        cv2.putText(
            colour,
            sides,
            (12, depth.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        Path(self.args.frame_out).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(self.args.frame_out, colour)
        # The PNG is the human view; the array is what a reader can re-analyse
        # without trusting the colour map.
        np.save(str(Path(self.args.frame_out).with_suffix(".npy")), depth.astype(np.float32))
        self.get_logger().info(f"saved stopped depth frame to {self.args.frame_out}")

    def close(self) -> None:
        self.csv_handle.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="results/avoidance_run.csv")
    parser.add_argument("--frame-out", default="results/avoidance_depth_frame.png")
    parser.add_argument("--contact-topic", default="")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--min-valid-m", type=float, default=0.25)
    parser.add_argument("--max-valid-m", type=float, default=12.0)
    parser.add_argument("--roi-width-fraction", type=float, default=0.6)
    parser.add_argument("--roi-height-fraction", type=float, default=0.5)
    args = parser.parse_args()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = AvoidanceRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
