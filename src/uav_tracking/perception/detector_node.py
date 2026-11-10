#!/usr/bin/env python3
"""YOLO11n ONNX detector node.

Subscribes
    sensor_msgs/Image            /camera/image_raw
Publishes
    uav_interfaces/TargetArray   /detections

This node deliberately stops at pixels. Turning a pixel into a world coordinate
needs the vehicle pose and attitude, which belongs to the control side; keeping
the split means the detector can be replayed from a rosbag with no PX4 running.

Run from the repository root, after building ``src/uav_interfaces``::

    python3 src/uav_tracking/perception/detector_node.py --config config/detector.yaml
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

# Running as a script: put the package root (src/) on the path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from uav_interfaces.msg import Target, TargetArray  # noqa: E402
from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.tracking import IouTracker  # noqa: E402
from uav_tracking.perception.yolo import YoloOnnxDetector  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]


class DetectorNode(Node):
    def __init__(self, config: dict) -> None:
        super().__init__("uav_detector")

        self.backend = str(config.get("backend", "yolo")).lower()
        self.detector: YoloOnnxDetector | None = None
        description = ""

        if self.backend == "yolo":
            model_path = Path(config["model_path"])
            if not model_path.is_absolute():
                model_path = REPO_ROOT / model_path
            self.detector = YoloOnnxDetector(
                model_path=model_path,
                input_size=config.get("input_size", 640),
                conf_threshold=config.get("conf_threshold", 0.35),
                iou_threshold=config.get("iou_threshold", 0.45),
                classes=tuple(config.get("classes", [0])),
                max_detections=config.get("max_detections", 10),
            )
            description = model_path.name
        elif self.backend == "baseline":
            # baseline_hsv_ranges is a list of [lower, upper] pairs, OR-ed
            # together; a red target needs two because hue wraps at 180.
            ranges = config.get("baseline_hsv_ranges")
            if ranges:
                self.hsv_lower = [tuple(r[0]) for r in ranges]
                self.hsv_upper = [tuple(r[1]) for r in ranges]
                description = f"HSV ranges {ranges}"
            else:
                self.hsv_lower = tuple(config.get("baseline_hsv_lower", [35, 80, 80]))
                self.hsv_upper = tuple(config.get("baseline_hsv_upper", [85, 255, 255]))
                description = f"HSV threshold {self.hsv_lower}..{self.hsv_upper}"
            self.min_area_px = int(config.get("baseline_min_area_px", 150))
        else:
            raise ValueError(f"unknown backend {self.backend!r}: expected 'yolo' or 'baseline'")

        self.bridge = CvBridge()
        self.tracker = IouTracker()
        self.min_period_s = 1.0 / float(config.get("publish_rate_hz", 5))
        self._last_publish_s = 0.0
        self._frames = 0
        self._infer_ms_total = 0.0

        self.publisher = self.create_publisher(TargetArray, "/detections", 10)
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"detector ready: backend={self.backend} ({description}), "
            f"rate <= {1.0 / self.min_period_s:.1f} Hz"
        )

    def on_image(self, msg: Image) -> None:
        """Throttle, convert, infer, publish.

        The throttle is applied *before* the conversion: dropping a frame after
        decoding it would waste most of the CPU this is meant to save. Returning
        early is deliberate — the control side uses "no message for 2 s" as its
        definition of a lost target, so silence while throttled is harmless
        because the throttle period is far below that timeout.
        """
        now = time.monotonic()
        if now - self._last_publish_s < self.min_period_s:
            return
        self._last_publish_s = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - surface the real cv_bridge error
            self.get_logger().error(f"image conversion failed: {exc}")
            return

        started = time.perf_counter()
        if self.backend == "baseline":
            detections = self._baseline(frame)
        else:
            detections = self.detector.infer(frame)
        elapsed_ms = (time.perf_counter() - started) * 1000.0

        # Identity is assigned here, after detection, so the message carries a
        # stable id for the same object across frames. 0 would mean "untracked";
        # the tracker starts at 1.
        detections = self.tracker.update(detections, time.monotonic())

        message = TargetArray()
        # The image header (stamp and frame_id) is passed through unchanged. The
        # consumer measures latency against this stamp, so re-stamping here would
        # silently report the processing time as zero.
        message.header = msg.header
        for detection in detections:
            target = Target()
            target.header = msg.header
            target.label = detection["label"]
            target.confidence = detection["confidence"]
            target.bbox_cx = detection["bbox_cx"]
            target.bbox_cy = detection["bbox_cy"]
            target.bbox_w = detection["bbox_w"]
            target.bbox_h = detection["bbox_h"]
            target.image_width = frame.shape[1]
            target.image_height = frame.shape[0]
            target.track_id = int(detection.get("track_id", 0))
            message.targets.append(target)

        # Published even when empty, on purpose: an empty array says "I am
        # alive and see nothing", which the consumer can distinguish from a dead
        # node. Staying silent would make both look identical.
        self.publisher.publish(message)

        self._frames += 1
        self._infer_ms_total += elapsed_ms
        if self._frames % 30 == 0:
            mean_ms = self._infer_ms_total / 30.0
            self.get_logger().info(f"{self._frames} frames, mean inference {mean_ms:.1f} ms")
            self._infer_ms_total = 0.0

    def _baseline(self, frame: np.ndarray) -> list[dict]:
        """Colour-threshold detection, same output shape as the YOLO backend."""
        found = detect_color_centroid(frame, self.hsv_lower, self.hsv_upper, self.min_area_px)
        if found is None:
            return []
        u, v, _area, width, height = found
        return [
            {
                "label": "target",
                "confidence": 1.0,
                "bbox_cx": u,
                "bbox_cy": v,
                "bbox_w": float(width),
                "bbox_h": float(height),
            }
        ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/detector.yaml")
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    rclpy.init()
    node = DetectorNode(config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM already tore the context down; shutting it down again
        # raises RCLError and prints a traceback on every normal exit.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
