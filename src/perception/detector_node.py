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

    python3 src/perception/detector_node.py --config config/detector.yaml
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import yaml

# Nodes are executed as scripts, so make the repository root importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from src.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_interfaces.msg import Target, TargetArray  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]


def letterbox(
    image: np.ndarray, size: int, colour: int = 114
) -> tuple[np.ndarray, float, int, int]:
    """Resize keeping aspect ratio and pad to a square. Returns (canvas, scale, left, top)."""
    height, width = image.shape[:2]
    scale = min(size / height, size / width)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), colour, dtype=np.uint8)
    top, left = (size - new_h) // 2, (size - new_w) // 2
    canvas[top : top + new_h, left : left + new_w] = resized
    return canvas, scale, left, top


class YoloOnnxDetector:
    """Minimal ONNX Runtime wrapper for a YOLO11 detection export (no NMS in the graph)."""

    def __init__(
        self,
        model_path: Path,
        input_size: int = 640,
        conf_threshold: float = 0.35,
        iou_threshold: float = 0.45,
        classes: tuple[int, ...] = (0,),
        max_detections: int = 10,
    ) -> None:
        if not model_path.exists():
            raise FileNotFoundError(
                f"model not found: {model_path}\n"
                "Export it first with `python3 scripts/export_onnx.py` "
                "and put the .onnx file under models/."
            )
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.input_size = int(input_size)
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.classes = np.asarray(classes, dtype=int)
        self.max_detections = int(max_detections)

    def infer(self, frame_bgr: np.ndarray) -> list[dict]:
        canvas, scale, left, top = letterbox(frame_bgr, self.input_size)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        outputs = self.session.run(None, {self.input_name: np.ascontiguousarray(blob)})[0]
        return self._postprocess(outputs, scale, left, top)

    def _postprocess(self, output: np.ndarray, scale: float, left: int, top: int) -> list[dict]:
        prediction = np.squeeze(output, axis=0)
        if prediction.shape[0] < prediction.shape[1]:  # (84, 8400) -> (8400, 84)
            prediction = prediction.T

        boxes_xywh = prediction[:, :4]
        scores_per_class = prediction[:, 4:]
        scores = scores_per_class[:, self.classes].max(axis=1)
        keep = scores >= self.conf_threshold
        if not np.any(keep):
            return []

        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]

        cx, cy, w, h = boxes_xywh.T
        x1 = (cx - w / 2.0 - left) / scale
        y1 = (cy - h / 2.0 - top) / scale
        width = w / scale
        height = h / scale

        # cv2.dnn.NMSBoxes expects [x, y, w, h] in the original image scale.
        nms_input = np.stack([x1, y1, width, height], axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(
            nms_input, scores.tolist(), self.conf_threshold, self.iou_threshold
        )

        detections: list[dict] = []
        for index in np.array(indices).flatten()[: self.max_detections]:
            bx, by, bw, bh = nms_input[int(index)]
            detections.append(
                {
                    "label": "target",
                    "confidence": float(scores[int(index)]),
                    "bbox_cx": float(bx + bw / 2.0),
                    "bbox_cy": float(by + bh / 2.0),
                    "bbox_w": float(bw),
                    "bbox_h": float(bh),
                }
            )
        detections.sort(key=lambda d: d["confidence"], reverse=True)
        return detections


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
            self.hsv_lower = tuple(config.get("baseline_hsv_lower", [35, 80, 80]))
            self.hsv_upper = tuple(config.get("baseline_hsv_upper", [85, 255, 255]))
            self.min_area_px = int(config.get("baseline_min_area_px", 150))
            description = f"HSV threshold {self.hsv_lower}..{self.hsv_upper}"
        else:
            raise ValueError(f"unknown backend {self.backend!r}: expected 'yolo' or 'baseline'")

        self.bridge = CvBridge()
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

        message = TargetArray()
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
            message.targets.append(target)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
