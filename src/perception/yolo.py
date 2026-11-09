"""Pure YOLO decode: letterbox, output parsing, NMS.

Everything here is a plain function over numpy arrays, so it can be tested
without ROS, without ONNX Runtime and without a model file. Box decoding and
NMS are where most detection bugs live; keeping them out of the node is what
makes them testable at all.
"""

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np


def letterbox(
    image: np.ndarray, size: int, colour: int = 114
) -> tuple[np.ndarray, float, int, int]:
    """Resize keeping aspect ratio and pad to a square.

    Returns ``(canvas, scale, left, top)`` where ``scale`` and the padding are
    what the decoder needs to map boxes back to the original frame.
    """
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected HxWx3 BGR image, got shape {image.shape}")
    if size <= 0:
        raise ValueError("size must be positive")

    height, width = image.shape[:2]
    scale = min(size / height, size / width)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas = np.full((size, size, 3), colour, dtype=np.uint8)
    top, left = (size - new_h) // 2, (size - new_w) // 2
    canvas[top : top + new_h, left : left + new_w] = resized
    return canvas, scale, left, top


def decode(
    output: np.ndarray,
    scale: float = 1.0,
    left: int = 0,
    top: int = 0,
    conf_threshold: float = 0.35,
    iou_threshold: float = 0.45,
    classes: Sequence[int] = (0,),
    max_detections: int = 10,
) -> list[dict]:
    """Turn a YOLO11 export ``(1, 4 + num_classes, num_boxes)`` into detections.

    Coordinates in the network output are relative to the letterboxed square;
    ``scale``/``left``/``top`` map them back to the original frame.
    """
    if scale <= 0.0:
        raise ValueError("scale must be positive")

    class_indices = np.asarray(classes, dtype=int).ravel()
    if class_indices.size == 0:
        raise ValueError("classes must not be empty")

    prediction = np.squeeze(output, axis=0) if output.ndim == 3 else output
    if prediction.ndim != 2:
        raise ValueError(f"expected a 2D prediction, got shape {output.shape}")

    # The export can come out as (4 + C, N) or (N, 4 + C). Decide by asking which
    # orientation can hold at least the requested classes; `rows < cols` alone
    # fails when there are fewer boxes than columns (e.g. a 1-box export).
    minimum_columns = int(class_indices.max()) + 5
    if prediction.shape[1] < minimum_columns <= prediction.shape[0]:
        prediction = prediction.T

    if prediction.shape[1] <= class_indices.max() + 4:
        raise ValueError(
            f"prediction has {prediction.shape[1]} columns, not enough for class "
            f"index {int(class_indices.max())}"
        )

    boxes = prediction[:, :4]
    scores = prediction[:, 4:][:, class_indices].max(axis=1)
    keep = scores >= conf_threshold
    if not np.any(keep):
        return []

    boxes = boxes[keep]
    scores = scores[keep]

    cx, cy, width, height = boxes.T
    nms_input = np.stack(
        [
            (cx - width / 2.0 - left) / scale,
            (cy - height / 2.0 - top) / scale,
            width / scale,
            height / scale,
        ],
        axis=1,
    ).tolist()

    # cv2.dnn.NMSBoxes expects [x, y, w, h] in the original image scale.
    indices = np.array(cv2.dnn.NMSBoxes(nms_input, scores.tolist(), conf_threshold, iou_threshold))
    detections: list[dict] = []
    for index in indices.flatten()[:max_detections]:
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

    detections.sort(key=lambda detection: detection["confidence"], reverse=True)
    return detections
