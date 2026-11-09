"""Colour-threshold detector used as the non-learned baseline.

The point of this module is to have an honest comparison in the results table:
a 15-line HSV threshold versus a CNN. It is also the fallback when the ONNX
model is unavailable, so the control loop can still be tested.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np


def detect_color_centroid(
    frame_bgr: np.ndarray,
    hsv_lower: Sequence[int],
    hsv_upper: Sequence[int],
    min_area_px: int = 150,
) -> tuple[float, float, int, int, int] | None:
    """Return ``(u, v, area_px, width_px, height_px)`` of the largest blob, or ``None``.

    ``cv2`` is imported lazily so the geometry tests can run on machines without
    OpenCV installed.
    """
    import cv2  # noqa: PLC0415 - lazy import keeps the pure tests dependency-free

    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError(f"expected HxWx3 BGR image, got shape {frame_bgr.shape}")

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(hsv_lower, dtype=np.uint8)
    upper = np.array(hsv_upper, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return None

    areas = stats[1:, cv2.CC_STAT_AREA]
    best = 1 + int(np.argmax(areas))
    area = int(stats[best, cv2.CC_STAT_AREA])
    if area < min_area_px:
        return None

    u, v = centroids[best]
    width = int(stats[best, cv2.CC_STAT_WIDTH])
    height = int(stats[best, cv2.CC_STAT_HEIGHT])
    return float(u), float(v), area, width, height
