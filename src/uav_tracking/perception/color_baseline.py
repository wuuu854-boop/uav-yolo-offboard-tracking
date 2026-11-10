"""Colour-threshold detector used as the non-learned baseline.

The point of this module is to have an honest comparison in the results table:
a 15-line HSV threshold versus a CNN. It is also the fallback when the ONNX
model is unavailable, so the control loop can still be tested.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np


def _as_ranges(
    hsv_lower: Sequence[int] | Sequence[Sequence[int]],
    hsv_upper: Sequence[int] | Sequence[Sequence[int]],
) -> list[tuple[Sequence[int], Sequence[int]]]:
    """Accept one ``[h, s, v]`` pair or several.

    A red target needs two ranges: hue wraps around 0/180, so ``[0, 10]`` and
    ``[170, 180]`` are the same colour. Anything else (green, blue) needs one.
    """
    if len(hsv_lower) == 3 and not isinstance(hsv_lower[0], (list, tuple)):
        return [(hsv_lower, hsv_upper)]
    return list(zip(hsv_lower, hsv_upper, strict=True))


def detect_color_centroid(
    frame_bgr: np.ndarray,
    hsv_lower: Sequence[int] | Sequence[Sequence[int]],
    hsv_upper: Sequence[int] | Sequence[Sequence[int]],
    min_area_px: int = 150,
) -> tuple[float, float, int, int, int] | None:
    """Return ``(u, v, area_px, width_px, height_px)`` of the largest blob, or ``None``.

    ``hsv_lower``/``hsv_upper`` accept either a single ``[h, s, v]`` pair or a
    list of pairs, which are OR-ed together.

    ``cv2`` is imported lazily so the geometry tests can run on machines without
    OpenCV installed.
    """
    import cv2  # noqa: PLC0415 - lazy import keeps the pure tests dependency-free

    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError(f"expected HxWx3 BGR image, got shape {frame_bgr.shape}")

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lower_values, upper_values in _as_ranges(hsv_lower, hsv_upper):
        lower = np.array(lower_values, dtype=np.uint8)
        upper = np.array(upper_values, dtype=np.uint8)
        part = cv2.inRange(hsv, lower, upper)
        mask = part if mask is None else cv2.bitwise_or(mask, part)
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
