"""Choosing which detection the mission should chase.

Picking the highest-confidence box is the obvious choice and usually the wrong
one: the highest scorer is often a distant object near the edge of the frame,
while the one the vehicle should approach is the one it is already flying at.
These three strategies are the ones that mattered in practice, and the default is
the one that fails most gracefully.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

STRATEGIES = ("nearest_centre", "nearest_previous", "highest_confidence", "tracked")


def select_target(
    detections: Sequence[dict],
    strategy: str = "nearest_centre",
    image_size: tuple[int, int] | None = None,
    previous: tuple[float, float] | None = None,
    locked_id: int | None = None,
) -> dict | None:
    """Return the detection to follow, or ``None`` when there is nothing to follow.

    Args:
        detections: dicts with ``confidence``, ``bbox_cx`` and ``bbox_cy``; the
            ``tracked`` strategy also needs ``track_id``.
        strategy: one of :data:`STRATEGIES`.
        image_size: ``(width, height)``; required by ``nearest_centre`` and used
            by ``tracked`` as its fallback.
        previous: last selected pixel; required by ``nearest_previous``, which
            falls back to confidence until one exists.
        locked_id: the ``track_id`` the mission committed to; only used by
            ``tracked``. ``None`` means "nothing locked yet".

    Raises:
        ValueError: on an unknown strategy or missing arguments.
    """
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown strategy {strategy!r}: expected one of {STRATEGIES}")
    if not detections:
        return None

    if strategy == "tracked":
        # Follow one object until it is gone. Without the lock the selector would
        # re-decide every frame, which is exactly how a mission ends up
        # oscillating between two targets that are both in view.
        if locked_id is not None:
            for detection in detections:
                if detection.get("track_id") == locked_id:
                    return detection
        # Nothing locked, or the locked object was lost: fall back so the mission
        # still makes progress instead of freezing.
        if image_size is not None:
            return select_target(detections, "nearest_centre", image_size)
        return max(detections, key=lambda detection: detection["confidence"])

    if strategy == "highest_confidence":
        return max(detections, key=lambda detection: detection["confidence"])

    if strategy == "nearest_centre":
        if image_size is None:
            raise ValueError("image_size is required by nearest_centre")
        centre_u, centre_v = image_size[0] / 2.0, image_size[1] / 2.0
        return min(
            detections,
            key=lambda detection: math.hypot(
                detection["bbox_cx"] - centre_u, detection["bbox_cy"] - centre_v
            ),
        )

    if previous is None:
        return max(detections, key=lambda detection: detection["confidence"])
    return min(
        detections,
        key=lambda detection: math.hypot(
            detection["bbox_cx"] - previous[0], detection["bbox_cy"] - previous[1]
        ),
    )
