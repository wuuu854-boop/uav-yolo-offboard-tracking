"""Quality metrics over a flight's detections.

The utility-inspection failure that costs the most is not a bad camera, it is
data that cannot be trusted: too few usable frames, gaps nobody noticed, a track
that silently became a different track, an image associated with the wrong
asset. Those four are arithmetic, so they live here as pure functions and the
script that runs them over a recording stays thin.
"""

from __future__ import annotations

import math

DEFAULT_THRESHOLDS = {
    "detection_rate_min": 0.6,
    "max_gap_s": 2.0,
    "association_p95_m": 1.5,
}


def nearest_target(
    east: float, north: float, targets: list[tuple[float, float]]
) -> tuple[int, float]:
    """Index and distance of the closest asset, in the local frame."""
    if not targets:
        raise ValueError("no targets to associate against")
    distances = [math.hypot(east - tx, north - ty) for tx, ty in targets]
    index = min(range(len(distances)), key=distances.__getitem__)
    return index, distances[index]


def percentiles(values: list[float]) -> dict[str, float]:
    """Median, p95 and max, or NaNs when there is nothing to summarise."""
    if not values:
        return {"median": float("nan"), "p95": float("nan"), "max": float("nan")}
    ordered = sorted(values)
    middle = len(ordered) // 2
    median = ordered[middle] if len(ordered) % 2 else (ordered[middle - 1] + ordered[middle]) / 2.0
    p95 = ordered[min(len(ordered) - 1, int(round(0.95 * (len(ordered) - 1))))]
    return {"median": median, "p95": p95, "max": ordered[-1]}


def longest_gap(times: list[float], total_s: float) -> float:
    """The longest stretch without a detection, including the ends.

    A run with fewer than two detections is one gap from start to finish: that is
    the honest reading, and it stops a single lucky frame from looking complete.
    """
    if len(times) < 2:
        return total_s
    return max(times[i + 1] - times[i] for i in range(len(times) - 1))


def verdict(metrics: dict, thresholds: dict) -> tuple[bool, list[str]]:
    """Pass/fail plus the reason for every failed metric."""
    reasons = []
    if metrics["detection_rate"] < thresholds["detection_rate_min"]:
        reasons.append(
            f"detection rate {metrics['detection_rate']:.0%} < "
            f"{thresholds['detection_rate_min']:.0%}"
        )
    if metrics["longest_gap_s"] > thresholds["max_gap_s"]:
        reasons.append(
            f"longest gap {metrics['longest_gap_s']:.1f}s > {thresholds['max_gap_s']:.1f}s"
        )
    if metrics["association_m"]["p95"] > thresholds["association_p95_m"]:
        reasons.append(
            f"association p95 {metrics['association_m']['p95']:.2f}m > "
            f"{thresholds['association_p95_m']:.1f}m"
        )
    return (not reasons), reasons
