"""The shot-sheet arithmetic: resolution, framing, and view coverage.

A shot sheet is the written standard for what a usable capture looks like: which
views, at what ground sample distance, framed how. The four questions it asks are
all arithmetic, so they live here, and the checker that applies them to a capture
stays thin.
"""

from __future__ import annotations

from collections.abc import Iterable

# A view's requirements, as parsed from config/shot_sheet.yaml.
ViewRequirement = dict


def ground_sample_distance_cm(altitude_m: float, focal_px: float) -> float:
    """Centimetres of ground per pixel.

    ``altitude / fx`` is metres per pixel for a nadir pinhole camera; the factor of
    100 turns it into the unit shot sheets are written in.
    """
    if altitude_m < 0.0:
        raise ValueError("altitude_m must be non-negative")
    if focal_px <= 0.0:
        raise ValueError("focal_px must be positive")
    return altitude_m / focal_px * 100.0


def is_fully_framed(
    bbox_cx: float,
    bbox_cy: float,
    bbox_w: float,
    bbox_h: float,
    width: int,
    height: int,
    margin_px: float,
) -> bool:
    """True when the whole box sits at least ``margin_px`` inside the frame.

    A target clipped by the edge is not a usable view even when it is detected:
    the reviewer cannot see the whole structure, and the association test cannot
    find its centre.
    """
    if margin_px < 0.0:
        raise ValueError("margin_px must be non-negative")
    left = bbox_cx - bbox_w / 2.0
    right = bbox_cx + bbox_w / 2.0
    top = bbox_cy - bbox_h / 2.0
    bottom = bbox_cy + bbox_h / 2.0
    return (
        left >= margin_px
        and top >= margin_px
        and right <= width - margin_px
        and bottom <= height - margin_px
    )


def view_rejections(
    view: ViewRequirement,
    altitude_m: float,
    focal_px: float,
    bbox: tuple[float, float, float, float],
    width: int,
    height: int,
) -> list[str]:
    """Every reason this frame does not satisfy ``view``; empty means it does."""
    reasons = []
    gsd = ground_sample_distance_cm(altitude_m, focal_px)
    if gsd > float(view.get("max_gsd_cm", float("inf"))):
        reasons.append(f"gsd {gsd:.2f} cm/px > {view['max_gsd_cm']} cm/px")
    if not is_fully_framed(*bbox, width, height, float(view.get("margin_px", 0.0))):
        reasons.append(f"target closer than {view.get('margin_px', 0)} px to the frame edge")
    return reasons


def coverage(
    observations: Iterable[tuple[int, str, float, list[str]]],
    views: list[ViewRequirement],
) -> dict:
    """Per-asset, per-view counts from ``(asset, view_name, gsd_cm, rejections)``.

    A frame with no rejections counts towards its view. The result also carries
    the rejection reasons, because "3 of 4 views" is only actionable when the
    missing one says why.
    """
    counts: dict[int, dict[str, int]] = {}
    rejected: dict[int, dict[str, list[str]]] = {}
    for asset, view_name, _gsd, reasons in observations:
        counts.setdefault(asset, {}).setdefault(view_name, 0)
        if reasons:
            for reason in reasons:
                rejected.setdefault(asset, {}).setdefault(reason, []).append(view_name)
        else:
            counts[asset][view_name] += 1

    passed = {}
    for asset, per_view in counts.items():
        passed[asset] = all(
            per_view.get(view["name"], 0) >= int(view.get("min_frames", 1)) for view in views
        )
    return {"counts": counts, "rejected": rejected, "passed": passed}
