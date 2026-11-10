"""Shot-sheet arithmetic: GSD, framing, and per-view coverage."""

from __future__ import annotations

import pytest

from uav_tracking.control.shot_sheet import (
    coverage,
    ground_sample_distance_cm,
    is_fully_framed,
    view_rejections,
)

VIEW = {"name": "nadir", "camera": "down", "max_gsd_cm": 1.3, "min_frames": 3, "margin_px": 24}


def test_gsd_is_altitude_over_focal_length():
    # 5 m at fx 538.4 -> 0.93 cm/px
    assert ground_sample_distance_cm(5.0, 538.4) == pytest.approx(0.9287, rel=1e-3)


def test_gsd_and_altitude_are_proportional():
    assert ground_sample_distance_cm(10.0, 500.0) == pytest.approx(
        2 * ground_sample_distance_cm(5.0, 500.0)
    )


def test_a_well_framed_box_passes():
    assert is_fully_framed(640, 480, 40, 40, 1280, 960, 24)


def test_a_box_touching_the_edge_fails():
    # 5 px from the top edge, with a 24 px margin required
    assert not is_fully_framed(640, 25, 40, 40, 1280, 960, 24)


def test_a_box_one_pixel_inside_the_margin_passes():
    assert is_fully_framed(640, 45, 40, 40, 1280, 960, 24)


def test_a_good_frame_has_no_rejections():
    reasons = view_rejections(VIEW, 4.9, 538.4, (640, 480, 40, 40), 1280, 960)

    assert reasons == []


def test_a_coarse_frame_is_rejected_for_resolution():
    reasons = view_rejections(VIEW, 9.0, 538.4, (640, 480, 40, 40), 1280, 960)

    assert any("gsd" in reason for reason in reasons)


def test_a_clipped_frame_is_rejected_for_framing():
    reasons = view_rejections(VIEW, 4.9, 538.4, (10, 480, 40, 40), 1280, 960)

    assert any("edge" in reason for reason in reasons)


def test_coverage_passes_with_enough_good_frames():
    observations = [(0, "nadir", 0.9, []) for _ in range(3)]
    result = coverage(observations, [VIEW])

    assert result["passed"][0]
    assert result["counts"][0]["nadir"] == 3


def test_coverage_fails_with_too_few_frames_and_keeps_the_reason():
    observations = [
        (0, "nadir", 0.9, []),
        (0, "nadir", 0.9, []),
        (0, "nadir", 2.1, ["gsd 2.10 cm/px > 1.3 cm/px"]),
    ]
    result = coverage(observations, [VIEW])

    assert not result["passed"][0]
    assert "gsd 2.10 cm/px > 1.3 cm/px" in result["rejected"][0]
