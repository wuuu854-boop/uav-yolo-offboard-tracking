"""Colour-threshold baseline. Skipped automatically when OpenCV is unavailable."""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402

GREEN_LOWER = (50, 100, 100)
GREEN_UPPER = (70, 255, 255)


def test_finds_the_centroid_and_box_of_a_green_rectangle():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (80, 60), (159, 179), (0, 255, 0), thickness=-1)

    found = detect_color_centroid(frame, GREEN_LOWER, GREEN_UPPER, min_area_px=200)

    assert found is not None
    u, v, area, width, height = found
    assert u == pytest.approx(120.0, abs=3.0)
    assert v == pytest.approx(120.0, abs=3.0)
    assert area == pytest.approx(80 * 120, rel=0.2)
    assert width == pytest.approx(80, rel=0.2)
    assert height == pytest.approx(120, rel=0.2)


def test_returns_none_on_an_empty_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert detect_color_centroid(frame, GREEN_LOWER, GREEN_UPPER, min_area_px=50) is None


def test_ignores_blobs_below_the_minimum_area():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (10, 10), (14, 14), (0, 255, 0), thickness=-1)  # 25 px
    assert detect_color_centroid(frame, GREEN_LOWER, GREEN_UPPER, min_area_px=500) is None


def test_rejects_a_frame_that_is_not_bgr():
    with pytest.raises(ValueError):
        detect_color_centroid(np.zeros((480, 640), dtype=np.uint8), GREEN_LOWER, GREEN_UPPER)


RED_LOWER = [[0, 120, 70], [170, 120, 70]]
RED_UPPER = [[10, 255, 255], [180, 255, 255]]


def test_two_ranges_are_or_ed_into_one_blob():
    """Red wraps at hue 180, so half the object falls in the other range."""
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(frame, (60, 80), (139, 159), (30, 30, 220), thickness=-1)  # hue ~1
    cv2.rectangle(frame, (100, 80), (179, 159), (40, 40, 240), thickness=-1)  # hue ~178

    found = detect_color_centroid(frame, RED_LOWER, RED_UPPER, min_area_px=200)

    assert found is not None
    u, _v, area, _w, _h = found
    assert 60 < u < 180
    assert area > 200


def test_a_single_range_still_works():
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(frame, (100, 80), (179, 159), (0, 0, 220), thickness=-1)

    found = detect_color_centroid(frame, RED_LOWER[0], RED_UPPER[0], min_area_px=200)

    assert found is not None
    assert found[0] == pytest.approx(139.0, abs=4.0)
