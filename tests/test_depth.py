"""Unit tests for the depth statistics and the stop gate.

These cover the decisions that must be right before a vehicle is allowed to act
on a depth camera: what counts as a valid return, which part of the frame may
command a stop, and how the gate releases.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uav_tracking.perception.depth import (
    DepthStats,
    Roi,
    StopGate,
    StopWatchdog,
    analyse_depth,
    depth_from_bytes,
    roi_bounds,
)


def test_depth_bytes_decode_to_height_by_width_float32():
    depth = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype="<f4")

    decoded = depth_from_bytes(depth.tobytes(), width=3, height=2)

    assert decoded.shape == (2, 3)
    np.testing.assert_allclose(decoded, depth)


def test_depth_bytes_reject_a_truncated_frame():
    with pytest.raises(ValueError, match="expected 24"):
        depth_from_bytes(b"\x00" * 20, width=3, height=2)


def test_depth_bytes_reject_a_non_positive_size():
    with pytest.raises(ValueError, match="invalid depth image size"):
        depth_from_bytes(b"", width=0, height=2)


def test_roi_defaults_to_the_centre_half_height():
    roi = roi_bounds(640, 480)

    assert roi == Roi(left=128, top=120, width=384, height=240)


def test_roi_is_clamped_to_the_frame_at_the_border():
    roi = roi_bounds(100, 100, width_fraction=0.5, height_fraction=0.5, center_x=0.0, center_y=1.0)

    assert (roi.left, roi.top) == (0, 50)
    assert (roi.width, roi.height) == (50, 50)


def test_roi_is_at_least_one_pixel_on_a_tiny_image():
    roi = roi_bounds(1, 1, width_fraction=0.01, height_fraction=0.01)

    assert (roi.width, roi.height) == (1, 1)
    assert (roi.left, roi.top) == (0, 0)


def test_roi_rejects_a_zero_size_image():
    with pytest.raises(ValueError, match="invalid image size"):
        roi_bounds(0, 10)


def test_roi_rejects_an_out_of_range_fraction():
    for fraction in (0.0, -0.1, 1.1):
        with pytest.raises(ValueError, match="fraction"):
            roi_bounds(10, 10, width_fraction=fraction)


def test_roi_rejects_an_out_of_range_centre():
    for centre in (-0.1, 1.1):
        with pytest.raises(ValueError, match="center_x"):
            roi_bounds(10, 10, center_x=centre)


def test_analyse_depth_finds_the_nearest_valid_pixel():
    # Default ROI on 10x10: columns 2..7, rows 2..6.
    depth = np.full((10, 10), 50.0, dtype=np.float32)
    depth[4:6, 4:6] = 1.5

    stats = analyse_depth(depth, min_distance_m=0.1, max_distance_m=20.0)

    assert stats == DepthStats(nearest_m=1.5, valid_pixels=4, roi_pixels=30)


def test_analyse_depth_ignores_zero_nan_and_infinity():
    depth = np.full((10, 10), 10.0, dtype=np.float32)
    depth[2, 2] = 0.0  # no return
    depth[2, 3] = np.nan
    depth[2, 4] = np.inf

    stats = analyse_depth(depth, min_distance_m=0.1, max_distance_m=20.0)

    assert stats.nearest_m == 10.0
    assert stats.valid_pixels == 27


def test_analyse_depth_honours_the_maximum_range():
    depth = np.full((10, 10), 50.0, dtype=np.float32)
    depth[5, 5] = 1.0

    stats = analyse_depth(depth, min_distance_m=0.1, max_distance_m=20.0)

    # The 1 m return is the only usable pixel; the 50 m background is clipped.
    assert stats.nearest_m == 1.0
    assert stats.valid_pixels == 1


def test_analyse_depth_returns_none_when_nothing_is_valid():
    depth = np.zeros((4, 4), dtype=np.float32)

    stats = analyse_depth(depth, min_distance_m=0.1, max_distance_m=20.0)

    assert stats.nearest_m is None
    assert stats.valid_pixels == 0
    assert stats.roi_pixels == 4


def test_analyse_depth_only_looks_inside_the_roi():
    # A 1 m object in the bottom row is outside the default centre window;
    # the centre sees the 5 m wall. If ROI selection were broken, the gate
    # would stop on the floor.
    depth = np.full((100, 100), 5.0, dtype=np.float32)
    depth[95:, :] = 1.0

    stats = analyse_depth(depth, min_distance_m=0.1, max_distance_m=20.0)

    assert stats.nearest_m == 5.0


def test_analyse_depth_rejects_a_non_2d_array():
    with pytest.raises(ValueError, match="2-D"):
        analyse_depth(np.zeros((2, 2, 2), dtype=np.float32))


def test_analyse_depth_rejects_an_inverted_range():
    with pytest.raises(ValueError, match="greater than min_distance_m"):
        analyse_depth(np.zeros((2, 2), dtype=np.float32), min_distance_m=5.0, max_distance_m=1.0)


def test_analyse_depth_rejects_a_non_finite_minimum():
    with pytest.raises(ValueError, match="min_distance_m"):
        analyse_depth(np.zeros((2, 2), dtype=np.float32), min_distance_m=math.nan)


def test_gate_starts_clear_and_stops_below_the_threshold():
    gate = StopGate(stop_distance_m=2.0, clear_distance_m=2.6)

    assert gate.update(5.0) is False
    assert gate.update(2.0) is True
    assert gate.stopped is True
    assert gate.transitions == 1


def test_gate_does_not_release_inside_the_hysteresis_band():
    gate = StopGate(stop_distance_m=2.0, clear_distance_m=2.6)
    gate.update(1.9)

    # 2.3 m is above the stop threshold but below the clear threshold: a
    # braking vehicle must not resume from one noisy frame.
    assert gate.update(2.3) is True
    assert gate.transitions == 1
    assert gate.update(2.6) is False
    assert gate.transitions == 2


def test_gate_ignores_a_missing_measurement():
    gate = StopGate(stop_distance_m=2.0, clear_distance_m=2.6)

    assert gate.update(None) is False
    gate.update(1.0)
    assert gate.update(None) is True
    assert gate.update(math.inf) is True
    assert gate.update(math.nan) is True


def test_gate_reset_forgets_the_latch():
    gate = StopGate(stop_distance_m=2.0, clear_distance_m=2.6)
    gate.update(1.0)

    gate.reset()

    assert gate.stopped is False
    assert gate.transitions == 0
    assert gate.last_distance_m is None
    assert gate.update(1.0) is True


def test_gate_rejects_thresholds_without_hysteresis():
    with pytest.raises(ValueError, match="greater than stop_distance_m"):
        StopGate(stop_distance_m=2.0, clear_distance_m=2.0)


def test_gate_rejects_a_non_positive_stop_threshold():
    for stop in (0.0, -1.0, math.nan):
        with pytest.raises(ValueError, match="stop_distance_m"):
            StopGate(stop_distance_m=stop, clear_distance_m=2.6)


def test_watchdog_is_inactive_until_the_avoidance_node_speaks():
    watchdog = StopWatchdog(timeout_s=1.0)

    # A mission without the depth node must keep working: absence is not a stop.
    assert watchdog.active(100.0) is False


def test_watchdog_reports_the_freshest_value():
    watchdog = StopWatchdog(timeout_s=1.0)
    watchdog.update(True, 10.0)
    assert watchdog.active(10.5) is True

    watchdog.update(False, 11.0)
    assert watchdog.active(11.5) is False


def test_watchdog_fails_safe_when_the_stream_goes_stale():
    watchdog = StopWatchdog(timeout_s=1.0)
    watchdog.update(False, 10.0)

    # 0.9 s of silence is tolerated; 1.1 s is a dead node, and dead is not clear.
    assert watchdog.active(10.9) is False
    assert watchdog.active(11.1) is True


def test_watchdog_reset_makes_it_optional_again():
    watchdog = StopWatchdog(timeout_s=1.0)
    watchdog.update(True, 10.0)

    watchdog.reset()

    assert watchdog.seen is False
    assert watchdog.active(100.0) is False


def test_watchdog_rejects_a_zero_timeout():
    with pytest.raises(ValueError, match="timeout_s"):
        StopWatchdog(timeout_s=0.0)


def test_watchdog_rejects_a_non_finite_clock_reading():
    watchdog = StopWatchdog(timeout_s=1.0)

    with pytest.raises(ValueError, match="now_s"):
        watchdog.update(True, math.nan)
    watchdog.update(True, 1.0)
    with pytest.raises(ValueError, match="now_s"):
        watchdog.active(math.inf)
