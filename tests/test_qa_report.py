"""The QA report's arithmetic: association, percentiles, pass/fail."""

from __future__ import annotations

import pytest

from uav_tracking.control.qa import (
    DEFAULT_THRESHOLDS,
    nearest_target,
    percentiles,
    verdict,
)


def test_the_nearest_asset_wins():
    index, distance = nearest_target(3.2, 0.4, [(3.5, 0.0), (23.5, 0.0)])

    assert index == 0
    assert distance == pytest.approx(0.5)


def test_a_detection_far_from_every_asset_reports_its_real_distance():
    _index, distance = nearest_target(50.0, 50.0, [(3.5, 0.0), (23.5, 0.0)])

    assert distance > 30.0


def test_no_assets_is_an_error():
    with pytest.raises(ValueError, match="no targets"):
        nearest_target(0.0, 0.0, [])


def test_percentiles_handle_even_and_odd_counts():
    assert percentiles([1.0, 2.0, 3.0])["median"] == pytest.approx(2.0)
    assert percentiles([1.0, 2.0])["median"] == pytest.approx(1.5)
    assert percentiles([])["max"] != percentiles([])["max"]  # NaN


def test_a_clean_run_passes():
    metrics = {
        "detection_rate": 0.95,
        "longest_gap_s": 0.5,
        "association_m": {"p95": 0.4},
    }

    passed, reasons = verdict(metrics, DEFAULT_THRESHOLDS)

    assert passed
    assert reasons == []


def test_each_failure_names_its_own_metric():
    metrics = {
        "detection_rate": 0.1,
        "longest_gap_s": 9.0,
        "association_m": {"p95": 4.0},
    }

    passed, reasons = verdict(metrics, DEFAULT_THRESHOLDS)

    assert not passed
    assert len(reasons) == 3
    assert "detection rate" in reasons[0]
    assert "gap" in reasons[1]
    assert "association" in reasons[2]
