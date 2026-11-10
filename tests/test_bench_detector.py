"""The benchmark arithmetic: percentiles and throughput from a latency list."""

from __future__ import annotations

import pytest

from scripts.bench_detector import summarise


def test_percentiles_and_throughput():
    p50, p95, fps = summarise([10.0, 20.0, 30.0, 40.0])

    assert p50 == pytest.approx(25.0)
    assert p95 == pytest.approx(38.5)
    # throughput comes from the mean, so 1000 / 25 ms
    assert fps == pytest.approx(40.0)


def test_a_slow_frame_lowers_throughput_more_than_the_median():
    p50, _p95, fps = summarise([10.0, 90.0])

    assert p50 == pytest.approx(50.0)
    assert fps == pytest.approx(20.0)
    assert fps < 1000.0 / p50 * 1.0001


def test_no_samples_is_an_error():
    with pytest.raises(ValueError, match="no samples"):
        summarise([])
