"""Benchmark summary: the function that turns CSV rows into README claims."""

from __future__ import annotations

import math

import pytest

from scripts.benchmark import (
    load_raw_latencies,
    load_rows,
    percentile,
    summarize,
    to_markdown,
)


def test_percentile_interpolates_between_neighbours():
    values = [1.0, 2.0, 3.0, 4.0]
    assert percentile(values, 50) == pytest.approx(2.5)
    assert percentile(values, 0) == pytest.approx(1.0)
    assert percentile(values, 100) == pytest.approx(4.0)


def test_percentile_of_one_value_is_that_value():
    assert percentile([7.0], 95) == pytest.approx(7.0)


def test_percentile_of_nothing_is_nan():
    assert math.isnan(percentile([], 50))


def test_summary_counts_successes_against_the_tolerance():
    rows = [
        {"trial": "1", "target_id": "0", "error_m": "0.5", "latency_p50_ms": "200"},
        {"trial": "2", "target_id": "0", "error_m": "1.0", "latency_p50_ms": "220"},
        {"trial": "3", "target_id": "1", "error_m": "2.0", "latency_p50_ms": "240"},
    ]
    summary = summarize(rows, tolerance=1.5)

    assert summary["trials"] == 3
    assert summary["successes"] == 2
    assert summary["error_median_m"] == pytest.approx(1.0)
    assert summary["error_p95_m"] == pytest.approx(1.9)
    assert summary["by_target"]["0"]["trials"] == 2


def test_raw_samples_are_pooled_before_the_percentile():
    # Per-trial medians are 100, 200, 300; their median is 200.
    # Pooled, the distribution is bimodal and the pooled p95 is 300.
    rows = [
        {"target_id": "0", "error_m": "0.5", "latency_p50_ms": "100"},
        {"target_id": "0", "error_m": "0.5", "latency_p50_ms": "200"},
        {"target_id": "0", "error_m": "0.5", "latency_p50_ms": "300"},
    ]
    raw = [100.0, 100.0, 100.0, 200.0, 200.0, 200.0, 300.0, 300.0, 300.0]

    summary = summarize(rows, raw_latencies=raw)

    assert summary["latency_p50_ms"] == pytest.approx(200.0)
    assert summary["latency_p95_ms"] == pytest.approx(300.0)
    assert summary["latency_source"] == "9 raw samples"


def test_without_raw_samples_the_source_is_stated_and_p95_is_nan():
    rows = [{"target_id": "0", "error_m": "0.5", "latency_p50_ms": "120"}]
    summary = summarize(rows)

    assert summary["latency_p50_ms"] == pytest.approx(120.0)
    assert math.isnan(summary["latency_p95_ms"])
    assert "per-trial medians" in summary["latency_source"]


def test_markdown_says_so_when_there_are_no_trials():
    assert "No trials recorded" in to_markdown(summarize([]))


def test_markdown_contains_the_numbers_and_the_latency_source():
    rows = [
        {"target_id": "0", "error_m": "0.4", "latency_p50_ms": "180"},
        {"target_id": "1", "error_m": "1.2", "latency_p50_ms": "200"},
    ]
    table = to_markdown(summarize(rows, raw_latencies=[150.0, 250.0]))

    assert "2/2" in table
    assert "| 0 | 1 |" in table
    assert "2 raw samples" in table


def test_load_rows_skips_empty_error_rows(tmp_path):
    path = tmp_path / "landing_error.csv"
    path.write_text(
        "trial,target_id,error_m,latency_p50_ms,latency_p95_ms,success,notes\n"
        "1,0,0.5,200,300,True,arrived\n"
        "2,,,,,,\n",
        encoding="utf-8",
    )
    rows = load_rows(path)
    assert len(rows) == 1
    assert rows[0]["error_m"] == "0.5"


def test_load_raw_latencies_reads_samples_and_tolerates_a_missing_file(tmp_path):
    missing = tmp_path / "latency_raw.csv"
    assert load_raw_latencies(missing) == []

    missing.write_text("trial,latency_ms\n1,101.5\n1,98.5\n", encoding="utf-8")
    assert load_raw_latencies(missing) == [101.5, 98.5]
