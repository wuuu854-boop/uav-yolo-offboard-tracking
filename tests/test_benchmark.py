"""Benchmark summary: the function that turns CSV rows into README claims."""

from __future__ import annotations

import math

import pytest

from scripts.benchmark import load_rows, percentile, summarize, to_markdown


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
    assert summary["latency_median_ms"] == pytest.approx(220.0)
    assert summary["by_target"]["0"]["trials"] == 2


def test_summary_tolerates_rows_without_latency():
    summary = summarize([{"error_m": "0.8", "latency_p50_ms": ""}])
    assert summary["trials"] == 1
    assert summary["successes"] == 1
    assert math.isnan(summary["latency_median_ms"])


def test_markdown_says_so_when_there_are_no_trials():
    assert "No trials recorded" in to_markdown(summarize([]))


def test_markdown_contains_the_numbers_and_per_target_table():
    summary = summarize(
        [
            {"target_id": "0", "error_m": "0.4", "latency_p50_ms": "180"},
            {"target_id": "1", "error_m": "1.2", "latency_p50_ms": "200"},
        ]
    )
    table = to_markdown(summary)
    assert "2/2" in table
    assert "| 0 | 1 |" in table
    assert "| 1 | 1 |" in table


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
