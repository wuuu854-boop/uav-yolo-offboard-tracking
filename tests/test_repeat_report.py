"""Unit tests for the repeatability aggregation."""

from __future__ import annotations

import math

import pytest

from scripts.repeat_report import render_markdown, summarise


def _record(scenario: str, index: int, **overrides) -> dict:
    record = {
        "scenario": scenario,
        "index": index,
        "verdict": "PASS",
        "final_error_m": 0.2,
        "min_clearance_m": 0.5,
        "contacts_total": 0,
        "landing_error_m": 0.3,
        "landing_success": True,
    }
    record.update(overrides)
    return record


def test_summarise_groups_and_counts():
    report = summarise(
        [
            _record("gate", 1),
            _record("gate", 2, verdict="FAIL", final_error_m=0.6, contacts_total=1),
            _record("gate", 3, final_error_m=0.4),
        ]
    )

    gate = report["gate"]
    assert gate["runs"] == 3
    assert gate["passes"] == 2
    assert gate["target_error_m"]["min"] == pytest.approx(0.2)
    assert gate["target_error_m"]["max"] == pytest.approx(0.6)
    assert gate["target_error_m"]["mean"] == pytest.approx(0.4)
    assert gate["contacts_total"] == 1


def test_render_contains_the_pass_fraction():
    markdown = render_markdown(summarise([_record("gate", 1)]))
    assert "1/1" in markdown
    assert "gate" in markdown


def test_nan_landing_is_preserved():
    report = summarise([_record("gate", 1, landing_error_m=math.nan)])
    assert math.isnan(report["gate"]["landing_error_m"]["min"])
