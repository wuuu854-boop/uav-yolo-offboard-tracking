"""The two dependency lists must stay identical."""

from __future__ import annotations

from scripts.check_dependencies import differences


def test_the_repository_lists_agree():
    assert differences() == []


def test_a_version_mismatch_is_reported():
    problems = differences({"numpy": ">=1.26,<2.0"}, {"numpy": ">=2.0"})
    assert any("numpy" in problem and ">" in problem for problem in problems)


def test_a_missing_entry_is_reported():
    problems = differences({"numpy": ">=1.26,<2.0"}, {})
    assert any("missing from requirements.txt" in problem for problem in problems)


def test_an_extra_entry_is_reported():
    problems = differences({}, {"opencv-python-headless": ">=4.9,<4.11"})
    assert any("missing from pyproject.toml" in problem for problem in problems)


def test_names_are_compared_case_insensitively():
    assert differences({"PyYAML": ">=6.0,<7.0"}, {"pyyaml": ">=6.0,<7.0"}) == []
