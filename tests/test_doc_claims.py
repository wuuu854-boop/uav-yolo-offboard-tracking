"""The README's quoted test count must match the suite."""

from __future__ import annotations

from scripts.check_doc_claims import claimed_badge, claimed_tests, count_tests


def test_repository_claim_matches_the_suite():
    assert claimed_tests() == count_tests()


def test_counting_ignores_helpers(tmp_path):
    (tmp_path / "test_example.py").write_text(
        "def helper():\n    pass\n\ndef test_one():\n    pass\n\ndef test_two():\n    pass\n",
        encoding="utf-8",
    )
    assert count_tests(tmp_path) == 2


def test_missing_claim_is_not_an_error(tmp_path):
    readme = tmp_path / "README.md"
    readme.write_text("# no number here\n", encoding="utf-8")
    assert claimed_tests(readme) is None


def test_a_stale_badge_is_reported(tmp_path):
    readme = tmp_path / "README.md"
    readme.write_text(
        "![tests](https://img.shields.io/badge/tests-3-brightgreen)",
        encoding="utf-8",
    )
    assert claimed_badge(readme) == 3
    assert claimed_badge(readme) != 175
