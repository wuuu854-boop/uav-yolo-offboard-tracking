"""Documentation links must keep working as the repository moves."""

from __future__ import annotations

from pathlib import Path

from scripts.check_links import broken_links, markdown_files


def test_repository_has_no_broken_local_links():
    root = Path(__file__).resolve().parents[1]
    assert broken_links(root) == []


def test_a_missing_target_is_reported(tmp_path):
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs" / "page.md").write_text(
        "See [the other page](../README.md) and [a gap](missing.md).\n", encoding="utf-8"
    )
    (tmp_path / "README.md").write_text("# ok\n", encoding="utf-8")

    assert broken_links(tmp_path) == ["docs/page.md: missing.md"]


def test_anchors_and_external_links_are_ignored(tmp_path):
    (tmp_path / "page.md").write_text(
        "[anchor](#section) [web](https://example.com/x) [mail](mailto:a@b.c)\n",
        encoding="utf-8",
    )
    assert broken_links(tmp_path) == []


def test_skipped_directories_are_not_scanned(tmp_path):
    hidden = tmp_path / ".venv"
    hidden.mkdir()
    (hidden / "note.md").write_text("[gone](nowhere.md)\n", encoding="utf-8")
    assert markdown_files(tmp_path) == []
