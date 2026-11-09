"""Guard against references to paths that no longer exist.

After the package rename a Makefile target and a shell script still pointed at
the pre-rename package directory. Nothing without a simulator exercises those
paths, so only a grep catches it. This is that grep, in CI.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SKIP_DIRS = {".git", ".venv", "__pycache__", ".pytest_cache", ".ruff_cache", "results", "models"}
CHECKED_SUFFIXES = {".py", ".sh", ".md", ".yml", ".toml"}
CHECKED_NAMES = {"Makefile"}

# Built from pieces so this file does not match itself.
OLD_PATHS = ("src/" + "perception", "src/" + "control")


def offending_files() -> list[str]:
    offenders: list[str] = []
    for path in REPO_ROOT.rglob("*"):
        if not path.is_file():
            continue
        if SKIP_DIRS.intersection(path.relative_to(REPO_ROOT).parts):
            continue
        if path.suffix not in CHECKED_SUFFIXES and path.name not in CHECKED_NAMES:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if any(old in text for old in OLD_PATHS):
            offenders.append(path.relative_to(REPO_ROOT).as_posix())
    return sorted(offenders)


def test_no_file_references_the_old_package_path():
    assert offending_files() == []
