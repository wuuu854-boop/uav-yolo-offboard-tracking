#!/usr/bin/env python3
"""Fail on broken relative links in Markdown, before a reader finds them.

Absolute links are not checked — they belong to other people's servers and
checking them in CI only produces flakes. What is checked is every `](path)`
that points inside this repository, including the anchors' target file.

    python3 scripts/check_links.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
LINK = re.compile(r"\]\(([^)]+)\)")
SKIP_DIRS = {".git", ".venv", "node_modules", "__pycache__", ".pytest_cache", ".ruff_cache"}


def markdown_files(root: Path) -> list[Path]:
    return [
        path
        for path in root.rglob("*.md")
        if not SKIP_DIRS.intersection(path.relative_to(root).parts)
    ]


def broken_links(root: Path) -> list[str]:
    problems: list[str] = []
    for path in markdown_files(root):
        text = path.read_text(encoding="utf-8")
        for raw in LINK.findall(text):
            target = raw.split("#", 1)[0].strip()
            if not target or target.startswith(("http://", "https://", "mailto:")):
                continue
            resolved = (path.parent / target).resolve()
            if not resolved.exists():
                problems.append(f"{path.relative_to(root).as_posix()}: {target}")
    return problems


def main() -> int:
    problems = broken_links(REPO_ROOT)
    if problems:
        print(f"{len(problems)} broken local link(s):")
        for problem in problems:
            print(f"  {problem}")
        return 1
    print(f"OK: {len(markdown_files(REPO_ROOT))} markdown files, no broken local links")
    return 0


if __name__ == "__main__":
    sys.exit(main())
