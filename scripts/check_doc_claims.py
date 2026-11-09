#!/usr/bin/env python3
"""Keep the test count quoted in the README equal to the real one.

A number in a README rots silently. This check makes the rot fail CI, so the
sentence "93 tests" cannot survive a commit that deletes or adds tests.

    python3 scripts/check_doc_claims.py

The count is taken from `def test_` definitions, which matches pytest collection
as long as the suite does not use parametrisation (it does not, on purpose: a
parametrised count is harder to quote honestly).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
README = REPO_ROOT / "README.md"
TEST_DEF = re.compile(r"^def test_", re.MULTILINE)
README_CLAIM = re.compile(r"\((\d+) tests\)")


def count_tests(root: Path = REPO_ROOT / "tests") -> int:
    """Count unit tests only.

    ``tests/ros`` is excluded: those tests need a sourced ROS 2 environment, so
    pytest skips the module as a whole elsewhere and the collectable total would
    disagree with the file count. They are covered by the `ROS build` workflow
    instead, and the README says so separately.
    """
    total = 0
    for path in root.rglob("test_*.py"):
        if "ros" in path.relative_to(root).parts:
            continue
        total += len(TEST_DEF.findall(path.read_text(encoding="utf-8")))
    return total


def claimed_tests(readme: Path = README) -> int | None:
    match = README_CLAIM.search(readme.read_text(encoding="utf-8"))
    return int(match.group(1)) if match else None


def main() -> int:
    actual = count_tests()
    claimed = claimed_tests()
    if claimed is None:
        print("README does not state a test count; nothing to check")
        return 0
    if claimed != actual:
        print(f"README claims {claimed} tests, the suite defines {actual}")
        print("Update README.md (or add the tests) in the same commit.")
        return 1
    print(f"OK: README and the suite agree on {actual} tests")
    return 0


if __name__ == "__main__":
    sys.exit(main())
