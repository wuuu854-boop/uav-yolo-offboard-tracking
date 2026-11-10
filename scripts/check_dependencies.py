#!/usr/bin/env python3
"""Keep pyproject.toml and requirements.txt saying the same thing.

They are two lists of the same dependencies, and on 2026-11-10 they drifted:
requirements.txt said `opencv-python-headless`, pyproject.toml still said
`opencv-python`, so `pip install -e .` silently put the GUI wheel back and the
container failed on libGL. Nothing caught it until a red CI job.

    python3 scripts/check_dependencies.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import tomllib

REPO_ROOT = Path(__file__).resolve().parents[1]
PYPROJECT = REPO_ROOT / "pyproject.toml"
REQUIREMENTS = REPO_ROOT / "requirements.txt"

_NAME = re.compile(r"^([A-Za-z0-9_.-]+)")


def normalise(requirement: str) -> tuple[str, str]:
    """Return ``(name, specifier)`` with the name lowercased and dashes unified."""
    text = requirement.strip()
    match = _NAME.match(text)
    if not match:
        raise ValueError(f"cannot parse requirement: {requirement!r}")
    name = match.group(1).lower().replace("_", "-")
    return name, text[match.end() :].replace(" ", "")


def from_pyproject(path: Path = PYPROJECT) -> dict[str, str]:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    return dict(normalise(item) for item in data["project"]["dependencies"])


def from_requirements(path: Path = REQUIREMENTS) -> dict[str, str]:
    entries = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("-"):
            continue
        name, specifier = normalise(stripped)
        entries[name] = specifier
    return entries


def _clean(mapping: dict[str, str]) -> dict[str, str]:
    """Normalise names and specifiers so the two sources can be compared."""
    return {
        name.lower().replace("_", "-"): value.replace(" ", "") for name, value in mapping.items()
    }


def differences(
    pyproject: dict[str, str] | None = None,
    requirements: dict[str, str] | None = None,
) -> list[str]:
    pyproject = _clean(pyproject if pyproject is not None else from_pyproject())
    requirements = _clean(requirements if requirements is not None else from_requirements())

    problems: list[str] = []
    for name in sorted(set(pyproject) | set(requirements)):
        if name not in requirements:
            problems.append(f"{name}: in pyproject.toml, missing from requirements.txt")
        elif name not in pyproject:
            problems.append(f"{name}: in requirements.txt, missing from pyproject.toml")
        elif pyproject[name] != requirements[name]:
            problems.append(
                f"{name}: pyproject has '{pyproject[name]}', requirements has '{requirements[name]}'"
            )
    return problems


def main() -> int:
    problems = differences()
    if problems:
        print("pyproject.toml and requirements.txt disagree:")
        for problem in problems:
            print(f"  - {problem}")
        print("Keep both lists identical; packaging installs one, CI the other.")
        return 1
    print(f"OK: pyproject.toml and requirements.txt agree on {len(from_pyproject())} dependencies")
    return 0


if __name__ == "__main__":
    sys.exit(main())
