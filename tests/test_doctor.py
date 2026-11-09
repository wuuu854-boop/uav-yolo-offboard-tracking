"""Doctor helpers: the pure parts, so they can be tested without a simulator."""

from __future__ import annotations

import sys
from pathlib import Path

from scripts.doctor import (
    FAIL,
    PASS,
    SKIP,
    Check,
    check_python,
    collect,
    format_report,
    load_versions,
    parse_version,
    same_minor,
)


def test_parse_version_handles_tags_and_prefixes():
    assert parse_version("v1.15.0") == (1, 15, 0)
    assert parse_version("release/1.15") == (1, 15)
    assert parse_version("8") == (8,)
    assert parse_version("no digits here") == ()


def test_same_minor_compares_only_major_and_minor():
    assert same_minor("v1.15.0", "release/1.15")
    assert not same_minor("v1.14.3", "release/1.15")
    assert not same_minor("v1", "1.15")  # not enough information to compare


def test_python_check_passes_for_the_running_interpreter():
    current = f"{sys.version_info.major}.{sys.version_info.minor}"
    assert check_python(current).status == PASS


def test_python_check_fails_for_a_wrong_version():
    result = check_python("2.7")
    assert result.status == FAIL
    assert "rclpy" in result.hint


def test_report_renders_hints_only_for_problems():
    report = format_report(
        [
            Check("python", PASS, "3.10.19"),
            Check("world", FAIL, "missing", "copy the sdf"),
        ]
    )
    assert "PASS" in report and "FAIL" in report
    assert "copy the sdf" in report
    assert report.index("python") < report.index("world")


def test_static_mode_skips_the_checks_that_need_a_running_stack():
    checks = collect({}, static=True)
    assert any(check.status == SKIP for check in checks)
    assert all(check.status in {PASS, FAIL, SKIP} for check in checks)


def test_versions_file_is_parseable_and_complete():
    versions = load_versions(Path("versions.yaml"))
    for key in ("ubuntu", "ros_distro", "python", "px4", "px4_msgs"):
        assert key in versions, f"versions.yaml lost the {key!r} key"
