#!/usr/bin/env python3
"""Environment doctor for the PX4 + Gazebo + ROS 2 + Python stack.

Every check maps to a failure that costs people days (see
``docs/troubleshooting.md``). The goal is to fail loudly *before* a simulation
run, with a hint that names the fix, instead of letting a wrong Python version
surface as an unrelated-looking import error ten minutes later.

Usage::

    python3 scripts/doctor.py            # everything
    python3 scripts/doctor.py --static   # no ROS / Gazebo needed (runs in CI)
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
VERSIONS_FILE = REPO_ROOT / "versions.yaml"

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str
    hint: str = ""


def run_command(command: list[str], timeout: float = 10.0) -> tuple[int, str]:
    """Run a command and return ``(returncode, combined output)``. Never raises."""
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, str(exc)
    return completed.returncode, (completed.stdout + completed.stderr).strip()


def parse_version(text: str) -> tuple[int, ...]:
    """Extract the first dotted number: ``'v1.15.0'`` -> ``(1, 15, 0)``."""
    match = re.search(r"(\d+(?:\.\d+)*)", text or "")
    if not match:
        return ()
    return tuple(int(part) for part in match.group(1).split("."))


def same_minor(left: str, right: str) -> bool:
    """True when both strings carry the same major.minor pair."""
    a, b = parse_version(left), parse_version(right)
    return len(a) >= 2 and len(b) >= 2 and a[:2] == b[:2]


def load_versions(path: Path = VERSIONS_FILE) -> dict:
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _os_release() -> dict[str, str]:
    data: dict[str, str] = {}
    try:
        text = Path("/etc/os-release").read_text(encoding="utf-8")
    except OSError:
        return data
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            data[key] = value.strip().strip('"')
    return data


def check_python(expected: str = "3.10") -> Check:
    actual = f"{sys.version_info.major}.{sys.version_info.minor}"
    if actual == expected:
        return Check("python", PASS, f"{actual}.{sys.version_info.micro}")
    return Check(
        "python",
        FAIL,
        f"{actual} (expected {expected})",
        f"ROS 2 was built for Python {expected}; a different interpreter breaks rclpy (troubleshooting #3)",
    )


def check_os(expected: str = "22.04") -> Check:
    release = _os_release()
    if not release:
        return Check("os", SKIP, "not a Linux host (/etc/os-release missing)")
    pretty = release.get("PRETTY_NAME", "unknown")
    if release.get("VERSION_ID") == expected:
        return Check("os", PASS, pretty)
    return Check(
        "os",
        WARN,
        f"{pretty} (verified on Ubuntu {expected})",
        "other versions usually work, expect drift",
    )


def check_ros(expected: str = "humble", static: bool = False) -> Check:
    distro = os.environ.get("ROS_DISTRO", "")
    if not distro:
        if static:
            return Check("ros", SKIP, "not sourced (--static)")
        return Check("ros", FAIL, "ROS_DISTRO is not set", "source /opt/ros/humble/setup.bash")
    if distro == expected:
        return Check("ros", PASS, distro)
    return Check(
        "ros",
        WARN,
        f"{distro} (expected {expected})",
        "message and QoS behaviour differs between distros",
    )


def check_rclpy(expected_python: str = "3.10", static: bool = False) -> Check:
    try:
        import rclpy  # noqa: F401, PLC0415 - import is the check
    except ImportError as exc:
        if static:
            return Check("rclpy", SKIP, "not installed (--static)")
        message = str(exc)
        if "_rclpy_pybind11" in message or "pybind" in message:
            return Check(
                "rclpy",
                FAIL,
                "rclpy found but its C extension does not match this interpreter",
                f"you are on Python {sys.version_info.major}.{sys.version_info.minor}, "
                f"rclpy was built for {expected_python} (troubleshooting #3)",
            )
        return Check(
            "rclpy",
            FAIL,
            "not importable",
            "source /opt/ros/humble/setup.bash, or rebuild the venv with --system-site-packages",
        )
    return Check("rclpy", PASS, getattr(rclpy, "__file__", "importable"))


def check_ros_gz() -> Check:
    code, output = run_command(["ros2", "pkg", "list"])
    if code != 0:
        return Check("ros_gz", SKIP, "ros2 CLI not available")
    packages = set(output.split())
    if "ros_gz_bridge" in packages:
        return Check("ros_gz", PASS, "ros_gz_bridge (Harmonic)")
    garden = {p for p in packages if p.startswith("ros_gzgarden")}
    if garden:
        return Check(
            "ros_gz",
            WARN,
            f"found {sorted(garden)[0]}, not ros_gz_bridge",
            "Garden is EOL; install ros-humble-ros-gz and remove ros_gzgarden (troubleshooting #2)",
        )
    return Check("ros_gz", FAIL, "ros_gz_bridge missing", "sudo apt install ros-humble-ros-gz")


def check_gazebo(expected_major: str = "8") -> Check:
    for command in (["gz", "sim", "--versions"], ["gz", "--versions"]):
        code, output = run_command(command)
        if code == 0 and output:
            versions = parse_version(output)
            if versions and str(versions[0]) == expected_major:
                return Check("gazebo", PASS, output.splitlines()[0])
            return Check(
                "gazebo",
                WARN,
                output.splitlines()[0],
                f"expected gz-sim {expected_major} (Harmonic)",
            )
    return Check(
        "gazebo", FAIL, "gz not found", "install Gazebo Harmonic and source its setup script"
    )


def px4_directory() -> Path:
    return Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot")).expanduser()


def check_px4(px4_dir: Path, expected: str = "") -> Check:
    if not px4_dir.is_dir():
        return Check(
            "px4",
            FAIL,
            f"not found: {px4_dir}",
            "git clone --recursive https://github.com/PX4/PX4-Autopilot.git",
        )
    code, output = run_command(["git", "-C", str(px4_dir), "describe", "--tags", "--always"])
    if code != 0:
        return Check("px4", WARN, f"{px4_dir} (not a git checkout)", "cannot verify the version")
    if expected and not same_minor(output, expected):
        return Check(
            "px4",
            WARN,
            f"{output} (expected {expected})",
            "record the real version in versions.yaml",
        )
    return Check("px4", PASS, output)


def check_px4_msgs(px4_version: str, expected_branch: str = "") -> Check:
    candidates = [REPO_ROOT / "src" / "px4_msgs", Path.cwd() / "src" / "px4_msgs"]
    workspace = os.environ.get("PX4_ROS2_WS")
    if workspace:
        candidates.insert(0, Path(workspace) / "src" / "px4_msgs")

    found = next((path for path in candidates if path.is_dir()), None)
    if found is None:
        return Check(
            "px4_msgs",
            SKIP,
            "not vendored in this workspace (expected if built elsewhere)",
            "pin it: git clone -b release/1.15 https://github.com/PX4/px4_msgs.git (troubleshooting #1)",
        )

    code, branch = run_command(["git", "-C", str(found), "rev-parse", "--abbrev-ref", "HEAD"])
    if code != 0:
        return Check("px4_msgs", WARN, f"{found} (not a git checkout)", "cannot verify the branch")
    if expected_branch and branch != expected_branch:
        return Check(
            "px4_msgs",
            FAIL,
            f"branch {branch} (expected {expected_branch})",
            "a mismatched px4_msgs breaks the build with missing message headers (troubleshooting #1)",
        )
    if px4_version and not same_minor(branch, px4_version):
        return Check(
            "px4_msgs",
            FAIL,
            f"branch {branch} vs PX4 {px4_version}",
            "px4_msgs and PX4 must share a minor version (troubleshooting #1)",
        )
    return Check("px4_msgs", PASS, branch)


def check_gz_assets(px4_dir: Path, world: str = "target_tracking") -> Check:
    """World file and camera model, which live in a PX4 git submodule."""
    gz_root = px4_dir / "Tools" / "simulation" / "gz"
    world_file = gz_root / "worlds" / f"{world}.sdf"
    model_dir = gz_root / "models" / "x500_mono_cam_down"

    if not (gz_root / "worlds").is_dir():
        return Check(
            "gz assets",
            FAIL,
            f"{gz_root}/worlds is missing or empty",
            'Tools/simulation/gz is a submodule: git -C "$PX4_DIR" submodule update --init --recursive',
        )
    if not world_file.is_file():
        return Check(
            "gz assets",
            FAIL,
            f"missing {world_file}",
            'cp sim/worlds/target_tracking.sdf "$PX4_DIR/Tools/simulation/gz/worlds/"',
        )
    if not model_dir.is_dir():
        return Check(
            "gz assets",
            FAIL,
            "x500_mono_cam_down model missing",
            'submodule is outdated: git -C "$PX4_DIR" submodule update --remote Tools/simulation/gz',
        )
    return Check("gz assets", PASS, f"{world}.sdf + x500_mono_cam_down")


def numpy_abi_problem(numpy_version: str, has_cv_bridge: bool) -> str | None:
    """A numpy 2.x next to a ROS-built cv_bridge segfaults (skills/06)."""
    if not has_cv_bridge:
        return None
    try:
        major = int(str(numpy_version).split(".", 1)[0])
    except ValueError:
        return None
    if major >= 2:
        return f"numpy {numpy_version} with cv_bridge"
    return None


def check_numpy_abi() -> Check:
    try:
        import numpy  # noqa: PLC0415 - the check is the import
    except ImportError:
        return Check("numpy abi", SKIP, "numpy not installed")

    try:
        import cv_bridge  # noqa: F401, PLC0415

        has_cv_bridge = True
    except ImportError:
        has_cv_bridge = False

    if not has_cv_bridge:
        return Check("numpy abi", SKIP, f"numpy {numpy.__version__}, no cv_bridge here")

    problem = numpy_abi_problem(numpy.__version__, True)
    if problem:
        return Check(
            "numpy abi",
            FAIL,
            problem,
            "pin numpy<2: the ROS cv_bridge is built against the 1.x ABI (skills/06)",
        )
    return Check("numpy abi", PASS, f"numpy {numpy.__version__} with cv_bridge")


def check_camera_topic(topic: str = "/camera/image_raw") -> Check:
    code, output = run_command(["ros2", "topic", "list"])
    if code != 0:
        return Check("camera topic", SKIP, "ros2 CLI not available")
    if topic in output.split():
        return Check("camera topic", PASS, topic)
    return Check(
        "camera topic",
        FAIL,
        f"{topic} not published",
        "start the simulation and the bridge, then: gz topic -l | grep image",
    )


def format_report(checks: list[Check]) -> str:
    width = max((len(check.name) for check in checks), default=5)
    lines = [f"{'check'.ljust(width)}  status  detail", "-" * (width + 48)]
    for check in checks:
        lines.append(f"{check.name.ljust(width)}  {check.status:<6}  {check.detail}")
        if check.hint and check.status in (FAIL, WARN):
            lines.append(f"{' ' * width}  {'':<6}  -> {check.hint}")
    return "\n".join(lines)


def collect(versions: dict, static: bool = False) -> list[Check]:
    checks = [
        check_python(str(versions.get("python", "3.10"))),
        check_os(str(versions.get("ubuntu", "22.04"))),
        check_ros(str(versions.get("ros_distro", "humble")), static=static),
        check_rclpy(str(versions.get("python", "3.10")), static=static),
    ]

    if static:
        checks.append(Check("dynamic checks", SKIP, "--static: ROS / Gazebo / PX4 checks skipped"))
        return checks

    px4_dir = px4_directory()
    px4_check = check_px4(px4_dir, str(versions.get("px4", "")))
    checks.extend(
        [
            check_ros_gz(),
            check_gazebo(str(versions.get("gz_sim", "8"))),
            px4_check,
            check_px4_msgs(str(versions.get("px4", "")), str(versions.get("px4_msgs", ""))),
            check_numpy_abi(),
            check_gz_assets(px4_dir),
            check_camera_topic(),
        ]
    )
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--static", action="store_true", help="skip checks that need a running stack"
    )
    parser.add_argument("--versions", default=str(VERSIONS_FILE))
    args = parser.parse_args()

    versions = load_versions(Path(args.versions))
    checks = collect(versions, static=args.static)
    print(format_report(checks))

    failures = [check for check in checks if check.status == FAIL]
    warnings = [check for check in checks if check.status == WARN]
    print(f"\n{len(checks)} checks: {len(failures)} failed, {len(warnings)} warnings")
    if failures:
        print("Fix the failures above before running the simulation.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
