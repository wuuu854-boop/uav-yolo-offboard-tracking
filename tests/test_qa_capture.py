"""The QA metrics, exercised end to end on a generated capture.

The flight recordings are too big for the repository, but the metrics are worth
protecting: this builds a six-frame capture whose geometry is known exactly, so a
change in the projection, the detector or the phase logic shows up as a failing
test rather than a surprise months later.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from scripts.qa_report import analyse_capture  # noqa: E402

# A tiny camera so the fixture frames fit in a few kilobytes and the geometry is
# easy to reason about: 160x120, 100 px focal length, principal point in the
# middle.
CAMERA = {
    "fx": 100.0,
    "fy": 100.0,
    "cx": 80.0,
    "cy": 60.0,
    "image_width": 160,
    "image_height": 120,
    "image_up_is_forward": True,
    "mount_offset_xyz": [0.0, 0.0, 0.0],
}
DETECTOR = {
    "baseline_hsv_ranges": [[[0, 120, 70], [10, 255, 255]], [[170, 120, 70], [180, 255, 255]]],
    "baseline_min_area_px": 40,
}
ALTITUDE = 2.0


def _write_capture(root, target_east: float = 1.0) -> float:
    """Six level frames with a red square where ``target_east`` projects."""
    root.mkdir(parents=True, exist_ok=True)
    poses = []
    # Facing east (yaw 0): forward is east, so a target at +east sits at the top.
    u = CAMERA["cx"] + CAMERA["fx"] * 0.0
    v = CAMERA["cy"] - CAMERA["fy"] * (target_east / ALTITUDE)
    for index in range(6):
        frame = np.full((120, 160, 3), 200, dtype=np.uint8)
        cv2.rectangle(frame, (int(u) - 8, int(v) - 8), (int(u) + 8, int(v) + 8), (20, 20, 210), -1)
        name = f"frame_{index:02d}.jpg"
        cv2.imwrite(str(root / name), frame)
        poses.append(
            {
                "file": name,
                "east": 0.0,
                "north": 0.0,
                "altitude": ALTITUDE,
                "yaw": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "wall_time": 100.0 + 0.5 * index,
            }
        )
    (root / "poses.jsonl").write_text(
        "\n".join(json.dumps(pose) for pose in poses) + "\n", encoding="utf-8"
    )
    return u


def test_a_clean_capture_passes_and_lands_on_the_asset(tmp_path):
    _write_capture(tmp_path / "capture")

    report = analyse_capture(tmp_path / "capture", CAMERA, DETECTOR, [(1.0, 0.0)])

    assert report["passed"], report["failures"]
    assert report["metrics"]["detections"] == 6
    assert report["metrics"]["association_m"]["median"] < 0.3
    assert report["metrics"]["track_ids"] == 1
    assert report["metrics"]["phases"]["level"]["frames"] == 6


def test_the_same_capture_fails_when_the_asset_is_declared_elsewhere(tmp_path):
    _write_capture(tmp_path / "capture")

    report = analyse_capture(tmp_path / "capture", CAMERA, DETECTOR, [(2.5, 0.0)])

    assert not report["passed"]
    assert any("association" in reason for reason in report["failures"])
    # The detections are still where they were; only the declared asset moved.
    assert report["metrics"]["association_m"]["median"] > 1.0


def test_a_capture_without_detections_fails_on_every_metric(tmp_path):
    capture = tmp_path / "empty"
    capture.mkdir()
    frame = np.full((120, 160, 3), 200, dtype=np.uint8)  # no red anywhere
    poses = []
    for index in range(4):
        name = f"frame_{index:02d}.jpg"
        cv2.imwrite(str(capture / name), frame)
        poses.append(
            {
                "file": name,
                "east": 0.0,
                "north": 0.0,
                "altitude": ALTITUDE,
                "yaw": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "wall_time": 100.0 + 0.5 * index,
            }
        )
    (capture / "poses.jsonl").write_text(
        "\n".join(json.dumps(pose) for pose in poses) + "\n", encoding="utf-8"
    )

    report = analyse_capture(capture, CAMERA, DETECTOR, [(1.0, 0.0)])

    assert not report["passed"]
    assert report["metrics"]["detections"] == 0
