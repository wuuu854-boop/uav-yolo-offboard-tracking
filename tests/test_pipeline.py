"""The pipeline as a whole: QA, shot sheet and features agree on one capture.

The three components have their own tests. This one exists because they are sold
— and trusted — as a single pass: it runs all three over the same generated
capture and checks that they tell the same story.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from scripts.export_geojson import build_features  # noqa: E402
from scripts.qa_report import analyse_capture  # noqa: E402
from scripts.shot_sheet_check import analyse_shots  # noqa: E402

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
# The tiny camera has a coarse GSD at 2 m, so the sheet in this test relaxes it;
# the real one lives in config/shot_sheet.yaml.
SHEET = {
    "views": [
        {"name": "nadir", "camera": "down", "max_gsd_cm": 3.0, "min_frames": 3, "margin_px": 5}
    ]
}
TARGET = (1.0, 0.0)
ALTITUDE = 2.0


def _capture(root, frames: int = 6) -> None:
    root.mkdir(parents=True, exist_ok=True)
    u = CAMERA["cx"]
    v = CAMERA["cy"] - CAMERA["fy"] * (TARGET[0] / ALTITUDE)
    poses = []
    for index in range(frames):
        frame = np.full((120, 160, 3), 200, dtype=np.uint8)
        cv2.rectangle(frame, (int(u) - 4, int(v) - 4), (int(u) + 4, int(v) + 4), (20, 20, 210), -1)
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


def test_all_three_components_agree_on_a_good_capture(tmp_path):
    capture = tmp_path / "capture"
    _capture(capture)
    targets = [TARGET]

    qa = analyse_capture(capture, CAMERA, DETECTOR, targets)
    shots = analyse_shots(capture, SHEET, CAMERA, DETECTOR, targets)
    features = build_features(capture, CAMERA, DETECTOR, targets, 52.52, 13.40)

    assert qa["passed"], qa["failures"]
    assert shots["passed"], shots["rejected"]
    assert len(features) == 6
    assert all(feature["properties"]["asset"] == 0 for feature in features)
    assert all("association_confidence" in feature["properties"] for feature in features)
    assert all(feature["properties"]["association_m"] < 0.3 for feature in features)


def test_a_moved_asset_fails_qa_but_still_produces_features(tmp_path):
    capture = tmp_path / "capture"
    _capture(capture)

    qa = analyse_capture(capture, CAMERA, DETECTOR, [(3.0, 0.0)])
    features = build_features(capture, CAMERA, DETECTOR, [(3.0, 0.0)], 52.52, 13.40)

    assert not qa["passed"]
    # The deliverable is still produced; the verdict is what says not to trust it.
    assert len(features) == 6
    assert all(feature["properties"]["association_m"] > 1.0 for feature in features)


def test_the_shot_sheet_rejects_a_capture_that_never_leaves_the_ground(tmp_path):
    capture = tmp_path / "low"
    capture.mkdir()
    frame = np.full((120, 160, 3), 200, dtype=np.uint8)
    cv2.rectangle(frame, (2, 50), (18, 66), (20, 20, 210), -1)  # clipped by the left edge
    poses = []
    for index in range(4):
        name = f"frame_{index:02d}.jpg"
        cv2.imwrite(str(capture / name), frame)
        poses.append(
            {
                "file": name,
                "east": 0.0,
                "north": 0.0,
                "altitude": 0.3,
                "yaw": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "wall_time": 100.0 + 0.5 * index,
            }
        )
    (capture / "poses.jsonl").write_text(
        "\n".join(json.dumps(pose) for pose in poses) + "\n", encoding="utf-8"
    )

    shots = analyse_shots(capture, SHEET, CAMERA, DETECTOR, [TARGET])

    # 0.3 m altitude is 0.3 cm/px, which satisfies the relaxed GSD, but the
    # projection puts the target at the frame edge, so framing rejects it.
    assert not shots["passed"]
    assert shots["rejected"]
