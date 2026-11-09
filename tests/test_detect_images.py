"""Batch image evaluation: the part that works without a simulator."""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from scripts.detect_images import evaluate, list_images, summarise, write_csv  # noqa: E402


def _write_image(path, green_square: bool) -> None:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    if green_square:
        cv2.rectangle(frame, (250, 180), (390, 300), (0, 255, 0), thickness=-1)
    cv2.imwrite(str(path), frame)


def _baseline(frame: np.ndarray) -> list[dict]:
    return [
        {"confidence": 1.0, "bbox_cx": 320.0, "bbox_cy": 240.0, "bbox_w": 140.0, "bbox_h": 120.0}
    ]


def test_list_images_finds_supported_suffixes_only(tmp_path):
    (tmp_path / "a.jpg").write_bytes(b"")
    (tmp_path / "b.PNG").write_bytes(b"")
    (tmp_path / "notes.txt").write_text("x", encoding="utf-8")
    (tmp_path / "nested").mkdir()
    (tmp_path / "nested" / "c.png").write_bytes(b"")

    assert [path.name for path in list_images(tmp_path)] == ["a.jpg", "b.PNG", "c.png"]


def test_evaluate_records_one_row_per_image(tmp_path):
    images = []
    for index, has_square in enumerate((True, False)):
        path = tmp_path / f"frame_{index}.png"
        _write_image(path, has_square)
        images.append(path)

    rows = evaluate(images, _baseline)

    assert [row["image"] for row in rows] == ["frame_0.png", "frame_1.png"]
    assert [row["detections"] for row in rows] == [1, 1]
    assert rows[0]["cx"] == 320.0
    assert isinstance(rows[0]["latency_ms"], float)


def test_unreadable_image_is_reported_not_skipped(tmp_path):
    broken = tmp_path / "broken.png"
    broken.write_text("not an image", encoding="utf-8")

    rows = evaluate([broken], _baseline)

    assert rows[0]["detections"] == -1


def test_summarise_counts_detections_and_latency(tmp_path):
    rows = evaluate([], _baseline)
    assert rows == []

    image = tmp_path / "one.png"
    _write_image(image, True)
    summary = summarise(evaluate([image], _baseline))
    assert "1 images" in summary
    assert "100%" in summary


def test_write_csv_round_trip(tmp_path):
    path = tmp_path / "out" / "rows.csv"
    write_csv(path, [{"image": "a.png", "detections": 1, "latency_ms": 2.5, "cx": 1.0, "cy": 2.0}])

    text = path.read_text(encoding="utf-8").splitlines()
    assert text[0] == "image,detections,latency_ms,cx,cy"
    assert text[1].startswith("a.png,1,2.5")


def test_write_csv_rejects_an_empty_run(tmp_path):
    with pytest.raises(ValueError):
        write_csv(tmp_path / "x.csv", [])
