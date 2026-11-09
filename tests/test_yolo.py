"""YOLO decoding: letterbox geometry, box mapping back to the frame, NMS."""

from __future__ import annotations

import numpy as np
import pytest

from src.perception.yolo import decode, letterbox


def _output(boxes: list[tuple[float, float, float, float]], class_scores: list[list[float]]):
    """Build a YOLO-shaped tensor: (1, 4 + num_classes, num_boxes)."""
    columns = np.array(boxes, dtype=float).T  # 4 x N
    scores = np.array(class_scores, dtype=float).T  # C x N
    return np.concatenate([columns, scores], axis=0)[None, ...]


def test_letterbox_keeps_a_square_image_untouched():
    image = np.zeros((640, 640, 3), dtype=np.uint8)
    canvas, scale, left, top = letterbox(image, 640)
    assert canvas.shape == (640, 640, 3)
    assert scale == pytest.approx(1.0)
    assert (left, top) == (0, 0)


def test_letterbox_pads_a_wide_image():
    image = np.zeros((480, 1280, 3), dtype=np.uint8)
    canvas, scale, left, top = letterbox(image, 640)
    assert canvas.shape == (640, 640, 3)
    assert scale == pytest.approx(0.5)
    assert left == 0
    assert top == 200  # (640 - 240) / 2


def test_letterbox_rejects_a_non_bgr_frame():
    with pytest.raises(ValueError):
        letterbox(np.zeros((480, 640), dtype=np.uint8), 640)


def test_decode_maps_boxes_back_through_the_letterbox():
    # One box centred at (320, 320) in the padded 640 square, image scaled by 0.5
    # and pushed down by 200 px -> centre (640, 240) in the original frame.
    output = _output([(320.0, 320.0, 64.0, 32.0)], [[0.9, 0.0, 0.0]])
    detections = decode(output, scale=0.5, left=0, top=200, classes=(0,))

    assert len(detections) == 1
    detection = detections[0]
    assert detection["bbox_cx"] == pytest.approx(640.0)
    assert detection["bbox_cy"] == pytest.approx(240.0)
    assert detection["bbox_w"] == pytest.approx(128.0)
    assert detection["bbox_h"] == pytest.approx(64.0)
    assert detection["confidence"] == pytest.approx(0.9)


def test_decode_drops_low_confidence_boxes():
    output = _output([(320.0, 320.0, 10.0, 10.0)], [[0.2, 0.0, 0.0]])
    assert decode(output, conf_threshold=0.35, classes=(0,)) == []


def test_decode_filters_by_class():
    # High score, but in class 1 while only class 0 is requested.
    output = _output([(320.0, 320.0, 10.0, 10.0)], [[0.1, 0.95, 0.0]])
    assert decode(output, classes=(0,)) == []


def test_decode_suppresses_overlapping_boxes():
    output = _output(
        [(320.0, 320.0, 100.0, 100.0), (322.0, 318.0, 100.0, 100.0)],
        [[0.9, 0.0, 0.0], [0.8, 0.0, 0.0]],
    )
    detections = decode(output, iou_threshold=0.45, classes=(0,))
    assert len(detections) == 1
    assert detections[0]["confidence"] == pytest.approx(0.9)


def test_decode_keeps_separate_boxes_and_sorts_by_confidence():
    output = _output(
        [(100.0, 100.0, 20.0, 20.0), (500.0, 500.0, 20.0, 20.0)],
        [[0.6, 0.0, 0.0], [0.95, 0.0, 0.0]],
    )
    detections = decode(output, classes=(0,))
    assert [round(d["confidence"], 2) for d in detections] == [0.95, 0.6]


def test_decode_respects_max_detections():
    boxes = [(100.0 + 50 * i, 100.0, 10.0, 10.0) for i in range(5)]
    scores = [[0.9 - 0.05 * i, 0.0, 0.0] for i in range(5)]
    detections = decode(_output(boxes, scores), classes=(0,), max_detections=2)
    assert len(detections) == 2


def test_decode_accepts_an_unbatched_tensor():
    output = _output([(320.0, 320.0, 10.0, 10.0)], [[0.9, 0.0, 0.0]])[0]
    assert len(decode(output, classes=(0,))) == 1


def test_decode_rejects_a_tensor_without_enough_classes():
    output = _output([(320.0, 320.0, 10.0, 10.0)], [[0.9]])
    with pytest.raises(ValueError):
        decode(output, classes=(5,))


def test_decode_rejects_a_non_positive_scale():
    with pytest.raises(ValueError):
        decode(np.zeros((1, 7, 2), dtype=float), scale=0.0)
