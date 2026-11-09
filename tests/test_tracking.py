"""Tracking: identity across frames, and what happens when a frame is missed."""

from __future__ import annotations

import pytest

from uav_tracking.perception.tracking import IouTracker, Track, iou


def _box(cx: float, cy: float, w: float = 40.0, h: float = 40.0) -> dict:
    return {"bbox_cx": cx, "bbox_cy": cy, "bbox_w": w, "bbox_h": h}


def test_iou_of_identical_boxes_is_one():
    assert iou(_box(10, 10), _box(10, 10)) == pytest.approx(1.0)


def test_iou_of_disjoint_boxes_is_zero():
    assert iou(_box(0, 0), _box(200, 200)) == 0.0


def test_iou_of_half_overlap_is_a_third():
    # Two 40x40 boxes offset by 20 px overlap in half the area.
    assert iou(_box(10, 10), _box(30, 10)) == pytest.approx(1 / 3, abs=1e-6)


def test_a_static_target_keeps_its_id():
    tracker = IouTracker()
    first = tracker.update([_box(100, 100)], now_s=0.0)
    second = tracker.update([_box(101, 99)], now_s=0.2)

    assert first[0]["track_id"] == second[0]["track_id"]
    assert len(tracker.active) == 1


def test_a_target_that_moves_slowly_keeps_its_id():
    tracker = IouTracker()
    tracker.update([_box(100, 100)], now_s=0.0)
    moved = tracker.update([_box(115, 100)], now_s=0.2)  # 15 px on a 40 px box

    assert moved[0]["track_id"] == 1


def test_two_targets_get_two_ids_and_do_not_swap():
    tracker = IouTracker()
    first = tracker.update([_box(100, 100), _box(500, 100)], now_s=0.0)
    second = tracker.update([_box(500, 100), _box(100, 100)], now_s=0.2)

    by_position = {(d["bbox_cx"], d["bbox_cy"]): d["track_id"] for d in second}
    assert by_position[(100.0, 100.0)] == first[0]["track_id"]
    assert by_position[(500.0, 100.0)] == first[1]["track_id"]
    assert first[0]["track_id"] != first[1]["track_id"]


def test_one_missed_frame_does_not_create_a_new_id():
    tracker = IouTracker(max_age_s=1.5)
    tracker.update([_box(100, 100)], now_s=0.0)
    assert tracker.update([], now_s=0.4) == []

    again = tracker.update([_box(100, 100)], now_s=0.6)
    assert again[0]["track_id"] == 1


def test_a_track_older_than_max_age_is_dropped_and_replaced():
    tracker = IouTracker(max_age_s=1.0)
    tracker.update([_box(100, 100)], now_s=0.0)

    later = tracker.update([_box(100, 100)], now_s=2.0)
    assert later[0]["track_id"] == 2
    assert len(tracker.active) == 1


def test_an_empty_frame_returns_no_detections():
    tracker = IouTracker()
    assert tracker.update([], now_s=0.0) == []


def test_the_input_order_is_preserved():
    tracker = IouTracker()
    result = tracker.update([_box(100, 100), _box(500, 100)], now_s=0.0)
    assert [d["bbox_cx"] for d in result] == [100.0, 500.0]


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError):
        IouTracker(iou_threshold=1.5)
    with pytest.raises(ValueError):
        IouTracker(max_age_s=0.0)


def test_a_track_renders_itself_as_a_detection():
    track = Track(track_id=7, bbox_cx=1.0, bbox_cy=2.0, bbox_w=3.0, bbox_h=4.0)
    assert track.as_detection()["track_id"] == 7
    assert track.as_detection()["bbox_cx"] == 1.0
