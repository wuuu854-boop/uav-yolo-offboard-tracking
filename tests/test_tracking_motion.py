"""The motion model: a moving box keeps its id, a static one is unaffected."""

from __future__ import annotations

import pytest

from uav_tracking.perception.tracking import IouTracker, Track


def box(cx: float, cy: float = 320.0, size: float = 40.0) -> dict:
    return {"bbox_cx": cx, "bbox_cy": cy, "bbox_w": size, "bbox_h": size}


def test_a_static_target_keeps_its_id_and_zero_velocity():
    tracker = IouTracker()

    first = tracker.update([box(100.0)], 0.0)[0]
    second = tracker.update([box(100.0)], 0.2)[0]

    assert first["track_id"] == second["track_id"] == 1
    track = tracker.active[0]
    assert track.vx == pytest.approx(0.0, abs=1e-9)
    assert track.vy == pytest.approx(0.0, abs=1e-9)


def test_a_moving_target_keeps_its_id_across_a_jump_too_large_for_iou():
    # 40 px box moving 30 px per frame: consecutive boxes overlap by 10 px,
    # IoU ~= 0.14, below the 0.2 threshold. Only the prediction can match them.
    tracker = IouTracker()

    ids = []
    for index in range(8):
        ids.append(tracker.update([box(100.0 + 30.0 * index)], 0.2 * index)[0]["track_id"])

    assert set(ids) == {1}, f"id switched: {ids}"
    assert tracker.active[0].vx == pytest.approx(150.0, rel=0.2)


def test_the_estimated_velocity_has_the_right_sign_and_scale():
    tracker = IouTracker(velocity_alpha=1.0)

    tracker.update([box(100.0)], 0.0)
    tracker.update([box(120.0)], 0.2)

    # 20 px in 0.2 s = 100 px/s
    assert tracker.active[0].vx == pytest.approx(100.0)
    assert tracker.active[0].vy == pytest.approx(0.0, abs=1e-9)


def test_a_missed_frame_does_not_lose_the_track():
    tracker = IouTracker()
    # Two hits establish the velocity before the gap.
    first = tracker.update([box(100.0)], 0.0)[0]["track_id"]
    tracker.update([box(130.0)], 0.2)
    tracker.update([], 0.4)  # nothing detected
    # Where the constant-velocity guess expects it: 130 + 150 * 0.2
    third = tracker.update([box(160.0)], 0.6)[0]["track_id"]

    assert first == third


def test_two_crossing_targets_keep_distinct_ids():
    tracker = IouTracker()

    first = tracker.update([box(100.0, cy=300.0), box(300.0, cy=340.0)], 0.0)
    assert len({detection["track_id"] for detection in first}) == 2
    first_id = next(d["track_id"] for d in first if d["bbox_cx"] < 200)
    second_id = next(d["track_id"] for d in first if d["bbox_cx"] > 200)

    first_x, second_x = 100.0, 300.0
    for step in range(1, 9):
        first_x += 40.0
        second_x -= 40.0
        annotated = tracker.update([box(first_x, cy=300.0), box(second_x, cy=340.0)], 0.2 * step)
        near_first = min(annotated, key=lambda d: abs(d["bbox_cx"] - first_x))
        near_second = min(annotated, key=lambda d: abs(d["bbox_cx"] - second_x))
        assert near_first["track_id"] == first_id
        assert near_second["track_id"] == second_id


def test_prediction_is_capped_after_a_long_gap():
    track = Track(track_id=1, bbox_cx=100.0, bbox_cy=100.0, bbox_w=40.0, bbox_h=40.0, vx=100.0)

    near = track.predicted(0.5, max_extrapolation_s=1.0)
    far = track.predicted(10.0, max_extrapolation_s=1.0)

    assert near["bbox_cx"] == pytest.approx(150.0)
    assert far["bbox_cx"] == pytest.approx(200.0)  # capped at 1 s of motion


def test_a_bad_velocity_alpha_is_rejected():
    with pytest.raises(ValueError, match="velocity_alpha"):
        IouTracker(velocity_alpha=0.0)
