"""Target selection: which box does the mission actually follow?"""

from __future__ import annotations

import pytest

from uav_tracking.control.selection import STRATEGIES, select_target

IMAGE = (1280, 960)


def _box(u: float, v: float, confidence: float) -> dict:
    return {"confidence": confidence, "bbox_cx": u, "bbox_cy": v}


def test_nothing_to_select_returns_none():
    assert select_target([]) is None


def test_nearest_centre_ignores_a_more_confident_edge_detection():
    edge = _box(20, 20, 0.95)
    middle = _box(650, 500, 0.60)
    assert select_target([edge, middle], "nearest_centre", IMAGE) is middle


def test_nearest_previous_keeps_the_same_object_between_frames():
    left = _box(200, 400, 0.55)
    right = _box(1000, 400, 0.90)
    chosen = select_target([left, right], "nearest_previous", IMAGE, previous=(210, 405))
    assert chosen is left


def test_nearest_previous_falls_back_to_confidence_before_the_first_fix():
    left = _box(200, 400, 0.55)
    right = _box(1000, 400, 0.90)
    assert select_target([left, right], "nearest_previous", IMAGE, previous=None) is right


def test_highest_confidence_is_still_available():
    edge = _box(20, 20, 0.95)
    middle = _box(650, 500, 0.60)
    assert select_target([edge, middle], "highest_confidence", IMAGE) is edge


def test_single_detection_is_returned_by_every_strategy():
    only = _box(100, 100, 0.4)
    for strategy in STRATEGIES:
        assert select_target([only], strategy, IMAGE, previous=(0, 0)) is only


def test_unknown_strategy_is_rejected():
    with pytest.raises(ValueError):
        select_target([_box(1, 1, 0.5)], "biggest", IMAGE)


def test_nearest_centre_requires_an_image_size():
    with pytest.raises(ValueError):
        select_target([_box(1, 1, 0.5)], "nearest_centre")


def test_tracked_follows_the_locked_id_even_if_another_is_better_looking():
    locked = {**_box(900, 100, 0.4), "track_id": 7}
    tempting = {**_box(650, 480, 0.95), "track_id": 8}

    chosen = select_target([locked, tempting], "tracked", IMAGE, locked_id=7)

    assert chosen is locked


def test_tracked_falls_back_to_the_centre_when_the_lock_is_gone():
    other = {**_box(650, 480, 0.6), "track_id": 9}

    chosen = select_target([other], "tracked", IMAGE, locked_id=7)

    assert chosen is other


def test_tracked_falls_back_to_confidence_without_a_frame_size():
    best = {**_box(900, 900, 0.9), "track_id": 3}
    worse = {**_box(10, 10, 0.2), "track_id": 4}

    assert select_target([best, worse], "tracked", locked_id=None) is best


def test_tracked_ignores_detections_without_ids():
    no_id = _box(650, 480, 0.8)
    assert select_target([no_id], "tracked", IMAGE, locked_id=1) is no_id
