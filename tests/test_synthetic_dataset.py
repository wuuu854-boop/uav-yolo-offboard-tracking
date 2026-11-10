"""The synthetic dataset generator: geometry and label sanity."""

from __future__ import annotations

import math
import random

import pytest

PIL = pytest.importorskip("PIL")
np = pytest.importorskip("numpy")

from scripts.make_synthetic_dataset import _square, make_sample  # noqa: E402


def test_a_square_has_four_corners_at_the_right_distance():
    corners = _square((100, 100), side=40, angle_deg=0.0)
    assert len(corners) == 4
    for x, y in corners:
        assert math.hypot(x - 100, y - 100) == pytest.approx(math.hypot(20, 20))


def test_rotation_does_not_move_the_centre():
    corners = _square((100, 100), side=40, angle_deg=25.0)
    centre_x = sum(point[0] for point in corners) / 4
    centre_y = sum(point[1] for point in corners) / 4
    assert centre_x == pytest.approx(100.0)
    assert centre_y == pytest.approx(100.0)


def test_a_sample_has_boxes_inside_the_image():
    rng = random.Random(3)
    image, boxes = make_sample(rng, size=640)

    assert image.size == (640, 640)
    assert 1 <= len(boxes) <= 2
    for x_min, y_min, x_max, y_max in boxes:
        assert 0.0 <= x_min < x_max <= 1.0
        assert 0.0 <= y_min < y_max <= 1.0
        # a real window, not a degenerate line
        assert (x_max - x_min) * 640 >= 20
        assert (y_max - y_min) * 640 >= 20


def test_the_pixels_are_not_uniform():
    rng = random.Random(11)
    image, boxes = make_sample(rng, size=320)
    array = np.asarray(image)

    assert boxes
    assert array.std() > 5.0  # noise plus the coloured square
