"""The per-detection association confidence."""

from __future__ import annotations

import math

import pytest

from uav_tracking.control.qa import association_confidence


def test_a_well_supported_detection_on_the_asset_is_certain():
    assert association_confidence(0.0, track_hits=3) == pytest.approx(1.0)


def test_a_single_frame_blip_is_worth_less():
    assert association_confidence(0.0, track_hits=1) == pytest.approx(1.0 / 3.0)


def test_one_sigma_is_the_expected_ratio():
    assert association_confidence(0.5, track_hits=3) == pytest.approx(math.exp(-0.5))


def test_distance_decays_smoothly_and_never_hides_a_near_miss():
    near = association_confidence(0.5, track_hits=3)
    far = association_confidence(2.0, track_hits=3)

    assert 0.0 < far < near < 1.0
    assert far == pytest.approx(math.exp(-8.0), rel=1e-6)


def test_more_hits_stop_helping_after_the_support_threshold():
    assert association_confidence(0.2, track_hits=10) == pytest.approx(
        association_confidence(0.2, track_hits=3)
    )


def test_bad_inputs_are_rejected():
    with pytest.raises(ValueError, match="distance_m"):
        association_confidence(-1.0)
    with pytest.raises(ValueError, match="sigma_m"):
        association_confidence(1.0, sigma_m=0.0)
