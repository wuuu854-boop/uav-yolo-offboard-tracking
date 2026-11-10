"""The local-ENU to WGS84 conversion used by the GeoJSON export."""

from __future__ import annotations

import math

import pytest

from scripts.export_geojson import enu_to_lonlat

LAT, LON = 52.52, 13.40


def test_the_origin_maps_to_itself():
    assert enu_to_lonlat(0.0, 0.0, LAT, LON) == pytest.approx((LON, LAT))


def test_north_moves_latitude_by_the_right_amount():
    _lon, lat = enu_to_lonlat(0.0, 1113.2, LAT, LON)  # 1113.2 m north

    assert lat == pytest.approx(LAT + 0.01, rel=1e-6)


def test_east_moves_longitude_scaled_by_latitude():
    lon, _lat = enu_to_lonlat(1000.0, 0.0, LAT, LON)

    expected = LON + 1000.0 / (111_320.0 * math.cos(math.radians(LAT)))
    assert lon == pytest.approx(expected, rel=1e-9)


def test_a_positive_offset_moves_east_and_north_not_west_or_south():
    lon, lat = enu_to_lonlat(100.0, 100.0, LAT, LON)

    assert lon > LON
    assert lat > LAT


def test_a_bad_origin_is_rejected():
    with pytest.raises(ValueError, match="origin_lat"):
        enu_to_lonlat(0.0, 0.0, 95.0, LON)
