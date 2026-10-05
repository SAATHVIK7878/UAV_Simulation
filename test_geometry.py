import math

import pytest

from uav_survey import geometry as g


def test_haversine_one_degree_of_latitude():
    assert g.haversine_m(0, 0, 1, 0) == pytest.approx(111_195, rel=1e-3)


def test_haversine_zero_distance():
    assert g.haversine_m(47.4, 8.5, 47.4, 8.5) == 0.0


def test_offset_roundtrip():
    lat, lon = g.offset_to_latlon(47.397742, 8.545594, 123.4, -56.7)
    east, north = g.latlon_to_offset(47.397742, 8.545594, lat, lon)
    assert east == pytest.approx(123.4, abs=1e-6)
    assert north == pytest.approx(-56.7, abs=1e-6)


def test_offset_matches_haversine_for_small_areas():
    lat, lon = g.offset_to_latlon(47.397742, 8.545594, 300.0, 400.0)
    assert g.haversine_m(47.397742, 8.545594, lat, lon) == pytest.approx(500.0, rel=2e-3)


def test_rotate_clockwise_quarter_turn_sends_east_to_south():
    x, y = g.rotate_clockwise((1.0, 0.0), 90.0)
    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(-1.0)


def test_rotate_clockwise_sends_north_to_east():
    x, y = g.rotate_clockwise((0.0, 1.0), 90.0)
    assert (x, y) == (pytest.approx(1.0), pytest.approx(0.0, abs=1e-12))


def test_rotate_preserves_length():
    p = g.rotate_clockwise((30.0, 40.0), 37.0)
    assert math.hypot(*p) == pytest.approx(50.0)


@pytest.mark.parametrize(
    "extent, spacing, expected",
    [
        (80, 20, [0, 20, 40, 60, 80]),
        (70, 20, [0, 20, 40, 60, 70]),   # far edge still covered
        (10, 20, [0, 10]),
        (0, 5, [0]),
    ],
)
def test_sweep_positions(extent, spacing, expected):
    assert g.sweep_positions(extent, spacing) == pytest.approx(expected)


def test_sweep_positions_rejects_bad_spacing():
    with pytest.raises(ValueError):
        g.sweep_positions(10, 0)


def test_lawnmower_alternates_direction():
    pts = g.lawnmower(40, 30, 20)
    assert pts == [(0, 0), (0, 30), (20, 30), (20, 0), (40, 0), (40, 30)]


def test_lawnmower_path_length():
    # 3 lines of 30 m plus 2 connectors of 20 m
    assert g.path_length_m(g.lawnmower(40, 30, 20)) == pytest.approx(130.0)


def test_point_segment_distance_interior_and_endpoints():
    a, b = (0.0, 0.0), (10.0, 0.0)
    assert g.point_segment_distance((5.0, 3.0), a, b) == pytest.approx(3.0)
    assert g.point_segment_distance((-4.0, 3.0), a, b) == pytest.approx(5.0)
    assert g.point_segment_distance((13.0, 4.0), a, b) == pytest.approx(5.0)


def test_point_segment_distance_degenerate_segment():
    assert g.point_segment_distance((3.0, 4.0), (0.0, 0.0), (0.0, 0.0)) == pytest.approx(5.0)


def test_distance_to_polyline_picks_nearest_segment():
    poly = [(0, 0), (10, 0), (10, 10)]
    assert g.distance_to_polyline((9.0, 6.0), poly) == pytest.approx(1.0)


def test_distance_to_polyline_empty_raises():
    with pytest.raises(ValueError):
        g.distance_to_polyline((0, 0), [])
