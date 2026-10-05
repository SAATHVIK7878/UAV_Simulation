"""Small-area geometry helpers (pure functions, no third-party dependencies).

A local East/North frame (metres) around an origin is used for survey areas of
up to a few kilometres, where a flat-earth approximation is accurate to well
under a metre per hundred metres.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

EARTH_RADIUS_M = 6_371_000.0
METRES_PER_DEG_LAT = math.pi * EARTH_RADIUS_M / 180.0  # ~111 195 m

Point = Tuple[float, float]  # (east_m, north_m)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two lat/lon points in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def offset_to_latlon(lat0: float, lon0: float, east_m: float, north_m: float) -> Point:
    """Convert a local (east, north) offset to (lat, lon)."""
    dlat = north_m / METRES_PER_DEG_LAT
    dlon = east_m / (METRES_PER_DEG_LAT * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def latlon_to_offset(lat0: float, lon0: float, lat: float, lon: float) -> Point:
    """Inverse of :func:`offset_to_latlon`; returns (east_m, north_m)."""
    north = (lat - lat0) * METRES_PER_DEG_LAT
    east = (lon - lon0) * METRES_PER_DEG_LAT * math.cos(math.radians(lat0))
    return east, north


def rotate_clockwise(point: Point, heading_deg: float) -> Point:
    """Rotate an (east, north) point clockwise (compass sense) about the origin."""
    theta = math.radians(heading_deg)
    x, y = point
    return (
        x * math.cos(theta) + y * math.sin(theta),
        -x * math.sin(theta) + y * math.cos(theta),
    )


def sweep_positions(extent_m: float, spacing_m: float) -> List[float]:
    """Positions of sweep lines from 0 to ``extent_m``.

    Lines are placed every ``spacing_m``; if the extent is not an exact
    multiple, a final line is added at ``extent_m`` so the far edge is covered.
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")
    if extent_m < 0:
        raise ValueError("extent_m must be non-negative")
    positions: List[float] = []
    i = 0
    while i * spacing_m <= extent_m + 1e-9:
        positions.append(round(i * spacing_m, 9))
        i += 1
    if extent_m - positions[-1] > 1e-6:
        positions.append(extent_m)
    return positions


def lawnmower(width_m: float, height_m: float, spacing_m: float) -> List[Point]:
    """Back-and-forth (boustrophedon) path covering a width x height rectangle.

    Sweep lines run north-south and are spaced along the east axis, starting at
    the south-west corner (0, 0).
    """
    points: List[Point] = []
    for i, east in enumerate(sweep_positions(width_m, spacing_m)):
        norths = (0.0, height_m) if i % 2 == 0 else (height_m, 0.0)
        for north in norths:
            points.append((east, north))
    return points


def path_length_m(points: Sequence[Point]) -> float:
    """Total length of a polyline in local metres."""
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Shortest distance from point ``p`` to the segment ``a``-``b``."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return math.dist(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def distance_to_polyline(p: Point, polyline: Sequence[Point]) -> float:
    """Shortest distance from ``p`` to any segment of ``polyline``."""
    if len(polyline) == 0:
        raise ValueError("polyline is empty")
    if len(polyline) == 1:
        return math.dist(p, polyline[0])
    return min(point_segment_distance(p, a, b) for a, b in zip(polyline, polyline[1:]))


def pairwise(items: Iterable[Point]) -> List[Tuple[Point, Point]]:
    seq = list(items)
    return list(zip(seq, seq[1:]))
