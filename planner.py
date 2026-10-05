"""Turns a :class:`SurveyConfig` plus an origin into a concrete survey plan."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

from .config import SurveyConfig, validate_origin
from .geometry import (
    lawnmower,
    offset_to_latlon,
    path_length_m,
    rotate_clockwise,
)


@dataclass(frozen=True)
class Waypoint:
    index: int
    lat_deg: float
    lon_deg: float
    alt_m: float          # relative to home
    east_m: float         # local offset from the origin
    north_m: float


@dataclass(frozen=True)
class SurveyPlan:
    config: SurveyConfig
    origin_lat_deg: float
    origin_lon_deg: float
    waypoints: Tuple[Waypoint, ...]

    # ------------------------------------------------------------ metrics
    @property
    def local_points(self) -> List[Tuple[float, float]]:
        return [(w.east_m, w.north_m) for w in self.waypoints]

    @property
    def survey_distance_m(self) -> float:
        """Length of the sweep path itself (first to last waypoint)."""
        return path_length_m(self.local_points)

    @property
    def total_distance_m(self) -> float:
        """Home -> survey path -> home, i.e. what the vehicle actually flies."""
        route = [(0.0, 0.0)] + self.local_points + [(0.0, 0.0)]
        return path_length_m(route)

    @property
    def estimated_duration_s(self) -> float:
        """Rough flight-time estimate: cruise distance / speed plus climb and descent.

        Ignores acceleration and wind, so treat it as a lower bound.
        """
        cruise = self.total_distance_m / self.config.speed_m_s
        vertical = 2 * self.config.altitude_m / min(self.config.speed_m_s, 3.0)
        return cruise + vertical

    @property
    def area_m2(self) -> float:
        return self.config.width_m * self.config.height_m

    @property
    def sweep_lines(self) -> int:
        return len(self.waypoints) // 2

    def reference_polyline(self) -> List[Tuple[float, float]]:
        """Home -> waypoints -> home in local metres, for cross-track analysis."""
        return [(0.0, 0.0)] + self.local_points + [(0.0, 0.0)]

    # --------------------------------------------------------------- i/o
    def to_dict(self) -> Dict[str, Any]:
        return {
            "format": "uav-survey-plan",
            "version": 1,
            "origin": {"lat_deg": self.origin_lat_deg, "lon_deg": self.origin_lon_deg},
            "config": self.config.to_dict(),
            "waypoints": [
                {
                    "index": w.index,
                    "lat_deg": w.lat_deg,
                    "lon_deg": w.lon_deg,
                    "alt_m": w.alt_m,
                    "east_m": w.east_m,
                    "north_m": w.north_m,
                }
                for w in self.waypoints
            ],
            "summary": {
                "sweep_lines": self.sweep_lines,
                "survey_distance_m": round(self.survey_distance_m, 2),
                "total_distance_m": round(self.total_distance_m, 2),
                "estimated_duration_s": round(self.estimated_duration_s, 1),
                "area_m2": self.area_m2,
            },
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SurveyPlan":
        if data.get("format") != "uav-survey-plan":
            raise ValueError("not a uav-survey-plan file")
        cfg = SurveyConfig.from_dict(data["config"])
        wps = tuple(
            Waypoint(
                index=int(w["index"]),
                lat_deg=float(w["lat_deg"]),
                lon_deg=float(w["lon_deg"]),
                alt_m=float(w["alt_m"]),
                east_m=float(w["east_m"]),
                north_m=float(w["north_m"]),
            )
            for w in data["waypoints"]
        )
        origin = data["origin"]
        return cls(cfg, float(origin["lat_deg"]), float(origin["lon_deg"]), wps)

    def save(self, path: "str | Path") -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return out

    @classmethod
    def load(cls, path: "str | Path") -> "SurveyPlan":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def build_plan(config: SurveyConfig, origin_lat: float, origin_lon: float) -> SurveyPlan:
    """Generate the survey plan for ``config`` starting at the given origin.

    The grid's south-west corner sits on the origin (home) and is rotated
    clockwise by ``config.heading_deg``.
    """
    config.validate()
    validate_origin(origin_lat, origin_lon)

    grid = lawnmower(config.width_m, config.height_m, config.spacing_m)
    waypoints: List[Waypoint] = []
    for i, p in enumerate(grid):
        east, north = rotate_clockwise(p, config.heading_deg)
        east, north = _clean(east), _clean(north)
        lat, lon = offset_to_latlon(origin_lat, origin_lon, east, north)
        waypoints.append(Waypoint(i, lat, lon, config.altitude_m, east, north))
    return SurveyPlan(config, origin_lat, origin_lon, tuple(waypoints))


def _clean(value: float) -> float:
    """Remove floating-point dust such as 6e-15 after rotation."""
    return 0.0 if math.isclose(value, 0.0, abs_tol=1e-9) else round(value, 9)
