"""Mission configuration with strict validation.

All numeric limits are deliberately conservative so that a typo (for example
an altitude of 300 instead of 30) is rejected before anything is uploaded to
the vehicle.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Dict, Mapping


class ConfigError(ValueError):
    """Raised when a configuration value is missing, unknown or out of range."""


# PX4 SITL + Gazebo default home position (Zurich). Used when no origin is given.
DEFAULT_ORIGIN_LAT = 47.397742
DEFAULT_ORIGIN_LON = 8.545594


@dataclass(frozen=True)
class SurveyConfig:
    # --- survey geometry ---
    width_m: float = 80.0          # extent towards the east before rotation
    height_m: float = 60.0         # extent towards the north before rotation
    spacing_m: float = 20.0        # distance between adjacent sweep lines
    heading_deg: float = 0.0       # clockwise rotation of the grid from north
    # --- flight ---
    altitude_m: float = 30.0       # relative to home
    speed_m_s: float = 5.0
    acceptance_radius_m: float = 2.0
    # --- safety ---
    min_start_battery_pct: float = 60.0   # refuse to take off below this
    min_battery_pct: float = 25.0         # abort and return to launch below this
    mission_timeout_s: float = 900.0      # abort and return to launch after this long
    landing_timeout_s: float = 180.0      # how long to wait for touchdown after the mission
    # --- link ---
    connection: str = "udp://:14540"

    def validate(self) -> "SurveyConfig":
        _in_range("width_m", self.width_m, 1.0, 2000.0)
        _in_range("height_m", self.height_m, 1.0, 2000.0)
        _in_range("spacing_m", self.spacing_m, 1.0, 500.0)
        if self.spacing_m > max(self.width_m, self.height_m):
            raise ConfigError(
                f"spacing_m ({self.spacing_m}) larger than both survey dimensions "
                f"({self.width_m} x {self.height_m}); no sweep would be produced"
            )
        _in_range("heading_deg", self.heading_deg, 0.0, 360.0, upper_inclusive=False)
        _in_range("altitude_m", self.altitude_m, 2.0, 120.0)
        _in_range("speed_m_s", self.speed_m_s, 0.5, 15.0)
        _in_range("acceptance_radius_m", self.acceptance_radius_m, 0.5, 20.0)
        _in_range("min_start_battery_pct", self.min_start_battery_pct, 1.0, 100.0)
        _in_range("min_battery_pct", self.min_battery_pct, 1.0, 99.0)
        if self.min_start_battery_pct <= self.min_battery_pct:
            raise ConfigError(
                "min_start_battery_pct must be greater than min_battery_pct, "
                "otherwise the mission would abort immediately after take-off"
            )
        _in_range("mission_timeout_s", self.mission_timeout_s, 10.0, 24 * 3600.0)
        _in_range("landing_timeout_s", self.landing_timeout_s, 5.0, 3600.0)
        if not isinstance(self.connection, str) or "://" not in self.connection:
            raise ConfigError(f"connection must look like 'udp://:14540', got {self.connection!r}")
        return self

    # ------------------------------------------------------------------ io
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SurveyConfig":
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ConfigError(f"unknown configuration keys: {', '.join(unknown)}")
        try:
            cfg = cls(**dict(data))
        except TypeError as exc:  # pragma: no cover - defensive
            raise ConfigError(str(exc)) from exc
        return cfg.validate()

    @classmethod
    def from_json_file(cls, path: "str | Path") -> "SurveyConfig":
        try:
            raw = json.loads(Path(path).read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ConfigError(f"config file not found: {path}") from exc
        except json.JSONDecodeError as exc:
            raise ConfigError(f"config file {path} is not valid JSON: {exc}") from exc
        if not isinstance(raw, dict):
            raise ConfigError(f"config file {path} must contain a JSON object")
        return cls.from_dict(raw)

    def with_overrides(self, **overrides: Any) -> "SurveyConfig":
        """Return a validated copy with the non-None overrides applied."""
        merged = self.to_dict()
        merged.update({k: v for k, v in overrides.items() if v is not None})
        return SurveyConfig.from_dict(merged)


def _in_range(name: str, value: Any, low: float, high: float, upper_inclusive: bool = True) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number, got {value!r}")
    if math.isnan(value) or math.isinf(value):
        raise ConfigError(f"{name} must be finite, got {value!r}")
    too_high = value > high if upper_inclusive else value >= high
    if value < low or too_high:
        bracket = "]" if upper_inclusive else ")"
        raise ConfigError(f"{name}={value} is outside the allowed range [{low}, {high}{bracket}")


def validate_origin(lat: float, lon: float) -> None:
    if not (-90.0 <= lat <= 90.0):
        raise ConfigError(f"origin latitude {lat} outside [-90, 90]")
    if not (-180.0 <= lon <= 180.0):
        raise ConfigError(f"origin longitude {lon} outside [-180, 180]")
    if abs(lat) > 85.0:
        raise ConfigError("origin latitude too close to a pole for the flat-earth approximation")
