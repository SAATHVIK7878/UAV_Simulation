"""Flight-log analysis: load a telemetry CSV, compute metrics, compare runs.

Everything here uses only the standard library so it runs anywhere; plotting
lives in :mod:`uav_survey.plotting` and needs matplotlib.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

from .geometry import distance_to_polyline, haversine_m, latlon_to_offset
from .planner import SurveyPlan

REQUIRED_COLUMNS = ("t_s", "lat_deg", "lon_deg", "rel_alt_m")


class LogFormatError(ValueError):
    """The telemetry CSV is missing columns, has bad values, or is too short."""


@dataclass(frozen=True)
class Sample:
    t_s: float
    lat_deg: float
    lon_deg: float
    rel_alt_m: float
    ground_speed_m_s: Optional[float] = None
    battery_pct: Optional[float] = None
    in_air: Optional[bool] = None
    flight_mode: str = ""


@dataclass(frozen=True)
class FlightMetrics:
    samples: int
    duration_s: float
    distance_m: float
    max_alt_m: float
    mean_speed_m_s: Optional[float]
    max_speed_m_s: Optional[float]
    battery_start_pct: Optional[float]
    battery_end_pct: Optional[float]

    @property
    def battery_used_pct(self) -> Optional[float]:
        if self.battery_start_pct is None or self.battery_end_pct is None:
            return None
        return self.battery_start_pct - self.battery_end_pct

    def to_dict(self) -> Dict[str, Optional[float]]:
        return {
            "samples": self.samples,
            "duration_s": _r(self.duration_s, 1),
            "distance_m": _r(self.distance_m, 1),
            "max_alt_m": _r(self.max_alt_m, 1),
            "mean_speed_m_s": _r(self.mean_speed_m_s, 2),
            "max_speed_m_s": _r(self.max_speed_m_s, 2),
            "battery_start_pct": _r(self.battery_start_pct, 1),
            "battery_end_pct": _r(self.battery_end_pct, 1),
            "battery_used_pct": _r(self.battery_used_pct, 1),
        }


@dataclass(frozen=True)
class PathDeviation:
    """Horizontal distance between the flown track and the planned path."""

    samples: int
    mean_m: float
    rms_m: float
    p95_m: float
    max_m: float

    def to_dict(self) -> Dict[str, float]:
        return {
            "samples": self.samples,
            "mean_m": round(self.mean_m, 2),
            "rms_m": round(self.rms_m, 2),
            "p95_m": round(self.p95_m, 2),
            "max_m": round(self.max_m, 2),
        }


@dataclass(frozen=True)
class RunSummary:
    label: str
    metrics: FlightMetrics
    deviation: Optional[PathDeviation]

    def to_row(self) -> Dict[str, object]:
        row: Dict[str, object] = {"run": self.label}
        row.update(self.metrics.to_dict())
        if self.deviation is not None:
            row.update({f"deviation_{k}": v for k, v in self.deviation.to_dict().items() if k != "samples"})
        return row


# ------------------------------------------------------------------ loading
def load_log(path: "str | Path") -> List[Sample]:
    """Read a telemetry CSV written by :class:`~uav_survey.telemetry.TelemetryRecorder`."""
    path = Path(path)
    try:
        fh = path.open("r", newline="", encoding="utf-8")
    except FileNotFoundError as exc:
        raise LogFormatError(f"log file not found: {path}") from exc

    with fh:
        reader = csv.DictReader(fh)
        header = reader.fieldnames or []
        missing = [c for c in REQUIRED_COLUMNS if c not in header]
        if missing:
            raise LogFormatError(f"{path} is missing required column(s): {', '.join(missing)}")

        samples: List[Sample] = []
        for line_no, row in enumerate(reader, start=2):
            if not (row.get("lat_deg") or "").strip() or not (row.get("lon_deg") or "").strip():
                continue  # no position fix yet
            try:
                samples.append(
                    Sample(
                        t_s=float(row["t_s"]),
                        lat_deg=float(row["lat_deg"]),
                        lon_deg=float(row["lon_deg"]),
                        rel_alt_m=float(row["rel_alt_m"]),
                        ground_speed_m_s=_opt_float(row.get("ground_speed_m_s")),
                        battery_pct=_opt_float(row.get("battery_pct")),
                        in_air=_opt_bool(row.get("in_air")),
                        flight_mode=(row.get("flight_mode") or "").strip(),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise LogFormatError(f"{path}:{line_no}: bad value ({exc})") from exc
    return samples


def trim_to_flight(samples: Sequence[Sample]) -> List[Sample]:
    """Keep only the span between the first and last sample marked ``in_air``.

    If the log has no ``in_air`` information the samples are returned unchanged.
    """
    airborne = [i for i, s in enumerate(samples) if s.in_air]
    if not airborne:
        return list(samples)
    return list(samples[airborne[0]: airborne[-1] + 1])


# ------------------------------------------------------------------ metrics
def compute_metrics(samples: Sequence[Sample]) -> FlightMetrics:
    """Distance, duration, speed and battery use over the airborne part of a log."""
    flight = trim_to_flight(samples)
    if len(flight) < 2:
        raise LogFormatError("need at least 2 airborne samples to compute metrics")

    distance = sum(
        haversine_m(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg)
        for a, b in zip(flight, flight[1:])
    )
    duration = flight[-1].t_s - flight[0].t_s
    if duration <= 0:
        raise LogFormatError("timestamps do not increase; cannot compute metrics")

    speeds = [s.ground_speed_m_s for s in flight if s.ground_speed_m_s is not None]
    batteries = [s.battery_pct for s in flight if s.battery_pct is not None]
    return FlightMetrics(
        samples=len(flight),
        duration_s=duration,
        distance_m=distance,
        max_alt_m=max(s.rel_alt_m for s in flight),
        mean_speed_m_s=distance / duration,
        max_speed_m_s=max(speeds) if speeds else None,
        battery_start_pct=batteries[0] if batteries else None,
        battery_end_pct=batteries[-1] if batteries else None,
    )


def path_deviation(samples: Sequence[Sample], plan: SurveyPlan) -> PathDeviation:
    """How far the vehicle strayed (horizontally) from the planned path.

    The reference path is home -> waypoints -> home, so the return leg is
    included. Altitude is ignored. This measures distance to the path, not
    timing along it.
    """
    flight = trim_to_flight(samples)
    if not flight:
        raise LogFormatError("no samples to compare against the plan")
    reference = plan.reference_polyline()
    errors = sorted(
        distance_to_polyline(
            latlon_to_offset(plan.origin_lat_deg, plan.origin_lon_deg, s.lat_deg, s.lon_deg),
            reference,
        )
        for s in flight
    )
    return PathDeviation(
        samples=len(errors),
        mean_m=sum(errors) / len(errors),
        rms_m=math.sqrt(sum(e * e for e in errors) / len(errors)),
        p95_m=_percentile(errors, 95.0),
        max_m=errors[-1],
    )


def summarize_run(path: "str | Path", plan: Optional[SurveyPlan] = None) -> RunSummary:
    samples = load_log(path)
    metrics = compute_metrics(samples)
    deviation = path_deviation(samples, plan) if plan is not None else None
    return RunSummary(Path(path).stem, metrics, deviation)


# ---------------------------------------------------------------- reporting
def format_report(summary: RunSummary) -> str:
    m = summary.metrics
    lines = [
        f"Run: {summary.label}",
        f"  Samples (airborne):  {m.samples}",
        f"  Flight duration:     {m.duration_s:.1f} s",
        f"  Distance flown:      {m.distance_m:.1f} m",
        f"  Mean ground speed:   {m.mean_speed_m_s:.2f} m/s",
        f"  Max altitude:        {m.max_alt_m:.1f} m",
    ]
    if m.max_speed_m_s is not None:
        lines.append(f"  Max ground speed:    {m.max_speed_m_s:.2f} m/s")
    if m.battery_used_pct is not None:
        lines.append(
            f"  Battery used:        {m.battery_used_pct:.1f} % "
            f"({m.battery_start_pct:.0f}% -> {m.battery_end_pct:.0f}%)"
        )
    d = summary.deviation
    if d is not None:
        lines += [
            "  Deviation from planned path (horizontal):",
            f"    mean {d.mean_m:.2f} m   rms {d.rms_m:.2f} m   p95 {d.p95_m:.2f} m   max {d.max_m:.2f} m",
        ]
    return "\n".join(lines)


def format_comparison(summaries: Sequence[RunSummary]) -> str:
    """Plain-text table, one run per row."""
    has_dev = any(s.deviation is not None for s in summaries)
    headers = ["run", "time_s", "dist_m", "speed_m/s", "max_alt_m", "batt_used_%"]
    if has_dev:
        headers += ["dev_mean_m", "dev_max_m"]
    rows: List[List[str]] = []
    for s in summaries:
        m = s.metrics
        row = [
            s.label,
            f"{m.duration_s:.1f}",
            f"{m.distance_m:.1f}",
            f"{m.mean_speed_m_s:.2f}",
            f"{m.max_alt_m:.1f}",
            "n/a" if m.battery_used_pct is None else f"{m.battery_used_pct:.1f}",
        ]
        if has_dev:
            d = s.deviation
            row += ["n/a", "n/a"] if d is None else [f"{d.mean_m:.2f}", f"{d.max_m:.2f}"]
        rows.append(row)
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    fmt = "  ".join(f"{{:<{w}}}" for w in widths)
    out = [fmt.format(*headers), fmt.format(*("-" * w for w in widths))]
    out += [fmt.format(*r) for r in rows]
    return "\n".join(out)


def write_summary_csv(summaries: Iterable[RunSummary], path: "str | Path") -> Path:
    rows = [s.to_row() for s in summaries]
    if not rows:
        raise ValueError("no runs to write")
    columns: List[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    return out


# ------------------------------------------------------------------ helpers
def _opt_float(text: Optional[str]) -> Optional[float]:
    if text is None or not text.strip():
        return None
    value = float(text)
    return None if math.isnan(value) else value


def _opt_bool(text: Optional[str]) -> Optional[bool]:
    if text is None or not text.strip():
        return None
    return text.strip().lower() in {"1", "true", "yes"}


def _percentile(sorted_values: Sequence[float], q: float) -> float:
    """Linear-interpolated percentile of an already sorted sequence."""
    if not sorted_values:
        raise ValueError("empty sequence")
    pos = (len(sorted_values) - 1) * q / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return sorted_values[lo]
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (pos - lo)


def _r(value: Optional[float], digits: int) -> Optional[float]:
    return None if value is None else round(value, digits)
