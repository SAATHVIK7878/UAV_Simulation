"""Figures for the paper: flown path vs plan, and the altitude/speed profile."""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

from .analysis import Sample, trim_to_flight
from .errors import UavError
from .geometry import latlon_to_offset
from .planner import SurveyPlan

# Okabe-Ito colours: distinguishable with common colour-vision deficiencies.
_PLANNED = "#0072B2"
_FLOWN = "#D55E00"
_HOME = "#009E73"


def plot_flight(
    samples: Sequence[Sample],
    out_path: "str | Path",
    plan: Optional[SurveyPlan] = None,
    title: str = "UAV survey flight",
    dpi: int = 200,
) -> Path:
    """Save a two-panel figure (top-down path, altitude/speed vs time)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise UavError("plotting needs matplotlib:  pip install matplotlib") from exc

    flight = trim_to_flight(samples)
    if len(flight) < 2:
        raise UavError("not enough airborne samples to plot")

    if plan is not None:
        lat0, lon0 = plan.origin_lat_deg, plan.origin_lon_deg
    else:
        lat0, lon0 = flight[0].lat_deg, flight[0].lon_deg
    track = [latlon_to_offset(lat0, lon0, s.lat_deg, s.lon_deg) for s in flight]
    t0 = flight[0].t_s

    fig, (ax_map, ax_prof) = plt.subplots(1, 2, figsize=(11, 4.8), gridspec_kw={"width_ratios": [1.1, 1]})

    if plan is not None:
        ref = plan.reference_polyline()
        ax_map.plot([p[0] for p in ref], [p[1] for p in ref], "--", color=_PLANNED, linewidth=1.6, label="planned path")
        ax_map.scatter([w.east_m for w in plan.waypoints], [w.north_m for w in plan.waypoints],
                       s=22, color=_PLANNED, zorder=3)
    ax_map.plot([p[0] for p in track], [p[1] for p in track], "-", color=_FLOWN, linewidth=1.4, label="flown path")
    ax_map.scatter([0], [0], marker="*", s=130, color=_HOME, zorder=4, label="home")
    ax_map.set_xlabel("East of home (m)")
    ax_map.set_ylabel("North of home (m)")
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.grid(True, alpha=0.3)
    ax_map.legend(loc="best", fontsize=8)
    ax_map.set_title("Top-down path")

    times = [s.t_s - t0 for s in flight]
    ax_prof.plot(times, [s.rel_alt_m for s in flight], color=_FLOWN, linewidth=1.4)
    ax_prof.set_xlabel("Time since take-off (s)")
    ax_prof.set_ylabel("Altitude above home (m)", color=_FLOWN)
    ax_prof.grid(True, alpha=0.3)
    ax_prof.set_title("Altitude and ground speed")

    speeds = [(s.t_s - t0, s.ground_speed_m_s) for s in flight if s.ground_speed_m_s is not None]
    if speeds:
        ax_speed = ax_prof.twinx()
        ax_speed.plot([t for t, _ in speeds], [v for _, v in speeds], color=_PLANNED, linewidth=1.0, alpha=0.9)
        ax_speed.set_ylabel("Ground speed (m/s)", color=_PLANNED)

    fig.suptitle(title)
    fig.tight_layout()
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=dpi)
    plt.close(fig)
    return out
