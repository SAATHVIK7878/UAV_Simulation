"""Command-line interface: ``uav-survey plan | fly | log | analyze``."""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence

from . import __version__
from .analysis import (
    RunSummary,
    format_comparison,
    format_report,
    load_log,
    summarize_run,
    write_summary_csv,
)
from .config import DEFAULT_ORIGIN_LAT, DEFAULT_ORIGIN_LON, SurveyConfig
from .errors import UavError
from .export import to_geojson, to_qgc_plan, write_json, write_waypoints_csv
from .mission import MissionRunner
from .planner import SurveyPlan, build_plan
from .telemetry import TelemetryRecorder

log = logging.getLogger("uav_survey")

# (flag, config field, help) for options that override the configuration file.
_CONFIG_FLAGS = [
    ("--width", "width_m", "survey width towards the east, metres"),
    ("--height", "height_m", "survey height towards the north, metres"),
    ("--spacing", "spacing_m", "distance between sweep lines, metres"),
    ("--heading", "heading_deg", "rotate the grid clockwise from north, degrees"),
    ("--altitude", "altitude_m", "altitude above home, metres"),
    ("--speed", "speed_m_s", "cruise speed, m/s"),
    ("--min-battery", "min_battery_pct", "return to launch below this battery level, percent"),
    ("--min-start-battery", "min_start_battery_pct", "refuse to take off below this battery level, percent"),
    ("--timeout", "mission_timeout_s", "return to launch if the mission takes longer than this, seconds"),
]


# ------------------------------------------------------------------ parser
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uav-survey",
        description="Plan, fly, log and analyse survey missions on PX4 SITL.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug-level logging")
    parser.add_argument("--log-file", help="also write log messages to this file")
    sub = parser.add_subparsers(dest="command", required=True)

    p_plan = sub.add_parser("plan", help="generate a survey plan and export it (no vehicle needed)")
    _add_config_args(p_plan)
    p_plan.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT, help="home latitude")
    p_plan.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON, help="home longitude")
    p_plan.add_argument("--out-dir", default="mission_out", help="where to write the plan files")
    p_plan.set_defaults(func=cmd_plan)

    p_fly = sub.add_parser("fly", help="upload and fly a survey mission with safety failsafes")
    _add_config_args(p_fly)
    p_fly.add_argument("--connection", help="MAVSDK address (default udp://:14540)")
    p_fly.add_argument("--plan", help="fly this plan.json instead of generating one at the vehicle's position")
    p_fly.add_argument("--out-dir", help="save the flown plan and result here (default runs/<timestamp>)")
    p_fly.add_argument("--connect-timeout", type=float, default=30.0, help="seconds to wait for the vehicle")
    p_fly.add_argument("--no-battery-check", action="store_true",
                       help="fly even if the battery level is unavailable (disables the low-battery failsafe)")
    p_fly.set_defaults(func=cmd_fly)

    p_log = sub.add_parser("log", help="record telemetry to CSV")
    p_log.add_argument("--out", default="flight_log.csv", help="output CSV path")
    p_log.add_argument("--connection", default="udp://:14540", help="MAVSDK address")
    p_log.add_argument("--rate", type=float, default=2.0, help="samples per second (default 2)")
    p_log.add_argument("--duration", type=float, help="stop after this many seconds")
    p_log.add_argument("--stop-on-land", action="store_true", help="stop once the vehicle lands after flying")
    p_log.set_defaults(func=cmd_log)

    p_an = sub.add_parser("analyze", help="compute metrics and figures from one or more logs")
    p_an.add_argument("--log", nargs="+", required=True, help="telemetry CSV file(s)")
    p_an.add_argument("--plan", help="plan.json, to measure deviation from the planned path")
    p_an.add_argument("--plot-dir", help="save a flight figure (PNG) per log into this folder")
    p_an.add_argument("--summary-csv", help="write a one-row-per-run comparison table")
    p_an.add_argument("--json", dest="json_out", help="write all metrics as JSON")
    p_an.set_defaults(func=cmd_analyze)
    return parser


def _add_config_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", help="JSON configuration file (see configs/default.json)")
    for flag, dest, help_text in _CONFIG_FLAGS:
        parser.add_argument(flag, dest=dest, type=float, help=help_text)


def _resolve_config(args: argparse.Namespace, base: Optional[SurveyConfig] = None) -> SurveyConfig:
    if base is None:
        base = SurveyConfig.from_json_file(args.config) if args.config else SurveyConfig()
    overrides = {dest: getattr(args, dest) for _, dest, _ in _CONFIG_FLAGS}
    overrides["connection"] = getattr(args, "connection", None)
    return base.with_overrides(**overrides)


# ---------------------------------------------------------------- commands
def cmd_plan(args: argparse.Namespace) -> int:
    config = _resolve_config(args)
    plan = build_plan(config, args.origin_lat, args.origin_lon)
    out = Path(args.out_dir)
    _save_plan_files(plan, out)
    print(_plan_summary(plan))
    print(f"\nFiles written to {out}/:  plan.json  mission.plan  waypoints.csv  path.geojson")
    return 0


def cmd_fly(args: argparse.Namespace) -> int:
    plan = SurveyPlan.load(args.plan) if args.plan else None
    base = plan.config if (plan is not None and not args.config) else None
    config = _resolve_config(args, base)
    out = Path(args.out_dir) if args.out_dir else Path("runs") / datetime.now().strftime("%Y%m%d-%H%M%S")

    runner = MissionRunner(config, require_battery=not args.no_battery_check)

    async def go():
        await runner.connect(args.connect_timeout)
        return await runner.run(plan)

    result = None
    try:
        result = asyncio.run(go())
    finally:
        if runner.last_plan is not None:
            _save_plan_files(runner.last_plan, out)
        if result is not None:
            write_json(result.to_dict(), out / "result.json")

    print(f"\nMission {result.status.value}: {result.detail}")
    print(f"  waypoints {result.waypoints_reached}/{result.waypoints_total}, "
          f"{result.duration_s:.0f} s, landed={result.landed}")
    print(f"  plan and result saved to {out}/")
    return 0 if result.success else 1


def cmd_log(args: argparse.Namespace) -> int:
    recorder = TelemetryRecorder(
        args.out, connection=args.connection, rate_hz=args.rate, stop_on_land=args.stop_on_land,
    )
    try:
        rows = asyncio.run(recorder.run(args.duration))
    except KeyboardInterrupt:
        print(f"\nstopped; samples so far are saved in {args.out}")
        return 0
    print(f"wrote {rows} samples to {args.out}")
    return 0


def cmd_analyze(args: argparse.Namespace) -> int:
    plan = SurveyPlan.load(args.plan) if args.plan else None
    summaries: List[RunSummary] = []
    for path in args.log:
        summary = summarize_run(path, plan)
        summaries.append(summary)
        print(format_report(summary))
        print()
        if args.plot_dir:
            from .plotting import plot_flight  # imported lazily: matplotlib is optional

            png = plot_flight(load_log(path), Path(args.plot_dir) / f"{summary.label}.png",
                              plan=plan, title=summary.label)
            print(f"  figure: {png}\n")

    if len(summaries) > 1:
        print("Comparison")
        print(format_comparison(summaries))
    if args.summary_csv:
        print(f"\nsummary table: {write_summary_csv(summaries, args.summary_csv)}")
    if args.json_out:
        data = {s.label: {"metrics": s.metrics.to_dict(),
                          "deviation": None if s.deviation is None else s.deviation.to_dict()}
                for s in summaries}
        write_json(data, args.json_out)
        print(f"metrics JSON: {args.json_out}")
    return 0


# ----------------------------------------------------------------- helpers
def _save_plan_files(plan: SurveyPlan, out: Path) -> None:
    plan.save(out / "plan.json")
    write_json(to_qgc_plan(plan), out / "mission.plan")
    write_waypoints_csv(plan, out / "waypoints.csv")
    write_json(to_geojson(plan), out / "path.geojson")


def _plan_summary(plan: SurveyPlan) -> str:
    c = plan.config
    return "\n".join([
        f"Survey area:        {c.width_m:g} x {c.height_m:g} m ({plan.area_m2:,.0f} m2), grid heading {c.heading_deg:g} deg",
        f"Sweep lines:        {plan.sweep_lines} (spacing {c.spacing_m:g} m)",
        f"Waypoints:          {len(plan.waypoints)} at {c.altitude_m:g} m, {c.speed_m_s:g} m/s",
        f"Survey path:        {plan.survey_distance_m:,.0f} m",
        f"Total with return:  {plan.total_distance_m:,.0f} m",
        f"Estimated time:     {plan.estimated_duration_s / 60:.1f} min (lower bound; ignores wind and acceleration)",
    ])


def _setup_logging(verbose: bool, log_file: Optional[str]) -> None:
    handlers: List[logging.Handler] = [logging.StreamHandler(sys.stderr)]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers,
        force=True,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    _setup_logging(args.verbose, args.log_file)
    try:
        return args.func(args)
    except (ValueError, FileNotFoundError, json.JSONDecodeError) as exc:
        # bad configuration, bad input file or bad log format
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except UavError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
