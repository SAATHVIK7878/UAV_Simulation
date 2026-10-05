import csv
import math

import pytest

from uav_survey.analysis import (
    LogFormatError,
    compute_metrics,
    format_comparison,
    format_report,
    load_log,
    path_deviation,
    summarize_run,
    trim_to_flight,
    write_summary_csv,
)
from uav_survey.config import SurveyConfig
from uav_survey.geometry import offset_to_latlon
from uav_survey.planner import build_plan
from uav_survey.telemetry import FIELDS

LAT, LON = 47.397742, 8.545594


def write_log(path, rows):
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({**{k: "" for k in FIELDS}, **row})
    return path


def straight_line_rows(n=11, step_m=10.0, dt=2.0, offset_north=0.0):
    """Fly due east at 5 m/s, 30 m up, battery draining 1% per sample."""
    rows = []
    for i in range(n):
        lat, lon = offset_to_latlon(LAT, LON, i * step_m, offset_north)
        rows.append({
            "t_s": i * dt, "lat_deg": f"{lat:.8f}", "lon_deg": f"{lon:.8f}",
            "rel_alt_m": 30.0, "ground_speed_m_s": 5.0,
            "battery_pct": 100 - i, "in_air": 1, "flight_mode": "MISSION",
        })
    return rows


def test_load_and_metrics(tmp_path):
    samples = load_log(write_log(tmp_path / "a.csv", straight_line_rows()))
    m = compute_metrics(samples)
    assert m.samples == 11
    assert m.duration_s == pytest.approx(20.0)
    assert m.distance_m == pytest.approx(100.0, rel=1e-3)
    assert m.mean_speed_m_s == pytest.approx(5.0, rel=1e-3)
    assert m.max_speed_m_s == 5.0
    assert m.max_alt_m == 30.0
    assert m.battery_used_pct == pytest.approx(10.0)


def test_metrics_ignore_time_on_the_ground(tmp_path):
    ground = {"t_s": 0, "lat_deg": f"{LAT:.8f}", "lon_deg": f"{LON:.8f}", "rel_alt_m": 0, "in_air": 0, "battery_pct": 100}
    rows = [ground, {**ground, "t_s": 5}] + [
        {**r, "t_s": r["t_s"] + 10} for r in straight_line_rows()
    ] + [{**ground, "t_s": 60}]
    samples = load_log(write_log(tmp_path / "g.csv", rows))
    assert len(trim_to_flight(samples)) == 11
    assert compute_metrics(samples).duration_s == pytest.approx(20.0)


def test_metrics_without_optional_columns(tmp_path):
    rows = [{k: r[k] for k in ("t_s", "lat_deg", "lon_deg", "rel_alt_m")} for r in straight_line_rows()]
    m = compute_metrics(load_log(write_log(tmp_path / "min.csv", rows)))
    assert m.battery_used_pct is None and m.max_speed_m_s is None
    assert "Battery used" not in format_report(summarize_run(tmp_path / "min.csv"))


def test_rows_without_position_fix_are_skipped(tmp_path):
    rows = [{"t_s": 0, "rel_alt_m": 0}] + straight_line_rows()
    assert len(load_log(write_log(tmp_path / "nofix.csv", rows))) == 11


def test_bad_logs_rejected(tmp_path):
    with pytest.raises(LogFormatError, match="not found"):
        load_log(tmp_path / "missing.csv")

    no_cols = tmp_path / "nocols.csv"
    no_cols.write_text("a,b\n1,2\n")
    with pytest.raises(LogFormatError, match="missing required"):
        load_log(no_cols)

    bad_val = tmp_path / "badval.csv"
    bad_val.write_text("t_s,lat_deg,lon_deg,rel_alt_m\n0,abc,8.5,1\n")
    with pytest.raises(LogFormatError, match=r"badval.csv:2"):
        load_log(bad_val)

    one = write_log(tmp_path / "one.csv", straight_line_rows(n=1))
    with pytest.raises(LogFormatError, match="at least 2"):
        compute_metrics(load_log(one))

    frozen = [{**r, "t_s": 1.0} for r in straight_line_rows()]
    with pytest.raises(LogFormatError, match="timestamps"):
        compute_metrics(load_log(write_log(tmp_path / "frozen.csv", frozen)))


def make_plan():
    return build_plan(SurveyConfig(width_m=100, height_m=1.0, spacing_m=100), LAT, LON)


def test_path_deviation_zero_on_path_and_exact_when_offset(tmp_path):
    # plan with width 100, height 1: waypoints (0,0) (0,1) (100,1) (100,0); reference also returns home.
    plan = make_plan()
    on_path = load_log(write_log(tmp_path / "on.csv", straight_line_rows(offset_north=0.0)))
    # the straight east line along north=0 lies on the return leg (100,0) -> (0,0)
    assert path_deviation(on_path, plan).max_m == pytest.approx(0.0, abs=1e-3)

    off = load_log(write_log(tmp_path / "off.csv", straight_line_rows(offset_north=-4.0)))
    d = path_deviation(off, plan)
    assert d.mean_m == pytest.approx(4.0, rel=1e-3)
    assert d.rms_m == pytest.approx(4.0, rel=1e-3)
    assert d.p95_m == pytest.approx(4.0, rel=1e-3)
    assert d.max_m == pytest.approx(4.0, rel=1e-3)


def test_summaries_report_and_csv(tmp_path):
    plan = make_plan()
    a = summarize_run(write_log(tmp_path / "run_a.csv", straight_line_rows()), plan)
    b = summarize_run(write_log(tmp_path / "run_b.csv", straight_line_rows(dt=1.0)), plan)
    assert a.label == "run_a"
    assert "Deviation from planned path" in format_report(a)

    table = format_comparison([a, b])
    assert "run_a" in table and "run_b" in table and "dev_mean_m" in table

    out = write_summary_csv([a, b], tmp_path / "out" / "summary.csv")
    with out.open() as fh:
        rows = list(csv.DictReader(fh))
    assert [r["run"] for r in rows] == ["run_a", "run_b"]
    assert float(rows[1]["mean_speed_m_s"]) == pytest.approx(10.0, rel=1e-3)
    assert "deviation_mean_m" in rows[0]


def test_summary_csv_needs_runs(tmp_path):
    with pytest.raises(ValueError):
        write_summary_csv([], tmp_path / "x.csv")


def test_percentile_interpolates():
    from uav_survey.analysis import _percentile

    assert _percentile([1.0, 2.0, 3.0, 4.0, 5.0], 50) == 3.0
    assert _percentile([0.0, 10.0], 95) == pytest.approx(9.5)
    assert math.isclose(_percentile([7.0], 95), 7.0)
