import asyncio
import csv
import json

import pytest

from uav_survey.analysis import load_log
from uav_survey.cli import main
from uav_survey.telemetry import FIELDS, TelemetryRecorder

from fakes import FakeDrone
from test_analysis import straight_line_rows, write_log


def read_rows(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


# ---------------------------------------------------------------- recorder
def test_recorder_stops_after_landing(tmp_path):
    out = tmp_path / "log.csv"
    drone = FakeDrone(battery_values=[88.4], in_air_values=[False, True, True, False], in_air_delay=0.05)
    rec = TelemetryRecorder(out, drone=drone, rate_hz=50, stop_on_land=True)
    rows = asyncio.run(asyncio.wait_for(rec.run(), 5))

    data = read_rows(out)
    assert rows == len(data) > 3
    assert list(data[0].keys()) == FIELDS
    flags = [r["in_air"] for r in data]
    assert "1" in flags and flags[-1] == "0"
    last = data[-1]
    assert float(last["ground_speed_m_s"]) == pytest.approx(5.0)     # hypot(3, 4)
    assert float(last["battery_pct"]) == pytest.approx(88.4)
    assert last["flight_mode"] == "MISSION"
    assert float(last["lat_deg"]) == pytest.approx(47.397742)
    times = [float(r["t_s"]) for r in data]
    assert times == sorted(times)


def test_recorder_respects_duration(tmp_path):
    out = tmp_path / "log.csv"
    rec = TelemetryRecorder(out, drone=FakeDrone(), rate_hz=50)
    rows = asyncio.run(asyncio.wait_for(rec.run(duration_s=0.15), 5))
    assert 2 <= rows <= 10


def test_recorder_rejects_silly_rates(tmp_path):
    with pytest.raises(ValueError):
        TelemetryRecorder(tmp_path / "x.csv", rate_hz=0)
    with pytest.raises(ValueError):
        TelemetryRecorder(tmp_path / "x.csv", rate_hz=500)


def test_recorder_output_loads_in_analysis(tmp_path):
    out = tmp_path / "log.csv"
    rec = TelemetryRecorder(out, drone=FakeDrone(in_air_values=[True]), rate_hz=50)
    asyncio.run(asyncio.wait_for(rec.run(duration_s=0.1), 5))
    samples = load_log(out)
    assert samples and samples[0].battery_pct == 90.0 and samples[0].in_air is True


# --------------------------------------------------------------------- cli
def test_cli_plan_writes_all_files(tmp_path, capsys):
    out = tmp_path / "mission"
    code = main(["plan", "--out-dir", str(out), "--width", "40", "--height", "30", "--spacing", "20", "--altitude", "25"])
    assert code == 0
    for name in ("plan.json", "mission.plan", "waypoints.csv", "path.geojson"):
        assert (out / name).exists(), name
    printed = capsys.readouterr().out
    assert "Waypoints:" in printed and "6 at 25 m" in printed
    assert json.loads((out / "mission.plan").read_text())["fileType"] == "Plan"


def test_cli_plan_uses_config_file_and_flag_overrides(tmp_path):
    cfg = tmp_path / "c.json"
    cfg.write_text('{"altitude_m": 15, "speed_m_s": 3, "width_m": 40, "height_m": 30, "spacing_m": 20}')
    out = tmp_path / "m"
    assert main(["plan", "--config", str(cfg), "--speed", "4", "--out-dir", str(out)]) == 0
    saved = json.loads((out / "plan.json").read_text())["config"]
    assert saved["altitude_m"] == 15 and saved["speed_m_s"] == 4


@pytest.mark.parametrize(
    "argv",
    [
        ["plan", "--altitude", "999"],
        ["plan", "--config", "/definitely/not/here.json"],
        ["plan", "--origin-lat", "123"],
        ["analyze", "--log", "/definitely/not/here.csv"],
    ],
)
def test_cli_bad_input_gives_clean_error(argv, tmp_path, capsys):
    code = main(argv + (["--out-dir", str(tmp_path / "o")] if argv[0] == "plan" else []))
    assert code == 2
    err = capsys.readouterr().err
    assert err.startswith("error:") and "Traceback" not in err


def test_cli_analyze_single_and_multiple_runs(tmp_path, capsys):
    a = write_log(tmp_path / "slow.csv", straight_line_rows(dt=2.0))
    b = write_log(tmp_path / "fast.csv", straight_line_rows(dt=1.0))
    plan_dir = tmp_path / "plan"
    assert main(["plan", "--out-dir", str(plan_dir), "--width", "100", "--height", "1", "--spacing", "100"]) == 0
    capsys.readouterr()

    summary = tmp_path / "summary.csv"
    metrics = tmp_path / "metrics.json"
    plots = tmp_path / "figs"
    code = main([
        "analyze", "--log", str(a), str(b), "--plan", str(plan_dir / "plan.json"),
        "--plot-dir", str(plots), "--summary-csv", str(summary), "--json", str(metrics),
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert "Run: slow" in out and "Comparison" in out and "Deviation from planned path" in out
    assert (plots / "slow.png").stat().st_size > 1000
    assert (plots / "fast.png").exists()
    assert len(read_rows(summary)) == 2
    data = json.loads(metrics.read_text())
    assert data["fast"]["metrics"]["duration_s"] == pytest.approx(10.0)
    assert data["slow"]["deviation"] is not None


def test_cli_analyze_without_plan_has_no_deviation(tmp_path, capsys):
    a = write_log(tmp_path / "solo.csv", straight_line_rows())
    assert main(["analyze", "--log", str(a)]) == 0
    out = capsys.readouterr().out
    assert "Distance flown" in out and "Deviation" not in out and "Comparison" not in out


def test_cli_rejects_a_non_plan_json(tmp_path, capsys):
    not_a_plan = tmp_path / "x.json"
    not_a_plan.write_text('{"hello": "world"}')
    a = write_log(tmp_path / "solo.csv", straight_line_rows())
    assert main(["analyze", "--log", str(a), "--plan", str(not_a_plan)]) == 2
    assert "uav-survey-plan" in capsys.readouterr().err


def test_cli_requires_a_subcommand():
    with pytest.raises(SystemExit) as exc:
        main([])
    assert exc.value.code == 2
