import csv
import json
import math

import pytest

from uav_survey.config import ConfigError, SurveyConfig
from uav_survey.export import to_geojson, to_qgc_plan, write_waypoints_csv
from uav_survey.geometry import haversine_m
from uav_survey.planner import SurveyPlan, build_plan

LAT, LON = 47.397742, 8.545594


def make_plan(**overrides):
    cfg = SurveyConfig(width_m=40, height_m=30, spacing_m=20, **overrides)
    return build_plan(cfg, LAT, LON)


def test_waypoint_count_and_altitude():
    plan = make_plan(altitude_m=25)
    assert len(plan.waypoints) == 6          # 3 sweep lines x 2 ends
    assert plan.sweep_lines == 3
    assert {w.alt_m for w in plan.waypoints} == {25}


def test_first_waypoint_is_home():
    first = make_plan().waypoints[0]
    assert (first.east_m, first.north_m) == (0.0, 0.0)
    assert (first.lat_deg, first.lon_deg) == (LAT, LON)


def test_distances():
    plan = make_plan()
    assert plan.survey_distance_m == pytest.approx(130.0)
    # survey path ends at (40, 30)... last waypoint is (40, 30); return leg is 50 m
    assert plan.total_distance_m == pytest.approx(130.0 + 50.0)


def test_estimated_duration_scales_with_speed():
    slow = make_plan(speed_m_s=2.0)
    fast = make_plan(speed_m_s=8.0)
    assert slow.estimated_duration_s > fast.estimated_duration_s


def test_rotation_preserves_distances_and_geometry():
    base = make_plan(heading_deg=0)
    rotated = make_plan(heading_deg=35)
    assert rotated.survey_distance_m == pytest.approx(base.survey_distance_m)
    assert rotated.total_distance_m == pytest.approx(base.total_distance_m)
    assert (rotated.waypoints[0].east_m, rotated.waypoints[0].north_m) == (0.0, 0.0)


def test_waypoint_lat_lon_matches_local_offsets():
    plan = make_plan(heading_deg=20)
    for w in plan.waypoints:
        assert haversine_m(LAT, LON, w.lat_deg, w.lon_deg) == pytest.approx(math.hypot(w.east_m, w.north_m), rel=2e-3, abs=1e-6)


def test_invalid_inputs_rejected():
    with pytest.raises(ConfigError):
        build_plan(SurveyConfig(), 95.0, 0.0)
    with pytest.raises(ConfigError):
        build_plan(SurveyConfig(altitude_m=500.0), LAT, LON)


def test_plan_json_roundtrip(tmp_path):
    plan = make_plan(heading_deg=12)
    path = plan.save(tmp_path / "sub" / "plan.json")
    loaded = SurveyPlan.load(path)
    assert loaded == plan


def test_plan_from_wrong_file_rejected():
    with pytest.raises(ValueError, match="uav-survey-plan"):
        SurveyPlan.from_dict({"format": "something-else"})


def test_qgc_plan_structure():
    plan = make_plan()
    doc = to_qgc_plan(plan)
    assert doc["fileType"] == "Plan"
    items = doc["mission"]["items"]
    assert len(items) == len(plan.waypoints) + 2
    assert items[0]["command"] == 22                      # take-off first
    assert items[-1]["command"] == 20                     # RTL last
    assert [i["doJumpId"] for i in items] == list(range(1, len(items) + 1))
    for item, w in zip(items[1:-1], plan.waypoints):
        assert item["command"] == 16
        assert item["params"][4] == pytest.approx(w.lat_deg)
        assert item["params"][5] == pytest.approx(w.lon_deg)
        assert item["params"][6] == w.alt_m
    assert doc["mission"]["plannedHomePosition"][:2] == [LAT, LON]
    json.dumps(doc)  # must be serialisable


def test_geojson_structure():
    plan = make_plan()
    fc = to_geojson(plan)
    line = fc["features"][0]["geometry"]
    assert line["type"] == "LineString"
    assert len(line["coordinates"]) == len(plan.waypoints) + 2   # home, waypoints, home
    assert line["coordinates"][0] == [LON, LAT] == line["coordinates"][-1]
    assert sum(f["geometry"]["type"] == "Point" for f in fc["features"]) == len(plan.waypoints) + 1


def test_waypoints_csv(tmp_path):
    plan = make_plan()
    path = write_waypoints_csv(plan, tmp_path / "wp.csv")
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == len(plan.waypoints)
    assert float(rows[1]["north_m"]) == pytest.approx(30.0)
