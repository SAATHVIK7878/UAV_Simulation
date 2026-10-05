import json

import pytest

from uav_survey.config import ConfigError, SurveyConfig, validate_origin


def test_defaults_are_valid():
    assert SurveyConfig().validate() is not None


@pytest.mark.parametrize(
    "field, value",
    [
        ("width_m", 0),
        ("height_m", -5),
        ("spacing_m", 0.1),
        ("altitude_m", 300),        # typo for 30
        ("altitude_m", 1),
        ("speed_m_s", 50),
        ("heading_deg", 360),
        ("min_battery_pct", 0),
        ("mission_timeout_s", 1),
        ("width_m", float("nan")),
        ("width_m", float("inf")),
        ("width_m", True),
        ("width_m", "80"),
    ],
)
def test_out_of_range_values_rejected(field, value):
    with pytest.raises(ConfigError):
        SurveyConfig.from_dict({field: value})


def test_spacing_larger_than_area_rejected():
    with pytest.raises(ConfigError, match="spacing"):
        SurveyConfig.from_dict({"width_m": 10, "height_m": 10, "spacing_m": 50})


def test_start_battery_must_exceed_abort_battery():
    with pytest.raises(ConfigError, match="min_start_battery_pct"):
        SurveyConfig.from_dict({"min_start_battery_pct": 20, "min_battery_pct": 25})


def test_unknown_keys_rejected():
    with pytest.raises(ConfigError, match="unknown"):
        SurveyConfig.from_dict({"altitde_m": 30})


def test_bad_connection_string_rejected():
    with pytest.raises(ConfigError, match="connection"):
        SurveyConfig.from_dict({"connection": "14540"})


def test_json_roundtrip(tmp_path):
    cfg = SurveyConfig(altitude_m=42.0, speed_m_s=6.0)
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps(cfg.to_dict()))
    assert SurveyConfig.from_json_file(path) == cfg


def test_partial_json_uses_defaults(tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text('{"altitude_m": 15}')
    cfg = SurveyConfig.from_json_file(path)
    assert cfg.altitude_m == 15 and cfg.speed_m_s == SurveyConfig().speed_m_s


def test_missing_and_malformed_files(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        SurveyConfig.from_json_file(tmp_path / "nope.json")
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    with pytest.raises(ConfigError, match="not valid JSON"):
        SurveyConfig.from_json_file(bad)
    arr = tmp_path / "arr.json"
    arr.write_text("[1, 2]")
    with pytest.raises(ConfigError, match="JSON object"):
        SurveyConfig.from_json_file(arr)


def test_with_overrides_ignores_none_and_validates():
    cfg = SurveyConfig().with_overrides(altitude_m=45.0, speed_m_s=None)
    assert cfg.altitude_m == 45.0 and cfg.speed_m_s == SurveyConfig().speed_m_s
    with pytest.raises(ConfigError):
        SurveyConfig().with_overrides(altitude_m=999.0)


def test_shipped_config_files_are_valid():
    from pathlib import Path

    for path in (Path(__file__).resolve().parents[1] / "configs").glob("*.json"):
        SurveyConfig.from_json_file(path)


def test_validate_origin():
    validate_origin(47.4, 8.5)
    for lat, lon in [(91, 0), (0, 181), (89, 0)]:
        with pytest.raises(ConfigError):
            validate_origin(lat, lon)
