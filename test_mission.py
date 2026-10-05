import asyncio
import types

import pytest

from uav_survey import link
from uav_survey.config import SurveyConfig
from uav_survey.errors import LinkError, MissionError, PreflightError
from uav_survey.mission import (
    MAX_ORIGIN_OFFSET_M,
    MissionRunner,
    MissionStatus,
    normalize_battery_pct,
)
from uav_survey.planner import build_plan

from fakes import LAT, LON, FakeDrone, progress

CONFIG = SurveyConfig(width_m=40, height_m=30, spacing_m=20)
FULL_RUN = [progress(i) for i in range(0, 7)]     # 6 waypoints -> done at current == 6


def make_runner(drone, **kwargs):
    kwargs.setdefault("plan_builder", lambda plan: plan)
    return MissionRunner(CONFIG, drone=drone, **kwargs)


def run(coro):
    return asyncio.run(coro)


async def fly(drone, plan=None, **kwargs):
    runner = make_runner(drone, **kwargs)
    await runner.connect()
    return await runner.run(plan)


# ------------------------------------------------------------------ helpers
@pytest.mark.parametrize(
    "value, expected",
    [(80, 80.0), (0, 0.0), (100.0, 100.0), (130, 100.0),
     (float("nan"), None), (float("inf"), None), (-1, None), (None, None), ("x", None)],
)
def test_normalize_battery_pct(value, expected):
    assert normalize_battery_pct(value) == expected


# ------------------------------------------------------------- happy path
def test_completed_mission_uploads_arms_starts_and_lands():
    drone = FakeDrone(progress_values=FULL_RUN)
    result = run(fly(drone))
    assert result.status is MissionStatus.COMPLETED and result.success
    assert result.waypoints_reached == 6 and result.waypoints_total == 6
    assert result.landed is True
    assert result.min_battery_pct == 90.0
    assert drone.calls == ["rtl_after_mission", "upload", "arm", "start"]   # no extra RTL needed
    assert drone.connected_to == CONFIG.connection
    assert len(drone.uploaded.waypoints) == 6


def test_result_serialises():
    result = run(fly(FakeDrone(progress_values=FULL_RUN)))
    data = result.to_dict()
    assert data["status"] == "completed" and data["waypoints_total"] == 6


# --------------------------------------------------------------- failsafes
def test_low_battery_aborts_and_returns_to_launch():
    drone = FakeDrone(battery_values=[80, 20, 20, 20, 20], progress_values=[progress(1)])
    result = run(fly(drone))
    assert result.status is MissionStatus.ABORTED_LOW_BATTERY
    assert "below" in result.detail
    assert result.min_battery_pct == 20
    assert "rtl" in drone.calls


def test_brief_low_readings_do_not_abort():
    # low readings never reach three in a row, and the mission finishes after the
    # whole battery stream has been consumed (progress is deliberately slower)
    drone = FakeDrone(
        battery_values=[80, 20, 20, 80, 20, 20, 80, 80],
        progress_values=[progress(1), progress(6)],
        progress_delay=0.03,
    )
    result = run(fly(drone))
    assert result.status is MissionStatus.COMPLETED
    assert result.min_battery_pct == 20
    assert "rtl" not in drone.calls


def test_timeout_aborts_and_returns_to_launch():
    drone = FakeDrone(progress_values=[progress(1)])
    result = run(fly(drone, timeout_override_s=0.05))
    assert result.status is MissionStatus.ABORTED_TIMEOUT
    assert drone.calls.count("rtl") == 1


def test_rtl_failure_is_reported_not_raised():
    drone = FakeDrone(progress_values=[progress(1)], rtl_error=RuntimeError("link lost"))
    result = run(fly(drone, timeout_override_s=0.05))
    assert result.status is MissionStatus.ABORTED_TIMEOUT
    assert drone.calls.count("rtl") == 1


def test_interrupt_commands_return_to_launch():
    async def scenario():
        drone = FakeDrone(progress_values=[progress(1)])
        runner = make_runner(drone)
        await runner.connect()
        task = asyncio.create_task(runner.run())
        await asyncio.sleep(0.15)           # mission is now in the monitoring phase
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        return drone

    drone = run(scenario())
    assert "rtl" in drone.calls


def test_monitor_stream_ending_is_a_failure_with_rtl():
    class Ends(FakeDrone):
        def _progress(self):
            async def gen():
                yield progress(1)
            return gen()

    drone = Ends()
    result = run(fly(drone))
    assert result.status is MissionStatus.FAILED
    assert "ended" in result.detail
    assert "rtl" in drone.calls


# ---------------------------------------------------------------- pre-flight
def test_low_start_battery_refused_before_anything_is_uploaded():
    drone = FakeDrone(battery_values=[40])
    with pytest.raises(PreflightError, match="below the 60%"):
        run(fly(drone))
    assert drone.calls == []


def test_unreadable_battery_refused(monkeypatch):
    original = MissionRunner._read_battery
    monkeypatch.setattr(MissionRunner, "_read_battery", lambda self: original(self, timeout_s=0.05))
    drone = FakeDrone(battery_values=[float("nan")])
    with pytest.raises(PreflightError, match="battery level is unavailable"):
        run(fly(drone))
    assert drone.calls == []


def test_unreadable_battery_allowed_when_explicitly_opted_out():
    drone = FakeDrone(battery_values=[float("nan")], progress_values=FULL_RUN)
    result = run(fly(drone, require_battery=False))
    assert result.status is MissionStatus.COMPLETED
    assert result.min_battery_pct is None


def test_gps_not_ready_times_out():
    drone = FakeDrone(healthy=False)

    async def go():
        runner = make_runner(drone)
        await runner.connect()
        await runner.preflight(timeout_s=0.05)

    with pytest.raises(PreflightError, match="GPS"):
        run(go())


def test_plan_made_for_another_place_is_refused():
    far_plan = build_plan(CONFIG, LAT + 0.01, LON)        # ~1.1 km away
    drone = FakeDrone(progress_values=FULL_RUN)
    with pytest.raises(PreflightError, match="away from the vehicle"):
        run(fly(drone, plan=far_plan))
    assert drone.calls == []


def test_plan_made_for_this_place_is_flown():
    plan = build_plan(CONFIG, LAT, LON)
    drone = FakeDrone(progress_values=FULL_RUN)
    result = run(fly(drone, plan=plan))
    assert result.success and drone.uploaded is plan
    assert MAX_ORIGIN_OFFSET_M > 0


def test_not_connected_is_an_error():
    runner = make_runner(None)
    with pytest.raises(MissionError, match="not connected"):
        run(runner.run())


# ------------------------------------------------------ upload / arm errors
def test_upload_failure_never_arms():
    drone = FakeDrone(upload_error=RuntimeError("storage full"))
    with pytest.raises(MissionError, match="upload failed"):
        run(fly(drone))
    assert "arm" not in drone.calls


def test_arm_failure_disarms_and_reports():
    drone = FakeDrone(arm_error=RuntimeError("preflight check failed"))
    with pytest.raises(MissionError, match="could not arm"):
        run(fly(drone))
    assert "disarm" in drone.calls and "start" not in drone.calls


# ------------------------------------------------------------------ landing
def test_wait_landed_times_out_if_vehicle_stays_up():
    drone = FakeDrone(in_air_values=[True])
    runner = make_runner(drone)
    assert run(runner._wait_landed(0.05)) is False


# --------------------------------------------------------------- link layer
def test_load_mavsdk_missing(monkeypatch):
    def fake_import(name):
        raise ImportError(name)

    monkeypatch.setattr(link, "importlib", types.SimpleNamespace(import_module=fake_import, metadata=link.importlib.metadata))
    with pytest.raises(LinkError, match="not installed"):
        link.load_mavsdk()


def test_load_mavsdk_rejects_v4(monkeypatch):
    def fake_import(name):
        if name == "mavsdk":
            return types.SimpleNamespace(System=object)
        raise ImportError(name)

    monkeypatch.setattr(link, "importlib", types.SimpleNamespace(import_module=fake_import, metadata=link.importlib.metadata))
    monkeypatch.setattr(link, "_major_version", lambda dist: 4)
    with pytest.raises(LinkError, match=r"4\.x.*pip install"):
        link.load_mavsdk()


def test_connect_times_out_with_helpful_message():
    class Silent(FakeDrone):
        async def _connection_state(self):
            await asyncio.Event().wait()
            yield  # pragma: no cover

    drone = Silent()
    with pytest.raises(LinkError, match="no vehicle detected"):
        run(link.connect_drone("udp://:14540", timeout_s=0.05, drone=drone))
