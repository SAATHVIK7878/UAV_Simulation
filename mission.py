"""Safe execution of a survey plan on a MAVSDK-connected vehicle.

Safety behaviour (all of it covered by tests using a fake vehicle):

* Pre-flight: GPS/home must be ready, battery must be readable and above
  ``min_start_battery_pct``, and a pre-made plan must have been generated for
  the place the vehicle actually is (within ``MAX_ORIGIN_OFFSET_M``).
* In flight: three monitors run side by side - mission progress, battery and a
  wall-clock timeout. The first to trigger decides the outcome.
* On a low-battery or timeout abort, or if cancelled (Ctrl+C), return-to-launch
  is commanded. The toolkit never leaves the vehicle hovering unattended.
"""
from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from .config import SurveyConfig
from .errors import MissionError, PreflightError
from .geometry import haversine_m
from .link import connect_drone, load_mavsdk
from .planner import SurveyPlan, build_plan

log = logging.getLogger(__name__)

LOW_BATTERY_CONFIRMATIONS = 3     # consecutive low readings before aborting
MAX_ORIGIN_OFFSET_M = 50.0        # max distance between plan origin and vehicle


class MissionStatus(str, Enum):
    COMPLETED = "completed"
    ABORTED_LOW_BATTERY = "aborted_low_battery"
    ABORTED_TIMEOUT = "aborted_timeout"
    FAILED = "failed"


@dataclass(frozen=True)
class MissionResult:
    status: MissionStatus
    detail: str
    waypoints_reached: int
    waypoints_total: int
    duration_s: float
    min_battery_pct: Optional[float]
    landed: bool

    @property
    def success(self) -> bool:
        return self.status is MissionStatus.COMPLETED

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status.value,
            "detail": self.detail,
            "waypoints_reached": self.waypoints_reached,
            "waypoints_total": self.waypoints_total,
            "duration_s": round(self.duration_s, 1),
            "min_battery_pct": None if self.min_battery_pct is None else round(self.min_battery_pct, 1),
            "landed": self.landed,
        }


def normalize_battery_pct(value: Any) -> Optional[float]:
    """Return a battery level in percent (0-100), or ``None`` if it is unusable.

    MAVSDK-Python 3.x reports ``remaining_percent`` in 0-100. PX4 reports NaN
    while it has no estimate, which must never be mistaken for a real reading.
    """
    try:
        pct = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(pct) or math.isinf(pct) or pct < 0.0:
        return None
    return min(pct, 100.0)


def to_mavsdk_plan(plan: SurveyPlan) -> Any:
    """Convert a :class:`SurveyPlan` into a MAVSDK ``MissionPlan``."""
    _, mission = load_mavsdk()
    nan = float("nan")
    cfg = plan.config
    items = [
        mission.MissionItem(
            w.lat_deg, w.lon_deg, w.alt_m, cfg.speed_m_s, True,
            nan, nan, mission.MissionItem.CameraAction.NONE,
            nan, nan, cfg.acceptance_radius_m, nan, nan,
            mission.MissionItem.VehicleAction.NONE,
        )
        for w in plan.waypoints
    ]
    return mission.MissionPlan(items)


class _State:
    """Mutable progress shared between the monitors and the final report."""

    def __init__(self, total: int) -> None:
        self.total = total
        self.reached = 0
        self.min_battery: Optional[float] = None

    def note_battery(self, pct: float) -> None:
        if self.min_battery is None or pct < self.min_battery:
            self.min_battery = pct


Outcome = Tuple[MissionStatus, str]


class MissionRunner:
    """Runs a survey mission with battery and timeout failsafes.

    Parameters
    ----------
    config:
        Supplies the safety limits and the connection string.
    drone:
        Inject an existing MAVSDK ``System`` (or a fake in tests).
    plan_builder:
        Converts a :class:`SurveyPlan` to the object ``upload_mission`` expects.
    require_battery:
        When ``True`` (default) an unreadable battery level aborts pre-flight.
    timeout_override_s:
        Replaces ``config.mission_timeout_s`` (used by tests).
    """

    def __init__(
        self,
        config: SurveyConfig,
        drone: Optional[Any] = None,
        plan_builder: Callable[[SurveyPlan], Any] = to_mavsdk_plan,
        require_battery: bool = True,
        timeout_override_s: Optional[float] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config.validate()
        self._drone = drone
        self._plan_builder = plan_builder
        self.require_battery = require_battery
        self._timeout_s = timeout_override_s if timeout_override_s is not None else config.mission_timeout_s
        self._clock = clock
        self.last_plan: Optional[SurveyPlan] = None  # the plan most recently flown (or being flown)

    # ----------------------------------------------------------- lifecycle
    async def connect(self, timeout_s: float = 30.0) -> None:
        self._drone = await connect_drone(self.config.connection, timeout_s, self._drone)

    def _require_drone(self) -> Any:
        if self._drone is None:
            raise MissionError("not connected - call connect() first")
        return self._drone

    # ----------------------------------------------------------- pre-flight
    async def preflight(self, timeout_s: float = 60.0) -> Tuple[float, float]:
        """Run pre-flight checks and return the vehicle's (lat, lon)."""
        drone = self._require_drone()

        log.info("waiting for GPS lock and home position")
        try:
            await asyncio.wait_for(self._wait_healthy(), timeout_s)
        except asyncio.TimeoutError as exc:
            raise PreflightError(
                f"GPS / home position not ready after {timeout_s:.0f}s - wait for the "
                "simulator to finish starting, then try again"
            ) from exc

        position = await asyncio.wait_for(self._first(drone.telemetry.position()), 10.0)

        if self.require_battery:
            pct = await self._read_battery()
            if pct is None:
                raise PreflightError(
                    "battery level is unavailable, so the low-battery failsafe cannot work. "
                    "Pass --no-battery-check to fly without it (not recommended)."
                )
            if pct < self.config.min_start_battery_pct:
                raise PreflightError(
                    f"battery at {pct:.0f}% is below the {self.config.min_start_battery_pct:.0f}% "
                    "required to start"
                )
            log.info("battery %.0f%% - ok", pct)
        return position.latitude_deg, position.longitude_deg

    async def _wait_healthy(self) -> None:
        async for health in self._drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return
        raise PreflightError("health stream ended before the vehicle reported a position lock")

    @staticmethod
    async def _first(stream: Any) -> Any:
        async for item in stream:
            return item
        raise PreflightError("a telemetry stream ended unexpectedly")

    async def _read_battery(self, timeout_s: float = 10.0) -> Optional[float]:
        async def first_valid() -> Optional[float]:
            async for b in self._drone.telemetry.battery():
                pct = normalize_battery_pct(b.remaining_percent)
                if pct is not None:
                    return pct
            return None

        try:
            return await asyncio.wait_for(first_valid(), timeout_s)
        except asyncio.TimeoutError:
            return None

    # --------------------------------------------------------------- flight
    async def run(self, plan: Optional[SurveyPlan] = None) -> MissionResult:
        """Fly ``plan`` (or a fresh one built around the vehicle's position)."""
        lat, lon = await self.preflight()

        if plan is None:
            plan = build_plan(self.config, lat, lon)
        else:
            offset = haversine_m(plan.origin_lat_deg, plan.origin_lon_deg, lat, lon)
            if offset > MAX_ORIGIN_OFFSET_M:
                raise PreflightError(
                    f"the plan was made for a point {offset:.0f} m away from the vehicle "
                    f"(limit {MAX_ORIGIN_OFFSET_M:.0f} m); regenerate it with the vehicle's "
                    "home position, or omit --plan to build one automatically"
                )
        self.last_plan = plan
        log.info(
            "plan: %d waypoints, %.0f m total, ~%.0f s",
            len(plan.waypoints), plan.total_distance_m, plan.estimated_duration_s,
        )

        started = self._clock()
        state = _State(total=len(plan.waypoints))
        await self._start(self._plan_builder(plan))

        status, detail = await self._supervise(state)
        log.log(logging.INFO if status is MissionStatus.COMPLETED else logging.WARNING,
                "mission %s: %s", status.value, detail)
        if status is not MissionStatus.COMPLETED:
            await self._safe_rtl()

        landed = await self._wait_landed(self.config.landing_timeout_s)
        return MissionResult(
            status=status,
            detail=detail,
            waypoints_reached=state.reached,
            waypoints_total=state.total,
            duration_s=self._clock() - started,
            min_battery_pct=state.min_battery,
            landed=landed,
        )

    async def _start(self, mav_plan: Any) -> None:
        drone = self._require_drone()
        try:
            await drone.mission.set_return_to_launch_after_mission(True)
            await drone.mission.upload_mission(mav_plan)
        except Exception as exc:
            raise MissionError(f"mission upload failed: {exc}") from exc
        log.info("mission uploaded")
        try:
            await drone.action.arm()
            await drone.mission.start_mission()
        except Exception as exc:
            await self._safe_disarm()
            raise MissionError(f"could not arm / start the mission: {exc}") from exc
        log.info("armed, mission started")

    async def _supervise(self, state: _State) -> Outcome:
        watchers = [self._watch_progress(state), self._watch_timeout()]
        if self.require_battery:
            watchers.append(self._watch_battery(state))
        tasks: List["asyncio.Task[Outcome]"] = [asyncio.create_task(w) for w in watchers]

        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        except asyncio.CancelledError:
            await self._cancel(tasks)
            log.warning("interrupted - commanding return-to-launch")
            await self._safe_rtl()
            raise
        await self._cancel(tasks)

        outcomes: List[Outcome] = []
        for task in done:
            if task.cancelled():
                continue
            exc = task.exception()
            if exc is not None:
                outcomes.append((MissionStatus.FAILED, f"monitor error: {exc!r}"))
            else:
                outcomes.append(task.result())

        for wanted in (
            MissionStatus.COMPLETED,
            MissionStatus.ABORTED_LOW_BATTERY,
            MissionStatus.ABORTED_TIMEOUT,
            MissionStatus.FAILED,
        ):
            for status, detail in outcomes:
                if status is wanted:
                    return status, detail
        return MissionStatus.FAILED, "no monitor produced an outcome"

    @staticmethod
    async def _cancel(tasks: List["asyncio.Task[Outcome]"]) -> None:
        for task in tasks:
            if not task.done():
                task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    # ------------------------------------------------------------- monitors
    async def _watch_progress(self, state: _State) -> Outcome:
        async for p in self._require_drone().mission.mission_progress():
            state.reached = max(state.reached, p.current)
            if p.total:
                state.total = p.total
            log.info("waypoint %d/%d", p.current, p.total)
            if p.total > 0 and p.current >= p.total:
                return MissionStatus.COMPLETED, f"all {p.total} waypoints reached"
        return MissionStatus.FAILED, "mission progress stream ended unexpectedly"

    async def _watch_battery(self, state: _State) -> Outcome:
        low = 0
        limit = self.config.min_battery_pct
        async for b in self._require_drone().telemetry.battery():
            pct = normalize_battery_pct(b.remaining_percent)
            if pct is None:
                continue
            state.note_battery(pct)
            if pct < limit:
                low += 1
                log.warning("battery low: %.0f%% (%d/%d)", pct, low, LOW_BATTERY_CONFIRMATIONS)
                if low >= LOW_BATTERY_CONFIRMATIONS:
                    return (
                        MissionStatus.ABORTED_LOW_BATTERY,
                        f"battery {pct:.0f}% stayed below the {limit:.0f}% limit",
                    )
            else:
                low = 0
        return MissionStatus.FAILED, "battery stream ended unexpectedly"

    async def _watch_timeout(self) -> Outcome:
        await asyncio.sleep(self._timeout_s)
        return MissionStatus.ABORTED_TIMEOUT, f"mission exceeded {self._timeout_s:.0f}s"

    # ------------------------------------------------------------- failsafes
    async def _safe_rtl(self) -> bool:
        """Command return-to-launch; never raises (it runs inside failure handling)."""
        try:
            await self._require_drone().action.return_to_launch()
            log.warning("return-to-launch commanded")
            return True
        except Exception:
            log.exception("RETURN-TO-LAUNCH FAILED - take manual control of the vehicle")
            return False

    async def _safe_disarm(self) -> None:
        try:
            await self._require_drone().action.disarm()
        except Exception:
            log.warning("disarm after a failed start did not succeed", exc_info=True)

    async def _wait_landed(self, timeout_s: float) -> bool:
        async def until_on_ground() -> None:
            async for in_air in self._require_drone().telemetry.in_air():
                if not in_air:
                    return

        try:
            await asyncio.wait_for(until_on_ground(), timeout_s)
        except asyncio.TimeoutError:
            log.warning("vehicle still in the air after %.0fs", timeout_s)
            return False
        log.info("vehicle on the ground")
        return True
