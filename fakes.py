"""A scriptable stand-in for a MAVSDK ``System`` so failsafe logic can be tested without a simulator."""
import asyncio
from types import SimpleNamespace
from typing import Iterable, Optional

LAT, LON = 47.397742, 8.545594


async def series(values: Iterable, delay: float = 0.002, hold: bool = True):
    """Yield ``values`` one by one, then stay silent (like a live telemetry stream)."""
    for value in values:
        await asyncio.sleep(delay)
        yield value
    if hold:
        await asyncio.Event().wait()  # cancelled by the code under test


def progress(current: int, total: int = 6):
    return SimpleNamespace(current=current, total=total)


def battery(pct):
    return SimpleNamespace(remaining_percent=pct)


class FakeDrone:
    def __init__(
        self,
        *,
        battery_values=(90.0,),
        progress_values=(),
        in_air_values=(True, False),
        in_air_delay: float = 0.002,
        progress_delay: float = 0.002,
        healthy: bool = True,
        upload_error: Optional[Exception] = None,
        arm_error: Optional[Exception] = None,
        rtl_error: Optional[Exception] = None,
        position=(LAT, LON),
    ) -> None:
        self._battery_values = list(battery_values)
        self._progress_values = list(progress_values)
        self._in_air_values = list(in_air_values)
        self._in_air_delay = in_air_delay
        self._progress_delay = progress_delay
        self._healthy = healthy
        self._position = position
        self._upload_error = upload_error
        self._arm_error = arm_error
        self._rtl_error = rtl_error

        self.calls = []
        self.uploaded = None
        self.connected_to = None

        self.core = SimpleNamespace(connection_state=self._connection_state)
        self.telemetry = SimpleNamespace(
            health=self._health,
            position=self._position_stream,
            battery=self._battery,
            in_air=self._in_air,
            velocity_ned=lambda: series([SimpleNamespace(north_m_s=3.0, east_m_s=4.0)]),
            flight_mode=lambda: series([SimpleNamespace(name="MISSION")]),
        )
        self.mission = SimpleNamespace(
            set_return_to_launch_after_mission=self._set_rtl_after,
            upload_mission=self._upload,
            start_mission=self._start,
            mission_progress=self._progress,
        )
        self.action = SimpleNamespace(arm=self._arm, disarm=self._disarm, return_to_launch=self._rtl)

    # --- connection -------------------------------------------------------
    async def connect(self, system_address=None):
        self.connected_to = system_address

    async def _connection_state(self):
        yield SimpleNamespace(is_connected=True)

    # --- telemetry --------------------------------------------------------
    async def _health(self):
        while True:
            await asyncio.sleep(0.002)
            yield SimpleNamespace(is_global_position_ok=self._healthy, is_home_position_ok=self._healthy)

    def _position_stream(self):
        lat, lon = self._position
        return series([SimpleNamespace(latitude_deg=lat, longitude_deg=lon, relative_altitude_m=30.0)])

    def _battery(self):
        return series([battery(v) for v in self._battery_values])

    def _in_air(self):
        return series(self._in_air_values, delay=self._in_air_delay)

    def _progress(self):
        return series(self._progress_values, delay=self._progress_delay)

    # --- mission / actions --------------------------------------------------
    async def _set_rtl_after(self, value):
        self.calls.append("rtl_after_mission")

    async def _upload(self, plan):
        if self._upload_error:
            raise self._upload_error
        self.uploaded = plan
        self.calls.append("upload")

    async def _start(self):
        self.calls.append("start")

    async def _arm(self):
        if self._arm_error:
            raise self._arm_error
        self.calls.append("arm")

    async def _disarm(self):
        self.calls.append("disarm")

    async def _rtl(self):
        self.calls.append("rtl")
        if self._rtl_error:
            raise self._rtl_error
