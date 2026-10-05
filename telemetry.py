"""Fixed-rate telemetry recording to CSV.

Each telemetry stream is consumed by its own task that only updates the latest
value; a separate sampler writes one row per tick. This gives evenly spaced
samples (good for speed/distance statistics) regardless of how often each
stream publishes.
"""
from __future__ import annotations

import asyncio
import csv
import logging
import math
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

from .link import connect_drone
from .mission import normalize_battery_pct

log = logging.getLogger(__name__)

FIELDS = [
    "t_s", "lat_deg", "lon_deg", "rel_alt_m",
    "ground_speed_m_s", "battery_pct", "in_air", "flight_mode",
]


class TelemetryRecorder:
    def __init__(
        self,
        out_path: "str | Path",
        connection: str = "udp://:14540",
        drone: Optional[Any] = None,
        rate_hz: float = 2.0,
        stop_on_land: bool = False,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not (0.1 <= rate_hz <= 50.0):
            raise ValueError("rate_hz must be between 0.1 and 50")
        self.out_path = Path(out_path)
        self.connection = connection
        self._drone = drone
        self.rate_hz = rate_hz
        self.stop_on_land = stop_on_land
        self._clock = clock
        self._latest: Dict[str, Any] = {}

    async def run(self, duration_s: Optional[float] = None, connect_timeout_s: float = 30.0) -> int:
        """Record until ``duration_s`` elapses, the vehicle lands, or the task is cancelled.

        Returns the number of rows written.
        """
        self._drone = await connect_drone(self.connection, connect_timeout_s, self._drone)
        drone = self._drone

        consumers = [
            asyncio.create_task(self._consume(drone.telemetry.position(), self._on_position)),
            asyncio.create_task(self._consume(drone.telemetry.velocity_ned(), self._on_velocity)),
            asyncio.create_task(self._consume(drone.telemetry.battery(), self._on_battery)),
            asyncio.create_task(self._consume(drone.telemetry.in_air(), self._on_in_air)),
            asyncio.create_task(self._consume(drone.telemetry.flight_mode(), self._on_mode)),
        ]
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        rows = 0
        try:
            with self.out_path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=FIELDS)
                writer.writeheader()
                log.info("recording to %s at %.1f Hz", self.out_path, self.rate_hz)
                rows = await self._sample_loop(writer, fh, duration_s)
        finally:
            for task in consumers:
                task.cancel()
            await asyncio.gather(*consumers, return_exceptions=True)
        log.info("wrote %d samples to %s", rows, self.out_path)
        return rows

    async def _sample_loop(self, writer: "csv.DictWriter[str]", fh: Any, duration_s: Optional[float]) -> int:
        period = 1.0 / self.rate_hz
        start = self._clock()
        rows = 0
        was_in_air = False
        while True:
            await asyncio.sleep(period)
            now = self._clock()
            elapsed = now - start
            if "lat_deg" in self._latest:  # wait for the first position fix
                row = self._row(elapsed)
                writer.writerow(row)
                fh.flush()
                rows += 1
                in_air = self._latest.get("in_air")
                was_in_air = was_in_air or bool(in_air)
                if self.stop_on_land and was_in_air and in_air is False:
                    log.info("vehicle landed - stopping recorder")
                    return rows
            if duration_s is not None and elapsed >= duration_s:
                return rows

    def _row(self, elapsed: float) -> Dict[str, Any]:
        latest = self._latest
        battery = latest.get("battery_pct")
        speed = latest.get("ground_speed_m_s")
        return {
            "t_s": f"{elapsed:.2f}",
            "lat_deg": f"{latest['lat_deg']:.8f}",
            "lon_deg": f"{latest['lon_deg']:.8f}",
            "rel_alt_m": f"{latest.get('rel_alt_m', float('nan')):.2f}",
            "ground_speed_m_s": "" if speed is None else f"{speed:.2f}",
            "battery_pct": "" if battery is None else f"{battery:.1f}",
            "in_air": "" if latest.get("in_air") is None else int(bool(latest["in_air"])),
            "flight_mode": latest.get("flight_mode", ""),
        }

    # --------------------------------------------------------------- streams
    @staticmethod
    async def _consume(stream: Any, handler: Callable[[Any], None]) -> None:
        async for item in stream:
            handler(item)

    def _on_position(self, p: Any) -> None:
        self._latest["lat_deg"] = p.latitude_deg
        self._latest["lon_deg"] = p.longitude_deg
        self._latest["rel_alt_m"] = p.relative_altitude_m

    def _on_velocity(self, v: Any) -> None:
        self._latest["ground_speed_m_s"] = math.hypot(v.north_m_s, v.east_m_s)

    def _on_battery(self, b: Any) -> None:
        pct = normalize_battery_pct(b.remaining_percent)
        if pct is not None:
            self._latest["battery_pct"] = pct

    def _on_in_air(self, value: Any) -> None:
        self._latest["in_air"] = bool(value)

    def _on_mode(self, mode: Any) -> None:
        self._latest["flight_mode"] = getattr(mode, "name", str(mode))
