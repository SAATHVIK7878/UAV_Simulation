"""Connecting to the vehicle through MAVSDK-Python.

MAVSDK-Python 4.x is a different, native API. This toolkit is written for the
gRPC-based API (``mavsdk`` 3.x, also published as ``mavsdk-grpc`` and imported
as ``mavsdk_grpc``), so the import is checked up front and a clear message is
raised instead of a confusing ``AttributeError`` mid-flight.
"""
from __future__ import annotations

import asyncio
import importlib
import importlib.metadata
import logging
from types import ModuleType
from typing import Any, Optional, Tuple

from .errors import LinkError

log = logging.getLogger(__name__)

INSTALL_HINT = "install the gRPC-based MAVSDK-Python with:  pip install 'mavsdk>=3,<4'"


def _major_version(dist: str) -> int:
    try:
        return int(importlib.metadata.version(dist).split(".")[0])
    except (importlib.metadata.PackageNotFoundError, ValueError):
        return 0


def load_mavsdk() -> Tuple[Any, ModuleType]:
    """Return ``(System, mission_module)`` from a compatible MAVSDK-Python install."""
    problems = []
    for name in ("mavsdk_grpc", "mavsdk"):
        try:
            module = importlib.import_module(name)
        except ImportError:
            continue
        if name == "mavsdk" and _major_version("mavsdk") >= 4:
            problems.append("mavsdk 4.x is installed, which has a different API")
            continue
        try:
            return module.System, importlib.import_module(f"{name}.mission")
        except (AttributeError, ImportError) as exc:
            problems.append(f"{name} is installed but unusable ({exc})")
    detail = "; ".join(problems) if problems else "MAVSDK-Python is not installed"
    raise LinkError(f"{detail}. To fix: {INSTALL_HINT}")


async def _wait_connected(drone: Any) -> None:
    async for state in drone.core.connection_state():
        if state.is_connected:
            return
    raise LinkError("connection state stream ended before a vehicle was found")


async def connect_drone(address: str, timeout_s: float = 30.0, drone: Optional[Any] = None) -> Any:
    """Connect to ``address`` and wait for a vehicle heartbeat.

    ``drone`` can be supplied to inject an already-created (or fake) System.
    """
    if drone is None:
        system_cls, _ = load_mavsdk()
        drone = system_cls()
    log.info("connecting to %s", address)
    await drone.connect(system_address=address)
    try:
        await asyncio.wait_for(_wait_connected(drone), timeout_s)
    except asyncio.TimeoutError as exc:
        raise LinkError(
            f"no vehicle detected on {address} within {timeout_s:.0f}s - "
            "is PX4 SITL running (make px4_sitl gazebo-classic_iris)?"
        ) from exc
    log.info("vehicle connected")
    return drone
