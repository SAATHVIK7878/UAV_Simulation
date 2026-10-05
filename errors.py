"""Exception hierarchy. The CLI turns any :class:`UavError` into a clean one-line message."""


class UavError(RuntimeError):
    """Base class for expected, user-facing failures."""


class LinkError(UavError):
    """MAVSDK is missing/incompatible or the vehicle could not be reached."""


class PreflightError(UavError):
    """A pre-flight check failed; nothing was uploaded or armed."""


class MissionError(UavError):
    """Uploading, arming or starting the mission failed."""
