"""Export a survey plan to formats other tools understand.

* QGroundControl ``.plan`` - open it in QGC (File > Open) to inspect or upload.
* GeoJSON - paste into https://geojson.io to see the path on a map.
* CSV - one waypoint per row, for spreadsheets or report tables.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Dict

from .planner import SurveyPlan

# MAVLink command / frame ids used in QGC plan files
_CMD_WAYPOINT = 16
_CMD_RTL = 20
_CMD_TAKEOFF = 22
_FRAME_GLOBAL = 0
_FRAME_GLOBAL_RELATIVE_ALT = 3
_FRAME_MISSION = 2
_FIRMWARE_PX4 = 12
_VEHICLE_MULTIROTOR = 2


def _simple_item(do_jump_id: int, command: int, frame: int, params: list, altitude: float) -> Dict[str, Any]:
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude,
        "AltitudeMode": 1 if frame == _FRAME_GLOBAL_RELATIVE_ALT else 0,
        "autoContinue": True,
        "command": command,
        "doJumpId": do_jump_id,
        "frame": frame,
        "params": params,
        "type": "SimpleItem",
    }


def to_qgc_plan(plan: SurveyPlan) -> Dict[str, Any]:
    """Build the JSON structure of a QGroundControl plan file."""
    cfg = plan.config
    items = []
    jump_id = 1

    items.append(
        _simple_item(
            jump_id, _CMD_TAKEOFF, _FRAME_GLOBAL_RELATIVE_ALT,
            [0, 0, 0, None, plan.origin_lat_deg, plan.origin_lon_deg, cfg.altitude_m],
            cfg.altitude_m,
        )
    )
    for w in plan.waypoints:
        jump_id += 1
        items.append(
            _simple_item(
                jump_id, _CMD_WAYPOINT, _FRAME_GLOBAL_RELATIVE_ALT,
                [0, 0, 0, None, w.lat_deg, w.lon_deg, w.alt_m],
                w.alt_m,
            )
        )
    jump_id += 1
    items.append(_simple_item(jump_id, _CMD_RTL, _FRAME_MISSION, [0, 0, 0, 0, 0, 0, 0], 0))

    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": cfg.speed_m_s,
            "firmwareType": _FIRMWARE_PX4,
            "hoverSpeed": cfg.speed_m_s,
            "items": items,
            "plannedHomePosition": [plan.origin_lat_deg, plan.origin_lon_deg, 0],
            "vehicleType": _VEHICLE_MULTIROTOR,
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def to_geojson(plan: SurveyPlan) -> Dict[str, Any]:
    """GeoJSON FeatureCollection: the path as a LineString plus one Point per waypoint."""
    coords = [[plan.origin_lon_deg, plan.origin_lat_deg]]
    coords += [[w.lon_deg, w.lat_deg] for w in plan.waypoints]
    coords.append([plan.origin_lon_deg, plan.origin_lat_deg])

    features = [
        {
            "type": "Feature",
            "properties": {"name": "survey path (home -> sweep -> home)"},
            "geometry": {"type": "LineString", "coordinates": coords},
        },
        {
            "type": "Feature",
            "properties": {"name": "home"},
            "geometry": {"type": "Point", "coordinates": [plan.origin_lon_deg, plan.origin_lat_deg]},
        },
    ]
    for w in plan.waypoints:
        features.append(
            {
                "type": "Feature",
                "properties": {"name": f"WP{w.index}", "alt_m": w.alt_m},
                "geometry": {"type": "Point", "coordinates": [w.lon_deg, w.lat_deg]},
            }
        )
    return {"type": "FeatureCollection", "features": features}


def write_json(data: Dict[str, Any], path: "str | Path") -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return out


def write_waypoints_csv(plan: SurveyPlan, path: "str | Path") -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["index", "lat_deg", "lon_deg", "alt_m", "east_m", "north_m"])
        for w in plan.waypoints:
            writer.writerow([w.index, f"{w.lat_deg:.8f}", f"{w.lon_deg:.8f}", w.alt_m,
                             f"{w.east_m:.3f}", f"{w.north_m:.3f}"])
    return out
