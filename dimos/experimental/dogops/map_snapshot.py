from __future__ import annotations

import math
from typing import Any


DEFAULT_RERUN_URL = (
    "http://127.0.0.1:9878/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9877%2Fproxy"
)
DEFAULT_DIMOS_ROOT = "/Users/chris/Documents/Workspace/dimos"
DEFAULT_LIVE_ROBOT_IP = "192.168.123.161"


def build_map_snapshot(
    state: dict[str, Any],
    report: dict[str, Any] | None = None,
    *,
    mode: str = "demo",
    live_overlay: dict[str, Any] | None = None,
    rerun_url: str | None = None,
    demo_rerun_url: str | None = None,
    live_rerun_url: str | None = None,
    live_robot_ip: str | None = None,
) -> dict[str, Any]:
    site = state.get("site") or {}
    zones = _zones(site)
    zone_by_id = {zone["id"]: zone for zone in zones}
    assets = _assets(site, zone_by_id)
    entity_by_id = {**zone_by_id, **{asset["id"]: asset for asset in assets}}
    packages = _packages(site, state, report or {}, zone_by_id)
    observations = _observations(state)
    route = _route(state, entity_by_id)
    robot_pose = _latest_pose(observations, route, zone_by_id)
    target = None
    overlay = live_overlay if mode == "live" else None
    if overlay:
        route = overlay.get("route") or route
        robot_pose = overlay.get("robot_pose") or robot_pose
        target = overlay.get("target")
    points = [
        {"x": item["x"], "y": item["y"]}
        for group in (zones, assets, packages, observations)
        for item in group
        if item.get("x") is not None and item.get("y") is not None
    ]
    for group in (route, [target] if target else []):
        for item in group:
            if item and item.get("x") is not None and item.get("y") is not None:
                points.append({"x": item["x"], "y": item["y"]})
    if robot_pose:
        points.append(robot_pose)

    modes = build_view_modes(
        demo_rerun_url=demo_rerun_url or rerun_url,
        live_rerun_url=live_rerun_url or rerun_url,
        live_robot_ip=live_robot_ip,
    )
    bounds = _bounds(points)
    costmap = (overlay or {}).get("costmap") or _costmap(bounds, zones, assets, packages, route)
    live_status = overlay or {
        "ok": False,
        "source": "DimOS live LCM topics",
        "status": "not_requested" if mode != "live" else "waiting_for_topics",
        "topics": {},
        "error": "",
    }
    return {
        "run": state.get("run") or {},
        "site": {
            "id": site.get("site_id"),
            "name": site.get("site_name"),
            "tag_family": site.get("tag_family"),
            "marker_length_m": site.get("marker_length_m"),
        },
        "bounds": bounds,
        "zones": zones,
        "assets": assets,
        "packages": packages,
        "observations": observations,
        "route": route,
        "robot_pose": robot_pose,
        "target": target,
        "costmap": costmap,
        "streams": {
            "semantic_source": "DogOps run state",
            "live_source": "DimOS live LCM topics",
            "rerun_url": modes["demo"]["rerun_url"],
        },
        "mode": mode,
        "default_mode": "demo",
        "modes": modes,
        "live": live_status,
        "counts": {
            "zones": len(zones),
            "assets": len(assets),
            "packages": len(packages),
            "observations_with_pose": len(observations),
            "route_events": len(route),
            "costmap_cells": len(costmap.get("cells") or []),
        },
    }


def build_view_modes(
    *,
    demo_rerun_url: str | None = None,
    live_rerun_url: str | None = None,
    live_robot_ip: str | None = None,
) -> dict[str, dict[str, str]]:
    robot_ip = live_robot_ip or DEFAULT_LIVE_ROBOT_IP
    demo_url = demo_rerun_url or DEFAULT_RERUN_URL
    live_url = live_rerun_url or DEFAULT_RERUN_URL
    return {
        "demo": {
            "label": "Demo Replay",
            "badge": "Replay",
            "rerun_url": demo_url,
            "summary": "Replay the full DimOS Go2 stack to prove heatmap, costmap, path, pose, and camera rendering without touching hardware.",
            "command": (
                f"cd {DEFAULT_DIMOS_ROOT}\n"
                "uv run dimos --replay --rerun-web --rerun-open web "
                "run unitree-go2-dogops --daemon"
            ),
            "control_note": "DogOps controls stay available for UI smoke; robot motion is replay/simulated.",
        },
        "live": {
            "label": "Live Dog",
            "badge": "Live",
            "rerun_url": live_url,
            "summary": "Use the same DimOS map/costmap/path/camera streams while DogOps sends commands to the real Go2 control path.",
            "command": (
                f"cd {DEFAULT_DIMOS_ROOT}\n"
                f"uv run dimos --robot-ip {robot_ip} --rerun-web --rerun-open web "
                "run unitree-go2-dogops --daemon"
            ),
            "control_note": "Dashboard controls call DogOps/DimOS endpoints; Rerun is visualization only.",
        },
    }


def _zones(site: dict[str, Any]) -> list[dict[str, Any]]:
    zones: list[dict[str, Any]] = []
    for zone in site.get("zones") or []:
        pose = zone.get("pose_hint") or {}
        zones.append(
            {
                "id": zone.get("id"),
                "label": zone.get("display_name") or zone.get("id"),
                "kind": zone.get("zone_kind"),
                "tag_id": zone.get("tag_id"),
                "x": _float_or_none(pose.get("x")),
                "y": _float_or_none(pose.get("y")),
                "theta_deg": _float_or_none(pose.get("theta_deg")),
                "radius_m": float(zone.get("radius_m") or 0.8),
                "no_go": bool(zone.get("no_go")),
                "source": pose.get("source") or "site_config",
            }
        )
    return zones


def _assets(site: dict[str, Any], zone_by_id: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    assets: list[dict[str, Any]] = []
    zone_offsets: dict[str, int] = {}
    for asset in site.get("assets") or []:
        zone_id = asset.get("zone_id")
        zone = zone_by_id.get(zone_id or "")
        x, y = _offset_from_zone(zone, zone_offsets, zone_id or "", radius=0.34)
        assets.append(
            {
                "id": asset.get("id"),
                "label": asset.get("display_name") or asset.get("id"),
                "kind": asset.get("asset_kind"),
                "zone_id": zone_id,
                "tag_id": asset.get("tag_id"),
                "x": x,
                "y": y,
                "expected_clear": asset.get("expected_clear"),
                "expected_status": asset.get("expected_status"),
            }
        )
    return assets


def _packages(
    site: dict[str, Any],
    state: dict[str, Any],
    report: dict[str, Any],
    zone_by_id: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    statuses = state.get("package_statuses") or {}
    report_packages = {item.get("package_id"): item for item in report.get("packages") or []}
    packages: list[dict[str, Any]] = []
    zone_offsets: dict[str, int] = {}
    for package in site.get("packages") or []:
        package_id = package.get("id")
        status = statuses.get(package_id) or report_packages.get(package_id) or {}
        expected_zone_id = status.get("expected_zone_id") or package.get("expected_zone_id")
        observed_zone_id = status.get("observed_zone_id")
        display_zone_id = observed_zone_id or expected_zone_id
        zone = zone_by_id.get(display_zone_id or "")
        x, y = _offset_from_zone(zone, zone_offsets, display_zone_id or "", radius=0.62)
        packages.append(
            {
                "id": package_id,
                "label": package.get("display_name") or package_id,
                "tag_id": package.get("tag_id"),
                "expected_zone_id": expected_zone_id,
                "observed_zone_id": observed_zone_id,
                "state": status.get("state") or "expected",
                "blocks_asset_id": status.get("blocks_asset_id"),
                "x": x,
                "y": y,
            }
        )
    return packages


def _observations(state: dict[str, Any]) -> list[dict[str, Any]]:
    observations: list[dict[str, Any]] = []
    for obs in state.get("observations") or []:
        pose = obs.get("pose") or {}
        x = _float_or_none(pose.get("x"))
        y = _float_or_none(pose.get("y"))
        if x is None or y is None:
            continue
        observations.append(
            {
                "id": obs.get("id"),
                "entity_id": obs.get("entity_id"),
                "zone_id": obs.get("zone_id"),
                "tag_id": obs.get("tag_id"),
                "x": x,
                "y": y,
                "theta_deg": _float_or_none(pose.get("theta_deg")),
                "source": obs.get("source") or pose.get("source") or "observation",
            }
        )
    return observations


def _route(state: dict[str, Any], entity_by_id: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    route: list[dict[str, Any]] = []
    for event in state.get("nav_events") or []:
        target_id = event.get("target_id")
        target = entity_by_id.get(target_id or "")
        route.append(
            {
                "id": event.get("id"),
                "action": event.get("action"),
                "target_id": target_id,
                "success": bool(event.get("success", True)),
                "guided": bool(event.get("guided")),
                "retries": int(event.get("retries") or 0),
                "elapsed_s": float(event.get("elapsed_s") or 0.0),
                "note": event.get("note") or "",
                "x": target.get("x") if target else None,
                "y": target.get("y") if target else None,
            }
        )
    return route


def _latest_pose(
    observations: list[dict[str, Any]],
    route: list[dict[str, Any]],
    zone_by_id: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    if observations:
        latest = observations[-1]
        return {"x": latest["x"], "y": latest["y"], "source": latest["source"]}
    for event in reversed(route):
        if event.get("success") and event.get("x") is not None and event.get("y") is not None:
            return {"x": event["x"], "y": event["y"], "source": "last_nav_event"}
    home = zone_by_id.get("HOME")
    if home and home.get("x") is not None and home.get("y") is not None:
        return {"x": home["x"], "y": home["y"], "source": "HOME"}
    return None


def _offset_from_zone(
    zone: dict[str, Any] | None,
    zone_offsets: dict[str, int],
    zone_id: str,
    *,
    radius: float,
) -> tuple[float | None, float | None]:
    if zone is None or zone.get("x") is None or zone.get("y") is None:
        return None, None
    index = zone_offsets.get(zone_id, 0)
    zone_offsets[zone_id] = index + 1
    angle = index * (math.pi * 0.55)
    return float(zone["x"]) + math.cos(angle) * radius, float(zone["y"]) + math.sin(angle) * radius


def _bounds(points: list[dict[str, Any]]) -> dict[str, float]:
    if not points:
        return {"min_x": -1.0, "max_x": 1.0, "min_y": -1.0, "max_y": 1.0}
    xs = [float(point["x"]) for point in points if point.get("x") is not None]
    ys = [float(point["y"]) for point in points if point.get("y") is not None]
    return {
        "min_x": min(xs) - 0.8,
        "max_x": max(xs) + 0.8,
        "min_y": min(ys) - 0.8,
        "max_y": max(ys) + 0.8,
    }


def _costmap(
    bounds: dict[str, float],
    zones: list[dict[str, Any]],
    assets: list[dict[str, Any]],
    packages: list[dict[str, Any]],
    route: list[dict[str, Any]],
) -> dict[str, Any]:
    columns = 24
    rows = 16
    width = bounds["max_x"] - bounds["min_x"]
    height = bounds["max_y"] - bounds["min_y"]
    cell_w = width / columns
    cell_h = height / rows
    cells: list[dict[str, float]] = []
    blocked_packages = [
        package
        for package in packages
        if package.get("state") in {"missing", "wrong_zone", "blocked"}
        or package.get("blocks_asset_id")
    ]
    no_go_zones = [zone for zone in zones if zone.get("no_go")]

    for row in range(rows):
        for column in range(columns):
            x = bounds["min_x"] + (column + 0.5) * cell_w
            y = bounds["min_y"] + (row + 0.5) * cell_h
            cost = 0.08
            for zone in no_go_zones:
                cost = max(cost, _radial_cost(x, y, zone, radius_m=1.1, peak=0.95))
            for package in blocked_packages:
                cost = max(cost, _radial_cost(x, y, package, radius_m=0.9, peak=0.82))
            for asset in assets:
                if asset.get("expected_clear") is False:
                    cost = max(cost, _radial_cost(x, y, asset, radius_m=0.65, peak=0.65))
            route_bonus = max(
                (_radial_cost(x, y, event, radius_m=0.45, peak=0.24) for event in route),
                default=0.0,
            )
            cells.append(
                {
                    "x": bounds["min_x"] + column * cell_w,
                    "y": bounds["min_y"] + row * cell_h,
                    "width": cell_w,
                    "height": cell_h,
                    "cost": min(1.0, max(cost, route_bonus)),
                }
            )
    return {
        "source": "DogOps-generated navigation costmap",
        "columns": columns,
        "rows": rows,
        "cells": cells,
    }


def _radial_cost(
    x: float,
    y: float,
    point: dict[str, Any],
    *,
    radius_m: float,
    peak: float,
) -> float:
    point_x = point.get("x")
    point_y = point.get("y")
    if point_x is None or point_y is None:
        return 0.0
    distance = math.hypot(x - float(point_x), y - float(point_y))
    if distance >= radius_m:
        return 0.0
    return peak * (1.0 - distance / radius_m)


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
