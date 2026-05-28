from __future__ import annotations

from html import escape
from itertools import pairwise
import json
import math
import os
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from dimos.experimental.dogops.map_authoring import load_map_authoring
from dimos.experimental.dogops.qr_cargo import get_latest_qr_events

MAP_WIDTH = 920
MAP_HEIGHT = 560
MAP_PADDING_M = 0.55
MAP_CELL_M = 0.18
ENTITY_OFFSETS_M = (
    (0.0, 0.0),
    (0.24, 0.22),
    (0.28, -0.22),
    (-0.26, 0.20),
    (-0.30, -0.18),
    (0.48, 0.0),
    (0.0, -0.46),
)
PACKAGE_OFFSETS_M = (
    (-0.22, -0.18),
    (0.0, -0.22),
    (0.22, -0.18),
    (-0.14, 0.18),
    (0.14, 0.18),
)


def build_map_data(
    state: dict[str, Any],
    report: dict[str, Any],
    *,
    live_overlay: dict[str, Any] | None = None,
    heatmap_snapshot: dict[str, Any] | None = None,
    authoring: dict[str, Any] | None = None,
    qr_events: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    site = state.get("site") or {}
    authoring = authoring or {}
    authored_entities = _authoring_entities(authoring)
    authored_incidents = _authoring_incidents(authoring)
    zones = site.get("zones") or []
    assets = site.get("assets") or []
    site_packages = site.get("packages") or []
    observations = state.get("observations") or []
    nav_events = state.get("nav_events") or []
    incidents = report.get("incidents") or []
    report_packages = report.get("packages") or []

    zone_points: dict[str, tuple[float, float]] = {}
    map_zones: list[dict[str, Any]] = []
    for zone in zones:
        pose = _zone_pose(zone)
        zone_id = str(zone["id"])
        authored = authored_entities.get(zone_id)
        if authored is not None:
            pose = _authoring_pose(authored)
        elif zone_id == "HOME" and authoring.get("home"):
            pose = _authoring_pose({"pose": authoring["home"]})
        if pose is None:
            continue
        zone_points[zone_id] = pose
        map_zones.append(
            {
                "id": zone_id,
                "display_name": _authoring_label(authoring, zone_id, zone.get("display_name") or zone_id),
                "zone_kind": zone.get("zone_kind", "zone"),
                "tag_id": _authoring_tag_id(authoring, zone_id, zone.get("tag_id")),
                "radius_m": float(zone.get("radius_m") or 0.8),
                "no_go": bool(zone.get("no_go")),
                "x": pose[0],
                "y": pose[1],
                "source": "dashboard_edit" if authored is not None or (zone_id == "HOME" and authoring.get("home")) else "site_config",
            }
        )

    entity_points = dict(zone_points)
    map_assets: list[dict[str, Any]] = []
    for index, asset in enumerate(assets):
        zone_id = str(asset.get("zone_id") or "")
        base = zone_points.get(zone_id)
        asset_id = str(asset["id"])
        authored = authored_entities.get(asset_id)
        authored_pose = _authoring_pose(authored) if authored is not None else None
        if base is None and authored_pose is None:
            continue
        pose = authored_pose or _offset_pose(base, index + 1)  # type: ignore[arg-type]
        entity_points[asset_id] = pose
        map_assets.append(
            {
                "id": asset_id,
                "display_name": _authoring_label(authoring, asset_id, asset.get("display_name") or asset_id),
                "asset_kind": asset.get("asset_kind", "asset"),
                "tag_id": _authoring_tag_id(authoring, asset_id, asset.get("tag_id")),
                "zone_id": str((authored or {}).get("zone_id") or zone_id),
                "x": pose[0],
                "y": pose[1],
                "source": "dashboard_edit" if authored_pose is not None else "site_config",
            }
        )

    site_package_by_id = {str(package["id"]): package for package in site_packages}
    package_counts_by_zone: dict[str, int] = {}
    map_packages: list[dict[str, Any]] = []
    for package in report_packages:
        package_id = str(package["package_id"])
        authored = authored_entities.get(package_id)
        observed_zone = package.get("observed_zone_id")
        expected_zone = package.get("expected_zone_id")
        zone_id = str(observed_zone or expected_zone or "")
        base = zone_points.get(zone_id)
        authored_pose = _authoring_pose(authored) if authored is not None else None
        if base is None and authored_pose is None:
            continue
        count = package_counts_by_zone.get(zone_id, 0)
        package_counts_by_zone[zone_id] = count + 1
        pose = authored_pose or _package_pose(base, count)  # type: ignore[arg-type]
        site_package = site_package_by_id.get(package_id) or {}
        entity_points[package_id] = pose
        map_packages.append(
            {
                "id": package_id,
                "display_name": _authoring_label(authoring, package_id, site_package.get("display_name") or package_id),
                "tag_id": _authoring_tag_id(authoring, package_id, site_package.get("tag_id")),
                "expected_zone_id": expected_zone,
                "observed_zone_id": observed_zone,
                "state": package.get("state", "unknown"),
                "blocks_asset_id": package.get("blocks_asset_id"),
                "x": pose[0],
                "y": pose[1],
                "source": "dashboard_edit" if authored_pose is not None else "site_config",
            }
        )

    for entity in (authoring.get("entities") or []):
        if not isinstance(entity, dict):
            continue
        entity_id = str(entity.get("id") or "")
        if not entity_id or entity_id in entity_points:
            continue
        pose = _authoring_pose(entity)
        if pose is None:
            continue
        kind = str(entity.get("kind") or "checkpoint")
        entity_points[entity_id] = pose
        if kind in {"zone", "checkpoint"}:
            zone_points[entity_id] = pose
            map_zones.append(
                {
                    "id": entity_id,
                    "display_name": entity.get("label") or entity_id,
                    "zone_kind": kind,
                    "tag_id": entity.get("tag_id"),
                    "radius_m": 0.45,
                    "no_go": False,
                    "x": pose[0],
                    "y": pose[1],
                    "source": "dashboard_edit",
                }
            )
        elif kind == "asset":
            map_assets.append(
                {
                    "id": entity_id,
                    "display_name": entity.get("label") or entity_id,
                    "asset_kind": "authored",
                    "tag_id": entity.get("tag_id"),
                    "zone_id": entity.get("zone_id"),
                    "x": pose[0],
                    "y": pose[1],
                    "source": "dashboard_edit",
                }
            )
        elif kind == "package":
            map_packages.append(
                {
                    "id": entity_id,
                    "display_name": entity.get("label") or entity_id,
                    "tag_id": entity.get("tag_id"),
                    "expected_zone_id": entity.get("zone_id"),
                    "observed_zone_id": entity.get("zone_id"),
                    "state": "authored",
                    "blocks_asset_id": None,
                    "x": pose[0],
                    "y": pose[1],
                    "source": "dashboard_edit",
                }
            )

    map_route: list[dict[str, Any]] = []
    authored_route = _selected_authoring_route(authoring)
    if authored_route is not None:
        for waypoint in authored_route.get("waypoints") or []:
            if not isinstance(waypoint, dict):
                continue
            pose = _authoring_pose(waypoint)
            if pose is None:
                continue
            target_id = str(waypoint.get("target_id") or waypoint.get("id") or "waypoint")
            map_route.append(
                {
                    "target_id": target_id,
                    "x": pose[0],
                    "y": pose[1],
                    "success": True,
                    "guided": False,
                    "retries": 0,
                    "note": "authored route",
                    "source": "dashboard_edit",
                }
            )
    else:
        for event in nav_events:
            if event.get("action") != "goto":
                continue
            target_id = str(event.get("target_id") or "")
            pose = entity_points.get(target_id)
            if pose is None:
                continue
            map_route.append(
                {
                    "target_id": target_id,
                    "x": pose[0],
                    "y": pose[1],
                    "success": bool(event.get("success", True)),
                    "guided": bool(event.get("guided", False)),
                    "retries": int(event.get("retries") or 0),
                    "note": event.get("note", ""),
                    "source": "nav_event",
                }
            )

    map_observations: list[dict[str, Any]] = []
    for observation in observations:
        zone_id = str(observation.get("zone_id") or "")
        base = zone_points.get(zone_id)
        if base is None:
            continue
        pose = _offset_pose(base, len(map_observations) + 1)
        map_observations.append(
            {
                "id": observation.get("id"),
                "zone_id": zone_id,
                "entity_id": observation.get("entity_id"),
                "tag_id": observation.get("tag_id"),
                "visible_tag_ids": _visible_tag_ids(observation),
                "source": observation.get("source", "unknown"),
                "x": pose[0],
                "y": pose[1],
            }
        )

    map_incidents: list[dict[str, Any]] = []
    for incident in incidents:
        entity_id = str(incident.get("entity_id") or "")
        location = authored_incidents.get(str(incident.get("id") or ""))
        pose = _authoring_pose(location) if location is not None else entity_points.get(entity_id)
        if pose is None and incident.get("related_package_id"):
            pose = entity_points.get(str(incident["related_package_id"]))
        if pose is None:
            continue
        map_incidents.append(
            {
                "id": incident.get("id"),
                "entity_id": entity_id,
                "related_package_id": incident.get("related_package_id"),
                "severity": incident.get("severity", "INFO"),
                "state": incident.get("state", "unknown"),
                "x": pose[0],
                "y": pose[1],
                "source": "dashboard_edit" if location is not None else "run_state",
            }
        )

    map_qr_cargo_events = _qr_cargo_overlays(qr_events or [], entity_points)

    live = live_overlay or {
        "ok": False,
        "source": "DimOS live LCM topics",
        "status": "not_requested",
        "error": "",
        "topics": {},
        "costmap": None,
        "path": [],
        "route": [],
        "robot_pose": None,
        "target": None,
    }
    live = _live_with_gathered_heatmap(live, heatmap_snapshot)
    points = [
        (item["x"], item["y"])
        for group in (map_zones, map_assets, map_packages, map_route, map_observations)
        for item in group
    ]
    points.extend(
        (item["x"], item["y"])
        for item in map_qr_cargo_events
        if item.get("x") is not None and item.get("y") is not None
    )
    points.extend(_no_go_shape_points(authoring))
    points.extend(_live_overlay_points(live))
    bounds = _map_bounds(points)
    return {
        "site_id": site.get("site_id"),
        "site_name": site.get("site_name"),
        "zones": map_zones,
        "assets": map_assets,
        "packages": map_packages,
        "route": map_route,
        "observations": map_observations,
        "incidents": map_incidents,
        "qr_cargo_events": map_qr_cargo_events,
        "no_go_shapes": authoring.get("no_go_shapes") or [],
        "tag_bindings": authoring.get("tag_bindings") or [],
        "authoring": {
            "schema_version": authoring.get("schema_version", 1),
            "updated_at": authoring.get("updated_at"),
            "selected_route_id": authoring.get("selected_route_id"),
            "entities": len(authoring.get("entities") or []),
            "no_go_shapes": len(authoring.get("no_go_shapes") or []),
            "routes": len(authoring.get("routes") or []),
            "tag_bindings": len(authoring.get("tag_bindings") or []),
        },
        "bounds": bounds,
        "live": live,
        "gathered_heatmap": _gathered_heatmap_summary(heatmap_snapshot),
        "layers": {
            "semantic": True,
            "heatmap": bool((live.get("costmap") or {}).get("cells")) if isinstance(live, dict) else False,
            "path": bool(live.get("path") or live.get("route")) if isinstance(live, dict) else False,
            "robot": bool(live.get("robot_pose")) if isinstance(live, dict) else False,
            "qr": bool(map_qr_cargo_events),
        },
    }


def _live_with_gathered_heatmap(
    live: dict[str, Any],
    heatmap_snapshot: dict[str, Any] | None,
) -> dict[str, Any]:
    if not isinstance(heatmap_snapshot, dict):
        return live
    costmap = heatmap_snapshot.get("costmap")
    cells = costmap.get("cells") if isinstance(costmap, dict) else None
    if not isinstance(cells, list) or not cells:
        return live
    result = dict(live)
    result["costmap"] = {
        **costmap,
        "source": f"Gathered heatmap {heatmap_snapshot.get('route_run_id') or ''}".strip(),
    }
    result["source"] = result["costmap"]["source"]
    result["status"] = "gathered_heatmap"
    result["gathered_heatmap"] = _gathered_heatmap_summary(heatmap_snapshot)
    if not result.get("ok"):
        result["ok"] = True
    return result


def _gathered_heatmap_summary(snapshot: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(snapshot, dict):
        return None
    costmap = snapshot.get("costmap") if isinstance(snapshot.get("costmap"), dict) else {}
    cells = costmap.get("cells") if isinstance(costmap, dict) else []
    return {
        "route_run_id": snapshot.get("route_run_id"),
        "area_id": snapshot.get("area_id"),
        "collected_at": snapshot.get("collected_at"),
        "cells": len(cells) if isinstance(cells, list) else 0,
        "source": snapshot.get("source"),
    }


def _live_overlay_points(live: dict[str, Any]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for point in [*(live.get("path") or []), *(live.get("route") or [])]:
        maybe_point = _xy_point(point)
        if maybe_point is not None:
            points.append(maybe_point)
    for point in (live.get("robot_pose"), live.get("target")):
        maybe_point = _xy_point(point)
        if maybe_point is not None:
            points.append(maybe_point)
    costmap = live.get("costmap") or {}
    cells = costmap.get("cells") if isinstance(costmap, dict) else None
    if isinstance(cells, list):
        for cell in cells:
            maybe_point = _xy_point(cell)
            if maybe_point is None:
                continue
            x, y = maybe_point
            width = _float_or_none(cell.get("width") if isinstance(cell, dict) else None) or 0.0
            height = _float_or_none(cell.get("height") if isinstance(cell, dict) else None) or 0.0
            points.append((x, y))
            points.append((x + width, y + height))
    return points


def _authoring_entities(authoring: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        str(entity.get("id")): entity
        for entity in authoring.get("entities") or []
        if isinstance(entity, dict) and entity.get("id")
    }


def _authoring_incidents(authoring: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        str(location.get("incident_id")): location
        for location in authoring.get("incident_locations") or []
        if isinstance(location, dict) and location.get("incident_id")
    }


def _authoring_pose(item: dict[str, Any] | None) -> tuple[float, float] | None:
    if not isinstance(item, dict):
        return None
    pose = item.get("pose") if isinstance(item.get("pose"), dict) else item
    if not isinstance(pose, dict):
        return None
    x = _float_or_none(pose.get("x"))
    y = _float_or_none(pose.get("y"))
    if x is None or y is None:
        return None
    return x, y


def _authoring_label(authoring: dict[str, Any], entity_id: str, fallback: object) -> str:
    entity = _authoring_entities(authoring).get(entity_id)
    if entity is None:
        return str(fallback)
    return str(entity.get("label") or fallback)


def _authoring_tag_id(
    authoring: dict[str, Any],
    entity_id: str,
    fallback: object | None,
) -> object | None:
    entity = _authoring_entities(authoring).get(entity_id)
    if entity is not None and entity.get("tag_id") is not None:
        return entity.get("tag_id")
    for binding in authoring.get("tag_bindings") or []:
        if isinstance(binding, dict) and binding.get("entity_id") == entity_id:
            return binding.get("tag_id")
    return fallback


def _selected_authoring_route(authoring: dict[str, Any]) -> dict[str, Any] | None:
    routes = [route for route in authoring.get("routes") or [] if isinstance(route, dict)]
    if not routes:
        return None
    selected_route_id = authoring.get("selected_route_id")
    if selected_route_id:
        for route in routes:
            if route.get("id") == selected_route_id and route.get("waypoints"):
                return route
    routes_with_points = [route for route in routes if route.get("waypoints")]
    return routes_with_points[0] if routes_with_points else None


def _no_go_shape_points(authoring: dict[str, Any]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for shape in authoring.get("no_go_shapes") or []:
        if not isinstance(shape, dict) or not shape.get("enabled", True):
            continue
        for point in shape.get("points") or []:
            maybe_point = _authoring_pose(point)
            if maybe_point is not None:
                points.append(maybe_point)
    return points


def _xy_point(item: Any) -> tuple[float, float] | None:
    if not isinstance(item, dict):
        return None
    x = _float_or_none(item.get("x"))
    y = _float_or_none(item.get("y"))
    if x is None or y is None:
        return None
    return x, y


def _float_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _qr_cargo_overlays(
    qr_events: list[dict[str, Any]],
    entity_points: dict[str, tuple[float, float]],
) -> list[dict[str, Any]]:
    overlays: list[dict[str, Any]] = []
    for event in qr_events:
        payload = event.get("qr_payload") if isinstance(event.get("qr_payload"), dict) else {}
        location_node_id = str(payload.get("location_node_id") or "")
        static_xy = entity_points.get(location_node_id)
        static_pose = _qr_static_pose(static_xy) if static_xy is not None else None
        detection_pose = _qr_detection_pose(event.get("robot_pose_at_detection"))
        map_position = detection_pose or static_pose
        pose_delta = _qr_pose_delta(detection_pose, static_pose)

        overlay = {
            "event_id": event.get("event_id"),
            "warehouse_id": payload.get("warehouse_id"),
            "location_node_id": location_node_id,
            "zone": payload.get("zone"),
            "shelf_id": payload.get("shelf_id"),
            "cargo_id": payload.get("cargo_id"),
            "task": payload.get("task"),
            "timestamp": event.get("timestamp"),
            "status": event.get("status"),
            "action_policy": event.get("action_policy") or "report_only",
            "robot_pose_at_detection": event.get("robot_pose_at_detection"),
            "static_location_node_pose": static_pose,
            "map_position": map_position,
            "pose_delta": pose_delta,
            "source": event.get("source"),
            "linked_entity_id": location_node_id if location_node_id in entity_points else None,
            "qr_payload": payload,
        }
        if map_position is not None:
            overlay["x"] = map_position["x"]
            overlay["y"] = map_position["y"]
        overlays.append(overlay)
    return overlays


def _qr_detection_pose(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    point = _xy_point(value)
    if point is None:
        return None
    return {
        "frame": str(value.get("frame") or "map"),
        "x": point[0],
        "y": point[1],
        "yaw": _float_or_none(value.get("yaw")),
        "source": "robot_pose_at_detection",
    }


def _qr_static_pose(point: tuple[float, float]) -> dict[str, Any]:
    return {
        "frame": "world",
        "x": point[0],
        "y": point[1],
        "theta_deg": None,
        "source": "site_or_authoring",
    }


def _qr_pose_delta(
    detection_pose: dict[str, Any] | None,
    static_pose: dict[str, Any] | None,
) -> dict[str, float] | None:
    if detection_pose is None or static_pose is None:
        return None
    dx = float(detection_pose["x"]) - float(static_pose["x"])
    dy = float(detection_pose["y"]) - float(static_pose["y"])
    return {"dx": dx, "dy": dy, "distance_m": math.hypot(dx, dy)}


def build_route_data(
    state: dict[str, Any],
    report: dict[str, Any],
    *,
    authoring: dict[str, Any] | None = None,
) -> dict[str, Any]:
    map_data = build_map_data(state, report, authoring=authoring)
    checkpoints = {
        str(checkpoint.get("target_id")): checkpoint
        for checkpoint in report.get("checkpoint_verifications") or []
    }
    nav_events = [event for event in state.get("nav_events") or [] if event.get("action") == "goto"]
    nav_by_target = {str(event.get("target_id")): event for event in nav_events}
    stops = []
    for index, stop in enumerate(map_data["route"], 1):
        target_id = str(stop["target_id"])
        event = nav_by_target.get(target_id) or {}
        checkpoint = checkpoints.get(target_id) or {}
        stops.append(
            {
                "sequence": index,
                "target_id": target_id,
                "x": stop["x"],
                "y": stop["y"],
                "success": bool(stop.get("success", True)),
                "guided": bool(stop.get("guided", False)),
                "retries": int(stop.get("retries") or 0),
                "elapsed_s": float(event.get("elapsed_s") or 0.0),
                "note": stop.get("note", ""),
                "expected_tag_id": checkpoint.get("expected_tag_id"),
                "verification_observation_id": checkpoint.get("observation_id"),
                "tag_verified": bool(checkpoint.get("verified", False)),
            }
        )
    nav = report.get("nav_summary") or {}
    return {
        "run_id": report.get("run_id"),
        "mission_id": report.get("mission_id"),
        "route_targets": nav.get("route_targets", len(stops)),
        "route_coverage": nav.get("route_coverage", 0.0),
        "waypoints_reached": nav.get("waypoints_reached", 0),
        "waypoints_total": nav.get("waypoints_total", len(stops)),
        "tag_reacquisition_attempts": nav.get("tag_reacquisition_attempts", 0),
        "tag_reacquisition_successes": nav.get("tag_reacquisition_successes", 0),
        "stops": stops,
    }


def build_poi_data(state: dict[str, Any], report: dict[str, Any]) -> dict[str, Any]:
    observations = state.get("observations") or []
    incidents = report.get("incidents") or []
    captures = []
    for observation in observations:
        observation_id = str(observation.get("id") or "")
        related_incident_ids = [
            str(incident.get("id"))
            for incident in incidents
            if observation_id in (incident.get("evidence_observation_ids") or [])
        ]
        captures.append(
            {
                "id": observation_id,
                "zone_id": observation.get("zone_id"),
                "entity_id": observation.get("entity_id"),
                "tag_id": observation.get("tag_id"),
                "visible_tag_ids": _visible_tag_ids(observation),
                "source": observation.get("source", "unknown"),
                "related_incident_ids": related_incident_ids,
            }
        )

    readings = []
    for asset in (state.get("site") or {}).get("assets") or []:
        asset_id = str(asset.get("id") or "")
        expected_state = asset.get("expected_state") or {}
        if asset.get("expected_clear") is not None:
            raw_clear = _latest_fact_value(observations, f"{asset_id}.clearance_clear")
            clearance_clear = _to_bool(raw_clear)
            if clearance_clear is None:
                clearance_clear = bool(asset.get("expected_clear"))
            readings.append(
                {
                    "asset_id": asset_id,
                    "kind": "clearance",
                    "state": "clear" if clearance_clear else "blocked",
                    "clearance_clear": clearance_clear,
                    "expected_clear": asset.get("expected_clear"),
                }
            )
        threshold = _to_float(expected_state.get("max_celsius"))
        if threshold is not None:
            reading = _to_float(_latest_fact_value(observations, f"{asset_id}.temperature_c"))
            source = "observation"
            if reading is None:
                reading = _to_float(expected_state.get("current_celsius"))
                source = "expected_state"
            if reading is None:
                reading = round(threshold - 2.0, 1)
                source = "deterministic_fallback"
            readings.append(
                {
                    "asset_id": asset_id,
                    "kind": "temperature",
                    "reading_celsius": reading,
                    "max_celsius": threshold,
                    "within_threshold": reading <= threshold,
                    "source": source,
                }
            )
    return {
        "run_id": report.get("run_id"),
        "captures": captures,
        "readings": readings,
    }


def _route_action_count(route: dict[str, Any]) -> int:
    return sum(
        len(waypoint.get("actions") or [])
        for waypoint in route.get("waypoints") or []
        if isinstance(waypoint, dict)
    )


def _route_action_rows(route: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for waypoint in route.get("waypoints") or []:
        if not isinstance(waypoint, dict):
            continue
        for action in waypoint.get("actions") or []:
            if not isinstance(action, dict):
                continue
            rows.append(
                {
                    "waypoint": waypoint.get("label") or waypoint.get("id") or "-",
                    "kind": action.get("kind") or "-",
                    "label": action.get("label") or action.get("kind") or "-",
                    "args": action.get("args") or {},
                }
            )
    return rows


def _render_saved_route_rows(authoring: dict[str, Any]) -> str:
    routes = [route for route in authoring.get("routes") or [] if isinstance(route, dict)]
    if not routes:
        return '<tr><td colspan="6">No saved routes</td></tr>'
    selected_route_id = authoring.get("selected_route_id")
    rows: list[str] = []
    for route in routes:
        route_id = str(route.get("id") or "")
        selected = route_id == selected_route_id
        waypoints = route.get("waypoints") or []
        rows.append(
            f'<tr class="{"is-selected-route" if selected else ""}">'
            f"<td><strong>{escape(str(route.get('label') or route_id))}</strong><br>"
            f"<span>{escape(route_id or '-')}</span></td>"
            f"<td>{'Yes' if selected else ''}</td>"
            f"<td>{len(waypoints)}</td>"
            f"<td>{_route_action_count(route)}</td>"
            "<td>-</td>"
            '<td><div class="route-table-actions">'
            f'<button type="button" data-route-table-action="select" data-route-id="{escape(route_id, quote=True)}">Select</button>'
            f'<button type="button" data-route-table-action="rename" data-route-id="{escape(route_id, quote=True)}">Rename</button>'
            f'<button type="button" data-route-table-action="duplicate" data-route-id="{escape(route_id, quote=True)}">Duplicate</button>'
            f'<button type="button" data-route-table-action="delete" data-route-id="{escape(route_id, quote=True)}">Delete</button>'
            "</div></td></tr>"
        )
        if selected:
            action_rows = _route_action_rows(route)
            if action_rows:
                detail = (
                    '<ol class="route-actions-list">'
                    + "".join(
                        "<li>"
                        f"<strong>{escape(str(action['waypoint']))}</strong>: "
                        f"{escape(str(action['label']))} "
                        f"<span>({escape(str(action['kind']))})</span> "
                        f"<code>{escape(json.dumps(action['args'], sort_keys=True))}</code>"
                        "</li>"
                        for action in action_rows
                    )
                    + "</ol>"
                )
            else:
                detail = "No actions saved for this route"
            rows.append(f'<tr class="route-actions-subrow"><td colspan="6">{detail}</td></tr>')
    return "".join(rows)


def render_site_map(
    state: dict[str, Any],
    report: dict[str, Any],
    *,
    authoring: dict[str, Any] | None = None,
    qr_events: list[dict[str, Any]] | None = None,
) -> str:
    map_data = build_map_data(state, report, authoring=authoring, qr_events=qr_events)
    if not map_data["zones"]:
        return '<div class="map-empty">Map data unavailable</div>'

    bounds = map_data["bounds"]
    bounds_attr = escape(json.dumps(bounds, separators=(",", ":")), quote=True)
    authoring_attr = escape(json.dumps(authoring or {}, separators=(",", ":")), quote=True)
    projector = _MapProjector(bounds)
    route_points = " ".join(
        f"{projector.x(point['x']):.1f},{projector.y(point['y']):.1f}"
        for point in map_data["route"]
    )
    grid = _render_grid(projector)
    floor_cells = _render_floor_cells(projector, map_data)
    point_cloud = _render_point_cloud(projector, map_data)
    no_go = "".join(_render_no_go_zone(projector, zone) for zone in map_data["zones"])
    no_go += "".join(
        _render_no_go_shape(projector, shape) for shape in map_data["no_go_shapes"]
    )
    zones = "".join(_render_zone(projector, zone) for zone in map_data["zones"])
    assets = "".join(_render_asset(projector, asset) for asset in map_data["assets"])
    packages = "".join(_render_package(projector, package) for package in map_data["packages"])
    observations = "".join(
        _render_observation(projector, observation) for observation in map_data["observations"]
    )
    incidents = "".join(
        _render_incident(projector, incident) for incident in map_data["incidents"]
    )
    qr_cargo = "".join(
        _render_qr_cargo_event(projector, event)
        for event in map_data["qr_cargo_events"]
    )
    route = ""
    if route_points:
        route = (
            f'<polyline class="map-route" points="{route_points}" />'
            + "".join(_render_route_stop(projector, stop, index) for index, stop in enumerate(map_data["route"], 1))
        )
    robot = _render_live_robot_pose()
    rerun_web_url = _trusted_rerun_web_url(os.environ.get("DOGOPS_RERUN_WEB_URL"))
    rerun_web_url_attr = escape(rerun_web_url, quote=True)
    legend = (
        '<div class="map-legend">'
        '<span><i class="legend-free"></i>free grid</span>'
        '<span><i class="legend-heatmap"></i>DimOS heatmap</span>'
        '<span><i class="legend-route"></i>trajectory</span>'
        '<span><i class="legend-live"></i>live odom</span>'
        '<span><i class="legend-tag"></i>tag return</span>'
        '<span><i class="legend-no-go"></i>no-go cost</span>'
        '<span><i class="legend-incident"></i>P1/P2 event</span>'
        '<span><i class="legend-qr"></i>QR cargo</span>'
        "</div>"
    )
    layer_controls = (
        '<div class="map-layer-controls" data-map-layer-controls>'
        '<button type="button" data-map-layer="semantic" aria-pressed="true">'
        '<i class="layer-semantic"></i>Semantic</button>'
        '<button type="button" data-map-layer="heatmap" aria-pressed="true">'
        '<i class="legend-heatmap"></i>Heatmap</button>'
        '<button type="button" data-map-layer="path" aria-pressed="true">'
        '<i class="legend-route"></i>Path</button>'
        '<button type="button" data-map-layer="robot" aria-pressed="true">'
        '<i class="legend-live"></i>Robot</button>'
        '<button type="button" data-map-layer="qr" aria-pressed="true">'
        '<i class="legend-qr"></i>QR</button>'
        "</div>"
    )
    edit_controls = (
        '<div class="map-edit-controls" data-map-edit-controls>'
        '<div class="map-edit-row" data-map-edit-general-row>'
        '<button type="button" data-map-edit-mode="select" aria-pressed="true">Select</button>'
        '<button type="button" data-map-edit-mode="home" aria-pressed="false">Set Home</button>'
        '<button type="button" data-map-edit-action="delete_selected">Delete</button>'
        '<button type="button" data-map-edit-action="save">Save</button>'
        '<button type="button" data-map-edit-action="reset">Reset</button>'
        '<button type="button" data-map-edit-action="export">Export</button>'
        "</div>"
        '<div class="map-edit-row" data-map-edit-label-row>'
        '<button type="button" data-map-edit-mode="zone" aria-pressed="false">Label</button>'
        '<button type="button" data-map-edit-mode="asset" aria-pressed="false">Asset</button>'
        '<button type="button" data-map-edit-mode="package" aria-pressed="false">Package</button>'
        '<button type="button" data-map-edit-mode="no_go" aria-pressed="false">No-Go</button>'
        '<button type="button" data-map-edit-mode="incident" aria-pressed="false">Incident</button>'
        '<button type="button" data-map-edit-mode="tag" aria-pressed="false">Bind Tag</button>'
        '<button type="button" data-map-edit-action="use_observation">Use Observation</button>'
        '<button type="button" data-map-edit-action="publish_no_go">Publish No-Go</button>'
        "</div>"
        '<div class="map-edit-row" data-map-edit-route-row>'
        '<button type="button" data-map-edit-mode="route" aria-pressed="false">Route</button>'
        '<button type="button" data-map-edit-action="route_select">Select Route</button>'
        '<button type="button" data-map-edit-action="dry_run_route">Dry Run Route</button>'
        '<button type="button" data-map-edit-action="run_route">Run Live Route</button>'
        '<label class="map-route-planner-toggle"><input type="checkbox" data-route-use-planner /> Use planner</label>'
        '<button type="button" data-map-edit-action="stop_route">Stop Route</button>'
        '<button type="button" data-map-edit-action="heatmap_run">Heatmap Run</button>'
        '<button type="button" data-map-edit-action="route_up">Route Up</button>'
        '<button type="button" data-map-edit-action="route_down">Route Down</button>'
        '<span class="map-route-summary" data-map-route-summary>Selected route: none. Next: Route1</span>'
        "</div>"
        "</div>"
    )
    route_action_controls = (
        '<div class="map-edit-row map-route-action-row" data-route-action-row hidden>'
        '<span class="map-route-summary" data-route-action-summary>Select a waypoint to add actions</span>'
        '<button type="button" data-route-action-kind="capture_image">Capture Image</button>'
        '<button type="button" data-route-action-kind="gemini_inspect_image">Gemini Inspect</button>'
        '<button type="button" data-route-action-kind="scan_qr">Scan QR</button>'
        '<button type="button" data-route-action-kind="scan_tags">Scan Tags</button>'
        '<button type="button" data-route-action-kind="wait">Wait</button>'
        '<button type="button" data-route-action-kind="inspect_asset">Inspect Asset</button>'
        '<button type="button" data-route-action-kind="verify_work_order">Verify Work Order</button>'
        '<button type="button" data-route-action-kind="operator_prompt">Operator Prompt</button>'
        "</div>"
    )
    return f"""
      <div class="map-shell" data-map-surface>
        {_render_rerun_surface(rerun_web_url)}
        {edit_controls}
        <svg class="site-map" role="img" aria-label="DogOps mission map"
          data-live-map-svg data-map-bounds="{bounds_attr}" data-map-authoring="{authoring_attr}"
          viewBox="0 0 {MAP_WIDTH} {MAP_HEIGHT}">
          <defs>
            <filter id="dogops-map-glow" x="-50%" y="-50%" width="200%" height="200%">
              <feGaussianBlur stdDeviation="2.5" result="blur" />
              <feMerge>
                <feMergeNode in="blur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
            <pattern id="dogops-map-hatch" width="8" height="8" patternUnits="userSpaceOnUse">
              <path d="M-2,8 L8,-2 M0,10 L10,0" class="map-hatch-line" />
            </pattern>
          </defs>
          <rect class="map-bg" x="0" y="0" width="{MAP_WIDTH}" height="{MAP_HEIGHT}" rx="8" />
          <g data-layer="heatmap" data-live-heatmap></g>
          <g data-layer="semantic">
            {floor_cells}
            {grid}
            {point_cloud}
            {no_go}
            {zones}
            {assets}
            {packages}
            {observations}
            {incidents}
          </g>
          <g data-layer="path">
            {route}
            <polyline class="map-dimos-path" data-live-path points="" />
          </g>
          <g data-layer="qr" data-qr-cargo-layer>
            {qr_cargo}
          </g>
          <g data-layer="robot">
            {robot}
          </g>
        </svg>
        {route_action_controls}
        {layer_controls}
        {legend}
        <div class="map-workflow">
          <a href="{rerun_web_url_attr}" target="_blank" rel="noreferrer" data-rerun-web-link>Open Rerun Web</a>
          <span class="map-command-status" data-map-command-status>Map command idle</span>
        </div>
        <div class="map-authoring-status" data-map-authoring-status>Map authoring idle</div>
        <div class="map-route-execution-status" data-route-execution-status>Execution: idle</div>
        <div class="map-live-status" data-live-map-status>Live odom: waiting for Go2</div>
      </div>
    """


def render_dashboard_html(
    state: dict[str, Any],
    report: dict[str, Any],
    *,
    robot_control_token: str | None = None,
    authoring: dict[str, Any] | None = None,
    qr_events: list[dict[str, Any]] | None = None,
) -> str:
    run = state["run"]
    nav = report.get("nav_summary") or {}
    packages = report.get("packages") or []
    incidents = report.get("incidents") or []
    work_orders = report.get("work_orders") or []
    packages_metric = f"{report['packages_observed']}/{report['packages_expected']}"
    nav_metric = f"{nav.get('waypoints_reached', 0)}/{nav.get('waypoints_total', 0)}"
    checkpoint_metric = f"{report.get('checkpoints_verified', 0)}/{report.get('checkpoints_total', 0)}"
    tag_recovery_metric = (
        f"{nav.get('tag_reacquisition_successes', 0)}/"
        f"{nav.get('tag_reacquisition_attempts', 0)}"
    )
    mean_target_time_metric = f"{nav.get('mean_elapsed_s', 0):.1f}s"
    route_coverage_metric = f"{float(nav.get('route_coverage', 0.0)) * 100:.0f}%"
    qr_events = qr_events or []
    map_html = render_site_map(state, report, authoring=authoring, qr_events=qr_events)
    route_table_rows = _render_saved_route_rows(authoring or {})
    qr_map_data = build_map_data(state, report, authoring=authoring, qr_events=qr_events)
    route_data = build_route_data(state, report, authoring=authoring)
    poi_data = build_poi_data(state, report)
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>DogOps SiteOps Agent</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f6f7f9;
      --ink: #17202a;
      --muted: #5b6776;
      --line: #d7dce3;
      --panel: #ffffff;
      --accent: #0f766e;
      --warn: #b45309;
      --danger: #b91c1c;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--ink);
      font: 14px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    header {{
      background: #111827;
      color: white;
      padding: 18px 28px;
      display: flex;
      justify-content: space-between;
      gap: 18px;
      align-items: end;
    }}
    h1, h2 {{ margin: 0; }}
    h1 {{ font-size: 22px; font-weight: 700; letter-spacing: 0; }}
    h2 {{ font-size: 15px; margin-bottom: 8px; }}
    main {{
      padding: 22px 28px 32px;
      display: grid;
      grid-template-columns: minmax(360px, 1.05fr) minmax(320px, 0.95fr);
      gap: 16px;
      align-items: start;
    }}
    section {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
    }}
    .map-stack {{ display: grid; gap: 16px; }}
    .ops-stack {{ display: grid; gap: 10px; }}
    .wide {{ grid-column: 1 / -1; }}
    .metric-row {{
      display: grid;
      grid-template-columns: repeat(5, minmax(120px, 1fr));
      gap: 10px;
    }}
    .ops-stack .metric-row {{ grid-template-columns: repeat(4, minmax(0, 1fr)); }}
    .metric {{
      border: 1px solid var(--line);
      border-radius: 6px;
      min-height: 66px;
      padding: 8px 10px;
      background: #fbfcfd;
    }}
    .metric strong {{ display: block; font-size: 19px; }}
    .muted {{ color: var(--muted); }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border-bottom: 1px solid var(--line); padding: 8px 6px; text-align: left; }}
    th {{ color: var(--muted); font-size: 12px; text-transform: uppercase; }}
    .state-resolved, .state-verified, .state-verified_closed, .state-found_ok {{ color: var(--accent); font-weight: 700; }}
    .state-open, .state-missing {{ color: var(--danger); font-weight: 700; }}
    .severity-P1 {{ color: var(--danger); font-weight: 700; }}
    .timeline {{ display: grid; gap: 8px; }}
    .timeline div {{ border-left: 3px solid var(--accent); padding-left: 10px; }}
    .evidence-grid {{
      display: grid;
      gap: 12px;
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
    }}
    .saved-image-grid {{
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
    }}
    .saved-image {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfcfd;
      overflow: hidden;
    }}
    .saved-image img {{
      display: block;
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: cover;
      background: #111827;
    }}
    .saved-image figcaption {{
      padding: 7px 9px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.35;
    }}
    .compact-list {{
      display: grid;
      gap: 8px;
      list-style: none;
      margin: 0;
      padding: 0;
    }}
    .compact-list li {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfcfd;
      padding: 8px 10px;
    }}
    .map-panel {{
      background: #07090d;
      border-color: #1d2430;
      color: #d8dee9;
      min-height: 675px;
      overflow: hidden;
    }}
    .map-panel h2 {{ color: #eef2f8; }}
    .map-shell {{ display: grid; gap: 10px; }}
    .camera-panel {{
      background: #07090d;
      border-color: #1d2430;
      color: #d8dee9;
      display: grid;
      gap: 10px;
    }}
    .camera-header {{
      align-items: center;
      display: flex;
      justify-content: space-between;
      gap: 12px;
    }}
    .camera-header h2 {{ color: #eef2f8; }}
    .camera-health {{ color: #fbbf24; font-weight: 650; }}
    .camera-view {{
      background: #03060b;
      border: 1px solid #1d2430;
      border-radius: 8px;
      display: grid;
      min-height: 260px;
      overflow: hidden;
      place-items: center;
      position: relative;
    }}
    .camera-view img {{
      background: #03060b;
      height: 100%;
      object-fit: contain;
      width: 100%;
    }}
    .camera-view img[hidden] {{ display: none; }}
    .camera-empty {{
      align-items: center;
      background: #03060b;
      color: #c8d0dc;
      display: grid;
      inset: 0;
      justify-items: center;
      padding: 18px;
      position: absolute;
      text-align: center;
    }}
    .camera-empty[hidden] {{ display: none; }}
    .camera-empty strong {{
      color: #eef2f8;
      display: block;
      margin-bottom: 6px;
    }}
    .camera-meta {{
      color: #c8d0dc;
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      font-size: 12px;
    }}
    .rerun-surface {{
      background: #03060b;
      border: 1px solid #1d2430;
      border-radius: 8px;
      display: grid;
      min-height: 220px;
      overflow: hidden;
      position: relative;
    }}
    .rerun-surface iframe {{
      background: #03060b;
      border: 0;
      display: block;
      height: 100%;
      min-height: 220px;
      width: 100%;
    }}
    .rerun-standby {{
      align-items: center;
      background:
        linear-gradient(135deg, rgba(82, 224, 196, 0.12), rgba(125, 211, 252, 0.08)),
        repeating-linear-gradient(90deg, rgba(148, 163, 184, 0.08) 0 1px, transparent 1px 34px),
        repeating-linear-gradient(0deg, rgba(148, 163, 184, 0.08) 0 1px, transparent 1px 34px),
        #05070c;
      display: grid;
      gap: 10px;
      inset: 0;
      justify-items: center;
      padding: 18px;
      position: absolute;
      text-align: center;
      z-index: 1;
    }}
    .rerun-standby[hidden] {{ display: none; }}
    .rerun-chip {{
      border: 1px solid #2d3a4f;
      border-radius: 999px;
      color: #d8fff6;
      font-size: 12px;
      font-weight: 700;
      padding: 5px 10px;
    }}
    .rerun-status {{
      color: #a9b4c4;
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
      min-height: 16px;
    }}
    .rerun-controls {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      justify-content: center;
    }}
    .rerun-controls button, .rerun-controls a {{
      border: 1px solid #334155;
      border-radius: 6px;
      background: #0d1119;
      color: #e5edf5;
      cursor: pointer;
      font: inherit;
      min-height: 34px;
      padding: 6px 10px;
      text-decoration: none;
    }}
    .rerun-controls button:hover, .rerun-controls a:hover {{ border-color: #52e0c4; }}
    .site-map {{
      aspect-ratio: 23 / 14;
      background: #03060b;
      border: 1px solid #1d2430;
      border-radius: 8px;
      display: block;
      max-height: 580px;
      min-height: 420px;
      width: 100%;
    }}
    .map-bg {{ fill: #05070c; }}
    .map-free-cell {{ fill: #484981; opacity: 0.42; }}
    .map-cost-cell {{ fill: #171b2b; opacity: 0.72; }}
    .map-grid {{ stroke: #18202c; stroke-width: 1; }}
    .map-grid-major {{ stroke: #2f4058; stroke-width: 1.2; }}
    .map-hatch-line {{ stroke: #f87171; stroke-width: 1; opacity: 0.35; }}
    .map-point {{ fill: #7dd3fc; opacity: 0.46; }}
    .map-point.hot {{ fill: #f0abfc; opacity: 0.62; }}
    .map-live-cost-cell {{ opacity: 0.64; stroke: rgba(255, 255, 255, 0.08); stroke-width: 0.4; }}
    .map-no-go {{
      fill: rgba(127, 29, 29, 0.42);
      stroke: #ef4444;
      stroke-dasharray: 8 5;
      stroke-width: 1.6;
    }}
    .map-no-go-hatch {{ fill: url(#dogops-map-hatch); opacity: 0.55; }}
    .map-route {{
      fill: none;
      filter: url(#dogops-map-glow);
      stroke: #52e0c4;
      stroke-linecap: round;
      stroke-linejoin: round;
      stroke-width: 4;
    }}
    .map-route-stop {{ fill: #05070c; stroke: #52e0c4; stroke-width: 2; transition: fill 120ms ease, r 120ms ease, stroke-width 120ms ease; }}
    .map-route-stop-marker {{ cursor: pointer; }}
    .map-route-stop-marker.is-selected .map-route-stop {{
      fill: #145c52;
      stroke: #d8fff6;
      stroke-width: 3;
    }}
    .map-route-stop-marker.is-selected .map-route-index {{ fill: #ffffff; font-size: 13px; }}
    .map-route-index {{
      dominant-baseline: central;
      fill: #d8fff6;
      font-size: 11px;
      font-weight: 700;
      text-anchor: middle;
    }}
    .map-zone-anchor {{ fill: #05070c; stroke: #8b95a7; stroke-width: 1.5; }}
    .map-zone-label {{
      fill: #b9c4d5;
      font-size: 11px;
      font-weight: 650;
      letter-spacing: 0.03em;
      paint-order: stroke;
      stroke: #05070c;
      stroke-linejoin: round;
      stroke-width: 4px;
      text-anchor: middle;
    }}
    .map-zone-label, .map-asset-label, .map-package-label {{
      fill: #e5edf5;
      font-size: 11px;
      paint-order: stroke;
      stroke: #05070c;
      stroke-width: 4px;
      stroke-linejoin: round;
    }}
    .map-asset {{ fill: #c7f9ff; stroke: #22d3ee; stroke-width: 1.5; }}
    .map-asset-label {{ fill: #d6fbff; }}
    .map-tag-face {{ fill: #05070c; stroke: #d1d5db; stroke-width: 1.5; }}
    .map-tag-core {{ fill: #d1d5db; }}
    .map-package {{ fill: #f59e0b; stroke: #fef3c7; stroke-width: 1.2; }}
    .map-package.state-found_ok {{ fill: #34d399; stroke: #bbf7d0; }}
    .map-package.state-missing {{ fill: #7f1d1d; stroke: #f87171; stroke-dasharray: 4 3; }}
    .map-package.state-wrong_zone, .map-package.state-blocking_asset {{
      fill: #fb923c;
      stroke: #fed7aa;
    }}
    .map-observation {{ fill: #05070c; stroke: #a78bfa; stroke-width: 2; }}
    .map-qr-cargo-ring {{ fill: rgba(250, 204, 21, 0.18); stroke: #facc15; stroke-width: 2.2; }}
    .map-qr-cargo-core {{ fill: #0f766e; stroke: #d9f99d; stroke-width: 1.4; }}
    .map-qr-cargo-label {{
      fill: #fef9c3;
      font-size: 11px;
      font-weight: 700;
      paint-order: stroke;
      stroke: #05070c;
      stroke-linejoin: round;
      stroke-width: 4px;
    }}
    .map-qr-cargo-marker.is-selected .map-qr-cargo-ring {{ stroke: #ffffff; stroke-width: 3.2; }}
    .qr-linked .map-zone-anchor, .qr-linked .map-asset, .qr-linked .map-package {{
      filter: url(#dogops-map-glow);
      stroke: #facc15;
      stroke-width: 3;
    }}
    .map-observation-ray {{ stroke: #a78bfa; stroke-dasharray: 4 5; stroke-width: 1.2; }}
    .map-incident {{ fill: none; filter: url(#dogops-map-glow); stroke: #fb7185; stroke-width: 2.4; }}
    .map-incident-label {{
      fill: #fecdd3;
      font-size: 11px;
      font-weight: 700;
      paint-order: stroke;
      stroke: #05070c;
      stroke-linejoin: round;
      stroke-width: 4px;
    }}
    .map-robot {{ fill: rgba(82, 224, 196, 0.12); stroke: #52e0c4; stroke-width: 1.5; }}
    .map-robot-core {{ fill: #52e0c4; stroke: #d8fff6; stroke-width: 1.2; }}
    .map-live-trace {{
      fill: none;
      filter: url(#dogops-map-glow);
      stroke: #facc15;
      stroke-linecap: round;
      stroke-linejoin: round;
      stroke-width: 3.5;
    }}
    .map-dimos-path {{
      fill: none;
      filter: url(#dogops-map-glow);
      stroke: #38bdf8;
      stroke-dasharray: 9 7;
      stroke-linecap: round;
      stroke-linejoin: round;
      stroke-width: 3;
    }}
    .map-live-robot-halo {{ fill: rgba(250, 204, 21, 0.16); stroke: #facc15; stroke-width: 1.5; }}
    .map-live-robot-core {{ fill: #facc15; stroke: #fff7cc; stroke-width: 1.2; }}
    .map-dimos-target-ring {{ fill: rgba(56, 189, 248, 0.16); stroke: #38bdf8; stroke-width: 2; }}
    .map-dimos-target-core {{ fill: #38bdf8; stroke: #e0f2fe; stroke-width: 1.2; }}
    .map-go-to-ring {{ fill: rgba(248, 113, 113, 0.14); stroke: #f87171; stroke-width: 2; }}
    .map-go-to-cross {{ stroke: #fecaca; stroke-linecap: round; stroke-width: 2; }}
    .map-axis-label {{ fill: #657184; font-size: 10px; }}
    .site-map.go-to-armed {{ cursor: crosshair; }}
    .map-legend, .map-layer-controls {{
      color: #a9b4c4;
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      font-size: 12px;
    }}
    .map-legend span, .map-layer-controls button {{
      align-items: center;
      display: inline-flex;
      gap: 6px;
    }}
    .map-legend i, .map-layer-controls i {{
      border-radius: 999px;
      display: inline-block;
      height: 10px;
      width: 10px;
    }}
    .map-layer-controls button {{
      background: transparent;
      border: 0;
      color: inherit;
      cursor: pointer;
      font: inherit;
      padding: 0;
    }}
    .map-layer-controls button[aria-pressed="true"] {{
      color: #d8fff6;
      font-weight: 700;
    }}
    .map-layer-controls button[aria-pressed="false"] {{ opacity: 0.48; }}
    .legend-free {{ background: #484981; }}
    .legend-heatmap {{ background: #f97316; }}
    .legend-route {{ background: #52e0c4; }}
    .legend-live {{ background: #facc15; }}
    .legend-tag {{ background: #a78bfa; }}
    .legend-no-go {{ background: #ef4444; }}
    .legend-incident {{ background: #fb7185; }}
    .layer-semantic {{ background: #a78bfa; }}
    .legend-qr {{ background: #facc15; }}
    .map-workflow {{
      align-items: center;
      color: #a9b4c4;
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      font-size: 12px;
    }}
    .map-workflow a {{
      color: #bfdbfe;
      font-weight: 700;
      text-decoration: none;
    }}
    .map-workflow a:hover {{ text-decoration: underline; }}
    .map-command-status {{
      color: #fecaca;
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
    }}
    .map-command-status.ok {{ color: #86efac; }}
    .map-command-status.error {{ color: #fca5a5; }}
    .map-edit-controls {{
      display: grid;
      gap: 8px;
    }}
    .map-edit-row {{
      align-items: center;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }}
    .map-edit-controls button {{
      background: #0d1119;
      border: 1px solid #334155;
      border-radius: 999px;
      color: #d8dee9;
      cursor: pointer;
      font: inherit;
      min-height: 30px;
      padding: 5px 10px;
    }}
    .map-edit-controls button[aria-pressed="true"] {{
      background: #123b36;
      border-color: #52e0c4;
      color: #d8fff6;
      font-weight: 700;
    }}
    .map-route-planner-toggle {{
      align-items: center;
      border: 1px solid #334155;
      border-radius: 999px;
      color: #cbd5e1;
      display: inline-flex;
      gap: 6px;
      min-height: 30px;
      padding: 5px 10px;
    }}
    .map-route-planner-toggle input {{
      accent-color: #52e0c4;
      margin: 0;
    }}
    .map-route-action-row[hidden] {{ display: none; }}
    .map-route-action-row {{
      background: rgba(8, 13, 22, 0.72);
      border: 1px solid #243244;
      border-radius: 8px;
      padding: 8px;
    }}
    .map-route-action-row button {{
      background: #123b36;
      border-color: #52e0c4;
      color: #d8fff6;
    }}
    .map-route-table {{
      background: #ffffff;
      border: 1px solid #d1d5db;
      border-radius: 8px;
      color: #111827;
      overflow: hidden;
    }}
    .map-route-table strong {{
      display: block;
      padding: 8px 10px;
    }}
    .map-route-table table {{
      border-collapse: collapse;
      width: 100%;
    }}
    .map-route-table th,
    .map-route-table td {{
      border-top: 1px solid #d1d5db;
      color: #111827;
      padding: 6px 8px;
      text-align: left;
      vertical-align: top;
    }}
    .map-route-table th {{
      background: #f3f4f6;
      color: #111827;
      font-size: 11px;
      text-transform: uppercase;
    }}
    .map-route-table tr.is-selected-route > td {{
      background: #dbeafe;
    }}
    .route-table-actions {{
      display: flex;
      flex-wrap: wrap;
      gap: 6px;
    }}
    .route-table-actions button {{
      background: #ffffff;
      border: 1px solid #9ca3af;
      border-radius: 999px;
      color: #111827;
      cursor: pointer;
      font: inherit;
      padding: 4px 8px;
    }}
    .route-actions-subrow td {{
      background: #f9fafb;
    }}
    .route-actions-list {{
      margin: 0;
      padding-left: 18px;
    }}
    .map-route-summary {{
      color: #a9b4c4;
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
      min-height: 30px;
      display: inline-flex;
      align-items: center;
    }}
    .map-live-status {{
      color: #f7d75d;
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
      min-height: 18px;
    }}
    .map-authoring-status {{
      color: #c4b5fd;
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
      min-height: 18px;
    }}
    .map-authoring-status.ok {{ color: #86efac; }}
    .map-authoring-status.error {{ color: #fca5a5; }}
    .map-route-execution-status {{
      color: #bfdbfe;
      font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
      min-height: 18px;
    }}
    .map-route-execution-status.ok {{ color: #86efac; }}
    .map-route-execution-status.error {{ color: #fca5a5; }}
    .route-run-history, .route-run-timeline {{
      background: #ffffff;
      border: 1px solid #d1d5db;
      border-radius: 8px;
      color: #111827;
      display: grid;
      gap: 6px;
      font-size: 12px;
      overflow: hidden;
    }}
    .route-run-history table, .route-run-timeline table {{
      border-collapse: collapse;
      width: 100%;
    }}
    .route-run-history th, .route-run-history td,
    .route-run-timeline th, .route-run-timeline td {{
      border-bottom: 1px solid #d1d5db;
      color: #111827;
      padding: 5px 4px;
      text-align: left;
      vertical-align: top;
    }}
    .route-run-history th, .route-run-timeline th {{
      background: #f3f4f6;
      color: #111827;
      font-size: 10px;
      text-transform: uppercase;
    }}
    .route-run-history tr.is-selected-route > td {{
      background: #ecfeff;
    }}
    .route-run-history button {{
      background: #ffffff;
      border: 1px solid #9ca3af;
      border-radius: 999px;
      color: #111827;
      cursor: pointer;
      font: inherit;
      padding: 4px 8px;
    }}
    .map-empty {{
      align-items: center;
      background: #101721;
      border: 1px solid #263241;
      border-radius: 8px;
      color: #cbd5e1;
      display: flex;
      min-height: 420px;
      justify-content: center;
    }}
    .robot-controls {{
      display: grid;
      grid-template-columns: repeat(3, minmax(72px, 1fr));
      gap: 6px;
    }}
    .keyboard-map {{
      color: var(--muted);
      display: grid;
      gap: 6px;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      margin-top: 8px;
    }}
    .keyboard-map span {{
      align-items: center;
      display: inline-flex;
      gap: 6px;
      min-width: 0;
    }}
    kbd {{
      background: #eef2f7;
      border: 1px solid #cfd6df;
      border-bottom-color: #b8c0cc;
      border-radius: 4px;
      color: #17202a;
      display: inline-block;
      font: 11px/1.1 ui-monospace, SFMono-Regular, Menlo, monospace;
      min-width: 22px;
      padding: 3px 5px;
      text-align: center;
    }}
    .posture-controls {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-bottom: 8px;
    }}
    .map-controls {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-bottom: 8px;
    }}
    .posture-controls button {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #f8fafc;
      color: var(--ink);
      cursor: pointer;
      font: inherit;
      min-height: 34px;
      padding: 6px 10px;
    }}
    .map-controls button {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #f8fafc;
      color: var(--ink);
      cursor: pointer;
      font: inherit;
      min-height: 34px;
      padding: 6px 10px;
    }}
    .posture-controls button:hover {{ border-color: var(--accent); }}
    .map-controls button:hover {{ border-color: var(--accent); }}
    .map-controls button[aria-pressed="true"] {{
      background: #fef2f2;
      border-color: var(--danger);
      color: var(--danger);
      font-weight: 700;
    }}
    .posture-controls button:disabled {{ cursor: wait; opacity: 0.65; }}
    .map-controls button:disabled {{ cursor: wait; opacity: 0.65; }}
    .motion-controls {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-bottom: 8px;
    }}
    .motion-controls button {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #f8fafc;
      color: var(--ink);
      cursor: pointer;
      font: inherit;
      min-height: 34px;
      padding: 6px 10px;
    }}
    .motion-controls button[aria-pressed="true"] {{
      background: #e6f4f1;
      border-color: var(--accent);
      color: var(--accent);
      font-weight: 700;
    }}
    .motion-controls button:disabled {{ cursor: wait; opacity: 0.65; }}
    .robot-controls button {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #f8fafc;
      color: var(--ink);
      cursor: pointer;
      font: inherit;
      min-height: 36px;
      padding: 6px 10px;
    }}
    .robot-controls button:hover {{ border-color: var(--accent); }}
    .robot-controls button:disabled {{ cursor: wait; opacity: 0.65; }}
    .robot-controls .hard-stop {{
      background: var(--danger);
      border-color: var(--danger);
      color: #ffffff;
      font-weight: 700;
    }}
    .robot-status {{
      min-height: 18px;
      margin-top: 8px;
      color: var(--muted);
    }}
    .robot-status.error {{ color: var(--danger); }}
    .robot-status.ok {{ color: var(--accent); }}
    .qr-cargo-status {{ color: var(--muted); margin-bottom: 8px; min-height: 18px; }}
    .qr-cargo-status.ok {{ color: var(--accent); }}
    .qr-cargo-status.error {{ color: var(--danger); }}
    .qr-cargo-table {{ overflow-x: auto; }}
    .qr-cargo-table table {{ min-width: 980px; }}
    .qr-actions {{ display: flex; flex-wrap: wrap; gap: 6px; min-width: 260px; }}
    .qr-actions button {{
      background: #f8fafc;
      border: 1px solid var(--line);
      border-radius: 6px;
      color: var(--ink);
      cursor: pointer;
      font: inherit;
      min-height: 30px;
      padding: 4px 8px;
    }}
    .qr-actions button:hover {{ border-color: var(--accent); }}
    .qr-actions button:disabled {{ cursor: not-allowed; opacity: 0.55; }}
    @media (max-width: 920px) {{
      main {{ gap: 12px; padding: 16px 18px 24px; }}
      section {{ padding: 10px; }}
      .ops-stack {{ gap: 8px; }}
      .map-panel {{ min-height: 0; }}
      .site-map {{ min-height: 320px; }}
      .keyboard-map {{
        font-size: 12px;
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }}
    }}
    @media (max-width: 720px) {{
      header {{ align-items: start; flex-direction: column; }}
      main {{ grid-template-columns: 1fr; padding: 14px; }}
      .metric-row {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
      .evidence-grid {{ grid-template-columns: 1fr; }}
      .site-map {{ min-height: 330px; }}
    }}
  </style>
</head>
<body>
  <header>
    <div>
      <h1>DogOps SiteOps Agent</h1>
      <div class="muted">Mission {escape(str(run["mission_id"]))} / run {escape(str(run["id"]))}</div>
    </div>
    <div>State: <strong>{escape(str(run["state"]))}</strong></div>
  </header>
  <main>
    <div class="map-stack">
      <section class="map-panel">
        <h2>Mission Map</h2>
        {map_html}
      </section>
      <section class="camera-panel">
        <div class="camera-header">
          <h2>DimOS Camera</h2>
          <span class="camera-health" data-camera-health>Waiting for /color_image</span>
        </div>
        <div class="camera-view">
          <img src="about:blank" alt="Latest DimOS color_image frame" hidden data-camera-frame>
          <div class="camera-empty" data-camera-empty>
            <span><strong>No camera frame yet</strong>Waiting for DimOS color_image on /color_image.</span>
          </div>
        </div>
        <div class="camera-meta">
          <span data-camera-topic>Topic: /color_image</span>
          <span data-camera-shape>Frame: pending</span>
          <span data-camera-age>Age: pending</span>
        </div>
      </section>
    </div>
    <div class="ops-stack">
      <section>
        <h2>Run Summary</h2>
        <div class="metric-row">
          {metric("Packages", packages_metric)}
          {metric("Exceptions", report["manifest_exceptions"])}
          {metric("Incidents", report["incidents_opened"])}
          {metric("Verified WOs", report["work_orders_verified_closed"])}
          {metric("Nav", nav_metric)}
          {metric("Tag Sign-In", checkpoint_metric)}
          {metric("Coverage", route_coverage_metric)}
        </div>
      </section>
      <section>
        <h2>QR Cargo</h2>
        {qr_cargo_panel(qr_map_data["qr_cargo_events"])}
      </section>
      <section>
        <h2>Robot Control</h2>
        <div class="posture-controls" data-posture-controls>
          <button type="button" data-posture="wake">Wake / Stand</button>
          <button type="button" data-posture="balance">Balance</button>
          <button type="button" data-posture="sleep">Sleep</button>
        </div>
        <div class="map-controls" data-map-controls>
          <button type="button" data-map-action="start">Start Live Map</button>
          <button type="button" data-map-action="gather_heatmap">Gather Heatmap</button>
          <button type="button" data-map-action="scan_zone">Scan Zone</button>
          <button type="button" data-map-action="origin">Set Map Origin</button>
          <button type="button" data-map-action="arm_go_to" aria-pressed="false">Arm Go To</button>
        </div>
        <div class="motion-controls" data-motion-controls>
          <button type="button" data-motion="nudge" aria-pressed="true">Nudge</button>
          <button type="button" data-motion="step" aria-pressed="false">Step</button>
          <button type="button" data-motion="walk" aria-pressed="false">Walk</button>
        </div>
        <div class="robot-controls" data-robot-controls>
          <span></span>
          <button type="button" data-command="forward" data-key-hint="W / Up">Forward</button>
          <span></span>
          <button type="button" data-command="left" data-key-hint="A / Left">Left</button>
          <button type="button" class="hard-stop" data-command="hard_stop" data-key-hint="Space / Esc">HARD STOP</button>
          <button type="button" data-command="right" data-key-hint="D / Right">Right</button>
          <button type="button" data-command="yaw_left" data-key-hint="Q">Yaw L</button>
          <button type="button" data-command="backward" data-key-hint="S / Down">Back</button>
          <button type="button" data-command="yaw_right" data-key-hint="E">Yaw R</button>
        </div>
        <div class="keyboard-map" data-keyboard-map aria-label="Keyboard controls">
          <span><kbd>W</kbd><kbd>Up</kbd>Forward</span>
          <span><kbd>S</kbd><kbd>Down</kbd>Back</span>
          <span><kbd>A</kbd><kbd>Left</kbd>Left</span>
          <span><kbd>D</kbd><kbd>Right</kbd>Right</span>
          <span><kbd>Q</kbd>Yaw L</span>
          <span><kbd>E</kbd>Yaw R</span>
          <span><kbd>Space</kbd><kbd>Esc</kbd>Hard stop</span>
        </div>
        <div class="robot-status" data-robot-status>Idle</div>
      </section>
      <section>
        <h2>Saved Routes</h2>
        <div class="map-route-table" data-route-table-panel>
          <table>
            <thead>
              <tr><th>Route</th><th>Selected</th><th>Waypoints</th><th>Actions</th><th>Last Run</th><th>Manage</th></tr>
            </thead>
            <tbody data-route-table>{route_table_rows}</tbody>
          </table>
        </div>
      </section>
      <section>
        <h2>Route Run History</h2>
        <div class="route-run-history">
          <table>
            <thead>
              <tr><th>Time</th><th>Run</th><th>Route</th><th>Mode</th><th>State</th><th>Progress</th><th>Map</th></tr>
            </thead>
            <tbody data-route-run-history>
              <tr><td colspan="7">No route runs recorded</td></tr>
            </tbody>
          </table>
        </div>
      </section>
      <section>
        <h2>Current Run Timeline</h2>
        <div class="route-run-timeline">
          <table>
            <thead>
              <tr><th>#</th><th>Kind</th><th>State</th><th>Target</th><th>Note</th></tr>
            </thead>
            <tbody data-route-run-timeline>
              <tr><td colspan="5">No active route timeline</td></tr>
            </tbody>
          </table>
        </div>
      </section>
      <section>
        <h2>Gemini Vision Evidence</h2>
        <div class="route-run-timeline">
          <table>
            <thead>
              <tr><th>Waypoint</th><th>Summary</th><th>Change</th><th>Severity</th><th>Confidence</th><th>Baseline</th></tr>
            </thead>
            <tbody data-gemini-evidence>
              <tr><td colspan="6">No Gemini analysis recorded</td></tr>
            </tbody>
          </table>
        </div>
      </section>
    </div>
    <section class="wide">
      <h2>Package Reconciliation</h2>
      {package_table(packages)}
    </section>
    <section>
      <h2>Incidents</h2>
      {incident_table(incidents)}
    </section>
    <section>
      <h2>Work Orders</h2>
      {work_order_table(work_orders)}
    </section>
    <section class="wide">
      <h2>Navigation Eval</h2>
      <div class="metric-row">
        {metric("Waypoints", nav_metric)}
        {metric("Retries", nav.get("retries_total", 0))}
        {metric("Guided", nav.get("guided_interventions", 0))}
        {metric("Tag Recovery", tag_recovery_metric)}
        {metric("Route Coverage", route_coverage_metric)}
        {metric("Mean Target Time", mean_target_time_metric)}
      </div>
    </section>
    <section class="wide">
      <h2>Route / POI Evidence</h2>
      <div class="evidence-grid">
        <div>
          <h2>Route Stops</h2>
          {route_table(route_data["stops"])}
        </div>
        <div>
          <h2>POI Evidence</h2>
          {poi_list(poi_data)}
        </div>
      </div>
    </section>
    <section class="wide">
      <h2>Saved Images</h2>
      <div class="saved-image-grid" data-saved-images>
        <p class="muted">No saved route images yet</p>
      </div>
    </section>
  </main>
  <script>
    (() => {{
      const controls = document.querySelector("[data-robot-controls]");
      const postureControls = document.querySelector("[data-posture-controls]");
      const motionControls = document.querySelector("[data-motion-controls]");
      const mapControls = document.querySelector("[data-map-controls]");
      const layerControls = document.querySelector("[data-map-layer-controls]");
      const mapEditControls = document.querySelector("[data-map-edit-controls]");
      const routeActionRow = document.querySelector("[data-route-action-row]");
      const routeActionSummary = document.querySelector("[data-route-action-summary]");
      const routeTable = document.querySelector("[data-route-table]");
      const rerunSurface = document.querySelector("[data-rerun-surface]");
      const rerunConnect = document.querySelector("[data-rerun-connect]");
      const rerunFrame = document.querySelector("[data-rerun-frame]");
      const rerunStandby = document.querySelector("[data-rerun-standby]");
      const rerunStatus = document.querySelector("[data-rerun-status]");
      const liveMapSvg = document.querySelector("[data-live-map-svg]");
      const liveMapStatus = document.querySelector("[data-live-map-status]");
      const mapCommandStatus = document.querySelector("[data-map-command-status]");
      const mapAuthoringStatus = document.querySelector("[data-map-authoring-status]");
      const routeExecutionStatus = document.querySelector("[data-route-execution-status]");
      const routeSummary = document.querySelector("[data-map-route-summary]");
      const routeUsePlanner = document.querySelector("[data-route-use-planner]");
      const routeRunHistory = document.querySelector("[data-route-run-history]");
      const routeRunTimeline = document.querySelector("[data-route-run-timeline]");
      const geminiEvidence = document.querySelector("[data-gemini-evidence]");
      const savedImages = document.querySelector("[data-saved-images]");
      const liveHeatmap = liveMapSvg ? liveMapSvg.querySelector("[data-live-heatmap]") : null;
      const livePath = liveMapSvg ? liveMapSvg.querySelector("[data-live-path]") : null;
      const liveTrace = liveMapSvg ? liveMapSvg.querySelector("[data-live-trace]") : null;
      const liveRobot = liveMapSvg ? liveMapSvg.querySelector("[data-live-robot]") : null;
      const liveTarget = liveMapSvg ? liveMapSvg.querySelector("[data-live-target]") : null;
      const goToMarker = liveMapSvg ? liveMapSvg.querySelector("[data-go-to-marker]") : null;
      const qrCargoLayer = liveMapSvg ? liveMapSvg.querySelector("[data-qr-cargo-layer]") : null;
      const qrCargoRows = document.querySelector("[data-qr-cargo-events]");
      const qrCargoStatus = document.querySelector("[data-qr-cargo-status]");
      const cameraFrame = document.querySelector("[data-camera-frame]");
      const cameraEmpty = document.querySelector("[data-camera-empty]");
      const cameraHealth = document.querySelector("[data-camera-health]");
      const cameraTopic = document.querySelector("[data-camera-topic]");
      const cameraShape = document.querySelector("[data-camera-shape]");
      const cameraAge = document.querySelector("[data-camera-age]");
      const status = document.querySelector("[data-robot-status]");
      if (!controls || !status) return;
      let motionProfile = "nudge";
      let robotBusy = false;
      let liveMapPolling = false;
      let dimosMapPolling = false;
      let routeExecutionPolling = false;
      let dimosRobotPoseActive = false;
      let goToArmed = false;
      let mapEditMode = "select";
      let mapAuthoring = null;
      let selectedMapObject = null;
      let dragMapObject = null;
      let liveMapBounds = null;
      let liveOverlayBounds = null;
      let latestRouteRuns = [];
      let savedImageUrls = [];
      let selectedRouteRunId = "";
      let latestQrEventsById = new Map();
      try {{
        liveMapBounds = liveMapSvg ? JSON.parse(liveMapSvg.dataset.mapBounds || "{{}}") : null;
      }} catch (_) {{
        liveMapBounds = null;
      }}
      try {{
        mapAuthoring = liveMapSvg ? JSON.parse(liveMapSvg.dataset.mapAuthoring || "{{}}") : null;
      }} catch (_) {{
        mapAuthoring = null;
      }}
      liveOverlayBounds = liveMapBounds;
      const liveMapSize = {{width: {MAP_WIDTH}, height: {MAP_HEIGHT}}};
      const motionLabels = {{
        nudge: "Nudge",
        step: "Step",
        walk: "Walk",
      }};
      const robotControlToken = {json.dumps(robot_control_token)};
      const keyboardCommands = new Map([
        ["KeyW", "forward"],
        ["ArrowUp", "forward"],
        ["KeyS", "backward"],
        ["ArrowDown", "backward"],
        ["KeyA", "left"],
        ["ArrowLeft", "left"],
        ["KeyD", "right"],
        ["ArrowRight", "right"],
        ["KeyQ", "yaw_left"],
        ["KeyE", "yaw_right"],
        ["Space", "hard_stop"],
        ["Escape", "hard_stop"],
      ]);
      const buttons = Array.from(document.querySelectorAll("[data-command], [data-posture], [data-motion], [data-map-action]"));
      const setBusy = (busy) => buttons.forEach((button) => {{
        button.disabled = busy && button.getAttribute("data-command") !== "hard_stop";
      }});
      const setStatus = (text, state) => {{
        status.textContent = text;
        status.className = `robot-status ${{state || ""}}`;
      }};
      const setMapCommandStatus = (text, state) => {{
        if (!mapCommandStatus) return;
        mapCommandStatus.textContent = text;
        mapCommandStatus.className = `map-command-status ${{state || ""}}`;
      }};
      const setMapAuthoringStatus = (text, state) => {{
        if (!mapAuthoringStatus) return;
        mapAuthoringStatus.textContent = text;
        mapAuthoringStatus.className = `map-authoring-status ${{state || ""}}`;
      }};
      const setRouteExecutionStatus = (text, state) => {{
        if (!routeExecutionStatus) return;
        routeExecutionStatus.textContent = text;
        routeExecutionStatus.className = `map-route-execution-status ${{state || ""}}`;
      }};
      const routeList = () => {{
        const current = mapAuthoring || {{}};
        return Array.isArray(current.routes) ? current.routes : [];
      }};
      const escapeCell = (value) => String(value ?? "").replace(/[&<>"']/g, (char) => ({{
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;",
        '"': "&quot;",
        "'": "&#39;",
      }}[char]));
      const routeActionCount = (route) => (route.waypoints || []).reduce(
        (total, waypoint) => total + ((waypoint.actions || []).length),
        0
      );
      const lastRunForRoute = (routeId) => latestRouteRuns.find((run) => run.route_id === routeId);
      const formatRouteTableTime = (seconds) => {{
        if (!seconds) return "-";
        try {{
          return new Date(Number(seconds) * 1000).toLocaleTimeString();
        }} catch (_) {{
          return "-";
        }}
      }};
      const routeActionRows = (route) => {{
        const rows = [];
        for (const waypoint of route.waypoints || []) {{
          for (const action of waypoint.actions || []) {{
            rows.push({{
              waypoint: waypoint.label || waypoint.id || "-",
              kind: action.kind || "-",
              label: action.label || action.kind || "-",
              args: action.args || {{}},
            }});
          }}
        }}
        return rows;
      }};
      const renderRouteTable = () => {{
        if (!routeTable) return;
        const current = mapAuthoring || {{}};
        const routes = routeList();
        if (!routes.length) {{
          routeTable.innerHTML = '<tr><td colspan="6">No saved routes</td></tr>';
          return;
        }}
        routeTable.innerHTML = routes.map((route) => {{
          const selected = route.id === current.selected_route_id;
          const waypoints = (route.waypoints || []).length;
          const actions = routeActionCount(route);
          const lastRun = lastRunForRoute(route.id);
          const lastRunText = lastRun
            ? `${{formatRouteTableTime(lastRun.started_at)}} / ${{lastRun.state || "-"}}`
            : "-";
          const mainRow = `<tr class="${{selected ? "is-selected-route" : ""}}"><td><strong>${{escapeCell(route.label || route.id)}}</strong><br><span>${{escapeCell(route.id || "-")}}</span></td><td>${{selected ? "Yes" : ""}}</td><td>${{waypoints}}</td><td>${{actions}}</td><td>${{escapeCell(lastRunText)}}</td><td><div class="route-table-actions"><button type="button" data-route-table-action="select" data-route-id="${{escapeCell(route.id)}}">Select</button><button type="button" data-route-table-action="rename" data-route-id="${{escapeCell(route.id)}}">Rename</button><button type="button" data-route-table-action="duplicate" data-route-id="${{escapeCell(route.id)}}">Duplicate</button><button type="button" data-route-table-action="delete" data-route-id="${{escapeCell(route.id)}}">Delete</button></div></td></tr>`;
          if (!selected) return mainRow;
          const actionRows = routeActionRows(route);
          const detail = actionRows.length
            ? `<ol class="route-actions-list">${{actionRows.map((action) => `<li><strong>${{escapeCell(action.waypoint)}}</strong>: ${{escapeCell(action.label)}} <span>(${{escapeCell(action.kind)}})</span> <code>${{escapeCell(JSON.stringify(action.args))}}</code></li>`).join("")}}</ol>`
            : "No actions saved for this route";
          return `${{mainRow}}<tr class="route-actions-subrow"><td colspan="6">${{detail}}</td></tr>`;
        }}).join("");
      }};
      const nextRouteId = (routes = routeList()) => {{
        const used = new Set(routes.map((route) => String(route.id || "")));
        let index = 1;
        while (used.has(`Route${{index}}`)) index += 1;
        return `Route${{index}}`;
      }};
      const updateRouteSummary = () => {{
        const current = mapAuthoring || {{}};
        const routes = routeList();
        if (!routes.length) {{
          if (routeSummary) routeSummary.textContent = `Selected route: none. Next: ${{nextRouteId(routes)}}`;
          renderRouteTable();
          return;
        }}
        const selected = routes.find((route) => route.id === current.selected_route_id) || null;
        const selectedText = selected
          ? `${{selected.id}} (${{(selected.waypoints || []).length}} waypoint${{(selected.waypoints || []).length === 1 ? "" : "s"}})`
          : "none";
        const routeIds = routes.map((route) => route.id).join(", ");
        if (routeSummary) routeSummary.textContent = `Selected route: ${{selectedText}}. Routes: ${{routeIds}}. Next: ${{nextRouteId(routes)}}`;
        renderRouteTable();
      }};
      updateRouteSummary();
      const setRerunStatus = (text) => {{
        if (rerunStatus) rerunStatus.textContent = text;
      }};
      const connectRerunSurface = () => {{
        if (!rerunSurface || !rerunFrame) return;
        const url = rerunSurface.getAttribute("data-rerun-url") || "";
        if (!url) return;
        if (rerunFrame.getAttribute("src") !== url) {{
          rerunFrame.setAttribute("src", url);
        }}
        if (rerunStandby) rerunStandby.hidden = true;
        setRerunStatus("Rerun visualization connected");
      }};
      const projectPoseWithBounds = (pose, bounds) => {{
        if (!bounds || !pose) return null;
        const spanX = Math.max(0.1, bounds.x_max - bounds.x_min);
        const spanY = Math.max(0.1, bounds.y_max - bounds.y_min);
        return {{
          x: ((pose.x - bounds.x_min) / spanX) * liveMapSize.width,
          y: liveMapSize.height - (((pose.y - bounds.y_min) / spanY) * liveMapSize.height),
        }};
      }};
      const projectLivePose = (pose) => projectPoseWithBounds(pose, liveMapBounds);
      const projectLiveOverlayPose = (pose) => projectPoseWithBounds(pose, liveOverlayBounds);
      const projectWorldPoint = (x, y) => projectLivePose({{x, y}});
      const projectLiveOverlayPoint = (x, y) => projectLiveOverlayPose({{x, y}});
      const worldFromSvgEvent = (event) => {{
        if (!liveMapSvg || !liveMapBounds) return null;
        const matrix = liveMapSvg.getScreenCTM();
        if (!matrix) return null;
        const point = liveMapSvg.createSVGPoint();
        point.x = event.clientX;
        point.y = event.clientY;
        const svgPoint = point.matrixTransform(matrix.inverse());
        const spanX = Math.max(0.1, liveMapBounds.x_max - liveMapBounds.x_min);
        const spanY = Math.max(0.1, liveMapBounds.y_max - liveMapBounds.y_min);
        return {{
          x: liveMapBounds.x_min + (svgPoint.x / liveMapSize.width) * spanX,
          y: liveMapBounds.y_min + ((liveMapSize.height - svgPoint.y) / liveMapSize.height) * spanY,
        }};
      }};
      const setGoToArmed = (armed) => {{
        goToArmed = armed;
        if (liveMapSvg) liveMapSvg.classList.toggle("go-to-armed", armed);
        if (mapControls) {{
          const button = mapControls.querySelector('[data-map-action="arm_go_to"]');
          if (button) button.setAttribute("aria-pressed", armed ? "true" : "false");
        }}
        setMapCommandStatus(armed ? "Map Go To armed" : "Map command idle", armed ? "ok" : "");
      }};
      const setGoToMarker = (target) => {{
        if (!goToMarker || !target) return;
        const projected = projectWorldPoint(target.x, target.y);
        if (!projected) return;
        goToMarker.style.display = "";
        goToMarker.setAttribute(
          "transform",
          `translate(${{projected.x.toFixed(1)}} ${{projected.y.toFixed(1)}})`
        );
      }};
      const authoringPoint = (target) => ({{
        x: Math.round(target.x * 1000) / 1000,
        y: Math.round(target.y * 1000) / 1000,
        theta_deg: null,
        source: "dashboard_edit",
      }});
      const postAuthoring = async (url, body, method = "POST") => {{
        setMapAuthoringStatus("Saving map edit...", "");
        try {{
          const response = await fetch(url, {{
            method,
            headers: {{
              "Content-Type": "application/json",
              "X-DogOps-Control-Token": robotControlToken,
            }},
            body: JSON.stringify(body),
          }});
          const result = await response.json();
          if (!response.ok || result.ok === false) {{
            throw new Error(result.message || result.error || "map_authoring_failed");
          }}
          mapAuthoring = result.authoring || mapAuthoring;
          updateRouteSummary();
          setMapAuthoringStatus("Map edit saved; refreshing", "ok");
          await refreshDimOSMap();
          window.setTimeout(() => window.location.reload(), 150);
          return result;
        }} catch (error) {{
          setMapAuthoringStatus(`Map edit failed: ${{error.message}}`, "error");
          return null;
        }}
      }};
      const setMapEditMode = (mode) => {{
        mapEditMode = mode || "select";
        if (mapEditControls) {{
          mapEditControls.querySelectorAll("[data-map-edit-mode]").forEach((button) => {{
            button.setAttribute("aria-pressed", button.getAttribute("data-map-edit-mode") === mapEditMode ? "true" : "false");
          }});
        }}
        setMapAuthoringStatus(mapEditMode === "select" ? "Map authoring idle" : `Map authoring: ${{mapEditMode}}`, mapEditMode === "select" ? "" : "ok");
      }};
      const clearRouteStopSelection = () => {{
        if (!liveMapSvg) return;
        liveMapSvg.querySelectorAll(".map-route-stop-marker.is-selected").forEach((item) => {{
          item.classList.remove("is-selected");
          const circle = item.querySelector(".map-route-stop");
          if (circle) circle.setAttribute("r", "9");
        }});
      }};
      const updateRouteActionControls = () => {{
        const hasRouteStop = selectedMapObject && selectedMapObject.kind === "route_stop";
        if (routeActionRow) routeActionRow.hidden = !hasRouteStop;
        if (routeActionSummary) {{
          routeActionSummary.textContent = hasRouteStop
            ? `Actions for ${{selectedMapObject.id}}`
            : "Select a waypoint to add actions";
        }}
      }};
      const selectMapObject = (target) => {{
        const item = target ? target.closest("[data-edit-kind][data-edit-id]") : null;
        clearRouteStopSelection();
        selectedMapObject = item ? {{
          kind: item.getAttribute("data-edit-kind"),
          id: item.getAttribute("data-edit-id"),
        }} : null;
        if (selectedMapObject && selectedMapObject.kind === "route_stop") {{
          item.classList.add("is-selected");
          const circle = item.querySelector(".map-route-stop");
          if (circle) circle.setAttribute("r", "18");
        }}
        updateRouteActionControls();
        setMapAuthoringStatus(
          selectedMapObject ? `Selected ${{selectedMapObject.kind}} ${{selectedMapObject.id}}` : "Map authoring idle",
          selectedMapObject ? "ok" : ""
        );
        return selectedMapObject;
      }};
      const entityKindForSelected = () => {{
        if (!selectedMapObject) return "checkpoint";
        if (["zone", "asset", "package"].includes(selectedMapObject.kind)) return selectedMapObject.kind;
        return "checkpoint";
      }};
      const deleteSelectedMapObject = async () => {{
        if (!selectedMapObject || !selectedMapObject.id) {{
          setMapAuthoringStatus("Select an authored object first", "error");
          return;
        }}
        const id = selectedMapObject.id;
        if (!window.confirm(`Delete authored map object ${{id}}?`)) return;
        const kind = selectedMapObject.kind;
        if (kind === "route_stop") {{
          await removeRouteWaypoint(id);
        }} else if (kind === "no_go_shape") {{
          await postAuthoring(`/api/map/no_go_shapes/${{encodeURIComponent(id)}}`, {{}}, "DELETE");
        }} else if (kind === "incident") {{
          const current = mapAuthoring || {{}};
          const incidentLocations = (current.incident_locations || []).filter((item) => item.incident_id !== id);
          await postAuthoring("/api/map/authoring", {{...current, incident_locations: incidentLocations}}, "PUT");
        }} else {{
          await postAuthoring(`/api/map/entities/${{encodeURIComponent(id)}}`, {{}}, "DELETE");
        }}
      }};
      const placeSelectedFromObservation = async () => {{
        if (!selectedMapObject || !selectedMapObject.id) {{
          setMapAuthoringStatus("Select an entity before using an observation", "error");
          return;
        }}
        const observationId = window.prompt("Observation id or blank to use tag id", "");
        const tagInput = observationId ? "" : window.prompt("Tag id", "");
        const payload = observationId
          ? {{observation_id: observationId, kind: entityKindForSelected()}}
          : {{tag_id: Number(tagInput), kind: entityKindForSelected()}};
        await postAuthoring(`/api/map/entities/${{encodeURIComponent(selectedMapObject.id)}}/from_observation`, payload);
      }};
      const selectedRoute = () => {{
        const current = mapAuthoring || {{}};
        const routes = routeList();
        if (!routes.length) return null;
        if (!current.selected_route_id) return null;
        return routes.find((route) => route.id === current.selected_route_id) || null;
      }};
      const routeExecutionText = (state) => {{
        if (!state || !state.state) return "Execution: idle";
        const total = Number(state.waypoints_total || 0);
        const reached = Number(state.waypoints_reached || 0);
        const active = state.active_waypoint_id ? ` active=${{state.active_waypoint_id}}` : "";
        const action = state.active_action_id ? ` action=${{state.active_action_id}}` : "";
        const run = state.route_run_id ? ` run=${{state.route_run_id}}` : "";
        const transport = state.transport ? ` transport=${{state.transport}}` : "";
        const error = state.last_error ? ` error=${{state.last_error}}` : "";
        return `Execution: ${{state.state}} ${{reached}}/${{total}}${{active}}${{action}}${{run}}${{transport}}${{error}}`;
      }};
      const formatRouteRunTime = (seconds) => {{
        if (!seconds) return "-";
        try {{
          return new Date(Number(seconds) * 1000).toLocaleTimeString();
        }} catch (_) {{
          return "-";
        }}
      }};
      const htmlEscape = (value) => String(value ?? "").replace(/[&<>"']/g, (char) => ({{
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;",
        '"': "&quot;",
        "'": "&#39;",
      }}[char]));
      const routeApiErrorText = (result, fallback) => {{
        const error = result && result.error ? String(result.error) : "";
        const message = result && result.message ? String(result.message) : "";
        if (error && message) return `${{error}}: ${{message}}`;
        return message || error || fallback;
      }};
      const renderRouteRunHistory = (runs) => {{
        if (!routeRunHistory) return;
        if (!Array.isArray(runs) || !runs.length) {{
          routeRunHistory.innerHTML = '<tr><td colspan="7">No route runs recorded</td></tr>';
          return;
        }}
        routeRunHistory.innerHTML = runs.slice(0, 8).map((run) => {{
          const progress = `${{Number(run.waypoints_reached || 0)}}/${{Number(run.waypoints_total || 0)}} wp, ${{Number(run.actions_completed || 0)}}/${{Number(run.actions_total || 0)}} act`;
          const mode = routeRunMode(run);
          const routeRunId = htmlEscape(run.route_run_id || "");
          const selectedClass = selectedRouteRunId && selectedRouteRunId === run.route_run_id ? ' class="is-selected-route"' : "";
          return `<tr${{selectedClass}}><td>${{htmlEscape(formatRouteRunTime(run.started_at))}}</td><td>${{htmlEscape(run.dogops_run_id || "-")}}</td><td>${{htmlEscape(run.route_id || "-")}}</td><td>${{htmlEscape(mode)}}</td><td>${{htmlEscape(run.state || "-")}}</td><td>${{htmlEscape(progress)}}</td><td><button type="button" data-route-run-select="${{routeRunId}}">Show</button></td></tr>`;
        }}).join("");
      }};
      const routeRunMode = (run) => {{
        if (run && (run.route_id === "GATHER_HEATMAP" || run.transport === "dimos_costmap_snapshot")) {{
          return "Costmap snapshot";
        }}
        return run && run.dry_run ? "Dry run" : "Live";
      }};
      const renderRouteRunTimeline = (events) => {{
        if (!routeRunTimeline) return;
        if (!Array.isArray(events) || !events.length) {{
          routeRunTimeline.innerHTML = '<tr><td colspan="5">No active route timeline</td></tr>';
          return;
        }}
        routeRunTimeline.innerHTML = events.slice(-10).map((event) => {{
          const target = event.target_id || event.waypoint_id || event.action_id || "-";
          return `<tr><td>${{htmlEscape(event.sequence || "")}}</td><td>${{htmlEscape(event.kind || "-")}}</td><td>${{htmlEscape(event.state || "-")}}</td><td>${{htmlEscape(target)}}</td><td>${{htmlEscape(event.note || "")}}</td></tr>`;
        }}).join("");
      }};
      const renderGeminiEvidence = (evidence) => {{
        if (!geminiEvidence) return;
        const rows = Array.isArray(evidence)
          ? evidence.filter((item) => item.kind === "gemini_vision_analysis")
          : [];
        if (!rows.length) {{
          geminiEvidence.innerHTML = '<tr><td colspan="6">No Gemini analysis recorded</td></tr>';
          return;
        }}
        geminiEvidence.innerHTML = rows.slice(-6).map((item) => {{
          const metadata = item.metadata || {{}};
          const changed = metadata.changed === true ? "Changed" : metadata.changed === false ? "No change" : "-";
          const confidence = Number.isFinite(Number(metadata.confidence)) ? Number(metadata.confidence).toFixed(2) : "-";
          const baseline = `${{metadata.baseline_match || "none"}}${{metadata.baseline_evidence_id ? ` / ${{metadata.baseline_evidence_id}}` : ""}}`;
          const summary = metadata.summary || metadata.change_summary || item.path || "-";
          return `<tr><td>${{htmlEscape(metadata.waypoint_id || "-")}}</td><td>${{htmlEscape(summary)}}</td><td>${{htmlEscape(changed)}}</td><td>${{htmlEscape(metadata.severity || "-")}}</td><td>${{htmlEscape(confidence)}}</td><td>${{htmlEscape(baseline)}}</td></tr>`;
        }}).join("");
      }};
      const renderSavedImages = async (images) => {{
        if (!savedImages) return;
        savedImageUrls.forEach((url) => URL.revokeObjectURL(url));
        savedImageUrls = [];
        const rows = Array.isArray(images) ? images.filter((item) => item.url) : [];
        if (!rows.length) {{
          savedImages.innerHTML = '<p class="muted">No saved route images yet</p>';
          return;
        }}
        const cards = await Promise.all(rows.slice(0, 12).map(async (item) => {{
          try {{
            const response = await fetch(item.url, {{
              cache: "no-store",
              headers: {{"X-DogOps-Control-Token": robotControlToken}},
            }});
            if (!response.ok) throw new Error("image_fetch_failed");
            const blob = await response.blob();
            const url = URL.createObjectURL(blob);
            savedImageUrls.push(url);
            const metadata = item.metadata || {{}};
            const runLabel = item.dogops_run_id || item.route_run_id || "-";
            const waypoint = metadata.waypoint_id || "-";
            const source = metadata.source || "image";
            return `<figure class="saved-image"><img src="${{url}}" alt="Saved route image"><figcaption>Run ${{htmlEscape(runLabel)}}<br>${{htmlEscape(waypoint)}} / ${{htmlEscape(source)}}</figcaption></figure>`;
          }} catch (_) {{
            return "";
          }}
        }}));
        savedImages.innerHTML = cards.filter(Boolean).join("") || '<p class="muted">No saved route images available</p>';
      }};
      const selectRouteRunOnMap = async (routeRunId) => {{
        if (!routeRunId) return;
        try {{
          const response = await fetch("/api/route-runs/" + encodeURIComponent(routeRunId), {{
            cache: "no-store",
            headers: {{"X-DogOps-Control-Token": robotControlToken}},
          }});
          const detail = await response.json();
          if (!response.ok || detail.ok === false) {{
            throw new Error(routeApiErrorText(detail, "route_run_detail_failed"));
          }}
          selectedRouteRunId = routeRunId;
          renderRouteRunTimeline(detail.timeline || detail.events || []);
          if (detail.map) {{
            updateDimOSMapLayers(detail.map);
            const heatmap = detail.map.gathered_heatmap;
            const cells = heatmap ? Number(heatmap.cells || 0) : 0;
            setMapCommandStatus(
              `Showing route run ${{routeRunId}}${{cells ? ` with saved heatmap (${{cells}} cells)` : ""}}`,
              "ok"
            );
          }} else {{
            setMapCommandStatus(`Route run ${{routeRunId}} has no saved map files`, "error");
          }}
          renderRouteRunHistory(latestRouteRuns);
        }} catch (error) {{
          setMapCommandStatus(`Route run load failed: ${{error.message}}`, "error");
        }}
      }};
      const refreshRouteRunHistory = async () => {{
        try {{
          const currentResponse = await fetch("/api/route-runs/current", {{
            cache: "no-store",
            headers: {{"X-DogOps-Control-Token": robotControlToken}},
          }});
          const current = await currentResponse.json();
          if (currentResponse.ok && current.ok !== false) {{
            renderRouteRunTimeline(current.timeline || current.events || []);
            renderGeminiEvidence(current.evidence || []);
          }}
          const listResponse = await fetch("/api/route-runs", {{
            cache: "no-store",
            headers: {{"X-DogOps-Control-Token": robotControlToken}},
          }});
          const list = await listResponse.json();
          if (listResponse.ok && list.ok !== false) {{
            latestRouteRuns = Array.isArray(list.route_runs) ? list.route_runs : [];
            renderRouteRunHistory(list.route_runs || []);
            renderRouteTable();
          }}
          const imagesResponse = await fetch("/api/route-runs/images", {{
            cache: "no-store",
            headers: {{"X-DogOps-Control-Token": robotControlToken}},
          }});
          const imageList = await imagesResponse.json();
          if (imagesResponse.ok && imageList.ok !== false) {{
            await renderSavedImages(imageList.images || []);
          }}
        }} catch (_) {{
          // Keep the latest rendered history visible if polling fails.
        }}
      }};
      const refreshRouteExecution = async () => {{
        if (routeExecutionPolling) return;
        routeExecutionPolling = true;
        try {{
          const response = await fetch("/api/map/routes/status", {{
            cache: "no-store",
            headers: {{"X-DogOps-Control-Token": robotControlToken}},
          }});
          const result = await response.json();
          if (!response.ok || result.ok === false) {{
            throw new Error(routeApiErrorText(result, "route_status_failed"));
          }}
          const state = result.route_execution || {{}};
          setRouteExecutionStatus(routeExecutionText(state), state.state === "failed" ? "error" : "ok");
          await refreshRouteRunHistory();
        }} catch (error) {{
          setRouteExecutionStatus(`Execution: status unavailable (${{error.message}})`, "error");
        }} finally {{
          routeExecutionPolling = false;
        }}
      }};
      const updateCamera = async () => {{
        try {{
          const response = await fetch("/api/camera/status");
          const camera = await response.json();
          if (cameraTopic) cameraTopic.textContent = `Topic: ${{camera.topic || "/color_image"}}`;
          if (cameraShape) {{
            cameraShape.textContent = camera.received
              ? `Frame: ${{camera.width}}x${{camera.height}} ${{camera.format || ""}}`
              : "Frame: pending";
          }}
          if (cameraAge) {{
            cameraAge.textContent = camera.age_s === null || camera.age_s === undefined
              ? "Age: pending"
              : `Age: ${{camera.age_s}}s`;
          }}
          if (cameraHealth) {{
            cameraHealth.textContent = camera.received
              ? "Receiving color_image"
              : `Waiting for ${{camera.topic || "/color_image"}}`;
          }}
          if (cameraFrame && cameraEmpty) {{
            if (camera.received) {{
              cameraFrame.src = `/api/camera/frame.jpg?ts=${{Date.now()}}`;
              cameraFrame.hidden = false;
              cameraEmpty.hidden = true;
            }} else {{
              cameraFrame.hidden = true;
              cameraFrame.src = "about:blank";
              cameraEmpty.hidden = false;
            }}
          }}
        }} catch (error) {{
          if (cameraHealth) cameraHealth.textContent = `Camera unavailable (${{error.message}})`;
          if (cameraFrame) cameraFrame.hidden = true;
          if (cameraEmpty) cameraEmpty.hidden = false;
        }}
      }};
      const runSelectedRoute = async (dryRun) => {{
        const route = selectedRoute();
        if (!route) {{
          setRouteExecutionStatus("Execution: select or author a route first", "error");
          return;
        }}
        const modeText = dryRun ? "dry run" : "live run";
        const usePlanner = !!(routeUsePlanner && routeUsePlanner.checked && !dryRun);
        const routeMode = dryRun ? "dry run" : (usePlanner ? "planner live run" : "direct live run");
        setRouteExecutionStatus(`Execution: starting ${{routeMode}} ${{route.id}}...`, "");
        try {{
          const response = await fetch("/api/map/routes/follow", {{
            method: "POST",
            headers: {{
              "Content-Type": "application/json",
              "X-DogOps-Control-Token": robotControlToken,
            }},
            body: JSON.stringify({{route_id: route.id, dry_run: !!dryRun, use_planner: usePlanner}}),
          }});
          const result = await response.json();
          const state = (result.mcp_result && result.mcp_result.route_execution) || result.route_execution || {{}};
          if (!response.ok || result.ok === false) {{
            throw new Error(routeApiErrorText(result, "follow_route_failed"));
          }}
          setRouteExecutionStatus(routeExecutionText(state), "ok");
        }} catch (error) {{
          setRouteExecutionStatus(`Execution failed (${{routeMode}}): ${{error.message}}`, "error");
        }} finally {{
          await refreshRouteExecution();
        }}
      }};
      const stopRouteExecution = async () => {{
        setRouteExecutionStatus("Execution: stopping route...", "");
        try {{
          const response = await fetch("/api/map/routes/stop", {{
            method: "POST",
            headers: {{
              "Content-Type": "application/json",
              "X-DogOps-Control-Token": robotControlToken,
            }},
            body: JSON.stringify({{}}),
          }});
          const result = await response.json();
          if (!response.ok || result.ok === false) {{
            throw new Error(routeApiErrorText(result, "stop_route_failed"));
          }}
          setRouteExecutionStatus(routeExecutionText(result.route_execution || {{}}), "ok");
        }} catch (error) {{
          setRouteExecutionStatus(`Stop route failed: ${{error.message}}`, "error");
        }} finally {{
          await refreshRouteExecution();
        }}
      }};
      const routeIndexForTarget = (route, targetId) => {{
        if (!route || !Array.isArray(route.waypoints)) return -1;
        return route.waypoints.findIndex((waypoint) => waypoint.id === targetId || waypoint.target_id === targetId);
      }};
      const saveRoutes = async (routes, selectedRouteId = undefined) => {{
        const current = mapAuthoring || {{}};
        await postAuthoring("/api/map/authoring", {{
          ...current,
          routes,
          selected_route_id: selectedRouteId !== undefined ? selectedRouteId : current.selected_route_id || (routes[0] && routes[0].id) || null,
        }}, "PUT");
      }};
      const routeById = (routeId) => routeList().find((route) => route.id === routeId);
      const selectRouteFromTable = async (routeId) => {{
        if (!routeById(routeId)) return;
        await postAuthoring(`/api/map/routes/${{encodeURIComponent(routeId)}}/select`, {{}});
      }};
      const renameRouteFromTable = async (routeId) => {{
        const routes = routeList();
        const route = routes.find((item) => item.id === routeId);
        if (!route) return;
        const newId = (window.prompt("Route id", route.id) || "").trim();
        if (!newId) return;
        if (newId !== route.id && routes.some((item) => item.id === newId)) {{
          setMapAuthoringStatus("Route id already exists", "error");
          return;
        }}
        const newLabel = (window.prompt("Route label", route.label || newId) || newId).trim() || newId;
        const renamed = routes.map((item) => item.id === routeId ? {{...item, id: newId, label: newLabel}} : item);
        const selectedRouteId = (mapAuthoring || {{}}).selected_route_id === routeId ? newId : (mapAuthoring || {{}}).selected_route_id;
        await saveRoutes(renamed, selectedRouteId);
      }};
      const duplicateRouteFromTable = async (routeId) => {{
        const routes = routeList();
        const route = routes.find((item) => item.id === routeId);
        if (!route) return;
        const defaultId = `${{route.id || "Route"}}_COPY`;
        const newId = (window.prompt("New route id", defaultId) || "").trim();
        if (!newId) return;
        if (routes.some((item) => item.id === newId)) {{
          setMapAuthoringStatus("Route id already exists", "error");
          return;
        }}
        const copy = JSON.parse(JSON.stringify(route));
        copy.id = newId;
        copy.label = `${{route.label || route.id || "Route"}} Copy`;
        await saveRoutes([...routes, copy], newId);
      }};
      const deleteRouteFromTable = async (routeId) => {{
        const routes = routeList();
        const route = routes.find((item) => item.id === routeId);
        if (!route) return;
        if (!window.confirm(`Delete route ${{routeId}}?`)) return;
        const remaining = routes.filter((item) => item.id !== routeId);
        const currentSelected = (mapAuthoring || {{}}).selected_route_id;
        const selectedRouteId = currentSelected === routeId ? ((remaining[0] && remaining[0].id) || null) : currentSelected;
        await saveRoutes(remaining, selectedRouteId);
      }};
      const handleRouteTableAction = async (button) => {{
        const routeId = button.getAttribute("data-route-id") || "";
        const action = button.getAttribute("data-route-table-action") || "";
        if (action === "select") {{
          await selectRouteFromTable(routeId);
        }} else if (action === "rename") {{
          await renameRouteFromTable(routeId);
        }} else if (action === "duplicate") {{
          await duplicateRouteFromTable(routeId);
        }} else if (action === "delete") {{
          await deleteRouteFromTable(routeId);
        }}
      }};
      const removeRouteWaypoint = async (targetId) => {{
        const current = mapAuthoring || {{}};
        const routes = Array.isArray(current.routes) ? [...current.routes] : [];
        const route = routes.find((item) => item.id === current.selected_route_id) || routes[0];
        const routeIndex = routes.indexOf(route);
        if (!route || routeIndex < 0) return;
        route.waypoints = (route.waypoints || []).filter((waypoint) => waypoint.id !== targetId && waypoint.target_id !== targetId);
        routes[routeIndex] = route;
        await saveRoutes(routes, route.id);
      }};
      const moveSelectedRouteWaypoint = async (direction) => {{
        if (!selectedMapObject || selectedMapObject.kind !== "route_stop") {{
          setMapAuthoringStatus("Select a route waypoint first", "error");
          return;
        }}
        const current = mapAuthoring || {{}};
        const routes = Array.isArray(current.routes) ? [...current.routes] : [];
        const route = routes.find((item) => item.id === current.selected_route_id) || routes[0];
        const routeIndex = routes.indexOf(route);
        const waypointIndex = routeIndex >= 0 ? routeIndexForTarget(route, selectedMapObject.id) : -1;
        const nextIndex = waypointIndex + direction;
        if (!route || waypointIndex < 0 || nextIndex < 0 || nextIndex >= route.waypoints.length) return;
        const waypoints = [...route.waypoints];
        [waypoints[waypointIndex], waypoints[nextIndex]] = [waypoints[nextIndex], waypoints[waypointIndex]];
        route.waypoints = waypoints;
        routes[routeIndex] = route;
        await saveRoutes(routes, route.id);
      }};
      const routeActionArgs = (kind, waypoint) => {{
        if (kind === "scan_tags") {{
          const raw = window.prompt("Expected AprilTag IDs, comma separated", "");
          const expected = (raw || "").split(",").map((item) => Number(item.trim())).filter((item) => Number.isFinite(item));
          return {{expected}};
        }}
        if (kind === "scan_qr") {{
          const raw = window.prompt("Expected QR payloads, comma separated", "");
          const expected = (raw || "").split(",").map((item) => item.trim()).filter(Boolean);
          return {{expected}};
        }}
        if (kind === "capture_image") {{
          return {{target: waypoint.target_id || waypoint.id}};
        }}
        if (kind === "gemini_inspect_image") {{
          return {{
            target: waypoint.target_id || waypoint.id,
            prompt: "Inspect this waypoint for physical changes, safety issues, package changes, and work-order evidence.",
            baseline_policy: "same_waypoint_latest_previous",
            require_baseline: false,
            model: "gemini-2.5-flash",
            max_image_bytes_inline: 20000000,
          }};
        }}
        if (kind === "wait") {{
          const seconds = Number(window.prompt("Wait seconds", "2") || 2);
          return {{seconds: Number.isFinite(seconds) ? seconds : 2}};
        }}
        return {{target: waypoint.target_id || waypoint.id}};
      }};
      const routeActionLabels = {{
        capture_image: "Capture Image",
        gemini_inspect_image: "Gemini Inspect",
        scan_qr: "Scan QR",
        scan_tags: "Scan Tags",
        wait: "Wait",
        inspect_asset: "Inspect Asset",
        verify_work_order: "Verify Work Order",
        operator_prompt: "Operator Prompt",
      }};
      const addActionToSelectedRouteWaypoint = async (kind) => {{
        if (!selectedMapObject || selectedMapObject.kind !== "route_stop") {{
          setMapAuthoringStatus("Select a route waypoint before adding an action", "error");
          return;
        }}
        const allowed = new Set(Object.keys(routeActionLabels));
        if (!allowed.has(kind)) {{
          setMapAuthoringStatus("Unsupported route action kind", "error");
          return;
        }}
        const current = mapAuthoring || {{}};
        const routes = Array.isArray(current.routes) ? [...current.routes] : [];
        const route = routes.find((item) => item.id === current.selected_route_id) || routes[0];
        const routeIndex = routes.indexOf(route);
        const waypointIndex = routeIndex >= 0 ? routeIndexForTarget(route, selectedMapObject.id) : -1;
        if (!route || waypointIndex < 0) {{
          setMapAuthoringStatus("Selected route waypoint was not found", "error");
          return;
        }}
        const waypoints = [...(route.waypoints || [])];
        const waypoint = {{...waypoints[waypointIndex]}};
        const actionId = mapEditId(kind.toUpperCase());
        const label = routeActionLabels[kind] || kind;
        const required = !new Set(["capture_image", "gemini_inspect_image"]).has(kind);
        waypoint.actions = [...(waypoint.actions || []), {{
          id: actionId,
          kind,
          label,
          required,
          timeout_s: 5.0,
          args: routeActionArgs(kind, waypoint),
        }}];
        waypoints[waypointIndex] = waypoint;
        route.waypoints = waypoints;
        routes[routeIndex] = route;
        await saveRoutes(routes, route.id);
        setMapAuthoringStatus(`Added ${{kind}} to ${{waypoint.id}}`, "ok");
      }};
      const selectRouteByPrompt = async () => {{
        const current = mapAuthoring || {{}};
        const routes = routeList();
        const existingRouteIds = routes.map((route) => route.id).join(", ") || "none";
        const routeIdInput = window.prompt(
          `Route id (existing: ${{existingRouteIds}}; new id creates a route)`,
          current.selected_route_id || (routes[0] && routes[0].id) || nextRouteId(routes)
        );
        const routeId = routeIdInput ? routeIdInput.trim() : "";
        if (!routeId) return;
        const existing = routes.find((route) => route.id === routeId);
        if (existing) {{
          await postAuthoring(`/api/map/routes/${{encodeURIComponent(routeId)}}/select`, {{}});
        }} else {{
          await saveRoutes([...routes, {{id: routeId, label: routeId, waypoints: [], mission_id: null}}], routeId);
        }}
      }};
      const mapEditId = (prefix) => `${{prefix}}-${{Date.now().toString(36)}}`;
      const applyMapEditAt = async (target) => {{
        const pose = authoringPoint(target);
        if (mapEditMode === "home") {{
          const current = mapAuthoring || {{}};
          await postAuthoring("/api/map/authoring", {{...current, home: pose}}, "PUT");
        }} else if (mapEditMode === "zone") {{
          const label = window.prompt("Label name", "CHECKPOINT");
          if (!label) return;
          await postAuthoring("/api/map/entities", {{
            id: label.trim().replace(/\\s+/g, "_").toUpperCase(),
            kind: "checkpoint",
            label,
            pose,
          }});
        }} else if (mapEditMode === "asset") {{
          const label = window.prompt("Asset id", "ASSET_1");
          if (!label) return;
          await postAuthoring("/api/map/entities", {{
            id: label.trim().replace(/\\s+/g, "_").toUpperCase(),
            kind: "asset",
            label,
            pose,
          }});
        }} else if (mapEditMode === "package") {{
          const label = window.prompt("Package id", "PKG-NEW");
          if (!label) return;
          await postAuthoring("/api/map/entities", {{
            id: label.trim().replace(/\\s+/g, "_").toUpperCase(),
            kind: "package",
            label,
            pose,
          }});
        }} else if (mapEditMode === "no_go") {{
          const size = 0.5;
          await postAuthoring("/api/map/no_go_shapes", {{
            id: mapEditId("NO_GO"),
            label: "Authored No-Go",
            shape: "rectangle",
            points: [
              {{...pose, x: pose.x - size, y: pose.y - size}},
              {{...pose, x: pose.x + size, y: pose.y + size}},
            ],
            enabled: true,
            dimos_constraint_status: "not_supported",
          }});
        }} else if (mapEditMode === "route") {{
          const current = mapAuthoring || {{}};
          const routes = [...routeList()];
          const newRouteId = nextRouteId(routes);
          const route = routes.find((item) => item.id === current.selected_route_id) || routes[0] || {{id: newRouteId, label: newRouteId, waypoints: [], mission_id: null}};
          const routeIndex = routes.indexOf(route);
          route.waypoints = [...(route.waypoints || []), {{
            id: mapEditId("WP"),
            label: `Waypoint ${{(route.waypoints || []).length + 1}}`,
            pose,
            target_id: null,
            required: true,
          }}];
          routes[routeIndex >= 0 ? routeIndex : 0] = route;
          await postAuthoring("/api/map/authoring", {{...current, routes, selected_route_id: route.id}}, "PUT");
        }} else if (mapEditMode === "incident") {{
          const incidentId = window.prompt("Incident id", "INC-001");
          if (!incidentId) return;
          await postAuthoring(`/api/map/incidents/${{encodeURIComponent(incidentId)}}/location`, {{
            pose,
            evidence_observation_ids: [],
          }});
        }} else if (mapEditMode === "tag") {{
          const tagId = Number(window.prompt("Tag id", "999"));
          const entityId = window.prompt("Entity id", "CHECKPOINT");
          if (!Number.isFinite(tagId) || !entityId) return;
          await postAuthoring("/api/map/tag_bindings", {{
            tag_id: tagId,
            entity_id: entityId.trim(),
            label: entityId.trim(),
            binding_kind: "checkpoint",
          }});
        }}
      }};
      const setLiveMapUnavailable = (text, keepRobot = false) => {{
        if (liveMapStatus) liveMapStatus.textContent = text;
        if (liveRobot && !keepRobot) liveRobot.style.display = "none";
      }};
      const heatColor = (cost) => {{
        if (cost >= 0.75) return "#dc2626";
        if (cost >= 0.5) return "#f97316";
        if (cost >= 0.28) return "#eab308";
        return "#22c55e";
      }};
      const renderLiveHeatmap = (costmap) => {{
        if (!liveHeatmap) return 0;
        liveHeatmap.textContent = "";
        const cells = costmap && Array.isArray(costmap.cells) ? costmap.cells : [];
        let rendered = 0;
        for (const cell of cells) {{
          const cost = Math.max(0, Math.min(1, Number(cell.cost || 0)));
          if (cost < 0.12) continue;
          const p1 = projectLiveOverlayPoint(Number(cell.x), Number(cell.y));
          const p2 = projectLiveOverlayPoint(Number(cell.x) + Number(cell.width || 0), Number(cell.y) + Number(cell.height || 0));
          if (!p1 || !p2) continue;
          const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
          rect.setAttribute("class", "map-live-cost-cell");
          rect.setAttribute("x", Math.min(p1.x, p2.x).toFixed(1));
          rect.setAttribute("y", Math.min(p1.y, p2.y).toFixed(1));
          rect.setAttribute("width", Math.abs(p2.x - p1.x).toFixed(1));
          rect.setAttribute("height", Math.abs(p2.y - p1.y).toFixed(1));
          rect.setAttribute("fill", heatColor(cost));
          rect.setAttribute("opacity", (0.18 + cost * 0.55).toFixed(2));
          liveHeatmap.appendChild(rect);
          rendered += 1;
        }}
        return rendered;
      }};
      const renderDimOSPath = (path) => {{
        if (!livePath) return 0;
        const points = Array.isArray(path) ? path : [];
        const projected = points
          .map((point) => projectLiveOverlayPoint(point.x, point.y))
          .filter(Boolean)
          .map((point) => `${{point.x.toFixed(1)}},${{point.y.toFixed(1)}}`);
        livePath.setAttribute("points", projected.join(" "));
        return projected.length;
      }};
      const renderDimOSTarget = (target) => {{
        if (!liveTarget) return;
        const projected = target ? projectLiveOverlayPoint(target.x, target.y) : null;
        if (!projected) {{
          liveTarget.style.display = "none";
          return;
        }}
        liveTarget.style.display = "";
        liveTarget.setAttribute(
          "transform",
          `translate(${{projected.x.toFixed(1)}} ${{projected.y.toFixed(1)}})`
        );
      }};
      const clearChildren = (node) => {{
        if (!node) return;
        while (node.firstChild) node.removeChild(node.firstChild);
      }};
      const qrText = (value, fallback = "unknown") => {{
        if (value === null || value === undefined || value === "") return fallback;
        return String(value);
      }};
      const formatQrTimestamp = (value) => {{
        const numeric = Number(value);
        if (!Number.isFinite(numeric)) return "unknown";
        return numeric.toFixed(3);
      }};
      const formatQrRobotPose = (pose) => {{
        if (!pose) return "unknown";
        const x = Number(pose.x);
        const y = Number(pose.y);
        const yaw = Number(pose.yaw);
        if (!Number.isFinite(x) || !Number.isFinite(y)) return "unknown";
        if (!Number.isFinite(yaw)) return x.toFixed(2) + ", " + y.toFixed(2);
        return x.toFixed(2) + ", " + y.toFixed(2) + ", yaw " + yaw.toFixed(2);
      }};
      const setQrCargoStatus = (text, state = "") => {{
        if (!qrCargoStatus) return;
        qrCargoStatus.textContent = text;
        qrCargoStatus.className = "qr-cargo-status " + state;
      }};
      const renderQrCargoRows = (events) => {{
        if (!qrCargoRows) return;
        const items = Array.isArray(events) ? events : [];
        latestQrEventsById = new Map(items.map((event) => [String(event.event_id || ""), event]));
        clearChildren(qrCargoRows);
        if (!items.length) {{
          const row = document.createElement("tr");
          row.setAttribute("data-qr-empty", "");
          const cell = document.createElement("td");
          cell.colSpan = 10;
          cell.className = "muted";
          cell.textContent = "No QR cargo events yet";
          row.appendChild(cell);
          qrCargoRows.appendChild(row);
          setQrCargoStatus("QR cargo idle", "");
          return;
        }}
        for (const event of items.slice(0, 50)) {{
          const eventId = String(event.event_id || "");
          const row = document.createElement("tr");
          row.setAttribute("data-qr-event-row", "");
          row.setAttribute("data-qr-event-id", eventId);
          const addCell = (text) => {{
            const cell = document.createElement("td");
            cell.textContent = text;
            row.appendChild(cell);
          }};
          addCell(formatQrTimestamp(event.timestamp));
          addCell(qrText(event.warehouse_id));
          addCell(qrText(event.location_node_id));
          addCell(qrText(event.cargo_id));
          addCell([event.zone, event.shelf_id].filter(Boolean).join(" / ") || "unknown");
          addCell(qrText(event.task));
          addCell(qrText(event.source));
          addCell(qrText(event.status) + " / " + qrText(event.action_policy, "report_only"));
          addCell(formatQrRobotPose(event.robot_pose_at_detection));
          const actions = document.createElement("td");
          actions.className = "qr-actions";
          const addButton = (label, attribute, disabled = false) => {{
            const button = document.createElement("button");
            button.type = "button";
            button.textContent = label;
            button.setAttribute(attribute, eventId);
            button.disabled = disabled;
            actions.appendChild(button);
          }};
          addButton("Select on map", "data-qr-select", !event.map_position);
          addButton("Promote to Package", "data-qr-promote-package");
          addButton("Promote to Label", "data-qr-promote-label");
          addButton("Bind Location Node", "data-qr-bind-location");
          addButton("Copy JSON", "data-qr-copy");
          row.appendChild(actions);
          qrCargoRows.appendChild(row);
        }}
        setQrCargoStatus("Latest QR cargo events: " + items.length, "ok");
      }};
      const renderQrCargoMarkers = (events) => {{
        if (!qrCargoLayer) return;
        clearChildren(qrCargoLayer);
        const items = Array.isArray(events) ? events : [];
        const svgNS = "http://www.w3.org/2000/svg";
        for (const event of items) {{
          if (!event.map_position) continue;
          const projected = projectLiveOverlayPoint(Number(event.map_position.x), Number(event.map_position.y));
          if (!projected) continue;
          const eventId = String(event.event_id || "");
          const group = document.createElementNS(svgNS, "g");
          group.setAttribute("class", "map-qr-cargo-marker");
          group.setAttribute("data-edit-kind", "qr_cargo");
          group.setAttribute("data-edit-id", eventId);
          group.setAttribute("data-qr-event-id", eventId);
          group.setAttribute("data-linked-entity-id", String(event.linked_entity_id || event.location_node_id || ""));
          const title = document.createElementNS(svgNS, "title");
          title.textContent = "QR cargo " + qrText(event.cargo_id, "cargo") + " / " + qrText(event.location_node_id);
          group.appendChild(title);
          const ring = document.createElementNS(svgNS, "circle");
          ring.setAttribute("class", "map-qr-cargo-ring");
          ring.setAttribute("cx", projected.x.toFixed(1));
          ring.setAttribute("cy", projected.y.toFixed(1));
          ring.setAttribute("r", "14");
          group.appendChild(ring);
          const core = document.createElementNS(svgNS, "rect");
          core.setAttribute("class", "map-qr-cargo-core");
          core.setAttribute("x", (projected.x - 5).toFixed(1));
          core.setAttribute("y", (projected.y - 5).toFixed(1));
          core.setAttribute("width", "10");
          core.setAttribute("height", "10");
          core.setAttribute("rx", "1.5");
          group.appendChild(core);
          const label = document.createElementNS(svgNS, "text");
          label.setAttribute("class", "map-qr-cargo-label");
          label.setAttribute("x", (projected.x + 14).toFixed(1));
          label.setAttribute("y", (projected.y + 5).toFixed(1));
          label.textContent = qrText(event.cargo_id, "cargo");
          group.appendChild(label);
          qrCargoLayer.appendChild(group);
        }}
      }};
      const clearQrCargoHighlight = () => {{
        if (!liveMapSvg) return;
        liveMapSvg.querySelectorAll(".map-qr-cargo-marker.is-selected, .qr-linked").forEach((item) => {{
          item.classList.remove("is-selected");
          item.classList.remove("qr-linked");
        }});
      }};
      const highlightLinkedEntity = (linkedId) => {{
        if (!liveMapSvg || !linkedId) return;
        liveMapSvg.querySelectorAll("[data-edit-id]").forEach((item) => {{
          if (item.getAttribute("data-edit-id") === linkedId) item.classList.add("qr-linked");
        }});
      }};
      const highlightQrCargoEvent = (eventId) => {{
        clearQrCargoHighlight();
        if (!qrCargoLayer) return;
        const marker = Array.from(qrCargoLayer.querySelectorAll("[data-qr-event-id]")).find(
          (item) => item.getAttribute("data-qr-event-id") === eventId
        );
        const event = latestQrEventsById.get(eventId);
        if (marker) {{
          marker.classList.add("is-selected");
          marker.scrollIntoView({{block: "nearest", inline: "nearest"}});
          setQrCargoStatus("Selected QR event " + eventId, "ok");
        }} else {{
          setQrCargoStatus("QR event has no map position", "error");
        }}
        if (event) highlightLinkedEntity(event.linked_entity_id || event.location_node_id);
      }};
      const postQrAction = async (eventId, action) => {{
        setQrCargoStatus("Saving QR cargo action...", "");
        try {{
          const response = await fetch("/api/qr/events/" + encodeURIComponent(eventId) + "/" + action, {{
            method: "POST",
            headers: {{
              "Content-Type": "application/json",
              "X-DogOps-Control-Token": robotControlToken,
            }},
            body: "{{}}",
          }});
          const result = await response.json();
          if (!response.ok || result.ok === false) throw new Error(result.message || result.error || "qr_action_failed");
          setQrCargoStatus("QR cargo action saved", "ok");
          await refreshDimOSMap();
          window.setTimeout(() => window.location.reload(), 150);
        }} catch (error) {{
          setQrCargoStatus("QR cargo action failed: " + error.message, "error");
        }}
      }};
      const copyQrEventJson = async (eventId) => {{
        const event = latestQrEventsById.get(eventId);
        if (!event) return;
        const raw = JSON.stringify(event, null, 2);
        try {{
          if (!navigator.clipboard) throw new Error("clipboard unavailable");
          await navigator.clipboard.writeText(raw);
          setQrCargoStatus("QR event JSON copied", "ok");
        }} catch (_) {{
          window.prompt("QR event JSON", raw);
        }}
      }};

      const updateDimOSMapLayers = (data) => {{
        const live = data && data.live ? data.live : null;
        if (!live) return;
        if (data.bounds) liveOverlayBounds = data.bounds;
        const heatmapCells = renderLiveHeatmap(live.costmap);
        const overlayPath = Array.isArray(live.path) && live.path.length
          ? live.path
          : Array.isArray(live.route) && live.route.length
            ? live.route
            : data.route || [];
        const pathPoints = renderDimOSPath(overlayPath);
        renderDimOSTarget(live.target);
        const qrEvents = Array.isArray(data.qr_cargo_events) ? data.qr_cargo_events : [];
        renderQrCargoMarkers(qrEvents);
        renderQrCargoRows(qrEvents);
        dimosRobotPoseActive = Boolean(live.robot_pose);
        if (live.robot_pose) {{
          const yawRad = Number.isFinite(live.robot_pose.theta_deg)
            ? live.robot_pose.theta_deg * Math.PI / 180
            : 0;
          updateLiveMap(
            {{ok: true, pose: {{x: live.robot_pose.x, y: live.robot_pose.y, yaw_rad: yawRad}}, trajectory: [], ts: Date.now() / 1000}},
            liveOverlayBounds
          );
        }}
        if (liveMapStatus) {{
          if (live.ok) {{
            liveMapStatus.textContent = `Live DimOS: heatmap=${{heatmapCells}} path=${{pathPoints}} source=${{live.source || "LCM topics"}}`;
          }} else if (!liveMapStatus.textContent.startsWith("Live odom: x=")) {{
            liveMapStatus.textContent = `Live DimOS: waiting for topics${{live.error ? ` (${{live.error}})` : ""}}`;
          }}
        }}
      }};
      const updateLiveMap = (data, bounds = liveMapBounds) => {{
        if (!liveMapSvg || !liveTrace || !liveRobot || !liveMapStatus) return;
        if (!data || !data.ok || !data.pose) {{
          if (dimosRobotPoseActive) return;
          const error = data && data.error ? data.error : "offline";
          setLiveMapUnavailable(`Live odom: ${{error}}`);
          return;
        }}
        const trajectory = Array.isArray(data.trajectory) ? data.trajectory : [];
        const points = trajectory
          .map((pose) => projectPoseWithBounds(pose, bounds))
          .filter(Boolean)
          .map((point) => `${{point.x.toFixed(1)}},${{point.y.toFixed(1)}}`);
        liveTrace.setAttribute("points", points.join(" "));
        const projected = projectPoseWithBounds(data.pose, bounds);
        if (!projected) {{
          if (!dimosRobotPoseActive) setLiveMapUnavailable("Live odom: map projection unavailable");
          return;
        }}
        const yawDeg = (data.pose.yaw_rad || 0) * 180 / Math.PI;
        liveRobot.style.display = "";
        liveRobot.setAttribute(
          "transform",
          `translate(${{projected.x.toFixed(1)}} ${{projected.y.toFixed(1)}}) rotate(${{(-yawDeg).toFixed(1)}})`
        );
        const ageS = Math.max(0, Date.now() / 1000 - (data.ts || 0));
        liveMapStatus.textContent = `Live odom: x=${{data.pose.x.toFixed(2)}}m y=${{data.pose.y.toFixed(2)}}m yaw=${{yawDeg.toFixed(0)}}deg age=${{ageS.toFixed(1)}}s`;
      }};
      const refreshLiveMap = async () => {{
        if (liveMapPolling || !liveMapSvg) return;
        liveMapPolling = true;
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), 1500);
        try {{
          const response = await fetch("/api/robot/pose", {{
            cache: "no-store",
            signal: controller.signal,
          }});
          const result = await response.json();
          updateLiveMap(result);
        }} catch (error) {{
          if (!dimosRobotPoseActive) setLiveMapUnavailable(`Live odom: ${{error.message}}`);
        }} finally {{
          window.clearTimeout(timeout);
          liveMapPolling = false;
        }}
      }};
      const refreshDimOSMap = async () => {{
        if (dimosMapPolling || !liveMapSvg) return;
        dimosMapPolling = true;
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), 1800);
        try {{
          const response = await fetch("/api/map", {{
            cache: "no-store",
            signal: controller.signal,
          }});
          const result = await response.json();
          updateDimOSMapLayers(result);
        }} catch (error) {{
          if (liveMapStatus && !liveMapStatus.textContent.startsWith("Live odom: x=")) {{
            liveMapStatus.textContent = `Live DimOS: ${{error.message}}`;
          }}
        }} finally {{
          window.clearTimeout(timeout);
          dimosMapPolling = false;
        }}
      }};
      const shouldIgnoreKeyboardEvent = (event) => {{
        if (event.defaultPrevented || event.repeat) return true;
        if (event.metaKey || event.ctrlKey || event.altKey) return true;
        const target = event.target;
        if (!target) return false;
        const tagName = target.tagName ? target.tagName.toLowerCase() : "";
        return (
          target.isContentEditable ||
          tagName === "input" ||
          tagName === "textarea" ||
          tagName === "select" ||
          tagName === "button"
        );
      }};
      const motionTextForCommand = (command) => (result) => {{
        if (command === "hard_stop") return "Hard stop sent";
        const profile = motionLabels[result.profile] || motionLabels[motionProfile] || motionProfile;
        if (!result.observed) return `Sent ${{profile}} ${{command}}`;
        const distanceCm = Math.round((result.observed_distance_m || 0) * 1000) / 10;
        const yawDeg = Math.round(Math.abs(result.observed_dyaw_rad || 0) * 1800 / Math.PI) / 10;
        if (distanceCm >= 0.5) return `Sent ${{profile}} ${{command}} / observed ${{distanceCm}} cm`;
        if (yawDeg >= 0.5) return `Sent ${{profile}} ${{command}} / observed ${{yawDeg}} deg`;
        return `Sent ${{profile}} ${{command}} / no clear odom movement`;
      }};
      const sendJogCommand = async (command, source = "button") => {{
        if (robotBusy && command !== "hard_stop") return;
        await sendRobotAction(
          "/api/robot/jog",
          {{command, profile: motionProfile, source}},
          motionTextForCommand(command)
        );
        await refreshLiveMap();
      }};
      const sendGoToTarget = async (target) => {{
        if (!target || robotBusy) return;
        setGoToMarker(target);
        setMapCommandStatus(`Sending Go To x=${{target.x.toFixed(2)}} y=${{target.y.toFixed(2)}}...`, "");
        const result = await sendRobotAction(
          "/api/robot/go_to",
          {{command: "go_to", x: target.x, y: target.y, source: "map_click"}},
          () => `Go To sent x=${{target.x.toFixed(2)}} y=${{target.y.toFixed(2)}}`
        );
        setMapCommandStatus(
          result ? `Go To sent x=${{target.x.toFixed(2)}} y=${{target.y.toFixed(2)}}` : "Go To failed",
          result ? "ok" : "error"
        );
        await refreshLiveMap();
      }};
      const gatherHeatmap = async () => {{
        if (robotBusy) return;
        const areaId = (window.prompt("Heatmap area label", "CURRENT_AREA") || "").trim();
        const durationRaw = window.prompt("Gather duration in seconds", "10");
        const durationS = Number(durationRaw || 10);
        robotBusy = true;
        setBusy(true);
        setStatus("Gathering heatmap...", "");
        setMapCommandStatus("Gathering heatmap from DimOS costmap...", "");
        try {{
          const headers = {{"Content-Type": "application/json"}};
          if (robotControlToken) headers["X-DogOps-Control-Token"] = robotControlToken;
          const response = await fetch("/api/map/heatmap/gather", {{
            method: "POST",
            headers,
            body: JSON.stringify({{
              command: "gather_heatmap",
              area_id: areaId,
              duration_s: Number.isFinite(durationS) ? durationS : 10,
            }}),
          }});
          const result = await response.json();
          if (!response.ok || result.ok === false) {{
            throw new Error(result.message || result.error || "gather_heatmap_failed");
          }}
          const cells = result.heatmap && result.heatmap.costmap && Array.isArray(result.heatmap.costmap.cells)
            ? result.heatmap.costmap.cells.length
            : 0;
          setStatus(`Gathered heatmap (${{cells}} cells)`, "ok");
          setMapCommandStatus(`Gathered heatmap run ${{result.route_run_id || ""}}`, "ok");
          await refreshDimOSMap();
          await refreshRouteRunHistory();
        }} catch (error) {{
          setStatus(`Gather heatmap failed: ${{error.message}}`, "error");
          setMapCommandStatus(`Gather heatmap failed: ${{error.message}}`, "error");
        }} finally {{
          robotBusy = false;
          setBusy(false);
        }}
      }};
      const scanSelectedZone = async () => {{
        if (robotBusy) return;
        const defaultZone = selectedMapObject && selectedMapObject.kind === "zone"
          ? selectedMapObject.id
          : "INBOUND_DOCK";
        const zoneId = (window.prompt("Zone id to scan", defaultZone) || "").trim();
        if (!zoneId) return;
        setMapCommandStatus(`Scanning ${{zoneId}}...`, "");
        const result = await sendRobotAction(
          "/api/robot/scan_zone",
          {{command: "scan_zone", zone_id: zoneId}},
          (response) => {{
            const mcp = response.mcp_result || {{}};
            const tags = Array.isArray(mcp.visible_tag_ids) ? mcp.visible_tag_ids.join(",") : "";
            const packages = Array.isArray(mcp.package_ids) ? mcp.package_ids.join(",") : "";
            const source = mcp.source || response.transport || "mcp";
            return `Scan ${{zoneId}} via ${{source}} / tags=${{tags || "none"}} packages=${{packages || "none"}}`;
          }}
        );
        if (result) {{
          const mcp = result.mcp_result || {{}};
          setMapCommandStatus(
            `Scan ${{zoneId}}: ${{mcp.source || result.transport || "mcp"}} tags=${{(mcp.visible_tag_ids || []).join(",") || "none"}}`,
            "ok"
          );
          await refreshDimOSMap();
          await refreshRouteRunHistory();
        }} else {{
          setMapCommandStatus(`Scan ${{zoneId}} failed`, "error");
        }}
      }};
      const sendRobotAction = async (url, body, successText) => {{
        robotBusy = true;
        setBusy(true);
        setStatus(`Sending ${{body.command}}...`, "");
        try {{
          const headers = {{"Content-Type": "application/json"}};
          if (robotControlToken) headers["X-DogOps-Control-Token"] = robotControlToken;
          const response = await fetch(url, {{
            method: "POST",
            headers,
            body: JSON.stringify(body),
          }});
          const result = await response.json();
          if (!response.ok || !result.ok) {{
            throw new Error(result.error || "command_failed");
          }}
          setStatus(successText(result), "ok");
          return result;
        }} catch (error) {{
          setStatus(`Robot command failed: ${{error.message}}`, "error");
          return null;
        }} finally {{
          robotBusy = false;
          setBusy(false);
        }}
      }};
      controls.addEventListener("click", async (event) => {{
        const button = event.target.closest("button[data-command]");
        if (!button) return;
        const command = button.getAttribute("data-command");
        button.blur();
        await sendJogCommand(command);
      }});
      if (rerunConnect) {{
        rerunConnect.addEventListener("click", () => {{
          connectRerunSurface();
        }});
      }}
      if (liveMapSvg) {{
        liveMapSvg.addEventListener("mousedown", (event) => {{
          if (mapEditMode !== "select") return;
          const item = selectMapObject(event.target);
          if (!item || !["zone", "asset", "package"].includes(item.kind)) return;
          dragMapObject = item;
        }});
        liveMapSvg.addEventListener("mouseup", async (event) => {{
          if (!dragMapObject) return;
          const item = dragMapObject;
          dragMapObject = null;
          const target = worldFromSvgEvent(event);
          if (!target) return;
          await postAuthoring(`/api/map/entities/${{encodeURIComponent(item.id)}}`, {{
            id: item.id,
            kind: item.kind,
            label: item.id,
            pose: authoringPoint(target),
          }}, "PUT");
        }});
        liveMapSvg.addEventListener("click", async (event) => {{
          const selected = selectMapObject(event.target);
          if (selected) {{
            event.preventDefault();
            return;
          }}
          if (mapEditMode !== "select") {{
            event.preventDefault();
            const target = worldFromSvgEvent(event);
            if (!target) {{
              setMapAuthoringStatus("Map edit target unavailable", "error");
              return;
            }}
            await applyMapEditAt(target);
            return;
          }}
          if (!goToArmed) return;
          event.preventDefault();
          const target = worldFromSvgEvent(event);
          setGoToArmed(false);
          if (!target) {{
            setMapCommandStatus("Go To target unavailable", "error");
            return;
          }}
          await sendGoToTarget(target);
        }});
      }}
      window.addEventListener("keydown", async (event) => {{
        if (shouldIgnoreKeyboardEvent(event)) return;
        const command = keyboardCommands.get(event.code);
        if (!command) return;
        event.preventDefault();
        await sendJogCommand(command, "keyboard");
      }});
      if (motionControls) {{
        motionControls.addEventListener("click", (event) => {{
          const button = event.target.closest("button[data-motion]");
          if (!button) return;
          motionProfile = button.getAttribute("data-motion") || "nudge";
          motionControls.querySelectorAll("[data-motion]").forEach((item) => {{
            item.setAttribute("aria-pressed", item === button ? "true" : "false");
          }});
          setStatus(`Motion: ${{motionLabels[motionProfile] || motionProfile}}`, "ok");
          button.blur();
        }});
      }}
      if (postureControls) {{
        postureControls.addEventListener("click", async (event) => {{
          const button = event.target.closest("button[data-posture]");
          if (!button) return;
          const command = button.getAttribute("data-posture");
          button.blur();
          await sendRobotAction(
            "/api/robot/posture",
            {{command}},
            () => command === "wake" ? "Wake / stand complete" : `Sent ${{command}}`
          );
          await refreshLiveMap();
        }});
      }}
      if (mapControls) {{
        mapControls.addEventListener("click", async (event) => {{
          const button = event.target.closest("button[data-map-action]");
          if (!button) return;
          const action = button.getAttribute("data-map-action");
          button.blur();
          if (action === "start") {{
            await sendRobotAction(
              "/api/robot/map_start",
              {{command: "map_start"}},
              () => "Live map connected"
            );
          }} else if (action === "origin") {{
            await sendRobotAction(
              "/api/robot/map_origin",
              {{command: "map_origin"}},
              () => "Map origin set"
            );
          }} else if (action === "gather_heatmap") {{
            await gatherHeatmap();
          }} else if (action === "scan_zone") {{
            await scanSelectedZone();
          }} else if (action === "arm_go_to") {{
            setGoToArmed(!goToArmed);
          }}
          await refreshLiveMap();
        }});
      }}
      if (routeTable) {{
        routeTable.addEventListener("click", async (event) => {{
          const routeTableButton = event.target.closest("button[data-route-table-action]");
          if (!routeTableButton) return;
          routeTableButton.blur();
          await handleRouteTableAction(routeTableButton);
        }});
      }}
      if (routeRunHistory) {{
        routeRunHistory.addEventListener("click", async (event) => {{
          const routeRunButton = event.target.closest("button[data-route-run-select]");
          if (!routeRunButton) return;
          routeRunButton.blur();
          await selectRouteRunOnMap(routeRunButton.getAttribute("data-route-run-select") || "");
        }});
      }}
      if (mapEditControls) {{
        mapEditControls.addEventListener("click", async (event) => {{
          const routeActionButton = event.target.closest("button[data-route-action-kind]");
          if (routeActionButton) {{
            routeActionButton.blur();
            await addActionToSelectedRouteWaypoint(routeActionButton.getAttribute("data-route-action-kind") || "");
            return;
          }}
          const modeButton = event.target.closest("button[data-map-edit-mode]");
          if (modeButton) {{
            setGoToArmed(false);
            setMapEditMode(modeButton.getAttribute("data-map-edit-mode") || "select");
            modeButton.blur();
            return;
          }}
          const actionButton = event.target.closest("button[data-map-edit-action]");
          if (!actionButton) return;
          const action = actionButton.getAttribute("data-map-edit-action");
          actionButton.blur();
          if (action === "save") {{
            const current = mapAuthoring || {{}};
            await postAuthoring("/api/map/authoring", current, "PUT");
          }} else if (action === "reset") {{
            if (!window.confirm("Remove all authored map edits for this run?")) return;
            await postAuthoring("/api/map/authoring", {{
              schema_version: 1,
              site_id: "",
              frame: "world",
              entities: [],
              no_go_shapes: [],
              routes: [],
              incident_locations: [],
              tag_bindings: [],
            }}, "PUT");
          }} else if (action === "export") {{
            const result = await postAuthoring("/api/map/export", {{}});
            if (result && result.exports) {{
              setMapAuthoringStatus("Map authoring exported to run exports directory", "ok");
            }}
          }} else if (action === "delete_selected") {{
            await deleteSelectedMapObject();
          }} else if (action === "use_observation") {{
            await placeSelectedFromObservation();
          }} else if (action === "route_select") {{
            await selectRouteByPrompt();
          }} else if (action === "dry_run_route") {{
            await runSelectedRoute(true);
          }} else if (action === "run_route") {{
            await runSelectedRoute(false);
          }} else if (action === "stop_route") {{
            await stopRouteExecution();
          }} else if (action === "heatmap_run") {{
            await gatherHeatmap();
            await refreshLiveMap();
          }} else if (action === "route_up") {{
            await moveSelectedRouteWaypoint(-1);
          }} else if (action === "route_down") {{
            await moveSelectedRouteWaypoint(1);
          }} else if (action === "publish_no_go") {{
            await postAuthoring("/api/map/no_go_shapes/publish", {{}});
          }}
        }});
      }}
      if (qrCargoRows) {{
        qrCargoRows.addEventListener("click", async (event) => {{
          const button = event.target.closest("button[data-qr-select], button[data-qr-promote-package], button[data-qr-promote-label], button[data-qr-bind-location], button[data-qr-copy]");
          if (!button) return;
          button.blur();
          if (button.hasAttribute("data-qr-select")) {{
            highlightQrCargoEvent(button.getAttribute("data-qr-select") || "");
          }} else if (button.hasAttribute("data-qr-promote-package")) {{
            await postQrAction(button.getAttribute("data-qr-promote-package") || "", "promote_to_package");
          }} else if (button.hasAttribute("data-qr-promote-label")) {{
            await postQrAction(button.getAttribute("data-qr-promote-label") || "", "promote_to_label");
          }} else if (button.hasAttribute("data-qr-bind-location")) {{
            await postQrAction(button.getAttribute("data-qr-bind-location") || "", "bind_location_node");
          }} else if (button.hasAttribute("data-qr-copy")) {{
            await copyQrEventJson(button.getAttribute("data-qr-copy") || "");
          }}
        }});
      }}
      if (layerControls) {{
        layerControls.addEventListener("click", (event) => {{
          const button = event.target.closest("button[data-map-layer]");
          if (!button || !liveMapSvg) return;
          const layer = button.getAttribute("data-map-layer");
          const pressed = button.getAttribute("aria-pressed") !== "true";
          button.setAttribute("aria-pressed", pressed ? "true" : "false");
          liveMapSvg.querySelectorAll(`[data-layer="${{layer}}"]`).forEach((item) => {{
            item.toggleAttribute("hidden", !pressed);
          }});
        }});
      }}
      refreshLiveMap();
      refreshDimOSMap();
      updateCamera();
      refreshRouteExecution();
      refreshRouteRunHistory();
      window.setInterval(refreshLiveMap, 1000);
      window.setInterval(refreshDimOSMap, 1500);
      window.setInterval(updateCamera, 750);
      window.setInterval(refreshRouteExecution, 1500);
      window.setInterval(refreshRouteRunHistory, 5000);
    }})();
  </script>
</body>
</html>
"""


class _MapProjector:
    def __init__(self, bounds: dict[str, float]) -> None:
        self.x_min = bounds["x_min"]
        self.x_max = bounds["x_max"]
        self.y_min = bounds["y_min"]
        self.y_max = bounds["y_max"]

    def x(self, value: float) -> float:
        span = max(0.1, self.x_max - self.x_min)
        return ((value - self.x_min) / span) * MAP_WIDTH

    def y(self, value: float) -> float:
        span = max(0.1, self.y_max - self.y_min)
        return MAP_HEIGHT - (((value - self.y_min) / span) * MAP_HEIGHT)

    def radius(self, value_m: float) -> float:
        span_x = max(0.1, self.x_max - self.x_min)
        span_y = max(0.1, self.y_max - self.y_min)
        px_per_m = min(MAP_WIDTH / span_x, MAP_HEIGHT / span_y)
        return max(22.0, value_m * px_per_m)

    def size(self, value_m: float) -> float:
        span_x = max(0.1, self.x_max - self.x_min)
        span_y = max(0.1, self.y_max - self.y_min)
        px_per_m = min(MAP_WIDTH / span_x, MAP_HEIGHT / span_y)
        return max(2.0, value_m * px_per_m)


def _zone_pose(zone: dict[str, Any]) -> tuple[float, float] | None:
    pose = zone.get("pose_hint") or {}
    try:
        return float(pose["x"]), float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return None


def _offset_pose(base: tuple[float, float], index: int) -> tuple[float, float]:
    dx, dy = ENTITY_OFFSETS_M[index % len(ENTITY_OFFSETS_M)]
    return base[0] + dx, base[1] + dy


def _package_pose(base: tuple[float, float], index: int) -> tuple[float, float]:
    dx, dy = PACKAGE_OFFSETS_M[index % len(PACKAGE_OFFSETS_M)]
    return base[0] + dx, base[1] + dy


def _visible_tag_ids(observation: dict[str, Any]) -> list[int]:
    facts = observation.get("facts") or {}
    raw = facts.get("visible_tag_ids")
    if isinstance(raw, str):
        tag_ids = []
        for item in raw.split(","):
            item = item.strip()
            if item:
                try:
                    tag_ids.append(int(item))
                except ValueError:
                    continue
        return tag_ids
    if isinstance(raw, list):
        return [int(item) for item in raw if isinstance(item, int | str)]
    tag_id = observation.get("tag_id")
    return [int(tag_id)] if isinstance(tag_id, int) else []


def _trusted_rerun_web_url(raw_url: str | None) -> str:
    fallback = "http://127.0.0.1:9877"
    if not raw_url:
        return fallback
    try:
        parsed = urlparse(raw_url)
    except ValueError:
        return fallback
    if parsed.scheme not in {"http", "https"}:
        return fallback
    host = (parsed.hostname or "").lower()
    if host not in {"127.0.0.1", "localhost", "::1"}:
        return fallback
    return raw_url


def _render_rerun_surface(rerun_web_url: str) -> str:
    rerun_web_url_attr = escape(rerun_web_url, quote=True)
    return (
        f'<div class="rerun-surface" data-rerun-surface data-rerun-url="{rerun_web_url_attr}">'
        '<iframe data-rerun-frame title="Rerun Web visualization" loading="lazy"></iframe>'
        '<div class="rerun-standby" data-rerun-standby>'
        '<span class="rerun-chip">Rerun Web Visualization</span>'
        '<div class="rerun-status" data-rerun-status>Rerun visualization standby</div>'
        '<div class="rerun-controls">'
        '<button type="button" data-rerun-connect>Connect Rerun</button>'
        f'<a href="{rerun_web_url_attr}" target="_blank" rel="noreferrer">Open</a>'
        "</div>"
        "</div>"
        "</div>"
    )


def _map_bounds(points: list[tuple[float, float]]) -> dict[str, float]:
    if not points:
        return {"x_min": -1.0, "x_max": 1.0, "y_min": -1.0, "y_max": 1.0}
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x_min = min(xs) - MAP_PADDING_M
    x_max = max(xs) + MAP_PADDING_M
    y_min = min(ys) - MAP_PADDING_M
    y_max = max(ys) + MAP_PADDING_M
    if math.isclose(x_min, x_max):
        x_min -= 1.0
        x_max += 1.0
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    return {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max}


def _render_floor_cells(projector: _MapProjector, map_data: dict[str, Any]) -> str:
    free_cells: set[tuple[int, int]] = set()
    cost_cells: set[tuple[int, int]] = set()
    route_positions = [(float(point["x"]), float(point["y"])) for point in map_data["route"]]

    for x, y in _route_samples(route_positions):
        _add_cells(free_cells, x, y, radius_m=0.38)

    for zone in map_data["zones"]:
        x = float(zone["x"])
        y = float(zone["y"])
        if zone.get("no_go"):
            _add_cells(cost_cells, x, y, radius_m=float(zone.get("radius_m") or 0.8))
        else:
            _add_cells(free_cells, x, y, radius_m=0.42)

    for item in [*map_data["assets"], *map_data["packages"], *map_data["observations"]]:
        _add_cells(free_cells, float(item["x"]), float(item["y"]), radius_m=0.24)

    free_cells -= cost_cells
    free_markup = "".join(_render_cell(projector, cell, "map-free-cell") for cell in sorted(free_cells))
    cost_markup = "".join(_render_cell(projector, cell, "map-cost-cell") for cell in sorted(cost_cells))
    return free_markup + cost_markup


def _render_point_cloud(projector: _MapProjector, map_data: dict[str, Any]) -> str:
    points: list[tuple[float, float, bool]] = []
    for index, observation in enumerate(map_data["observations"]):
        x = float(observation["x"])
        y = float(observation["y"])
        visible_tags = observation.get("visible_tag_ids") or []
        count = max(4, len(visible_tags) * 3)
        for point_index in range(count):
            angle = (index * 0.91) + (point_index * 2.399)
            radius = 0.05 + ((point_index % 5) * 0.035)
            points.append(
                (
                    x + math.cos(angle) * radius,
                    y + math.sin(angle) * radius,
                    bool(visible_tags),
                )
            )

    for package in map_data["packages"]:
        x = float(package["x"])
        y = float(package["y"])
        points.extend(
            [
                (x - 0.035, y - 0.025, True),
                (x + 0.038, y - 0.015, True),
                (x + 0.004, y + 0.041, True),
            ]
        )

    markup = []
    for x, y, hot in points:
        css_class = "map-point hot" if hot else "map-point"
        markup.append(
            f'<circle class="{css_class}" cx="{projector.x(x):.1f}" '
            f'cy="{projector.y(y):.1f}" r="2.1" />'
        )
    return "".join(markup)


def _route_samples(route_positions: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not route_positions:
        return []
    samples = [route_positions[0]]
    for start, end in pairwise(route_positions):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        steps = max(1, int(distance / MAP_CELL_M))
        for step in range(1, steps + 1):
            ratio = step / steps
            samples.append((start[0] + dx * ratio, start[1] + dy * ratio))
    return samples


def _add_cells(cells: set[tuple[int, int]], x: float, y: float, *, radius_m: float) -> None:
    radius_cells = max(1, math.ceil(radius_m / MAP_CELL_M))
    center_x = round(x / MAP_CELL_M)
    center_y = round(y / MAP_CELL_M)
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            cell_x = center_x + dx
            cell_y = center_y + dy
            world_x = cell_x * MAP_CELL_M
            world_y = cell_y * MAP_CELL_M
            if math.hypot(world_x - x, world_y - y) <= radius_m:
                cells.add((cell_x, cell_y))


def _render_cell(projector: _MapProjector, cell: tuple[int, int], css_class: str) -> str:
    world_x = cell[0] * MAP_CELL_M
    world_y = cell[1] * MAP_CELL_M
    size = projector.size(MAP_CELL_M) * 0.9
    x = projector.x(world_x) - size / 2
    y = projector.y(world_y) - size / 2
    return f'<rect class="{css_class}" x="{x:.1f}" y="{y:.1f}" width="{size:.1f}" height="{size:.1f}" />'


def _render_grid(projector: _MapProjector) -> str:
    x_start = math.floor(projector.x_min * 2) / 2
    x_stop = math.ceil(projector.x_max * 2) / 2
    y_start = math.floor(projector.y_min * 2) / 2
    y_stop = math.ceil(projector.y_max * 2) / 2
    lines = []
    x_value = x_start
    while x_value <= x_stop + 0.001:
        x = projector.x(float(x_value))
        is_major = math.isclose(x_value % 1.0, 0.0, abs_tol=0.001)
        css_class = "map-grid-major" if is_major else "map-grid"
        lines.append(
            f'<line class="{css_class}" x1="{x:.1f}" y1="0" x2="{x:.1f}" y2="{MAP_HEIGHT}" />'
        )
        if is_major:
            label = round(x_value)
            lines.append(
                f'<text class="map-axis-label" x="{x + 4:.1f}" y="{MAP_HEIGHT - 8}">{label}m</text>'
            )
        x_value += 0.5

    y_value = y_start
    while y_value <= y_stop + 0.001:
        y = projector.y(float(y_value))
        is_major = math.isclose(y_value % 1.0, 0.0, abs_tol=0.001)
        css_class = "map-grid-major" if is_major else "map-grid"
        lines.append(
            f'<line class="{css_class}" x1="0" y1="{y:.1f}" x2="{MAP_WIDTH}" y2="{y:.1f}" />'
        )
        if is_major:
            label = round(y_value)
            lines.append(f'<text class="map-axis-label" x="8" y="{y - 4:.1f}">{label}m</text>')
        y_value += 0.5
    return "".join(lines)


def _render_no_go_zone(projector: _MapProjector, zone: dict[str, Any]) -> str:
    if not zone.get("no_go"):
        return ""
    x = projector.x(float(zone["x"]))
    y = projector.y(float(zone["y"]))
    radius = projector.size(float(zone.get("radius_m") or 0.8))
    width = radius * 1.55
    height = radius * 1.25
    title = escape(str(zone.get("display_name") or zone["id"]))
    return (
        f'<g data-edit-kind="zone" data-edit-id="{escape(str(zone["id"]))}"><title>{title}</title>'
        f'<rect class="map-no-go" x="{x - width / 2:.1f}" y="{y - height / 2:.1f}" '
        f'width="{width:.1f}" height="{height:.1f}" rx="4" />'
        f'<rect class="map-no-go-hatch" x="{x - width / 2:.1f}" y="{y - height / 2:.1f}" '
        f'width="{width:.1f}" height="{height:.1f}" rx="4" />'
        "</g>"
    )


def _render_no_go_shape(projector: _MapProjector, shape: dict[str, Any]) -> str:
    if not isinstance(shape, dict) or not shape.get("enabled", True):
        return ""
    points = [_authoring_pose(point) for point in shape.get("points") or []]
    points = [point for point in points if point is not None]
    if len(points) < 2:
        return ""
    title = escape(
        f"{shape.get('label') or shape.get('id') or 'No-go shape'} / "
        f"{shape.get('dimos_constraint_status') or 'not_supported'}"
    )
    if shape.get("shape") == "rectangle":
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        x1 = projector.x(min(xs))
        x2 = projector.x(max(xs))
        y1 = projector.y(max(ys))
        y2 = projector.y(min(ys))
        x = min(x1, x2)
        y = min(y1, y2)
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        return (
            f'<g data-edit-kind="no_go_shape" data-edit-id="{escape(str(shape.get("id") or ""))}" '
            f'data-authored-no-go="{escape(str(shape.get("id") or ""))}"><title>{title}</title>'
            f'<rect class="map-no-go" x="{x:.1f}" y="{y:.1f}" '
            f'width="{width:.1f}" height="{height:.1f}" rx="4" />'
            f'<rect class="map-no-go-hatch" x="{x:.1f}" y="{y:.1f}" '
            f'width="{width:.1f}" height="{height:.1f}" rx="4" />'
            "</g>"
        )
    svg_points = " ".join(
        f"{projector.x(point[0]):.1f},{projector.y(point[1]):.1f}" for point in points
    )
    return (
        f'<g data-edit-kind="no_go_shape" data-edit-id="{escape(str(shape.get("id") or ""))}" '
        f'data-authored-no-go="{escape(str(shape.get("id") or ""))}"><title>{title}</title>'
        f'<polygon class="map-no-go" points="{svg_points}" />'
        f'<polygon class="map-no-go-hatch" points="{svg_points}" />'
        "</g>"
    )


def _render_zone(projector: _MapProjector, zone: dict[str, Any]) -> str:
    x = projector.x(float(zone["x"]))
    y = projector.y(float(zone["y"]))
    label = escape(str(zone["id"]))
    title = escape(str(zone.get("display_name") or zone["id"]))
    return (
        f'<g data-edit-kind="zone" data-edit-id="{escape(str(zone["id"]))}"><title>{title}</title>'
        f'<line class="map-zone-anchor" x1="{x - 8:.1f}" y1="{y:.1f}" x2="{x + 8:.1f}" y2="{y:.1f}" />'
        f'<line class="map-zone-anchor" x1="{x:.1f}" y1="{y - 8:.1f}" x2="{x:.1f}" y2="{y + 8:.1f}" />'
        f'<circle class="map-zone-anchor" cx="{x:.1f}" cy="{y:.1f}" r="4.5" />'
        f'<text class="map-zone-label" x="{x:.1f}" y="{y + 19:.1f}">{label}</text>'
        "</g>"
    )


def _render_asset(projector: _MapProjector, asset: dict[str, Any]) -> str:
    x = projector.x(float(asset["x"]))
    y = projector.y(float(asset["y"]))
    label = escape(str(asset["id"]))
    title = escape(str(asset.get("display_name") or asset["id"]))
    return (
        f'<g data-edit-kind="asset" data-edit-id="{escape(str(asset["id"]))}"><title>{title}</title>'
        f'<rect class="map-asset" x="{x - 7:.1f}" y="{y - 7:.1f}" width="14" height="14" rx="2" />'
        f'<rect class="map-tag-face" x="{x - 3:.1f}" y="{y - 3:.1f}" width="6" height="6" />'
        f'<text class="map-asset-label" x="{x + 10:.1f}" y="{y + 4:.1f}">{label}</text>'
        "</g>"
    )


def _render_package(projector: _MapProjector, package: dict[str, Any]) -> str:
    x = projector.x(float(package["x"]))
    y = projector.y(float(package["y"]))
    state = str(package.get("state") or "unknown")
    label = escape(str(package["id"]))
    title = escape(f"{package['id']} / {state}")
    return (
        f'<g data-edit-kind="package" data-edit-id="{escape(str(package["id"]))}"><title>{title}</title>'
        f'<rect class="map-package state-{escape(state)}" x="{x - 7:.1f}" y="{y - 7:.1f}" '
        f'width="14" height="14" rx="2" transform="rotate(45 {x:.1f} {y:.1f})" />'
        f'<text class="map-package-label" x="{x + 11:.1f}" y="{y + 4:.1f}">{label}</text>'
        "</g>"
    )


def _render_observation(projector: _MapProjector, observation: dict[str, Any]) -> str:
    zone_x = projector.x(float(observation["x"]))
    zone_y = projector.y(float(observation["y"]))
    label = escape(str(observation["id"]))
    title = escape(
        f"{observation.get('id')} tags {','.join(str(tag) for tag in observation['visible_tag_ids'])}"
    )
    return (
        f'<g data-edit-kind="observation" data-edit-id="{escape(str(observation["id"]))}"><title>{title}</title>'
        f'<circle class="map-observation" cx="{zone_x:.1f}" cy="{zone_y:.1f}" r="5.5" />'
        f'<text class="map-asset-label" x="{zone_x + 8:.1f}" y="{zone_y - 7:.1f}">{label}</text>'
        "</g>"
    )


def _render_incident(projector: _MapProjector, incident: dict[str, Any]) -> str:
    x = projector.x(float(incident["x"]))
    y = projector.y(float(incident["y"]))
    label = escape(str(incident["id"]))
    title = escape(
        f"{incident.get('id')} {incident.get('severity')} {incident.get('state')}"
    )
    return (
        f'<g data-edit-kind="incident" data-edit-id="{escape(str(incident["id"]))}"><title>{title}</title>'
        f'<circle class="map-incident" cx="{x:.1f}" cy="{y:.1f}" r="15" />'
        f'<text class="map-incident-label" x="{x + 14:.1f}" y="{y - 13:.1f}">{label}</text>'
        "</g>"
    )


def _render_qr_cargo_event(projector: _MapProjector, event: dict[str, Any]) -> str:
    position = event.get("map_position")
    if not isinstance(position, dict):
        return ""
    x = projector.x(float(position["x"]))
    y = projector.y(float(position["y"]))
    event_id = escape(str(event.get("event_id") or ""))
    cargo_id = escape(str(event.get("cargo_id") or "cargo"))
    location_node_id = escape(str(event.get("location_node_id") or "unknown"))
    title = escape(
        f"QR cargo {event.get('cargo_id') or 'cargo'} / "
        f"{event.get('location_node_id') or 'unknown'} / "
        f"{event.get('action_policy') or 'report_only'}"
    )
    return (
        f'<g class="map-qr-cargo-marker" data-edit-kind="qr_cargo" '
        f'data-edit-id="{event_id}" data-qr-event-id="{event_id}" '
        f'data-linked-entity-id="{location_node_id}"><title>{title}</title>'
        f'<circle class="map-qr-cargo-ring" cx="{x:.1f}" cy="{y:.1f}" r="14" />'
        f'<rect class="map-qr-cargo-core" x="{x - 5:.1f}" y="{y - 5:.1f}" '
        f'width="10" height="10" rx="1.5" />'
        f'<text class="map-qr-cargo-label" x="{x + 14:.1f}" y="{y + 5:.1f}">{cargo_id}</text>'
        "</g>"
    )


def _render_route_stop(projector: _MapProjector, stop: dict[str, Any], index: int) -> str:
    x = projector.x(float(stop["x"]))
    y = projector.y(float(stop["y"]))
    title = escape(str(stop.get("target_id") or "route stop"))
    return (
        f'<g class="map-route-stop-marker" data-edit-kind="route_stop" '
        f'data-edit-id="{escape(str(stop.get("target_id") or ""))}"><title>{title}</title>'
        f'<circle class="map-route-stop" cx="{x:.1f}" cy="{y:.1f}" r="9" />'
        f'<text class="map-route-index" x="{x:.1f}" y="{y + 1:.1f}">{index}</text>'
        "</g>"
    )


def _render_live_robot_pose() -> str:
    return (
        '<polyline class="map-live-trace" data-live-trace points="" />'
        '<g data-live-robot style="display:none" transform="translate(0 0)">'
        "<title>Live Go2 odometry pose</title>"
        '<circle class="map-live-robot-halo" cx="0" cy="0" r="18" />'
        '<path class="map-live-robot-core" d="M 14 0 L -9 -8 L -5 0 L -9 8 Z" />'
        "</g>"
        '<g data-live-target style="display:none" transform="translate(0 0)">'
        "<title>DimOS planner target</title>"
        '<circle class="map-dimos-target-ring" cx="0" cy="0" r="14" />'
        '<circle class="map-dimos-target-core" cx="0" cy="0" r="4" />'
        "</g>"
        '<g data-go-to-marker style="display:none" transform="translate(0 0)">'
        "<title>DimOS go_to target</title>"
        '<circle class="map-go-to-ring" cx="0" cy="0" r="13" />'
        '<line class="map-go-to-cross" x1="-17" y1="0" x2="-6" y2="0" />'
        '<line class="map-go-to-cross" x1="6" y1="0" x2="17" y2="0" />'
        '<line class="map-go-to-cross" x1="0" y1="-17" x2="0" y2="-6" />'
        '<line class="map-go-to-cross" x1="0" y1="6" x2="0" y2="17" />'
        "</g>"
    )


def _render_scan_item(observation: dict[str, Any]) -> str:
    tag_ids = ", ".join(str(tag_id) for tag_id in observation["visible_tag_ids"]) or "none"
    return (
        "<li>"
        f"<strong>{escape(str(observation['id']))}</strong> "
        f"{escape(str(observation['zone_id']))} / tags {escape(tag_ids)}"
        "</li>"
    )


def metric(label: str, value: object) -> str:
    return (
        '<div class="metric">'
        f"<span class=\"muted\">{escape(label)}</span>"
        f"<strong>{escape(str(value))}</strong>"
        "</div>"
    )


def qr_cargo_panel(events: list[dict[str, Any]]) -> str:
    rows = []
    for event in events[:50]:
        event_id = escape(str(event.get("event_id") or ""))
        timestamp = _format_timestamp(event.get("timestamp"))
        robot_pose = _format_qr_robot_pose(event.get("robot_pose_at_detection"))
        zone_shelf = " / ".join(
            item
            for item in [str(event.get("zone") or ""), str(event.get("shelf_id") or "")]
            if item
        ) or "unknown"
        select_disabled = "" if event.get("map_position") else " disabled"
        rows.append(
            f'<tr data-qr-event-row data-qr-event-id="{event_id}">'
            f"<td>{escape(timestamp)}</td>"
            f"<td>{escape(str(event.get('warehouse_id') or 'unknown'))}</td>"
            f"<td>{escape(str(event.get('location_node_id') or 'unknown'))}</td>"
            f"<td>{escape(str(event.get('cargo_id') or 'unknown'))}</td>"
            f"<td>{escape(zone_shelf)}</td>"
            f"<td>{escape(str(event.get('task') or 'unknown'))}</td>"
            f"<td>{escape(str(event.get('source') or 'unknown'))}</td>"
            f"<td>{escape(str(event.get('status') or 'unknown'))} / "
            f"{escape(str(event.get('action_policy') or 'report_only'))}</td>"
            f"<td>{escape(robot_pose)}</td>"
            '<td class="qr-actions">'
            f'<button type="button" data-qr-select="{event_id}"{select_disabled}>Select on map</button>'
            f'<button type="button" data-qr-promote-package="{event_id}">Promote to Package</button>'
            f'<button type="button" data-qr-promote-label="{event_id}">Promote to Label</button>'
            f'<button type="button" data-qr-bind-location="{event_id}">Bind Location Node</button>'
            f'<button type="button" data-qr-copy="{event_id}">Copy JSON</button>'
            "</td>"
            "</tr>"
        )
    if not rows:
        rows.append(
            '<tr data-qr-empty><td colspan="10" class="muted">No QR cargo events yet</td></tr>'
        )
    return (
        '<div class="qr-cargo-status" data-qr-cargo-status>QR cargo idle</div>'
        '<div class="qr-cargo-table"><table><thead><tr>'
        '<th>Time</th><th>WH</th><th>Location Node</th><th>Cargo</th>'
        '<th>Zone/Shelf</th><th>Task</th><th>Source</th><th>Status/Policy</th>'
        '<th>Robot Pose</th><th>Actions</th></tr></thead>'
        f'<tbody data-qr-cargo-events>{"".join(rows)}</tbody></table></div>'
    )


def _format_timestamp(value: object) -> str:
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return "unknown"


def _format_qr_robot_pose(value: object) -> str:
    if not isinstance(value, dict):
        return "unknown"
    x = _float_or_none(value.get("x"))
    y = _float_or_none(value.get("y"))
    yaw = _float_or_none(value.get("yaw"))
    if x is None or y is None:
        return "unknown"
    if yaw is None:
        return f"{x:.2f}, {y:.2f}"
    return f"{x:.2f}, {y:.2f}, yaw {yaw:.2f}"


def package_table(packages: list[dict[str, Any]]) -> str:
    rows = []
    for package in packages:
        state = str(package["state"])
        rows.append(
            "<tr>"
            f"<td>{escape(str(package['package_id']))}</td>"
            f"<td>{escape(str(package['expected_zone_id']))}</td>"
            f"<td>{escape(str(package.get('observed_zone_id') or 'not observed'))}</td>"
            f"<td class=\"state-{escape(state)}\">{escape(state)}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>Package</th><th>Expected</th><th>Observed</th>"
        "<th>State</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def incident_table(incidents: list[dict[str, Any]]) -> str:
    rows = []
    for incident in incidents:
        severity = str(incident["severity"])
        state = str(incident["state"])
        rows.append(
            "<tr>"
            f"<td>{escape(str(incident['id']))}</td>"
            f"<td class=\"severity-{escape(severity)}\">{escape(severity)}</td>"
            f"<td>{escape(str(incident['title']))}</td>"
            f"<td class=\"state-{escape(state)}\">{escape(state)}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>ID</th><th>Severity</th><th>Title</th><th>State</th>"
        "</tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def work_order_table(work_orders: list[dict[str, Any]]) -> str:
    rows = []
    for work_order in work_orders:
        state = str(work_order["state"])
        rows.append(
            "<tr>"
            f"<td>{escape(str(work_order['id']))}</td>"
            f"<td>{escape(str(work_order['incident_id']))}</td>"
            f"<td>{escape(str(work_order['assignee']))}</td>"
            f"<td class=\"state-{escape(state)}\">{escape(state)}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>ID</th><th>Incident</th><th>Assignee</th><th>State</th>"
        "</tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def checkpoint_table(checkpoints: list[dict[str, Any]]) -> str:
    rows = []
    for checkpoint in checkpoints:
        verified = bool(checkpoint.get("verified"))
        state = "verified" if verified else "missing"
        tag_id = checkpoint.get("expected_tag_id")
        observation_id = checkpoint.get("observation_id") or "not observed"
        rows.append(
            "<tr>"
            f"<td>{escape(str(checkpoint['target_id']))}</td>"
            f"<td>{escape(str(tag_id if tag_id is not None else 'none'))}</td>"
            f"<td>{escape(str(observation_id))}</td>"
            f"<td class=\"state-{state}\">{state}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>Target</th><th>Tag</th><th>Observation</th><th>State</th>"
        "</tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def route_table(stops: list[dict[str, Any]]) -> str:
    rows = []
    for stop in stops:
        state = "verified" if stop.get("tag_verified") else "missing"
        tag_id = stop.get("expected_tag_id")
        rows.append(
            "<tr>"
            f"<td>{escape(str(stop['sequence']))}</td>"
            f"<td>{escape(str(stop['target_id']))}</td>"
            f"<td>{escape(str(tag_id if tag_id is not None else 'none'))}</td>"
            f"<td>{escape(str(stop.get('retries', 0)))}</td>"
            f"<td class=\"state-{state}\">{state}</td>"
            "</tr>"
        )
    return (
        "<table><thead><tr><th>#</th><th>Target</th><th>Tag</th><th>Retries</th>"
        "<th>State</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


def poi_list(poi_data: dict[str, Any]) -> str:
    captures = poi_data.get("captures") or []
    readings = poi_data.get("readings") or []
    items = []
    for capture in captures[:4]:
        tags = ", ".join(str(tag_id) for tag_id in capture.get("visible_tag_ids") or []) or "none"
        incidents = capture.get("related_incident_ids") or []
        incident_text = f" / incidents {', '.join(incidents)}" if incidents else ""
        items.append(
            "<li>"
            f"<strong>{escape(str(capture['id']))}</strong> "
            f"{escape(str(capture.get('zone_id') or 'unknown'))} / tags {escape(tags)}"
            f"{escape(incident_text)}"
            "</li>"
        )
    for reading in readings[:3]:
        if reading.get("kind") == "temperature":
            label = (
                f"{reading['asset_id']} {reading['reading_celsius']}C "
                f"<= {reading['max_celsius']}C"
            )
        else:
            label = f"{reading['asset_id']} {reading.get('state', 'unknown')}"
        items.append(f"<li><strong>{escape(str(reading['kind']))}</strong> {escape(label)}</li>")
    return '<ul class="compact-list">' + "".join(items) + "</ul>"


def write_dashboard_html(run_dir: str | Path, *, robot_control_token: str | None = None) -> Path:
    root = Path(run_dir)
    state = _read_json(root / "state.json")
    report = _read_json(root / "report.json")
    authoring = load_map_authoring(
        root,
        site_id=str((state.get("site") or {}).get("site_id") or ""),
    ).model_dump(mode="json")
    html_path = root / "dashboard.html"
    html_path.write_text(
        render_dashboard_html(
            state,
            report,
            robot_control_token=robot_control_token,
            authoring=authoring,
            qr_events=get_latest_qr_events(root),
        ),
        encoding="utf-8",
    )
    return html_path


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _latest_fact_value(observations: list[dict[str, Any]], key: str) -> object | None:
    for observation in reversed(observations):
        facts = observation.get("facts") or {}
        if key in facts:
            return facts[key]
    return None


def _to_float(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _to_bool(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "clear"}:
            return True
        if normalized in {"false", "no", "0", "blocked"}:
            return False
    return None
