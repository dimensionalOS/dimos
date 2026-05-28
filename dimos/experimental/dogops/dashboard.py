from __future__ import annotations

import asyncio
from collections import deque
from concurrent.futures import TimeoutError as FutureTimeoutError
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import os
from pathlib import Path
import secrets
import shlex
import shutil
import subprocess
import threading
import time
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

from pydantic import ValidationError

from dimos.experimental.dogops.dashboard_static import (
    build_map_data,
    build_poi_data,
    build_route_data,
    write_dashboard_html,
)
from dimos.experimental.dogops.heatmap_runs import (
    gather_heatmap_run,
    heatmap_snapshot_for_route_run,
    latest_heatmap_snapshot,
)
from dimos.experimental.dogops.live_camera import DogOpsLiveCameraAdapter
from dimos.experimental.dogops.live_map import DogOpsLiveMapAdapter
from dimos.experimental.dogops.map_authoring import (
    EditableIncidentLocation,
    EditableMapEntity,
    EditableNoGoShape,
    EditableRoute,
    EditableTagBinding,
    MapAuthoringState,
    delete_entity,
    delete_no_go_shape,
    delete_route,
    delete_tag_binding,
    export_authoring_yaml,
    load_map_authoring,
    replace_entity,
    replace_incident_location,
    replace_no_go_shape,
    replace_route,
    replace_tag_binding,
    save_map_authoring,
    select_route,
    publish_no_go_constraints,
    validation_error_message,
)
from dimos.experimental.dogops.qr_cargo import (
    append_qr_event,
    get_latest_qr_events,
    get_qr_event,
    load_qr_events,
    qr_events_path,
)
from dimos.experimental.dogops.route_executor import (
    DogOpsRouteExecutor,
    load_route_execution,
    request_route_stop,
)
from dimos.experimental.dogops.route_run_store import RouteRunStore
from dimos.experimental.dogops.store import DogOpsStore

try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.core.module import Module
except ModuleNotFoundError:

    class Module:
        @classmethod
        def blueprint(cls, **kwargs: object) -> dict[str, object]:
            return {"module": cls.__name__, "kwargs": kwargs}


DEFAULT_JOG_DURATION_S = 0.35
MAX_JOG_DURATION_S = 2.00
MAX_LINEAR_SPEED = 0.65
MAX_ANGULAR_SPEED = 1.10
DIRECT_ROUTE_PULSE_S = 0.35
DIRECT_ROUTE_MAX_SPEED = 0.35
DIRECT_ROUTE_MAX_ANGULAR_SPEED = 0.65
DIRECT_ROUTE_GAIN = 0.8
DIRECT_ROUTE_REACH_RADIUS_M = 0.35
DIRECT_ROUTE_WAYPOINT_TIMEOUT_S = 20.0
ROBOT_CALL_TIMEOUT_S = 8.0
DIMOS_MCP_CALL_TIMEOUT_S = 12.0
DIMOS_ROUTE_CALL_TIMEOUT_S = 180.0
WEBRTC_COMMAND_TIMEOUT_S = 2.0
MCP_UNAVAILABLE_MARKERS = (
    "no running mcp server",
    "mcp server is not running",
    "connection refused",
    "failed to connect",
    "tool not found",
)
HARD_STOP_REPEATS = 6
HARD_STOP_INTERVAL_S = 0.05
ROBOT_POSE_HISTORY_LIMIT = 240
MOTION_PROFILES: dict[str, tuple[float, float, float]] = {
    "nudge": (0.35, 1.0, 1.0),
    "step": (1.00, 2.3, 2.0),
    "walk": (2.00, 4.0, 3.0),
}
DEFAULT_MOTION_PROFILE = "nudge"
ROBOT_JOG_COMMANDS: dict[str, tuple[float, float, float]] = {
    "forward": (0.15, 0.0, 0.0),
    "backward": (-0.15, 0.0, 0.0),
    "left": (0.0, 0.15, 0.0),
    "right": (0.0, -0.15, 0.0),
    "yaw_left": (0.0, 0.0, 0.35),
    "yaw_right": (0.0, 0.0, -0.35),
    "hard_stop": (0.0, 0.0, 0.0),
    "stop": (0.0, 0.0, 0.0),
}
HARD_STOP_COMMANDS = {"hard_stop", "stop"}
ROBOT_POSTURE_COMMANDS = {"wake", "balance", "sleep"}
ROBOT_CONTROL_TOKEN_HEADER = "X-DogOps-Control-Token"
DEFAULT_ROBOT_IP = (
    os.environ.get("DOGOPS_ROBOT_IP")
    or os.environ.get("GO2_IP")
    or os.environ.get("ROBOT_IP")
    or "192.168.12.1"
)
_ROBOT_SESSIONS: dict[str, _RobotMotionSession] = {}
_ROBOT_SESSIONS_LOCK = threading.Lock()
_AUTHORING_LOCK = threading.Lock()
_QR_EVENTS_LOCK = threading.Lock()
_LIVE_MAP_ADAPTER = DogOpsLiveMapAdapter()
_LIVE_CAMERA_ADAPTER = DogOpsLiveCameraAdapter()


class DogOpsDashboardServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        *,
        live_map_adapter: DogOpsLiveMapAdapter,
        live_camera_adapter: DogOpsLiveCameraAdapter | None = None,
    ) -> None:
        self.live_map_adapter = live_map_adapter
        self.live_camera_adapter = live_camera_adapter
        super().__init__(server_address, handler_class)

    def server_close(self) -> None:
        for adapter_name in ("live_map_adapter", "live_camera_adapter"):
            stop = getattr(getattr(self, adapter_name, None), "stop", None)
            if stop is not None:
                stop()
        with _ROBOT_SESSIONS_LOCK:
            sessions = list(_ROBOT_SESSIONS.values())
            _ROBOT_SESSIONS.clear()
        for session in sessions:
            session.close()
        super().server_close()


def make_dashboard_server(run_dir: str | Path, host: str, port: int) -> ThreadingHTTPServer:
    root = Path(run_dir)
    robot_control_token = os.environ.get("DOGOPS_DASHBOARD_TOKEN") or secrets.token_urlsafe(32)
    write_dashboard_html(root, robot_control_token=robot_control_token)
    token = robot_control_token

    class Handler(DogOpsDashboardHandler):
        run_dir = root
        robot_control_token = token
        robot_ip = DEFAULT_ROBOT_IP

    return DogOpsDashboardServer(
        (host, port),
        Handler,
        live_map_adapter=_LIVE_MAP_ADAPTER,
        live_camera_adapter=_LIVE_CAMERA_ADAPTER,
    )


def serve_dashboard(run_dir: str | Path, host: str = "127.0.0.1", port: int = 8765) -> None:
    server = make_dashboard_server(run_dir, host, port)
    address = f"http://{host}:{server.server_address[1]}"
    print(f"DogOps dashboard serving {Path(run_dir)} at {address}")
    try:
        server.serve_forever()
    finally:
        server.server_close()


class DogOpsDashboardModule(Module):
    def __init__(
        self,
        *,
        run_dir: str | Path = ".dogops/runs/latest",
        host: str = "127.0.0.1",
        port: int = 8765,
        **_: object,
    ) -> None:
        if _:
            super().__init__(**_)
        self.run_dir = Path(run_dir)
        self.host = host
        self.port = port

    def write_dashboard(self) -> str:
        return str(write_dashboard_html(self.run_dir))

    def serve(self) -> None:
        serve_dashboard(self.run_dir, self.host, self.port)

    def status(self) -> dict[str, object]:
        return {
            "run_dir": str(self.run_dir),
            "dashboard_html": str(self.run_dir / "dashboard.html"),
            "host": self.host,
            "port": self.port,
            "exists": (self.run_dir / "dashboard.html").exists(),
        }


class DogOpsDashboardHandler(BaseHTTPRequestHandler):
    run_dir: Path
    robot_control_token: str
    robot_ip: str

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path in {"/", "/dashboard.html"}:
            self._send_file(self.run_dir / "dashboard.html", "text/html; charset=utf-8")
        elif path == "/api/state":
            self._send_file(self.run_dir / "state.json", "application/json")
        elif path == "/api/report":
            self._send_file(self.run_dir / "report.json", "application/json")
        elif path == "/api/nav":
            report = self._read_json(self.run_dir / "report.json")
            self._send_json(report.get("nav_summary") or {})
        elif path == "/api/camera/status":
            if self._authorize_local_read():
                self._send_json(_LIVE_CAMERA_ADAPTER.status())
        elif path == "/api/camera/frame.jpg":
            if self._authorize_local_read():
                self._send_camera_frame()
        elif path == "/api/map":
            state = self._read_json(self.run_dir / "state.json")
            report = self._read_json(self.run_dir / "report.json")
            authoring = self._load_authoring(state)
            self._send_json(
                build_map_data(
                    state,
                    report,
                    live_overlay=_LIVE_MAP_ADAPTER.snapshot(),
                    heatmap_snapshot=latest_heatmap_snapshot(self.run_dir),
                    authoring=authoring.model_dump(mode="json"),
                    qr_events=load_qr_events(self.run_dir),
                )
            )
        elif path == "/api/map/authoring":
            state = self._read_json(self.run_dir / "state.json")
            self._send_json(self._load_authoring(state).model_dump(mode="json"))
        elif path == "/api/qr/events":
            self._send_qr_events()
        elif path == "/api/qr/events/latest":
            self._send_latest_qr_events(parse_qs(parsed.query))
        elif path.startswith("/api/qr/events/"):
            self._send_qr_event(unquote(path.split("/")[-1]))
        elif path == "/api/map/routes/status":
            if self._authorize_map_authoring_write():
                self._route_execution_status()
        elif path == "/api/route-runs":
            if self._authorize_map_authoring_write():
                self._route_runs_list()
        elif path == "/api/route-runs/current":
            if self._authorize_map_authoring_write():
                self._route_runs_current()
        elif path == "/api/route-runs/images":
            if self._authorize_map_authoring_write():
                self._route_run_images()
        elif path.startswith("/api/route-runs/"):
            if self._authorize_map_authoring_write():
                self._route_runs_detail(path)
        elif path == "/api/robot/pose":
            if self._authorize_local_read():
                self._send_json(_robot_pose_snapshot(self.robot_ip))
        elif path == "/api/route":
            state = self._read_json(self.run_dir / "state.json")
            report = self._read_json(self.run_dir / "report.json")
            authoring = self._load_authoring(state)
            self._send_json(build_route_data(state, report, authoring=authoring.model_dump(mode="json")))
        elif path == "/api/poi":
            state = self._read_json(self.run_dir / "state.json")
            report = self._read_json(self.run_dir / "report.json")
            self._send_json(build_poi_data(state, report))
        else:
            self._send_json({"error": "not_found", "path": path}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path.startswith("/api/work_orders/") and path.endswith("/ready_to_verify"):
            work_order_id = path.split("/")[3]
            self._mark_work_order_ready(work_order_id)
        elif path == "/api/operator/event":
            self._record_operator_event()
        elif path == "/api/qr/events":
            if self._authorize_map_authoring_write():
                self._record_qr_event()
        elif path.startswith("/api/qr/events/") and path.endswith("/promote_to_package"):
            if self._authorize_map_authoring_write():
                self._promote_qr_event_to_package(_qr_event_id_from_path(path))
        elif path.startswith("/api/qr/events/") and path.endswith("/promote_to_label"):
            if self._authorize_map_authoring_write():
                self._promote_qr_event_to_label(_qr_event_id_from_path(path))
        elif path.startswith("/api/qr/events/") and path.endswith("/bind_location_node"):
            if self._authorize_map_authoring_write():
                self._bind_qr_location_node(_qr_event_id_from_path(path))
        elif path == "/api/robot/jog":
            if self._authorize_robot_control():
                self._robot_jog()
        elif path == "/api/robot/posture":
            if self._authorize_robot_control():
                self._robot_posture()
        elif path == "/api/robot/go_to":
            if self._authorize_robot_control():
                self._robot_go_to()
        elif path == "/api/robot/map_start":
            if self._authorize_robot_control():
                self._robot_map_start()
        elif path == "/api/robot/map_origin":
            if self._authorize_robot_control():
                self._robot_map_origin()
        elif path == "/api/robot/scan_zone":
            if self._authorize_robot_control():
                self._robot_scan_zone()
        elif path == "/api/map/heatmap/gather":
            if self._authorize_robot_control():
                self._gather_heatmap()
        elif path == "/api/map/entities":
            if self._authorize_map_authoring_write():
                self._upsert_map_entity()
        elif path == "/api/map/no_go_shapes":
            if self._authorize_map_authoring_write():
                self._upsert_no_go_shape()
        elif path == "/api/map/routes":
            if self._authorize_map_authoring_write():
                self._upsert_map_route()
        elif path == "/api/map/routes/follow":
            if self._authorize_map_authoring_write():
                self._follow_map_route()
        elif path == "/api/map/routes/stop":
            if self._authorize_map_authoring_write():
                self._stop_map_route()
        elif path.startswith("/api/map/incidents/") and path.endswith("/location"):
            if self._authorize_map_authoring_write():
                parts = path.split("/")
                incident_id = parts[4] if len(parts) >= 6 else ""
                self._upsert_incident_location(incident_id)
        elif path == "/api/map/tag_bindings":
            if self._authorize_map_authoring_write():
                self._upsert_tag_binding()
        elif path == "/api/map/export":
            if self._authorize_map_authoring_write():
                self._export_map_authoring()
        elif path == "/api/map/no_go_shapes/publish":
            if self._authorize_map_authoring_write():
                self._publish_no_go_shapes()
        elif path.startswith("/api/map/entities/") and path.endswith("/from_observation"):
            if self._authorize_map_authoring_write():
                parts = path.split("/")
                entity_id = parts[4] if len(parts) >= 6 else ""
                self._place_entity_from_observation(entity_id)
        elif path.startswith("/api/map/routes/") and path.endswith("/select"):
            if self._authorize_map_authoring_write():
                parts = path.split("/")
                route_id = parts[4] if len(parts) >= 6 else ""
                self._select_map_route(route_id)
        else:
            self._send_json({"error": "not_found", "path": path}, HTTPStatus.NOT_FOUND)

    def do_PUT(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/map/authoring":
            if self._authorize_map_authoring_write():
                self._replace_map_authoring()
        elif path.startswith("/api/map/entities/"):
            if self._authorize_map_authoring_write():
                self._upsert_map_entity(path.split("/")[-1])
        elif path.startswith("/api/map/no_go_shapes/"):
            if self._authorize_map_authoring_write():
                self._upsert_no_go_shape(path.split("/")[-1])
        elif path.startswith("/api/map/routes/"):
            if self._authorize_map_authoring_write():
                self._upsert_map_route(path.split("/")[-1])
        else:
            self._send_json({"error": "not_found", "path": path}, HTTPStatus.NOT_FOUND)

    def do_DELETE(self) -> None:
        path = urlparse(self.path).path
        if path.startswith("/api/map/entities/"):
            if self._authorize_map_authoring_write():
                self._delete_map_entity(path.split("/")[-1])
        elif path.startswith("/api/map/no_go_shapes/"):
            if self._authorize_map_authoring_write():
                self._delete_no_go_shape(path.split("/")[-1])
        elif path.startswith("/api/map/routes/"):
            if self._authorize_map_authoring_write():
                self._delete_map_route(path.split("/")[-1])
        elif path.startswith("/api/map/tag_bindings/"):
            if self._authorize_map_authoring_write():
                self._delete_tag_binding(path.split("/")[-1])
        else:
            self._send_json({"error": "not_found", "path": path}, HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: object) -> None:
        return

    def _load_authoring(self, state: dict[str, Any] | None = None) -> MapAuthoringState:
        if state is None:
            state = self._read_json(self.run_dir / "state.json")
        site_id = str((state.get("site") or {}).get("site_id") or "")
        return load_map_authoring(self.run_dir, site_id=site_id)

    def _send_authoring(self, authoring: MapAuthoringState) -> None:
        state = self._read_json(self.run_dir / "state.json")
        report = self._read_json(self.run_dir / "report.json")
        payload = authoring.model_dump(mode="json")
        self._send_json(
            {
                "ok": True,
                "authoring": payload,
                "map": build_map_data(
                    state,
                    report,
                    live_overlay=_LIVE_MAP_ADAPTER.snapshot(),
                    heatmap_snapshot=latest_heatmap_snapshot(self.run_dir),
                    authoring=payload,
                    qr_events=load_qr_events(self.run_dir),
                ),
            }
        )

    def _handle_authoring_error(self, exc: Exception) -> None:
        message = (
            validation_error_message(exc)
            if isinstance(exc, (ValidationError, ValueError))
            else str(exc)
        )
        self._send_json(
            {"ok": False, "error": "invalid_map_authoring", "message": message},
            HTTPStatus.BAD_REQUEST,
        )

    def _persist_authoring_mutation(self, mutate: Any) -> MapAuthoringState:
        with _AUTHORING_LOCK:
            authoring = mutate(self._load_authoring())
            self._save_authoring(authoring)
            return authoring

    def _save_authoring(self, authoring: MapAuthoringState) -> None:
        save_map_authoring(self.run_dir, authoring)
        write_dashboard_html(self.run_dir, robot_control_token=self.robot_control_token)

    def _replace_map_authoring(self) -> None:
        state = self._read_json(self.run_dir / "state.json")
        payload = self._read_body_json()
        payload.setdefault("site_id", str((state.get("site") or {}).get("site_id") or ""))
        try:
            authoring = MapAuthoringState.model_validate(payload)
            with _AUTHORING_LOCK:
                self._save_authoring(authoring)
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _upsert_map_entity(self, entity_id: str | None = None) -> None:
        payload = self._read_body_json()
        if entity_id:
            payload["id"] = entity_id
        try:
            entity = EditableMapEntity.model_validate(payload)
            authoring = self._persist_authoring_mutation(
                lambda existing: replace_entity(existing, entity)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _place_entity_from_observation(self, entity_id: str) -> None:
        payload = self._read_body_json()
        state = self._read_json(self.run_dir / "state.json")
        try:
            observation = _resolve_observation(state, payload)
            pose = _observation_pose(state, observation)
            existing = _authored_or_site_entity(state, self._load_authoring(), entity_id)
            if existing is None:
                existing = {
                    "id": entity_id,
                    "kind": str(payload.get("kind") or "checkpoint"),
                    "label": entity_id,
                }
            existing["pose"] = {
                "x": pose[0],
                "y": pose[1],
                "theta_deg": pose[2],
                "source": "observation",
            }
            if observation.get("tag_id") is not None:
                existing["tag_id"] = observation.get("tag_id")
            entity = EditableMapEntity.model_validate(existing)
            authoring = self._persist_authoring_mutation(
                lambda current: replace_entity(current, entity)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _delete_map_entity(self, entity_id: str) -> None:
        authoring = self._persist_authoring_mutation(
            lambda existing: delete_entity(existing, entity_id)
        )
        self._send_authoring(authoring)

    def _upsert_no_go_shape(self, shape_id: str | None = None) -> None:
        payload = self._read_body_json()
        if shape_id:
            payload["id"] = shape_id
        payload.setdefault("dimos_constraint_status", "not_supported")
        try:
            shape = EditableNoGoShape.model_validate(payload)
            authoring = self._persist_authoring_mutation(
                lambda existing: replace_no_go_shape(existing, shape)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _delete_no_go_shape(self, shape_id: str) -> None:
        authoring = self._persist_authoring_mutation(
            lambda existing: delete_no_go_shape(existing, shape_id)
        )
        self._send_authoring(authoring)

    def _upsert_map_route(self, route_id: str | None = None) -> None:
        payload = self._read_body_json()
        if route_id:
            payload["id"] = route_id
        try:
            route = EditableRoute.model_validate(payload)
            authoring = self._persist_authoring_mutation(
                lambda existing: replace_route(existing, route)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _delete_map_route(self, route_id: str) -> None:
        authoring = self._persist_authoring_mutation(
            lambda existing: delete_route(existing, route_id)
        )
        self._send_authoring(authoring)

    def _select_map_route(self, route_id: str) -> None:
        try:
            authoring = self._persist_authoring_mutation(
                lambda existing: select_route(existing, route_id)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _gather_heatmap(self) -> None:
        payload = self._read_body_json()
        area_id = str(payload.get("area_id") or "").strip()
        try:
            duration_s = float(payload.get("duration_s") or 0.0)
        except (TypeError, ValueError):
            duration_s = 0.0
        result = gather_heatmap_run(
            self.run_dir,
            live_snapshot=_LIVE_MAP_ADAPTER.snapshot(),
            live_snapshot_reader=_LIVE_MAP_ADAPTER.snapshot,
            area_id=area_id,
            duration_s=max(0.0, duration_s),
        )
        status = HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT
        self._send_json(result, status)

    def _follow_map_route(self) -> None:
        payload = self._read_body_json()
        route_id = payload.get("route_id")
        route_id_arg = str(route_id).strip() if route_id is not None else None
        dry_run = bool(payload.get("dry_run", False))
        use_planner = bool(payload.get("use_planner", False))
        try:
            result = _run_robot_call(
                lambda: _run_robot_follow_route(
                    route_id_arg,
                    dry_run,
                    use_planner=use_planner,
                    robot_ip=self.robot_ip,
                    run_dir=self.run_dir,
                ),
                timeout_s=DIMOS_ROUTE_CALL_TIMEOUT_S,
            )
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_mcp_unavailable",
                    "message": str(exc),
                    **self._route_execution_payload(),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "follow_route_timeout",
                    "message": str(exc),
                    **self._route_execution_payload(),
                },
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            if _is_mcp_unavailable_error(exc):
                self._send_json(
                    {
                        "ok": False,
                        "error": "dimos_mcp_unavailable",
                        "message": str(exc),
                        **self._route_execution_payload(),
                    },
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            self._send_json(
                {
                    "ok": False,
                    "error": "follow_route_failed",
                    "message": str(exc),
                    **self._route_execution_payload(),
                },
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        route_execution = _mcp_route_execution(result)
        self._send_json(
            {
                "ok": True,
                "command": "follow_route",
                "route_mode": (
                    "planner"
                    if use_planner and not dry_run
                    else ("dry_run" if dry_run else "direct")
                ),
                **(result or {}),
                **self._route_execution_payload(route_execution=route_execution),
            }
        )

    def _stop_map_route(self) -> None:
        request_route_stop(self.run_dir)
        hard_stop: dict[str, Any] = {"attempted": True, "ok": False}
        try:
            hard_stop_result = _run_robot_call(lambda: _run_route_hard_stop(self.robot_ip))
        except Exception as exc:
            hard_stop["error"] = str(exc)
        else:
            hard_stop.update({"ok": True, **(hard_stop_result or {})})
        try:
            result = _run_robot_call(_run_robot_stop_route)
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_mcp_unavailable",
                    "message": str(exc),
                    "hard_stop": hard_stop,
                    **self._route_execution_payload(),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "stop_route_timeout",
                    "message": str(exc),
                    "hard_stop": hard_stop,
                    **self._route_execution_payload(),
                },
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            if _is_mcp_unavailable_error(exc):
                self._send_json(
                    {
                        "ok": False,
                        "error": "dimos_mcp_unavailable",
                        "message": str(exc),
                        "hard_stop": hard_stop,
                        **self._route_execution_payload(),
                    },
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            self._send_json(
                {
                    "ok": False,
                    "error": "stop_route_failed",
                    "message": str(exc),
                    "hard_stop": hard_stop,
                    **self._route_execution_payload(),
                },
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        route_execution = _mcp_route_execution(result)
        self._send_json(
            {
                "ok": True,
                "command": "stop_route",
                "hard_stop": hard_stop,
                **(result or {}),
                **self._route_execution_payload(route_execution=route_execution),
            }
        )

    def _route_execution_status(self) -> None:
        self._send_json({"ok": True, **self._route_execution_payload()})

    def _route_runs_list(self) -> None:
        store = RouteRunStore(self.run_dir)
        self._send_json({"ok": True, "route_runs": store.list_route_runs()})

    def _route_runs_current(self) -> None:
        store = RouteRunStore(self.run_dir)
        current = store.current_route_run()
        route_events = store.route_run_events(current["route_run_id"]) if current else []
        self._send_json(
            {
                "ok": True,
                "route_run": current,
                "events": route_events,
                "timeline": self._unified_timeline(route_events, current) if current else [],
                "evidence": store.route_run_evidence(current["route_run_id"]) if current else [],
            }
        )

    def _route_run_images(self) -> None:
        store = RouteRunStore(self.run_dir)
        images: list[dict[str, Any]] = []
        for route_run in store.list_route_runs(limit=12):
            route_run_id = str(route_run.get("route_run_id") or "")
            if not route_run_id:
                continue
            for evidence in store.route_run_evidence(route_run_id):
                if evidence.get("kind") != "image" or not evidence.get("path"):
                    continue
                evidence_id = str(evidence.get("evidence_id") or "")
                images.append(
                    {
                        "evidence_id": evidence_id,
                        "route_run_id": route_run_id,
                        "dogops_run_id": route_run.get("dogops_run_id"),
                        "route_id": route_run.get("route_id"),
                        "created_at": evidence.get("created_at"),
                        "mime_type": evidence.get("mime_type"),
                        "metadata": evidence.get("metadata") or {},
                        "url": f"/api/route-runs/{route_run_id}/evidence/{evidence_id}/file",
                    }
                )
        images.sort(key=lambda item: float(item.get("created_at") or 0.0), reverse=True)
        self._send_json({"ok": True, "images": images[:12]})

    def _route_runs_detail(self, path: str) -> None:
        parts = [part for part in path.split("/") if part]
        route_run_id = parts[2] if len(parts) >= 3 else ""
        if not route_run_id:
            self._send_json({"ok": False, "error": "missing_route_run_id"}, HTTPStatus.BAD_REQUEST)
            return
        store = RouteRunStore(self.run_dir)
        route_run = store.route_run_detail(route_run_id)
        if not route_run:
            self._send_json({"ok": False, "error": "route_run_not_found"}, HTTPStatus.NOT_FOUND)
            return
        if len(parts) == 4 and parts[3] == "events":
            self._send_json({"ok": True, "events": store.route_run_events(route_run_id)})
            return
        if len(parts) == 4 and parts[3] == "evidence":
            self._send_json({"ok": True, "evidence": store.route_run_evidence(route_run_id)})
            return
        if len(parts) == 6 and parts[3] == "evidence" and parts[5] == "file":
            self._send_route_run_evidence_file(store, route_run, parts[4])
            return
        route_events = store.route_run_events(route_run_id)
        evidence = store.route_run_evidence(route_run_id)
        self._send_json(
            {
                "ok": True,
                "route_run": route_run,
                "events": route_events,
                "timeline": self._unified_timeline(route_events, route_run),
                "evidence": evidence,
                "map": self._route_run_map(route_run, evidence),
            }
        )

    def _send_route_run_evidence_file(
        self,
        store: RouteRunStore,
        route_run: dict[str, Any],
        evidence_id: str,
    ) -> None:
        evidence = next(
            (
                item
                for item in store.route_run_evidence(str(route_run["route_run_id"]))
                if item.get("evidence_id") == evidence_id
            ),
            None,
        )
        if evidence is None or evidence.get("kind") != "image" or not evidence.get("path"):
            self._send_json({"ok": False, "error": "evidence_image_not_found"}, HTTPStatus.NOT_FOUND)
            return
        mime_type = str(evidence.get("mime_type") or "")
        if mime_type not in {"image/jpeg", "image/png", "image/webp"}:
            self._send_json({"ok": False, "error": "unsupported_evidence_image_type"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
            return
        run_root = Path(str(route_run.get("run_dir") or self.run_dir)).resolve()
        evidence_path = Path(str(evidence["path"]))
        resolved = evidence_path.resolve()
        try:
            resolved.relative_to(run_root)
        except ValueError:
            self._send_json({"ok": False, "error": "evidence_path_outside_run"}, HTTPStatus.FORBIDDEN)
            return
        self._send_file(resolved, mime_type)

    def _route_run_map(
        self,
        route_run: dict[str, Any],
        evidence: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        timeline_run_dir = Path(str(route_run.get("run_dir") or self.run_dir))
        state_path = timeline_run_dir / "state.json"
        report_path = timeline_run_dir / "report.json"
        if not state_path.exists() or not report_path.exists():
            return None
        state = self._read_json(state_path)
        report = self._read_json(report_path)
        authoring = load_map_authoring(
            timeline_run_dir,
            site_id=str((state.get("site") or {}).get("site_id") or ""),
        ).model_dump(mode="json")
        route_snapshot = route_run.get("selected_route_snapshot")
        if isinstance(route_snapshot, dict) and route_snapshot.get("waypoints"):
            routes = [
                route
                for route in authoring.get("routes") or []
                if isinstance(route, dict) and route.get("id") != route_snapshot.get("id")
            ]
            routes.append(route_snapshot)
            authoring["routes"] = routes
            authoring["selected_route_id"] = route_snapshot.get("id")
        heatmap_snapshot = heatmap_snapshot_for_route_run(
            timeline_run_dir,
            str(route_run.get("route_run_id") or ""),
            evidence=evidence,
        )
        return build_map_data(
            state,
            report,
            heatmap_snapshot=heatmap_snapshot,
            authoring=authoring,
            qr_events=load_qr_events(timeline_run_dir),
        )

    def _unified_timeline(
        self,
        route_events: list[dict[str, Any]],
        route_run: dict[str, Any],
    ) -> list[dict[str, Any]]:
        timeline_run_dir = Path(str(route_run.get("run_dir") or self.run_dir))
        state = self._read_optional_json(timeline_run_dir / "state.json")
        report = self._read_optional_json(timeline_run_dir / "report.json")
        route_run_id = str(route_run.get("route_run_id") or "")
        dogops_run_id = str(route_run.get("dogops_run_id") or state.get("run", {}).get("id") or timeline_run_dir.name)
        rows = [
            {
                "event_id": f"TL-{event.get('event_id') or event.get('sequence')}",
                "route_run_id": event.get("route_run_id") or route_run_id,
                "ts": event.get("ts") or 0,
                "sequence": event.get("sequence"),
                "kind": event.get("kind") or "route",
                "state": event.get("state") or "",
                "target_id": event.get("target_id") or event.get("waypoint_id") or event.get("action_id"),
                "note": event.get("note") or "",
            }
            for event in route_events
        ]
        for observation in state.get("observations") or []:
            rows.append(
                {
                    "event_id": f"OBS-{observation.get('id')}",
                    "route_run_id": route_run_id,
                    "ts": observation.get("ts") or 0,
                    "sequence": "",
                    "kind": "observation",
                    "state": "recorded",
                    "target_id": observation.get("entity_id") or observation.get("zone_id"),
                    "note": f"observation {observation.get('id')}",
                }
            )
        for incident in report.get("incidents") or []:
            rows.append(
                {
                    "event_id": f"INC-{incident.get('id')}",
                    "route_run_id": route_run_id,
                    "ts": incident.get("ts_open") or 0,
                    "sequence": "",
                    "kind": "incident",
                    "state": incident.get("state") or "",
                    "target_id": incident.get("entity_id"),
                    "note": incident.get("title") or incident.get("id") or "",
                }
            )
        incidents_by_id = {item.get("id"): item for item in report.get("incidents") or []}
        for work_order in report.get("work_orders") or []:
            incident = incidents_by_id.get(work_order.get("incident_id")) or {}
            rows.append(
                {
                    "event_id": f"WO-{work_order.get('id')}",
                    "route_run_id": route_run_id,
                    "ts": incident.get("ts_closed") or incident.get("ts_open") or 0,
                    "sequence": "",
                    "kind": "work_order",
                    "state": work_order.get("state") or "",
                    "target_id": work_order.get("incident_id"),
                    "note": work_order.get("requested_action") or work_order.get("id") or "",
                }
            )
        for checkpoint in report.get("checkpoint_verifications") or []:
            rows.append(
                {
                    "event_id": f"VER-{checkpoint.get('target_id')}",
                    "route_run_id": route_run_id,
                    "ts": state.get("run", {}).get("ended_at") or state.get("run", {}).get("started_at") or 0,
                    "sequence": "",
                    "kind": "verification",
                    "state": "verified" if checkpoint.get("verified") else "missing",
                    "target_id": checkpoint.get("target_id"),
                    "note": f"expected tag {checkpoint.get('expected_tag_id')}",
                }
            )
        rows.sort(key=lambda item: (float(item.get("ts") or 0), str(item.get("kind") or "")))
        for index, row in enumerate(rows, 1):
            row["sequence"] = row.get("sequence") or index
        store = RouteRunStore(timeline_run_dir)
        store.replace_timeline_events(dogops_run_id, rows, route_run_id=route_run_id)
        return store.timeline_events(
            dogops_run_id=dogops_run_id,
            route_run_id=route_run_id,
        )

    def _route_execution_payload(
        self,
        *,
        route_execution: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        state = self._read_json(self.run_dir / "state.json")
        authoring = self._load_authoring(state)
        return {
            "route_execution": route_execution
            or load_route_execution(self.run_dir).model_dump(mode="json"),
            "authoring": authoring.model_dump(mode="json"),
            "route_run": RouteRunStore(self.run_dir).current_route_run(),
            "live": _LIVE_MAP_ADAPTER.snapshot(),
        }

    def _upsert_incident_location(self, incident_id: str) -> None:
        payload = self._read_body_json()
        payload["incident_id"] = incident_id or payload.get("incident_id")
        try:
            location = EditableIncidentLocation.model_validate(payload)
            authoring = self._persist_authoring_mutation(
                lambda existing: replace_incident_location(existing, location)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _upsert_tag_binding(self) -> None:
        payload = self._read_body_json()
        try:
            binding = EditableTagBinding.model_validate(payload)

            def add_binding(existing: MapAuthoringState) -> MapAuthoringState:
                if any(item.tag_id == binding.tag_id for item in existing.tag_bindings):
                    raise ValueError(f"duplicate tag id: {binding.tag_id}")
                return replace_tag_binding(existing, binding)

            authoring = self._persist_authoring_mutation(add_binding)
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _delete_tag_binding(self, tag_id: str) -> None:
        try:
            parsed_tag_id = int(tag_id)
        except ValueError:
            self._send_json(
                {"ok": False, "error": "invalid_tag_id", "tag_id": tag_id},
                HTTPStatus.BAD_REQUEST,
            )
            return
        authoring = self._persist_authoring_mutation(
            lambda existing: delete_tag_binding(existing, parsed_tag_id)
        )
        self._send_authoring(authoring)

    def _export_map_authoring(self) -> None:
        authoring = self._load_authoring()
        paths = export_authoring_yaml(self.run_dir, authoring)
        self._send_json({"ok": True, "exports": paths})

    def _publish_no_go_shapes(self) -> None:
        try:
            with _AUTHORING_LOCK:
                authoring = publish_no_go_constraints(self._load_authoring())
                self._save_authoring(authoring)
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return
        self._send_authoring(authoring)

    def _mark_work_order_ready(self, work_order_id: str) -> None:
        store = DogOpsStore.load_existing(self.run_dir)
        state = store.state
        assert state is not None
        for work_order in state.work_orders:
            if work_order.id == work_order_id:
                work_order.state = "ready_to_verify"
                store.update_work_order(work_order)
                store.write_state(state.run.id)
                store.write_report(state.run.id)
                write_dashboard_html(self.run_dir)
                self._send_json({"ok": True, "work_order_id": work_order_id, "state": "ready_to_verify"})
                return
        self._send_json(
            {"ok": False, "error": "unknown_work_order", "work_order_id": work_order_id},
            HTTPStatus.NOT_FOUND,
        )

    def _record_operator_event(self) -> None:
        payload = self._read_body_json()
        events_path = self.run_dir / "operator_events.jsonl"
        with events_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")
        self._send_json({"ok": True, "path": str(events_path)})

    def _send_qr_events(self) -> None:
        events = load_qr_events(self.run_dir)
        self._send_json(
            {
                "ok": True,
                "events": events,
                "count": len(events),
                "path": str(qr_events_path(self.run_dir)),
            }
        )

    def _send_latest_qr_events(self, query: dict[str, list[str]]) -> None:
        events = get_latest_qr_events(self.run_dir, limit=_limit_from_query(query))
        self._send_json({"ok": True, "events": events, "count": len(events)})

    def _send_qr_event(self, event_id: str) -> None:
        event = get_qr_event(self.run_dir, event_id)
        if event is None:
            self._send_json(
                {"ok": False, "error": "unknown_qr_event", "event_id": event_id},
                HTTPStatus.NOT_FOUND,
            )
            return
        self._send_json({"ok": True, "event": event})

    def _record_qr_event(self) -> None:
        payload = self._read_body_json()
        try:
            with _QR_EVENTS_LOCK:
                event = append_qr_event(self.run_dir, payload)
                write_dashboard_html(
                    self.run_dir,
                    robot_control_token=self.robot_control_token,
                )
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(
                {"ok": False, "error": "invalid_qr_event", "message": str(exc)},
                HTTPStatus.BAD_REQUEST,
            )
            return
        self._send_json(
            {
                "ok": True,
                "event": event,
                "path": str(qr_events_path(self.run_dir)),
            },
            HTTPStatus.CREATED,
        )

    def _promote_qr_event_to_package(self, event_id: str) -> None:
        self._promote_qr_event_to_authoring(event_id, entity_kind="package")

    def _promote_qr_event_to_label(self, event_id: str) -> None:
        self._promote_qr_event_to_authoring(event_id, entity_kind="checkpoint")

    def _bind_qr_location_node(self, event_id: str) -> None:
        self._promote_qr_event_to_authoring(event_id, entity_kind="checkpoint")

    def _promote_qr_event_to_authoring(self, event_id: str, *, entity_kind: str) -> None:
        event = get_qr_event(self.run_dir, event_id)
        if event is None:
            self._send_json(
                {"ok": False, "error": "unknown_qr_event", "event_id": event_id},
                HTTPStatus.NOT_FOUND,
            )
            return

        payload = event.get("qr_payload") if isinstance(event.get("qr_payload"), dict) else {}
        if entity_kind == "package":
            entity_id = str(payload.get("cargo_id") or event_id)
            label = entity_id
            zone_id = str(payload.get("location_node_id") or payload.get("zone") or "")
        else:
            entity_id = str(payload.get("location_node_id") or event_id)
            label = entity_id
            zone_id = str(payload.get("zone") or "")

        try:
            position = self._qr_authoring_position(event)
            if position is None:
                raise ValueError("QR event has no map position to promote")
            entity = EditableMapEntity.model_validate(
                {
                    "id": entity_id,
                    "kind": entity_kind,
                    "label": label,
                    "pose": {
                        "x": position["x"],
                        "y": position["y"],
                        "theta_deg": None,
                        "source": "qr_cargo_event",
                    },
                    "zone_id": zone_id or None,
                    "source_id": event_id,
                }
            )
            authoring = self._persist_authoring_mutation(
                lambda existing: replace_entity(existing, entity)
            )
        except (ValidationError, ValueError) as exc:
            self._handle_authoring_error(exc)
            return

        self._send_authoring(authoring)

    def _qr_authoring_position(self, event: dict[str, Any]) -> dict[str, Any] | None:
        state = self._read_json(self.run_dir / "state.json")
        report = self._read_json(self.run_dir / "report.json")
        authoring = self._load_authoring(state).model_dump(mode="json")
        map_data = build_map_data(
            state,
            report,
            live_overlay=_LIVE_MAP_ADAPTER.snapshot(),
            authoring=authoring,
            qr_events=[event],
        )
        overlays = map_data.get("qr_cargo_events") or []
        if not overlays:
            return None
        overlay = overlays[0]
        position = overlay.get("map_position")
        return position if isinstance(position, dict) else None

    def _robot_jog(self) -> None:
        payload = self._read_body_json()
        command = str(payload.get("command", "stop"))
        if command not in ROBOT_JOG_COMMANDS:
            self._send_json(
                {"ok": False, "error": "unknown_robot_command", "command": command},
                HTTPStatus.BAD_REQUEST,
            )
            return

        robot_ip = self.robot_ip

        try:
            linear_x, linear_y, angular_z, duration_s, profile = _resolve_motion_request(
                command, payload
            )
            if command in HARD_STOP_COMMANDS:
                motion_result = _run_robot_call(lambda: _publish_robot_hard_stop(robot_ip))
            else:
                motion_result = _run_robot_call(
                    lambda: _publish_robot_jog(linear_x, linear_y, angular_z, duration_s, robot_ip)
                )
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_motion_unavailable",
                    "message": str(exc),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {"ok": False, "error": "robot_command_timeout", "message": str(exc)},
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": "robot_command_failed", "message": str(exc)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        self._send_json(
            {
                "ok": True,
                "command": command,
                "duration_s": 0.0 if command in HARD_STOP_COMMANDS else duration_s,
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z,
                "profile": profile,
                **(motion_result or {}),
            }
        )

    def _robot_posture(self) -> None:
        payload = self._read_body_json()
        command = str(payload.get("command", ""))
        if command not in ROBOT_POSTURE_COMMANDS:
            self._send_json(
                {"ok": False, "error": "unknown_posture_command", "command": command},
                HTTPStatus.BAD_REQUEST,
            )
            return

        robot_ip = self.robot_ip
        try:
            ok = _run_robot_call(lambda: _run_robot_posture(command, robot_ip))
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_motion_unavailable",
                    "message": str(exc),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {"ok": False, "error": "posture_command_timeout", "message": str(exc)},
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": "posture_command_failed", "message": str(exc)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        self._send_json({"ok": bool(ok), "command": command})

    def _robot_go_to(self) -> None:
        payload = self._read_body_json()
        try:
            x, y = _go_to_target(payload)
        except (ValidationError, ValueError) as exc:
            self._send_json(
                {"ok": False, "error": "invalid_go_to_target", "message": str(exc)},
                HTTPStatus.BAD_REQUEST,
            )
            return

        source = str(payload.get("source") or "dashboard")
        try:
            result = _run_robot_call(lambda: _run_robot_go_to(x, y))
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_mcp_unavailable",
                    "message": str(exc),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {"ok": False, "error": "go_to_timeout", "message": str(exc)},
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": "go_to_failed", "message": str(exc)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        self._send_json(
            {
                "ok": True,
                "command": "go_to",
                "x": x,
                "y": y,
                "source": source,
                **(result or {}),
            }
        )

    def _robot_scan_zone(self) -> None:
        payload = self._read_body_json()
        zone_id = str(payload.get("zone_id") or "").strip()
        if not zone_id:
            self._send_json(
                {"ok": False, "error": "missing_zone_id"},
                HTTPStatus.BAD_REQUEST,
            )
            return

        try:
            result = _run_robot_call(lambda: _run_robot_scan_zone(zone_id))
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_mcp_unavailable",
                    "message": str(exc),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {"ok": False, "error": "scan_zone_timeout", "message": str(exc)},
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            if _is_mcp_unavailable_error(exc):
                self._send_json(
                    {
                        "ok": False,
                        "error": "dimos_mcp_unavailable",
                        "message": str(exc),
                    },
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            self._send_json(
                {"ok": False, "error": "scan_zone_failed", "message": str(exc)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        self._send_json({"ok": True, "command": "scan_zone", "zone_id": zone_id, **(result or {})})

    def _robot_map_start(self) -> None:
        robot_ip = self.robot_ip
        try:
            result = _run_robot_call(lambda: _start_robot_map(robot_ip))
        except ModuleNotFoundError as exc:
            self._send_json(
                {
                    "ok": False,
                    "error": "dimos_motion_unavailable",
                    "message": str(exc),
                },
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except TimeoutError as exc:
            self._send_json(
                {"ok": False, "error": "map_start_timeout", "message": str(exc)},
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": "map_start_failed", "message": str(exc)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        self._send_json({"ok": True, "command": "map_start", **result})

    def _robot_map_origin(self) -> None:
        robot_ip = self.robot_ip
        try:
            result = _run_robot_call(lambda: _reset_robot_map_origin(robot_ip))
        except TimeoutError as exc:
            self._send_json(
                {"ok": False, "error": "map_origin_timeout", "message": str(exc)},
                HTTPStatus.GATEWAY_TIMEOUT,
            )
            return
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": "map_origin_failed", "message": str(exc)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        self._send_json({"ok": bool(result), "command": "map_origin"})

    def _authorize_local_read(self) -> bool:
        host = self.headers.get("Host", "")
        if not _is_loopback_host(_host_name(host)):
            self._send_json({"ok": False, "error": "local_read_only"}, HTTPStatus.FORBIDDEN)
            return False
        return True

    def _authorize_robot_control(self) -> bool:
        host = self.headers.get("Host", "")
        if not _is_loopback_host(_host_name(host)):
            self._send_json({"ok": False, "error": "robot_control_local_only"}, HTTPStatus.FORBIDDEN)
            return False

        origin = self.headers.get("Origin")
        if origin and not _origin_matches_host(origin, host):
            self._send_json({"ok": False, "error": "robot_control_bad_origin"}, HTTPStatus.FORBIDDEN)
            return False

        expected = self.robot_control_token
        provided = self.headers.get(ROBOT_CONTROL_TOKEN_HEADER, "")
        if not secrets.compare_digest(provided, expected):
            self._send_json({"ok": False, "error": "robot_control_forbidden"}, HTTPStatus.FORBIDDEN)
            return False

        return True

    def _authorize_map_authoring_write(self) -> bool:
        host = self.headers.get("Host", "")
        if not _is_loopback_host(_host_name(host)):
            self._send_json({"ok": False, "error": "map_authoring_local_only"}, HTTPStatus.FORBIDDEN)
            return False

        origin = self.headers.get("Origin")
        if origin and not _origin_matches_host(origin, host):
            self._send_json({"ok": False, "error": "map_authoring_bad_origin"}, HTTPStatus.FORBIDDEN)
            return False

        expected = self.robot_control_token
        provided = self.headers.get(ROBOT_CONTROL_TOKEN_HEADER, "")
        if not secrets.compare_digest(provided, expected):
            self._send_json({"ok": False, "error": "map_authoring_forbidden"}, HTTPStatus.FORBIDDEN)
            return False

        return True

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.exists():
            self._send_json({"error": "missing_file", "path": str(path)}, HTTPStatus.NOT_FOUND)
            return
        payload = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = json.dumps(payload, indent=2, sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _send_camera_frame(self) -> None:
        try:
            payload = _LIVE_CAMERA_ADAPTER.frame_jpeg()
        except Exception as exc:
            self._send_json(
                {"ok": False, "error": "camera_frame_unavailable", "message": str(exc)},
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        if payload is None:
            self._send_json(
                {"ok": False, "error": "camera_frame_pending"},
                HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _read_json(self, path: Path) -> dict[str, Any]:
        return json.loads(path.read_text(encoding="utf-8"))

    def _read_optional_json(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        return self._read_json(path)

    def _read_body_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))


def _run_robot_call(fn: Any, *, timeout_s: float = ROBOT_CALL_TIMEOUT_S) -> Any:
    result: dict[str, Any] = {}

    def target() -> None:
        try:
            result["value"] = fn()
        except BaseException as exc:
            result["error"] = exc

    thread = threading.Thread(target=target, daemon=True)
    thread.start()
    thread.join(timeout=timeout_s)
    if thread.is_alive():
        raise TimeoutError(f"robot command exceeded {timeout_s:.1f}s")
    if "error" in result:
        raise result["error"]
    return result.get("value")


def _resolve_observation(state: dict[str, Any], payload: dict[str, Any]) -> dict[str, Any]:
    observations = state.get("observations") or []
    observation_id = payload.get("observation_id")
    tag_id = payload.get("tag_id")
    for observation in reversed(observations):
        if observation_id and observation.get("id") == observation_id:
            return observation
        if tag_id is not None and observation.get("tag_id") == int(tag_id):
            return observation
        visible_tags = (observation.get("facts") or {}).get("visible_tag_ids")
        if tag_id is not None and _tag_id_in_visible_tags(int(tag_id), visible_tags):
            return observation
    raise ValueError("matching observation not found")


def _observation_pose(
    state: dict[str, Any],
    observation: dict[str, Any],
) -> tuple[float, float, float | None]:
    pose = observation.get("pose") or {}
    x = _finite_or_none(pose.get("x"))
    y = _finite_or_none(pose.get("y"))
    theta = _finite_or_none(pose.get("theta_deg"))
    if x is not None and y is not None:
        return x, y, theta
    zone_id = str(observation.get("zone_id") or "")
    for zone in (state.get("site") or {}).get("zones") or []:
        if zone.get("id") != zone_id:
            continue
        zone_pose = zone.get("pose_hint") or {}
        x = _finite_or_none(zone_pose.get("x"))
        y = _finite_or_none(zone_pose.get("y"))
        theta = _finite_or_none(zone_pose.get("theta_deg"))
        if x is not None and y is not None:
            return x, y, theta
    raise ValueError("observation has no map pose")


def _authored_or_site_entity(
    state: dict[str, Any],
    authoring: MapAuthoringState,
    entity_id: str,
) -> dict[str, Any] | None:
    for entity in authoring.entities:
        if entity.id == entity_id:
            return entity.model_dump(mode="json")
    site = state.get("site") or {}
    for kind, collection in (
        ("zone", site.get("zones") or []),
        ("asset", site.get("assets") or []),
        ("package", site.get("packages") or []),
    ):
        for item in collection:
            if item.get("id") != entity_id:
                continue
            return {
                "id": entity_id,
                "kind": kind,
                "label": item.get("display_name") or entity_id,
                "tag_id": item.get("tag_id"),
                "zone_id": item.get("zone_id") or item.get("expected_zone_id"),
            }
    return None


def _tag_id_in_visible_tags(tag_id: int, visible_tags: object) -> bool:
    if isinstance(visible_tags, str):
        return str(tag_id) in {item.strip() for item in visible_tags.split(",")}
    if isinstance(visible_tags, list):
        return any(str(item) == str(tag_id) for item in visible_tags)
    return False


def _finite_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _live_robot_pose(snapshot: dict[str, Any]) -> dict[str, float] | None:
    pose = snapshot.get("robot_pose") or snapshot.get("pose")
    if not isinstance(pose, dict):
        return None
    x = _finite_or_none(pose.get("x"))
    y = _finite_or_none(pose.get("y"))
    if x is None or y is None:
        return None
    theta = _finite_or_none(pose.get("theta_deg"))
    return {"x": x, "y": y, "theta_deg": theta or 0.0}


def _resolve_motion_request(
    command: str,
    payload: dict[str, Any],
) -> tuple[float, float, float, float, str]:
    linear_x, linear_y, angular_z = ROBOT_JOG_COMMANDS[command]
    requested_profile = str(payload.get("profile") or DEFAULT_MOTION_PROFILE)
    profile = requested_profile if requested_profile in MOTION_PROFILES else DEFAULT_MOTION_PROFILE
    profile_duration_s, linear_scale, angular_scale = MOTION_PROFILES[profile]

    try:
        duration_s = float(payload.get("duration_s", profile_duration_s))
    except (TypeError, ValueError):
        duration_s = profile_duration_s
    duration_s = max(0.05, min(duration_s, MAX_JOG_DURATION_S))

    linear_x = _cap(linear_x * linear_scale, MAX_LINEAR_SPEED)
    linear_y = _cap(linear_y * linear_scale, MAX_LINEAR_SPEED)
    angular_z = _cap(angular_z * angular_scale, MAX_ANGULAR_SPEED)
    return linear_x, linear_y, angular_z, duration_s, profile


def _cap(value: float, limit: float) -> float:
    return max(-limit, min(value, limit))


def _go_to_target(payload: dict[str, Any]) -> tuple[float, float]:
    try:
        x = float(payload["x"])
        y = float(payload["y"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("go_to requires numeric x and y") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("go_to target must be finite")
    return x, y


def _limit_from_query(query: dict[str, list[str]], *, default: int = 50) -> int:
    raw = (query.get("limit") or [str(default)])[0]
    try:
        limit = int(raw)
    except (TypeError, ValueError):
        return default
    return max(1, min(limit, 500))


def _qr_event_id_from_path(path: str) -> str:
    parts = path.split("/")
    return unquote(parts[4]) if len(parts) >= 6 else ""


def _host_parts(host_header: str) -> tuple[str, int | None]:
    try:
        parsed = urlparse(f"//{host_header.strip()}")
        return (parsed.hostname or "").lower(), parsed.port
    except ValueError:
        return "", None


def _host_name(host_header: str) -> str:
    return _host_parts(host_header)[0]


def _is_loopback_host(hostname: str) -> bool:
    if hostname == "localhost":
        return True
    try:
        return ipaddress.ip_address(hostname).is_loopback
    except ValueError:
        return False


def _origin_matches_host(origin: str, host_header: str) -> bool:
    try:
        parsed_origin = urlparse(origin)
        origin_host = (parsed_origin.hostname or "").lower()
        origin_port = parsed_origin.port
    except ValueError:
        return False

    request_host, request_port = _host_parts(host_header)
    if parsed_origin.scheme not in {"http", "https"}:
        return False
    if not _is_loopback_host(origin_host):
        return False
    return origin_host == request_host and origin_port == request_port


def _publish_robot_jog(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    duration_s: float,
    robot_ip: str,
) -> dict[str, Any]:
    return _publish_wireless_controller_pulse(
        _joystick_payload(linear_x, linear_y, angular_z),
        duration_s,
        robot_ip,
    )


def _publish_robot_hard_stop(robot_ip: str) -> dict[str, Any]:
    return _publish_wireless_controller_pulse({"lx": 0, "ly": 0, "rx": 0, "ry": 0}, 0.15, robot_ip)


def _publish_wireless_controller_pulse(
    payload: dict[str, float | int],
    duration_s: float,
    robot_ip: str,
) -> dict[str, Any]:
    from unitree_webrtc_connect.constants import RTC_TOPIC
    from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

    async def publish() -> None:
        connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=robot_ip)
        await connection.connect()
        try:
            await connection.datachannel.disableTrafficSaving(True)
            deadline = time.monotonic() + max(0.0, duration_s)
            while time.monotonic() < deadline:
                connection.datachannel.pub_sub.publish_without_callback(
                    RTC_TOPIC["WIRELESS_CONTROLLER"],
                    data=payload,
                )
                await asyncio.sleep(0.02)
            for _ in range(HARD_STOP_REPEATS):
                connection.datachannel.pub_sub.publish_without_callback(
                    RTC_TOPIC["WIRELESS_CONTROLLER"],
                    data={"lx": 0, "ly": 0, "rx": 0, "ry": 0},
                )
                await asyncio.sleep(HARD_STOP_INTERVAL_S)
        finally:
            await connection.disconnect()

    asyncio.run(publish())
    return {"observed": False, "transport": "unitree_webrtc_wireless_controller", "ok": True}


def _run_robot_posture(command: str, robot_ip: str) -> bool:
    session = _get_robot_session(robot_ip)
    try:
        return session.posture(command)
    finally:
        if command == "sleep":
            _close_robot_session(robot_ip)


def _run_robot_go_to(x: float, y: float) -> dict[str, Any]:
    return _call_dimos_mcp_skill("go_to", {"x": x, "y": y})


def _run_robot_scan_zone(zone_id: str) -> dict[str, Any]:
    return _call_dimos_mcp_skill("scan_zone", {"zone_id": zone_id})


def _run_route_hard_stop(robot_ip: str) -> dict[str, Any]:
    return _publish_robot_hard_stop(robot_ip)


class _DirectRouteGoalPublisher:
    transport_name = "direct_webrtc"

    def __init__(
        self,
        robot_ip: str,
        live_snapshot_reader: Any,
        *,
        reach_radius_m: float = DIRECT_ROUTE_REACH_RADIUS_M,
        waypoint_timeout_s: float = DIRECT_ROUTE_WAYPOINT_TIMEOUT_S,
    ) -> None:
        self.robot_ip = robot_ip
        self.live_snapshot_reader = live_snapshot_reader
        self.reach_radius_m = reach_radius_m
        self.waypoint_timeout_s = waypoint_timeout_s
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def publish_goal(self, *, x: float, y: float, z: float, frame_id: str) -> dict[str, Any]:
        del z, frame_id
        with self._lock:
            self._stop_event.set()
            previous = self._thread
            if previous is not None and previous.is_alive():
                previous.join(timeout=0.5)
            self._stop_event = threading.Event()
            self._thread = threading.Thread(
                target=self._drive_to_waypoint,
                args=(x, y, self._stop_event),
                daemon=True,
            )
            self._thread.start()
        return {"transport": self.transport_name, "x": x, "y": y}

    def stop(self) -> None:
        with self._lock:
            self._stop_event.set()
            thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        with _ROBOT_SESSIONS_LOCK:
            session = _ROBOT_SESSIONS.get(self.robot_ip)
        if session is not None and not session.closed:
            try:
                session.hard_stop()
            except Exception:
                pass

    def _drive_to_waypoint(self, x: float, y: float, stop_event: threading.Event) -> None:
        session = _get_robot_session(self.robot_ip)
        deadline = time.time() + self.waypoint_timeout_s
        try:
            while not stop_event.is_set() and time.time() < deadline:
                pose = _live_robot_pose(self.live_snapshot_reader())
                if pose is None:
                    time.sleep(0.05)
                    continue
                dx = x - pose["x"]
                dy = y - pose["y"]
                distance = math.hypot(dx, dy)
                if distance <= self.reach_radius_m:
                    session.hard_stop()
                    return
                yaw = math.radians(float(pose.get("theta_deg") or 0.0))
                forward = math.cos(yaw) * dx + math.sin(yaw) * dy
                lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
                linear_x = _clamp(
                    forward * DIRECT_ROUTE_GAIN,
                    -DIRECT_ROUTE_MAX_SPEED,
                    DIRECT_ROUTE_MAX_SPEED,
                )
                linear_y = _clamp(
                    lateral * DIRECT_ROUTE_GAIN,
                    -DIRECT_ROUTE_MAX_SPEED,
                    DIRECT_ROUTE_MAX_SPEED,
                )
                target_heading = math.atan2(dy, dx)
                heading_error = _wrap_angle(target_heading - yaw)
                angular_z = _clamp(
                    heading_error * 0.6,
                    -DIRECT_ROUTE_MAX_ANGULAR_SPEED,
                    DIRECT_ROUTE_MAX_ANGULAR_SPEED,
                )
                session.jog(linear_x, linear_y, angular_z, DIRECT_ROUTE_PULSE_S)
        finally:
            try:
                session.hard_stop()
            except Exception:
                pass


def _run_robot_follow_route(
    route_id: str | None,
    dry_run: bool,
    *,
    use_planner: bool = False,
    robot_ip: str = DEFAULT_ROBOT_IP,
    run_dir: Path | None = None,
) -> dict[str, Any]:
    if dry_run or use_planner:
        return _run_robot_follow_route_with_planner(route_id, dry_run)
    return _run_robot_follow_route_direct(route_id, robot_ip=robot_ip, run_dir=run_dir)


def _run_robot_follow_route_with_planner(route_id: str | None, dry_run: bool) -> dict[str, Any]:
    args: dict[str, Any] = {"dry_run": dry_run}
    if route_id:
        args["route_id"] = route_id
    return _call_dimos_mcp_skill("follow_route", args, timeout_s=DIMOS_ROUTE_CALL_TIMEOUT_S)


def _run_robot_follow_route_direct(
    route_id: str | None,
    *,
    robot_ip: str,
    run_dir: Path | None,
) -> dict[str, Any]:
    route_run_dir = run_dir or Path(".dogops/runs/latest")
    live_adapter = _LIVE_MAP_ADAPTER
    publisher = _DirectRouteGoalPublisher(robot_ip, live_adapter.snapshot)
    executor = DogOpsRouteExecutor(
        route_run_dir,
        goal_publisher=publisher,
        live_snapshot_reader=live_adapter.snapshot,
        stop_handler=lambda: _run_route_hard_stop(robot_ip),
        frame="world",
        reach_radius_m=DIRECT_ROUTE_REACH_RADIUS_M,
        waypoint_timeout_s=DIRECT_ROUTE_WAYPOINT_TIMEOUT_S,
        max_retries=0,
        require_goal_confirmation=False,
    )
    try:
        state = executor.follow_route(route_id, dry_run=False)
    finally:
        publisher.stop()
    return {
        "ok": state.state == "completed",
        "transport": "direct_webrtc",
        "skill": "follow_route",
        "mcp_result": {
            "ok": state.state == "completed",
            "skill": "follow_route",
            "route_id": state.route_id,
            "state": state.state,
            "active_waypoint_id": state.active_waypoint_id,
            "waypoints_total": state.waypoints_total,
            "waypoints_reached": state.waypoints_reached,
            "transport": state.transport,
            "dry_run": False,
            "last_error": state.last_error,
            "route_execution": state.model_dump(mode="json"),
        },
    }


def _run_robot_stop_route() -> dict[str, Any]:
    return _call_dimos_mcp_skill("stop_route", {})


def _run_robot_route_status() -> dict[str, Any]:
    return _call_dimos_mcp_skill("route_status", {})


def _mcp_route_execution(result: Any) -> dict[str, Any] | None:
    if not isinstance(result, dict):
        return None
    mcp_result = result.get("mcp_result")
    if isinstance(mcp_result, dict) and isinstance(mcp_result.get("route_execution"), dict):
        return mcp_result["route_execution"]
    if isinstance(result.get("route_execution"), dict):
        return result["route_execution"]
    return None


def _is_mcp_unavailable_error(exc: Exception) -> bool:
    message = str(exc).lower()
    return any(marker in message for marker in MCP_UNAVAILABLE_MARKERS)


def _call_dimos_mcp_skill(
    skill_name: str,
    args: dict[str, Any],
    *,
    timeout_s: float = DIMOS_MCP_CALL_TIMEOUT_S,
) -> dict[str, Any]:
    command = _dimos_mcp_call_command(skill_name, args)
    try:
        result = subprocess.run(
            command,
            cwd=_dimos_command_cwd(),
            env=_dimos_command_env(),
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout_s,
        )
    except FileNotFoundError as exc:
        raise ModuleNotFoundError(
            f"DimOS MCP command is unavailable: {command[0]}"
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"dimos mcp call {skill_name} timed out after {timeout_s:.1f}s"
        ) from exc

    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    if result.returncode != 0:
        detail = stderr or stdout or "no output"
        raise RuntimeError(f"dimos mcp call {skill_name} failed: {detail}")
    payload: dict[str, Any] = {
        "transport": "dimos_mcp",
        "skill": skill_name,
    }
    if stdout:
        try:
            decoded = json.loads(stdout)
        except json.JSONDecodeError:
            payload["stdout"] = stdout
        else:
            if isinstance(decoded, dict):
                if decoded.get("ok") is False:
                    detail = decoded.get("error") or decoded.get("message") or decoded
                    raise RuntimeError(f"dimos mcp call {skill_name} returned error: {detail}")
                payload["mcp_result"] = decoded
            else:
                payload["mcp_result"] = {"value": decoded}
    return payload


def _dimos_mcp_call_command(skill_name: str, args: dict[str, Any]) -> list[str]:
    encoded_args = json.dumps(args, separators=(",", ":"))
    raw_prefix = os.environ.get("DOGOPS_DIMOS_MCP_CALL")
    if raw_prefix:
        return [*shlex.split(raw_prefix), skill_name, "--json-args", encoded_args]
    if shutil.which("uv") is not None:
        return ["uv", "run", "dimos", "mcp", "call", skill_name, "--json-args", encoded_args]
    return ["dimos", "mcp", "call", skill_name, "--json-args", encoded_args]


def _dimos_command_cwd() -> str | None:
    for name in ("DOGOPS_DIMOS_ROOT", "DIMOS_ROOT"):
        root = os.environ.get(name)
        if root and Path(root).exists():
            return root
    return None


def _dimos_command_env() -> dict[str, str]:
    env = dict(os.environ)
    for key in ("NO_PROXY", "no_proxy"):
        existing = env.get(key, "")
        entries = [item.strip() for item in existing.split(",") if item.strip()]
        for host in ("127.0.0.1", "localhost"):
            if host not in entries:
                entries.append(host)
        env[key] = ",".join(entries)
    return env


def _get_robot_session(robot_ip: str) -> _RobotMotionSession:
    with _ROBOT_SESSIONS_LOCK:
        session = _ROBOT_SESSIONS.get(robot_ip)
        if session is not None and not session.closed:
            return session

    new_session = _RobotMotionSession(robot_ip)
    with _ROBOT_SESSIONS_LOCK:
        session = _ROBOT_SESSIONS.get(robot_ip)
        if session is not None and not session.closed:
            new_session.close()
            return session
        _ROBOT_SESSIONS[robot_ip] = new_session
        return new_session


def _close_robot_session(robot_ip: str) -> None:
    with _ROBOT_SESSIONS_LOCK:
        session = _ROBOT_SESSIONS.pop(robot_ip, None)
    if session is not None:
        session.close()


def _robot_pose_snapshot(robot_ip: str) -> dict[str, Any]:
    with _ROBOT_SESSIONS_LOCK:
        session = _ROBOT_SESSIONS.get(robot_ip)
    if session is None or session.closed:
        return {
            "ok": False,
            "connected": False,
            "source": "unitree_go2_odom",
            "robot_ip": robot_ip,
            "error": "robot_session_not_started",
        }
    return session.pose_snapshot()


def _start_robot_map(robot_ip: str) -> dict[str, Any]:
    return _get_robot_session(robot_ip).pose_snapshot()


def _reset_robot_map_origin(robot_ip: str) -> bool:
    with _ROBOT_SESSIONS_LOCK:
        session = _ROBOT_SESSIONS.get(robot_ip)
    if session is None or session.closed:
        return False
    return session.reset_map_origin()


class _RobotMotionSession:
    def __init__(self, robot_ip: str) -> None:
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

        self.robot_ip = robot_ip
        self.rtc_topic = RTC_TOPIC
        self.sport_cmd = SPORT_CMD
        self.connection = self._make_connection(robot_ip)
        self.lock = threading.RLock()
        self.pose_lock = threading.RLock()
        self.closed = False
        self.mode = "connected"
        self._latest_pose: tuple[float, float, float] | None = None
        self._latest_pose_ts: float | None = None
        self._map_origin_pose: tuple[float, float, float] | None = None
        self._pose_history: deque[tuple[float, float, float, float]] = deque(
            maxlen=ROBOT_POSE_HISTORY_LIMIT
        )
        self._odom_subscription = self.connection.raw_odom_stream().subscribe(self._set_pose)

    def _make_connection(self, robot_ip: str) -> Any:
        from dimos.robot.unitree.connection import UnitreeWebRTCConnection

        return UnitreeWebRTCConnection(robot_ip)

    def posture(self, command: str) -> bool:
        with self.lock:
            if command == "wake":
                ok = bool(self._sport("StandUp"))
                time.sleep(3.0)
                self._sport("BalanceStand")
                self.mode = "balance"
                return ok
            if command == "balance":
                self._sport("BalanceStand")
                self.mode = "balance"
                return True
            if command == "sleep":
                self.hard_stop()
                self._sport("StandDown")
                self.mode = "sleep"
                return True
            raise ValueError(f"unknown posture command: {command}")

    def jog(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_s: float,
    ) -> dict[str, Any]:
        with self.lock:
            restore_obstacles = bool(linear_x or linear_y)
            try:
                self._ensure_motion_ready(disable_obstacles=restore_obstacles)
                before = self._wait_pose()
                self._sport_move(linear_x, linear_y, angular_z)
                time.sleep(duration_s)
                self.hard_stop()
                after = self._wait_pose()
                return _pose_delta(before, after)
            finally:
                if restore_obstacles:
                    self.connection.set_obstacle_avoidance(True)

    def hard_stop(self) -> None:
        with self.lock:
            try:
                self._sport("StopMove")
            except Exception:
                pass
            for _ in range(HARD_STOP_REPEATS):
                self._send_joystick({"lx": 0, "ly": 0, "rx": 0, "ry": 0})
                time.sleep(HARD_STOP_INTERVAL_S)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self._odom_subscription.dispose()
        except Exception:
            pass
        try:
            self.connection.stop()
        except Exception:
            pass

    def pose_snapshot(self) -> dict[str, Any]:
        with self.pose_lock:
            latest = self._latest_pose
            latest_ts = self._latest_pose_ts
            origin = self._map_origin_pose
            history = list(self._pose_history)
        if latest is None or latest_ts is None or origin is None:
            return {
                "ok": False,
                "connected": not self.closed,
                "source": "unitree_go2_odom",
                "robot_ip": self.robot_ip,
                "error": "waiting_for_odom",
            }
        x, y, yaw = _relative_map_pose(latest, origin)
        return {
            "ok": True,
            "connected": not self.closed,
            "source": "unitree_go2_odom",
            "robot_ip": self.robot_ip,
            "ts": latest_ts,
            "pose": {"x": x, "y": y, "yaw_rad": yaw},
            "raw_pose": {"x": latest[0], "y": latest[1], "yaw_rad": latest[2]},
            "origin_raw_pose": {"x": origin[0], "y": origin[1], "yaw_rad": origin[2]},
            "trajectory": [
                {"x": item[0], "y": item[1], "yaw_rad": item[2], "ts": item[3]}
                for item in history
            ],
        }

    def reset_map_origin(self) -> bool:
        with self.pose_lock:
            if self._latest_pose is None:
                return False
            now = self._latest_pose_ts or time.time()
            self._map_origin_pose = self._latest_pose
            x, y, yaw = _relative_map_pose(self._latest_pose, self._map_origin_pose)
            self._pose_history.clear()
            self._pose_history.append((x, y, yaw, now))
            return True

    def _ensure_balance(self) -> None:
        if self.mode != "balance":
            self._sport("BalanceStand")
            self.mode = "balance"

    def _ensure_motion_ready(self, *, disable_obstacles: bool) -> None:
        if disable_obstacles:
            self.connection.set_obstacle_avoidance(False)
        self._ensure_balance()

    def _sport(self, command: str) -> Any:
        return self._request(
            self.rtc_topic["SPORT_MOD"],
            {"api_id": self.sport_cmd[command]},
        )

    def _sport_move(self, linear_x: float, linear_y: float, angular_z: float) -> Any:
        return self._request(
            self.rtc_topic["SPORT_MOD"],
            {
                "api_id": self.sport_cmd["Move"],
                "parameter": {"x": linear_x, "y": linear_y, "z": angular_z},
            },
        )

    def _request(self, topic: str, data: dict[str, Any]) -> Any:
        async def send_request() -> Any:
            return await self.connection.conn.datachannel.pub_sub.publish_request_new(topic, data)

        future = asyncio.run_coroutine_threadsafe(send_request(), self.connection.loop)
        try:
            result = future.result(timeout=WEBRTC_COMMAND_TIMEOUT_S)
        except FutureTimeoutError as exc:
            self.closed = True
            raise TimeoutError(f"WebRTC request timed out after {WEBRTC_COMMAND_TIMEOUT_S:.1f}s") from exc
        status_code = _response_status_code(result)
        if status_code not in {None, 0}:
            raise RuntimeError(f"WebRTC request failed with status code {status_code}")
        return result

    def _send_joystick(self, data: dict[str, float | int]) -> None:
        async def send_joystick() -> None:
            self.connection.conn.datachannel.pub_sub.publish_without_callback(
                self.rtc_topic["WIRELESS_CONTROLLER"],
                data=data,
            )

        future = asyncio.run_coroutine_threadsafe(send_joystick(), self.connection.loop)
        try:
            future.result(timeout=WEBRTC_COMMAND_TIMEOUT_S)
        except FutureTimeoutError as exc:
            self.closed = True
            raise TimeoutError(f"WebRTC joystick timed out after {WEBRTC_COMMAND_TIMEOUT_S:.1f}s") from exc

    def _set_pose(self, msg: Any) -> None:
        pose = _pose_xy_yaw(msg)
        now = time.time()
        with self.pose_lock:
            if self._map_origin_pose is None:
                self._map_origin_pose = pose
            self._latest_pose = pose
            self._latest_pose_ts = now
            x, y, yaw = _relative_map_pose(pose, self._map_origin_pose)
            self._pose_history.append((x, y, yaw, now))

    def _wait_pose(self) -> tuple[float, float, float] | None:
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if self._latest_pose is not None:
                return self._latest_pose
            time.sleep(0.02)
        return None


def _joystick_payload(linear_x: float, linear_y: float, angular_z: float) -> dict[str, float | int]:
    return {
        "lx": -linear_y,
        "ly": linear_x,
        "rx": -angular_z,
        "ry": 0,
    }


def _pose_xy_yaw(msg: Any) -> tuple[float, float, float]:
    pose = msg["data"]["pose"] if isinstance(msg, dict) else msg
    position = pose["position"] if isinstance(pose, dict) else pose.position
    orientation = pose["orientation"] if isinstance(pose, dict) else pose.orientation
    x = float(position["x"] if isinstance(position, dict) else position.x)
    y = float(position["y"] if isinstance(position, dict) else position.y)
    qx = float(orientation["x"] if isinstance(orientation, dict) else orientation.x)
    qy = float(orientation["y"] if isinstance(orientation, dict) else orientation.y)
    qz = float(orientation["z"] if isinstance(orientation, dict) else orientation.z)
    qw = float(orientation["w"] if isinstance(orientation, dict) else orientation.w)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return x, y, yaw


def _relative_map_pose(
    pose: tuple[float, float, float],
    origin: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    heading = -origin[2]
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    x = (dx * cos_h) - (dy * sin_h)
    y = (dx * sin_h) + (dy * cos_h)
    yaw = _wrap_angle(pose[2] - origin[2])
    return x, y, yaw


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _pose_delta(
    before: tuple[float, float, float] | None,
    after: tuple[float, float, float] | None,
) -> dict[str, Any]:
    if before is None or after is None:
        return {"observed": False}
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    dyaw = _wrap_angle(after[2] - before[2])
    return {
        "observed": True,
        "observed_dx_m": dx,
        "observed_dy_m": dy,
        "observed_distance_m": math.hypot(dx, dy),
        "observed_dyaw_rad": dyaw,
    }


def _response_status_code(result: Any) -> int | None:
    if not isinstance(result, dict):
        return None
    data = result.get("data")
    if not isinstance(data, dict):
        return None
    status = data.get("status")
    if not isinstance(status, dict):
        return None
    code = status.get("code")
    return int(code) if code is not None else None
