from __future__ import annotations

from html import unescape
import json
import math
from pathlib import Path
import re
import sys
import threading
import time
from typing import Any
import urllib.request

import pytest

from dimos.experimental.dogops import dashboard, dashboard_static, live_camera
from dimos.experimental.dogops.dashboard import DogOpsDashboardModule, make_dashboard_server
from dimos.experimental.dogops.dashboard_static import (
    build_map_data,
    build_poi_data,
    build_route_data,
    write_dashboard_html,
)
from dimos.experimental.dogops.live_map import (
    DogOpsLiveMapAdapter,
    LIVE_TOPIC_MAX_AGE_S,
    _extend_dimos_package_path,
    _grid_to_costmap,
)
from dimos.experimental.dogops.map_authoring import (
    EditableMapEntity,
    EditableMapPoint,
    EditableRoute,
    EditableRouteWaypoint,
    MapAuthoringState,
    save_map_authoring,
)
from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.route_executor import DogOpsRouteExecutor, save_route_execution
from dimos.experimental.dogops.route_actions import EditableRouteAction
from dimos.experimental.dogops.route_run_store import RouteRunStore


def _get_json(url: str) -> dict[str, object]:
    with urllib.request.urlopen(url, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def _get_json_with_status(
    url: str,
    *,
    headers: dict[str, str] | None = None,
) -> tuple[int, dict[str, object]]:
    request = urllib.request.Request(url, headers=headers or {}, method="GET")
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.status, json.loads(exc.read().decode("utf-8"))


def _post_json(
    url: str,
    payload: dict[str, Any],
    *,
    headers: dict[str, str] | None = None,
) -> tuple[int, dict[str, object]]:
    request_headers = {"Content-Type": "application/json", **(headers or {})}
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=request_headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.status, json.loads(exc.read().decode("utf-8"))


def _put_json(
    url: str,
    payload: dict[str, Any],
    *,
    headers: dict[str, str] | None = None,
) -> tuple[int, dict[str, object]]:
    request_headers = {"Content-Type": "application/json", **(headers or {})}
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=request_headers,
        method="PUT",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.status, json.loads(exc.read().decode("utf-8"))


def _delete_json(
    url: str,
    *,
    headers: dict[str, str] | None = None,
) -> tuple[int, dict[str, object]]:
    request = urllib.request.Request(url, headers=headers or {}, method="DELETE")
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.status, json.loads(exc.read().decode("utf-8"))


def _robot_headers(server) -> dict[str, str]:
    return {
        dashboard.ROBOT_CONTROL_TOKEN_HEADER: server.RequestHandlerClass.robot_control_token,
    }


def test_dashboard_static_html_contains_closed_loop_result(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)

    html_path = write_dashboard_html(run_dir)
    content = html_path.read_text(encoding="utf-8")

    assert "DogOps SiteOps Agent" in content
    assert "Mission Map" in content
    assert "DimOS Camera" in content
    assert "Waiting for /color_image" in content
    assert 'data-camera-frame' in content
    assert "/api/camera/frame.jpg" in content
    assert '<div class="map-stack">' in content
    assert content.index('<div class="map-stack">') < content.index('<div class="ops-stack">')
    assert content.index('<h2>Mission Map</h2>') < content.index('<h2>DimOS Camera</h2>')
    assert content.index('<h2>DimOS Camera</h2>') < content.index('<h2>Run Summary</h2>')
    assert 'data-map-surface' in content
    assert "map-route" in content
    assert "map-free-cell" in content
    assert "map-live-cost-cell" in content
    assert "map-dimos-path" in content
    assert 'data-map-layer="heatmap"' in content
    assert 'data-live-heatmap' in content
    assert 'data-live-path' in content
    assert 'data-live-target' in content
    assert "refreshDimOSMap" in content
    assert 'data-map-action="gather_heatmap"' in content
    assert 'data-map-action="scan_zone"' in content
    assert "/api/map/heatmap/gather" in content
    assert "/api/robot/scan_zone" in content
    assert 'data-map-edit-label-row' in content
    assert 'data-map-edit-route-row' in content
    assert 'data-map-edit-action="dry_run_route"' in content
    assert 'data-map-edit-action="run_route"' in content
    assert 'data-map-edit-action="stop_route"' in content
    assert 'data-map-edit-action="heatmap_run"' in content
    assert 'data-route-action-row' in content
    assert 'data-route-action-kind="capture_image"' in content
    assert 'data-route-action-kind="gemini_inspect_image"' in content
    assert 'data-saved-images' in content
    assert 'data-route-action-kind="scan_qr"' in content
    assert 'data-route-action-kind="scan_tags"' in content
    assert 'data-route-action-kind="wait"' in content
    assert 'data-route-action-kind="inspect_asset"' in content
    assert 'data-route-action-kind="verify_work_order"' in content
    assert 'data-route-action-kind="operator_prompt"' in content
    assert "Saved Routes" in content
    assert 'data-route-table' in content
    assert 'data-route-table-action="select"' in content
    assert 'data-route-table-action="rename"' in content
    assert 'data-route-table-action="duplicate"' in content
    assert 'data-route-table-action="delete"' in content
    assert "route-actions-subrow" in content
    assert "routeActionRows(route)" in content
    assert "handleRouteTableAction" in content
    assert 'class="map-route-stop-marker"' in content
    assert 'circle.setAttribute("r", "18")' in content
    assert 'circle.setAttribute("r", "9")' in content
    assert 'run.route_id === "GATHER_HEATMAP"' in content
    assert "Costmap snapshot" in content
    assert "new Set(Object.keys(routeActionLabels))" in content
    assert "routeActionArgs(kind, waypoint)" in content
    assert 'data-route-execution-status' in content
    assert "/api/map/routes/follow" in content
    assert "/api/map/routes/stop" in content
    assert "/api/map/routes/status" in content
    assert "runSelectedRoute(true)" in content
    assert "runSelectedRoute(false)" in content
    assert "addActionToSelectedRouteWaypoint" in content
    assert "Dry run" in content
    assert "Live" in content
    assert "map-point" in content
    assert "map-robot-core" in content
    assert "free grid" in content
    assert "tag return" in content
    assert "no-go cost" in content
    assert 'data-rerun-surface' in content
    assert 'data-rerun-connect' in content
    assert 'data-rerun-frame' in content
    assert 'data-rerun-url=' in content
    assert 'data-rerun-web-link' in content
    assert "Rerun Web Visualization" in content
    assert "connectRerunSurface" in content
    assert 'data-map-command-status' in content
    assert 'data-map-action="arm_go_to"' in content
    assert 'data-map-edit-mode="home"' in content
    assert 'data-map-edit-mode="no_go"' in content
    assert 'data-map-edit-action="use_observation"' in content
    assert 'data-map-edit-action="delete_selected"' in content
    assert 'data-map-edit-action="route_select"' in content
    assert 'data-map-edit-action="route_up"' in content
    assert 'data-map-edit-action="route_down"' in content
    assert 'data-map-edit-action="publish_no_go"' in content
    assert 'data-map-edit-action="export"' in content
    assert 'data-map-edit-label-row' in content
    assert 'data-map-edit-route-row' in content
    assert 'data-map-route-summary' in content
    assert "Selected route: none. Next: Route1" in content
    assert "AUTHORED_ROUTE" not in content
    assert 'data-map-authoring-status' in content
    assert "/api/map/authoring" in content
    assert "/api/map/entities" in content
    assert "/api/map/no_go_shapes" in content
    assert "/api/map/no_go_shapes/publish" in content
    assert "/api/map/tag_bindings" in content
    assert "/from_observation" in content
    assert "if (!current.selected_route_id) return null" in content
    assert "selected_route_id: route.id" in content
    assert 'data-go-to-marker' in content
    assert "/api/robot/go_to" in content
    assert "worldFromSvgEvent" in content
    assert "map-zone no-go" not in content
    assert "OBS-003" in content
    assert "PKG-104" in content
    assert "INC-001" in content
    assert "Navigation Eval" in content
    assert "Route / POI Evidence" in content
    assert "Route Stops" in content
    assert "POI Evidence" in content
    assert "Robot Control" in content
    assert "Checkpoint Sign-In" not in content
    assert "Mission Timeline" not in content
    assert "What Changed" not in content
    assert "Current Run Timeline" in content
    assert "Current Timeline" not in content
    assert 'class="scan-strip"' not in content
    assert "Tag Sign-In" in content
    assert "OBS-005" in content
    assert 'data-command="forward"' in content
    assert 'data-command="hard_stop"' in content
    assert 'data-command="yaw_left" data-key-hint="Q"' in content
    assert 'data-command="yaw_right" data-key-hint="E"' in content
    assert 'data-key-hint="W / Up"' in content
    assert 'data-key-hint="Space / Esc"' in content
    assert 'data-keyboard-map' in content
    assert '["KeyW", "forward"]' in content
    assert '["KeyQ", "yaw_left"]' in content
    assert '["KeyE", "yaw_right"]' in content
    assert '["ArrowDown", "backward"]' in content
    assert '["Space", "hard_stop"]' in content
    assert '["Escape", "hard_stop"]' in content
    assert "shouldIgnoreKeyboardEvent" in content
    assert 'data-posture="wake"' in content
    assert 'data-posture="sleep"' in content
    assert 'data-motion="nudge"' in content
    assert 'data-motion="step"' in content
    assert 'data-motion="walk"' in content
    assert "X-DogOps-Control-Token" in content


def test_dashboard_static_embeds_full_authoring_state(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            site_id="dogops_demo_site",
            entities=[
                EditableMapEntity(
                    id="CHECKPOINT_X",
                    kind="checkpoint",
                    label="Checkpoint X",
                    pose=EditableMapPoint(x=1.0, y=2.0),
                )
            ],
            routes=[
                EditableRoute(
                    id="ROUTE_X",
                    label="Route X",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP_X",
                            label="Waypoint X",
                            pose=EditableMapPoint(x=3.0, y=4.0),
                        )
                    ],
                )
            ],
        ),
    )

    content = write_dashboard_html(run_dir).read_text(encoding="utf-8")
    match = re.search(r'data-map-authoring="([^"]+)"', content)
    assert match is not None
    authoring = json.loads(unescape(match.group(1)))

    assert authoring["entities"][0]["id"] == "CHECKPOINT_X"
    assert authoring["routes"][0]["waypoints"][0]["id"] == "WP_X"
    assert not isinstance(authoring["entities"], int)
    assert not isinstance(authoring["routes"], int)


def test_dashboard_map_layer_controls_match_svg_layers(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)

    html_path = write_dashboard_html(run_dir)
    content = html_path.read_text(encoding="utf-8")

    controls = set(re.findall(r'data-map-layer="([^"]+)"', content))
    layers = set(re.findall(r'data-layer="([^"]+)"', content))
    assert controls == {"semantic", "heatmap", "path", "robot", "qr"}
    assert controls <= layers
    assert 'querySelectorAll(`[data-layer="${layer}"]`)' in content
    assert 'item.toggleAttribute("hidden", !pressed)' in content
    assert "let dimosRobotPoseActive = false" in content
    assert "if (dimosRobotPoseActive) return" in content
    assert "let liveOverlayBounds = null" in content
    assert "if (data.bounds) liveOverlayBounds = data.bounds" in content
    assert "const projectWorldPoint = (x, y) => projectLivePose({x, y})" in content
    assert "const projectLiveOverlayPoint = (x, y) => projectLiveOverlayPose({x, y})" in content


def test_dashboard_map_controls_are_grouped_near_legend(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)

    html_path = write_dashboard_html(run_dir)
    content = html_path.read_text(encoding="utf-8")

    label_row = re.search(r'<div class="map-edit-row" data-map-edit-label-row>(.*?)</div>', content)
    route_row = re.search(r'<div class="map-edit-row" data-map-edit-route-row>(.*?)</div>', content)
    assert label_row is not None
    assert route_row is not None
    assert 'data-map-edit-mode="zone"' in label_row.group(1)
    assert 'data-map-edit-mode="asset"' in label_row.group(1)
    assert 'data-map-edit-mode="no_go"' in label_row.group(1)
    assert 'data-map-edit-mode="route"' not in label_row.group(1)
    assert 'data-map-edit-mode="route"' in route_row.group(1)
    assert 'data-map-edit-action="route_select"' in route_row.group(1)
    assert 'data-map-edit-action="run_route"' in route_row.group(1)
    assert 'data-map-edit-action="heatmap_run"' in route_row.group(1)
    assert 'data-map-edit-action="route_add_action"' not in route_row.group(1)
    assert 'data-map-edit-action="route_down"' in route_row.group(1)
    assert 'data-map-route-summary' in route_row.group(1)
    assert '<th>Last Run</th>' in content

    svg_end = content.index("</svg>")
    layer_controls = content.index('<div class="map-layer-controls"', svg_end)
    legend = content.index('<div class="map-legend">', layer_controls)
    assert svg_end < layer_controls < legend
    assert '<i class="legend-heatmap"></i>Heatmap' in content
    assert ".map-legend, .map-layer-controls" in content
    assert "const nextRouteId" in content
    assert "new id creates a route" in content
    assert "label: routeId" in content
    assert "routeTable.addEventListener(\"click\"" in content
    assert 'data-route-run-select="' in content
    assert "const overlayPath = Array.isArray(live.path) && live.path.length" in content
    assert "? live.path" in content
    assert ": data.route || []" in content
    assert "mapEditControls.addEventListener(\"click\"" in content
    assert ".map-route-table {" in content
    assert ".route-run-history, .route-run-timeline {" in content
    assert "background: #ffffff;" in content
    assert "color: #111827;" in content


def test_dashboard_saved_routes_table_renders_selected_actions_and_escapes(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id='ROUTE_"A"&',
            routes=[
                EditableRoute(
                    id='ROUTE_"A"&',
                    label='Route <A> "quoted"',
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="Waypoint <One>",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                            actions=[
                                EditableRouteAction(
                                    id="ACT-1",
                                    kind="scan_qr",
                                    label='Scan <QR> "now"',
                                    args={"expected": ['PAYLOAD<1>&"']},
                                )
                            ],
                        )
                    ],
                ),
                EditableRoute(
                    id="ROUTE_B",
                    label="Route B",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-2",
                            label="Waypoint 2",
                            pose=EditableMapPoint(x=2.0, y=3.0),
                        )
                    ],
                ),
            ],
        ),
    )

    html_path = write_dashboard_html(run_dir)
    content = html_path.read_text(encoding="utf-8")

    assert "Saved Routes" in content
    assert "Route &lt;A&gt; &quot;quoted&quot;" in content
    assert 'data-route-id="ROUTE_&quot;A&quot;&amp;"' in content
    assert '<tr class="route-actions-subrow">' in content
    assert "Waypoint &lt;One&gt;" in content
    assert "Scan &lt;QR&gt; &quot;now&quot;" in content
    assert "PAYLOAD&lt;1&gt;&amp;\\&quot;" in content
    assert "<td>1</td><td>1</td>" in content
    assert "Route B" in content


def test_dashboard_rerun_web_url_stays_loopback_only() -> None:
    fallback = "http://127.0.0.1:9877"

    assert dashboard_static._trusted_rerun_web_url(None) == fallback
    assert dashboard_static._trusted_rerun_web_url("http://127.0.0.1:9877") == fallback
    assert (
        dashboard_static._trusted_rerun_web_url("http://localhost:9877/?dataset=dogops")
        == "http://localhost:9877/?dataset=dogops"
    )
    assert (
        dashboard_static._trusted_rerun_web_url("https://[::1]:9877")
        == "https://[::1]:9877"
    )
    assert dashboard_static._trusted_rerun_web_url("https://rerun.example.com") == fallback
    assert dashboard_static._trusted_rerun_web_url("javascript:alert(1)") == fallback


def test_dashboard_module_writes_dashboard_and_reports_status(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    module = DogOpsDashboardModule(run_dir=run_dir, port=18765)

    html_path = module.write_dashboard()
    status = module.status()

    assert html_path.endswith("dashboard.html")
    assert status["exists"] is True
    assert status["port"] == 18765


def test_dashboard_api_serves_state_report_and_nav(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        with urllib.request.urlopen(f"{base_url}/", timeout=5) as response:
            html = response.read().decode("utf-8")
        state = _get_json(f"{base_url}/api/state")
        report = _get_json(f"{base_url}/api/report")
        nav = _get_json(f"{base_url}/api/nav")
        camera_status = _get_json(f"{base_url}/api/camera/status")
        map_data = _get_json(f"{base_url}/api/map")
        authoring = _get_json(f"{base_url}/api/map/authoring")
        route = _get_json(f"{base_url}/api/route")
        poi = _get_json(f"{base_url}/api/poi")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert "DogOps SiteOps Agent" in html
    assert state["run"]["state"] == "done"  # type: ignore[index]
    assert report["manifest_exceptions"] == 2
    assert report["checkpoints_verified"] == 4
    assert report["checkpoint_verifications"][2]["target_id"] == "COOLING_1"  # type: ignore[index]
    assert report["checkpoint_verifications"][2]["expected_tag_id"] == 41  # type: ignore[index]
    assert nav["waypoints_reached"] == 4
    assert camera_status["source"] == "DimOS color_image"
    assert camera_status["topic"] == "/color_image"
    assert "received" in camera_status
    assert [stop["target_id"] for stop in map_data["route"]] == [
        "HOME",
        "INBOUND_DOCK",
        "COOLING_1",
        "QA_HOLD",
    ]
    assert [stop["target_id"] for stop in route["stops"]] == [
        "HOME",
        "INBOUND_DOCK",
        "COOLING_1",
        "QA_HOLD",
    ]
    assert route["stops"][2]["tag_verified"] is True  # type: ignore[index]
    assert any(capture["id"] == "OBS-003" for capture in poi["captures"])  # type: ignore[index]
    assert any(reading["asset_id"] == "TEMP_1" for reading in poi["readings"])  # type: ignore[index]
    assert any(package["id"] == "PKG-104" for package in map_data["packages"])
    assert map_data["live"]["source"] == "DimOS live LCM topics"  # type: ignore[index]
    assert "costmap" in map_data["live"]  # type: ignore[operator]
    assert authoring["schema_version"] == 1
    assert authoring["site_id"] == "dogops_demo_site"


def test_dashboard_camera_status_and_frame_proxy(tmp_path, monkeypatch) -> None:
    jpeg = (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
        b"\xff\xdb\x00C\x00" + (b"\x08" * 64) + b"\xff\xd9"
    )

    class FakeLiveCameraAdapter:
        def status(self) -> dict[str, object]:
            return {
                "ok": True,
                "source": "DimOS color_image",
                "topic": "/color_image",
                "status": "receiving",
                "error": "",
                "received": True,
                "age_s": 0.1,
                "width": 640,
                "height": 360,
                "format": "RGB",
                "frame_id": "camera_front",
            }

        def frame_jpeg(self) -> bytes:
            return jpeg

    monkeypatch.setattr(dashboard, "_LIVE_CAMERA_ADAPTER", FakeLiveCameraAdapter())
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        camera_status = _get_json(f"{base_url}/api/camera/status")
        with urllib.request.urlopen(f"{base_url}/api/camera/frame.jpg", timeout=5) as response:
            frame_content_type = response.headers["Content-Type"]
            frame_payload = response.read()
        forbidden_status, forbidden_result = _get_json_with_status(
            f"{base_url}/api/camera/status",
            headers={"Host": "192.0.2.10"},
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert camera_status["ok"] is True
    assert camera_status["width"] == 640
    assert camera_status["height"] == 360
    assert frame_content_type == "image/jpeg"
    assert frame_payload == jpeg
    assert forbidden_status == 403
    assert forbidden_result["error"] == "local_read_only"


def test_live_camera_adapter_marks_stale_frames_pending() -> None:
    class FakeFrame:
        width = 640
        height = 360
        format = "RGB"
        frame_id = "camera_front"

        def to_base64(self, *, quality: int = 75) -> str:
            assert quality == 75
            return "ZmFrZQ=="

    adapter = live_camera.DogOpsLiveCameraAdapter()
    adapter._started = True
    adapter._latest = (time.time() - live_camera.LIVE_CAMERA_MAX_AGE_S - 0.1, FakeFrame())

    status = adapter.status()

    assert status["ok"] is False
    assert status["received"] is False
    assert status["stale"] is True
    assert status["status"] == "stale_frame"
    assert adapter.frame_jpeg() is None


def test_dashboard_map_authoring_endpoints_persist_and_compose(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, entity_result = _post_json(
            f"{base_url}/api/map/entities",
            {
                "id": "CHECKPOINT_X",
                "kind": "checkpoint",
                "label": "Checkpoint X",
                "pose": {"x": 7.0, "y": 8.0, "source": "dashboard_edit"},
                "tag_id": 222,
            },
            headers=_robot_headers(server),
        )
        status_shape, shape_result = _post_json(
            f"{base_url}/api/map/no_go_shapes",
            {
                "id": "NO_GO_EDIT",
                "label": "Edited No-Go",
                "shape": "rectangle",
                "points": [
                    {"x": 6.0, "y": 6.0, "source": "dashboard_edit"},
                    {"x": 7.0, "y": 7.0, "source": "dashboard_edit"},
                ],
                "enabled": True,
            },
            headers=_robot_headers(server),
        )
        status_route, route_result = _post_json(
            f"{base_url}/api/map/routes",
            {
                "id": "ROUTE_EDIT",
                "label": "Edited Route",
                "waypoints": [
                    {
                        "id": "WP1",
                        "label": "Waypoint 1",
                        "target_id": "CHECKPOINT_X",
                        "pose": {"x": 7.0, "y": 8.0, "source": "dashboard_edit"},
                    }
                ],
            },
            headers=_robot_headers(server),
        )
        map_data = _get_json(f"{base_url}/api/map")
        with urllib.request.urlopen(f"{base_url}/dashboard.html", timeout=5) as response:
            html = response.read().decode("utf-8")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert status_shape == 200
    assert status_route == 200
    assert entity_result["ok"] is True
    assert shape_result["authoring"]["no_go_shapes"][0]["dimos_constraint_status"] == "not_supported"  # type: ignore[index]
    assert route_result["authoring"]["routes"][0]["id"] == "ROUTE_EDIT"  # type: ignore[index]
    assert (run_dir / "map_authoring.json").exists()
    assert any(zone["id"] == "CHECKPOINT_X" for zone in map_data["zones"])
    assert map_data["route"][0]["target_id"] == "CHECKPOINT_X"  # type: ignore[index]
    assert map_data["no_go_shapes"][0]["id"] == "NO_GO_EDIT"  # type: ignore[index]
    match = re.search(r'data-map-authoring="([^"]+)"', html)
    assert match is not None
    embedded_authoring = json.loads(unescape(match.group(1)))
    assert embedded_authoring["entities"][0]["id"] == "CHECKPOINT_X"
    assert embedded_authoring["routes"][0]["id"] == "ROUTE_EDIT"


def test_dashboard_map_authoring_rejects_duplicate_tag_binding(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    payload = {
        "tag_id": 222,
        "entity_id": "CHECKPOINT_X",
        "label": "Checkpoint X",
        "binding_kind": "checkpoint",
    }
    try:
        first_status, first = _post_json(
            f"{base_url}/api/map/tag_bindings",
            payload,
            headers=_robot_headers(server),
        )
        second_status, second = _post_json(
            f"{base_url}/api/map/tag_bindings",
            payload,
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert first_status == 200
    assert first["ok"] is True
    assert second_status == 400
    assert second["ok"] is False
    assert second["error"] == "invalid_map_authoring"


def test_dashboard_map_authoring_write_requires_token(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/map/entities",
            {
                "id": "CHECKPOINT_FORBIDDEN",
                "kind": "checkpoint",
                "label": "Checkpoint Forbidden",
                "pose": {"x": 1.0, "y": 2.0, "source": "dashboard_edit"},
            },
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 403
    assert result["ok"] is False
    assert result["error"] == "map_authoring_forbidden"
    assert not (run_dir / "map_authoring.json").exists()


def test_dashboard_map_authoring_write_rejects_cross_origin(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/map/entities",
            {
                "id": "CHECKPOINT_BAD_ORIGIN",
                "kind": "checkpoint",
                "label": "Checkpoint Bad Origin",
                "pose": {"x": 1.0, "y": 2.0, "source": "dashboard_edit"},
            },
            headers={**_robot_headers(server), "Origin": "https://example.com"},
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 403
    assert result["ok"] is False
    assert result["error"] == "map_authoring_bad_origin"
    assert not (run_dir / "map_authoring.json").exists()


def test_dashboard_map_authoring_delete_and_export(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        _post_json(
            f"{base_url}/api/map/entities",
            {
                "id": "CHECKPOINT_DELETE",
                "kind": "checkpoint",
                "label": "Checkpoint Delete",
                "pose": {"x": 2.0, "y": 3.0, "source": "dashboard_edit"},
            },
            headers=_robot_headers(server),
        )
        delete_status, delete_result = _delete_json(
            f"{base_url}/api/map/entities/CHECKPOINT_DELETE",
            headers=_robot_headers(server),
        )
        export_status, export_result = _post_json(
            f"{base_url}/api/map/export",
            {},
            headers=_robot_headers(server),
        )
        authoring = _get_json(f"{base_url}/api/map/authoring")
        map_data = _get_json(f"{base_url}/api/map")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert delete_status == 200
    assert delete_result["ok"] is True
    assert export_status == 200
    assert export_result["ok"] is True
    assert not any(
        entity["id"] == "CHECKPOINT_DELETE"
        for entity in authoring["entities"]  # type: ignore[index]
    )
    assert not any(zone["id"] == "CHECKPOINT_DELETE" for zone in map_data["zones"])
    site_yaml = (run_dir / "exports" / "site_authoring.yaml")
    assert site_yaml.exists()
    assert "CHECKPOINT_DELETE" not in site_yaml.read_text(encoding="utf-8")


def test_dashboard_map_authoring_observation_placement_and_route_select(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_place, placed = _post_json(
            f"{base_url}/api/map/entities/COOLING_1/from_observation",
            {"observation_id": "OBS-003", "kind": "asset"},
            headers=_robot_headers(server),
        )
        status_route, route_result = _post_json(
            f"{base_url}/api/map/routes",
            {
                "id": "ROUTE_SELECT",
                "label": "Route Select",
                "waypoints": [
                    {
                        "id": "WP_SELECT",
                        "label": "Waypoint Select",
                        "target_id": "COOLING_1",
                        "pose": {"x": 8.0, "y": 9.0, "source": "dashboard_edit"},
                    }
                ],
            },
            headers=_robot_headers(server),
        )
        status_select, selected = _post_json(
            f"{base_url}/api/map/routes/ROUTE_SELECT/select",
            {},
            headers=_robot_headers(server),
        )
        map_data = _get_json(f"{base_url}/api/map")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_place == 200
    assert status_route == 200
    assert status_select == 200
    assert placed["authoring"]["entities"][0]["id"] == "COOLING_1"  # type: ignore[index]
    assert route_result["authoring"]["selected_route_id"] == "ROUTE_SELECT"  # type: ignore[index]
    assert selected["authoring"]["selected_route_id"] == "ROUTE_SELECT"  # type: ignore[index]
    assert map_data["authoring"]["selected_route_id"] == "ROUTE_SELECT"  # type: ignore[index]
    assert map_data["route"][0]["target_id"] == "COOLING_1"  # type: ignore[index]


def test_dashboard_no_go_publish_keeps_unsupported_without_publisher(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        _post_json(
            f"{base_url}/api/map/no_go_shapes",
            {
                "id": "NO_GO_PUBLISH",
                "label": "Publish No-Go",
                "shape": "rectangle",
                "points": [
                    {"x": 1.0, "y": 1.0, "source": "dashboard_edit"},
                    {"x": 2.0, "y": 2.0, "source": "dashboard_edit"},
                ],
                "enabled": True,
            },
            headers=_robot_headers(server),
        )
        status, result = _post_json(
            f"{base_url}/api/map/no_go_shapes/publish",
            {},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["authoring"]["no_go_shapes"][0]["dimos_constraint_status"] == "not_supported"  # type: ignore[index]


def test_dashboard_no_go_publish_persists_published_status(
    tmp_path,
    monkeypatch,
) -> None:
    monkeypatch.setenv("DOGOPS_NO_GO_PUBLISH_COMMAND", "true")
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        _post_json(
            f"{base_url}/api/map/no_go_shapes",
            {
                "id": "NO_GO_PUBLISH",
                "label": "Publish No-Go",
                "shape": "rectangle",
                "points": [
                    {"x": 1.0, "y": 1.0, "source": "dashboard_edit"},
                    {"x": 2.0, "y": 2.0, "source": "dashboard_edit"},
                ],
                "enabled": True,
            },
            headers=_robot_headers(server),
        )
        status, result = _post_json(
            f"{base_url}/api/map/no_go_shapes/publish",
            {},
            headers=_robot_headers(server),
        )
        authoring = _get_json(f"{base_url}/api/map/authoring")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["authoring"]["no_go_shapes"][0]["dimos_constraint_status"] == "published"  # type: ignore[index]
    assert authoring["no_go_shapes"][0]["dimos_constraint_status"] == "published"  # type: ignore[index]


def test_dashboard_map_data_projects_site_route_and_observations(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    state = run_offline_simulation(out=run_dir)
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))

    map_data = build_map_data(state.model_dump(mode="json"), report)

    assert map_data["site_id"] == "dogops_demo_site"
    assert {zone["id"] for zone in map_data["zones"]} >= {"HOME", "INBOUND_DOCK", "QA_HOLD"}
    assert [stop["target_id"] for stop in map_data["route"]] == [
        "HOME",
        "INBOUND_DOCK",
        "COOLING_1",
        "QA_HOLD",
    ]
    assert any(observation["id"] == "OBS-003" for observation in map_data["observations"])
    assert any(incident["id"] == "INC-001" for incident in map_data["incidents"])
    assert map_data["live"]["status"] == "not_requested"


def test_dashboard_map_data_includes_dimos_live_layers(tmp_path, monkeypatch) -> None:
    class FakeLiveMapAdapter:
        def snapshot(self) -> dict[str, object]:
            return {
                "ok": True,
                "source": "DimOS live LCM topics",
                "status": "receiving",
                "error": "",
                "topics": {"global_costmap": {"received": True}},
                "costmap": {
                    "source": "DimOS live costmap",
                    "columns": 1,
                    "rows": 1,
                    "cells": [{"x": 1.0, "y": 2.0, "width": 0.5, "height": 0.5, "cost": 0.9}],
                },
                "path": [{"x": 1.0, "y": 2.0}, {"x": 2.0, "y": 3.0}],
                "route": [{"target_id": "LIVE-PATH-001", "x": 1.0, "y": 2.0}],
                "robot_pose": {"x": 1.2, "y": 2.1, "theta_deg": 45.0, "source": "odom"},
                "target": {"x": 2.0, "y": 3.0, "theta_deg": None, "source": "target"},
            }

    monkeypatch.setattr(dashboard, "_LIVE_MAP_ADAPTER", FakeLiveMapAdapter())
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        map_data = _get_json(f"{base_url}/api/map")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert map_data["live"]["ok"] is True  # type: ignore[index]
    assert map_data["live"]["costmap"]["source"] == "DimOS live costmap"  # type: ignore[index]
    assert map_data["live"]["path"][1]["x"] == 2.0  # type: ignore[index]
    assert map_data["live"]["target"]["source"] == "target"  # type: ignore[index]
    assert map_data["layers"]["heatmap"] is True  # type: ignore[index]
    assert map_data["layers"]["path"] is True  # type: ignore[index]


def test_dashboard_gather_heatmap_persists_snapshot_and_history(tmp_path, monkeypatch) -> None:
    class FakeLiveMapAdapter:
        def snapshot(self) -> dict[str, object]:
            return {
                "ok": True,
                "source": "DimOS live LCM topics",
                "status": "receiving",
                "error": "",
                "topics": {"navigation_costmap": {"received": True}},
                "costmap": {
                    "source": "DimOS live costmap",
                    "columns": 1,
                    "rows": 1,
                    "cells": [{"x": 1.0, "y": 2.0, "width": 0.5, "height": 0.5, "cost": 0.75}],
                },
                "path": [],
                "route": [],
                "robot_pose": {"x": 1.2, "y": 2.1, "theta_deg": 45.0, "source": "odom"},
                "target": None,
            }

    monkeypatch.setattr(dashboard, "_LIVE_MAP_ADAPTER", FakeLiveMapAdapter())
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/map/heatmap/gather",
            {"area_id": "AISLE_1", "duration_s": 0},
            headers=_robot_headers(server),
        )
        map_data = _get_json(f"{base_url}/api/map")
        status_runs, route_runs = _get_json_with_status(
            f"{base_url}/api/route-runs",
            headers=_robot_headers(server),
        )
        status_detail, detail = _get_json_with_status(
            f"{base_url}/api/route-runs/{result['route_run_id']}",
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert status_runs == 200
    assert status_detail == 200
    assert result["ok"] is True
    assert result["run_kind"] == "gather_heatmap"
    assert result["heatmap"]["area_id"] == "AISLE_1"  # type: ignore[index]
    assert (run_dir / "heatmaps" / "latest_heatmap.json").is_file()
    assert (run_dir / "heatmaps" / f"{result['route_run_id']}.json").is_file()
    assert map_data["layers"]["heatmap"] is True  # type: ignore[index]
    assert map_data["gathered_heatmap"]["area_id"] == "AISLE_1"  # type: ignore[index]
    assert map_data["live"]["costmap"]["source"].startswith("Gathered heatmap")  # type: ignore[index]
    assert route_runs["route_runs"][0]["route_id"] == "GATHER_HEATMAP"  # type: ignore[index]
    assert route_runs["route_runs"][0]["transport"] == "dimos_costmap_snapshot"  # type: ignore[index]
    assert detail["map"]["gathered_heatmap"]["route_run_id"] == result["route_run_id"]  # type: ignore[index]
    assert detail["map"]["live"]["costmap"]["cells"][0]["cost"] == 0.75  # type: ignore[index]


def test_dashboard_gather_heatmap_without_costmap_records_failed_history(tmp_path, monkeypatch) -> None:
    class FakeLiveMapAdapter:
        def snapshot(self) -> dict[str, object]:
            return {
                "ok": False,
                "source": "DimOS live LCM topics",
                "status": "waiting",
                "error": "no_costmap",
                "topics": {"navigation_costmap": {"received": False}},
                "costmap": {"source": "DimOS live costmap", "cells": []},
                "path": [],
                "route": [],
                "robot_pose": None,
                "target": None,
            }

    monkeypatch.setattr(dashboard, "_LIVE_MAP_ADAPTER", FakeLiveMapAdapter())
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/map/heatmap/gather",
            {"area_id": "AISLE_1", "duration_s": 0},
            headers=_robot_headers(server),
        )
        map_data = _get_json(f"{base_url}/api/map")
        status_runs, route_runs = _get_json_with_status(
            f"{base_url}/api/route-runs",
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 409
    assert status_runs == 200
    assert result["ok"] is False
    assert result["error"] == "heatmap_unavailable"
    assert not (run_dir / "heatmaps" / "latest_heatmap.json").exists()
    assert map_data["gathered_heatmap"] is None
    assert route_runs["route_runs"][0]["route_id"] == "GATHER_HEATMAP"  # type: ignore[index]
    assert route_runs["route_runs"][0]["state"] == "failed"  # type: ignore[index]
    assert route_runs["route_runs"][0]["last_error"] == result["message"]  # type: ignore[index]


def test_dashboard_route_action_authoring_persists_valid_actions(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"
    route_payload = EditableRoute(
        id="ROUTE_A",
        label="Route A",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-1",
                label="Waypoint 1",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="ACT-CAPTURE",
                        kind="capture_image",
                        label="Take picture",
                        required=True,
                        timeout_s=5.0,
                        args={},
                    ),
                    EditableRouteAction(
                        id="ACT-GEMINI",
                        kind="gemini_inspect_image",
                        label="Gemini inspect",
                        required=True,
                        timeout_s=5.0,
                        args={"target": "WP-1"},
                    ),
                    EditableRouteAction(
                        id="ACT-TAGS",
                        kind="scan_tags",
                        label="Scan AprilTags",
                        required=True,
                        timeout_s=5.0,
                        args={"expected": [101, 102]},
                    ),
                    EditableRouteAction(
                        id="ACT-QR",
                        kind="scan_qr",
                        label="Scan QR",
                        required=True,
                        timeout_s=5.0,
                        args={"expected": ["QR-1"]},
                    ),
                    EditableRouteAction(
                        id="ACT-WAIT",
                        kind="wait",
                        label="Wait",
                        required=True,
                        timeout_s=5.0,
                        args={"seconds": 2.0},
                    ),
                    EditableRouteAction(
                        id="ACT-ASSET",
                        kind="inspect_asset",
                        label="Inspect asset",
                        required=True,
                        timeout_s=5.0,
                        args={"target": "ASSET_1"},
                    ),
                    EditableRouteAction(
                        id="ACT-WO",
                        kind="verify_work_order",
                        label="Verify work order",
                        required=True,
                        timeout_s=5.0,
                        args={"target": "WO-001"},
                    ),
                    EditableRouteAction(
                        id="ACT-PROMPT",
                        kind="operator_prompt",
                        label="Operator prompt",
                        required=True,
                        timeout_s=5.0,
                        args={"target": "WP-1"},
                    ),
                ],
            )
        ],
    ).model_dump(mode="json")

    try:
        status, _ = _post_json(
            f"{base_url}/api/map/routes",
            route_payload,
            headers=_robot_headers(server),
        )
        authoring = _get_json(f"{base_url}/api/map/authoring")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    routes = authoring["routes"]  # type: ignore[index]
    actions = routes[0]["waypoints"][0]["actions"]  # type: ignore[index]
    assert [action["kind"] for action in actions] == [
        "capture_image",
        "gemini_inspect_image",
        "scan_tags",
        "scan_qr",
        "wait",
        "inspect_asset",
        "verify_work_order",
        "operator_prompt",
    ]
    assert actions[0]["required"] is True
    assert actions[1]["args"] == {"target": "WP-1"}
    assert actions[2]["args"] == {"expected": [101, 102]}
    assert actions[3]["args"] == {"expected": ["QR-1"]}
    assert actions[4]["args"] == {"seconds": 2.0}
    assert actions[5]["args"] == {"target": "ASSET_1"}
    assert actions[6]["args"] == {"target": "WO-001"}
    assert actions[7]["args"] == {"target": "WP-1"}


def test_dashboard_map_data_bounds_include_live_overlay(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    state = run_offline_simulation(out=run_dir)
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))

    map_data = build_map_data(
        state.model_dump(mode="json"),
        report,
        live_overlay={
            "ok": True,
            "source": "DimOS live LCM topics",
            "status": "receiving",
            "error": "",
            "topics": {},
            "costmap": {
                "cells": [{"x": 99.0, "y": 101.0, "width": 2.0, "height": 3.0, "cost": 1.0}]
            },
            "path": [{"x": -20.0, "y": -10.0}],
            "route": [],
            "robot_pose": {"x": 120.0, "y": 130.0},
            "target": {"x": -30.0, "y": -40.0},
        },
    )

    assert map_data["bounds"]["x_min"] <= -30.0
    assert map_data["bounds"]["y_min"] <= -40.0
    assert map_data["bounds"]["x_max"] >= 120.0
    assert map_data["bounds"]["y_max"] >= 130.0


def test_live_map_adapter_does_not_assume_local_dimos_checkout(monkeypatch) -> None:
    before = list(sys.path)
    monkeypatch.delenv("DIMOS_ROOT", raising=False)

    _extend_dimos_package_path()

    assert sys.path == before


def test_live_map_adapter_snapshot_converts_recorded_dimos_messages() -> None:
    class Pose:
        def __init__(self, x: float, y: float, yaw: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.yaw = yaw

    class Path:
        poses = [Pose(1.0, 2.0), Pose(3.0, 4.0)]

    class Costmap:
        width = 2
        height = 2
        resolution = 0.5
        origin = Pose(-1.0, -2.0)
        grid: list[list[int]]

    adapter = DogOpsLiveMapAdapter()
    adapter._started = True
    global_costmap = Costmap()
    global_costmap.grid = [[0, 25], [50, 75]]
    navigation_costmap = Costmap()
    navigation_costmap.grid = [[100, 0], [0, 0]]

    adapter._record("global_costmap", global_costmap)
    adapter._record("navigation_costmap", navigation_costmap)
    adapter._record("odom", Pose(0.2, 0.3, math.pi / 2))
    adapter._record("path", Path())
    adapter._record("clicked_point", Pose(5.0, 6.0))

    snapshot = adapter.snapshot()

    assert snapshot["ok"] is True
    assert snapshot["status"] == "receiving"
    assert snapshot["topics"]["global_costmap"]["received"] is True
    assert snapshot["topics"]["navigation_costmap"]["received"] is True
    assert snapshot["topics"]["clicked_point"]["received"] is True
    assert snapshot["costmap"]["cells"][0]["cost"] == 1.0
    assert snapshot["path"] == [
        {"x": 1.0, "y": 2.0, "theta_deg": 0.0, "source": "path"},
        {"x": 3.0, "y": 4.0, "theta_deg": 0.0, "source": "path"},
    ]
    assert snapshot["route"][1]["target_id"] == "LIVE-PATH-002"
    assert snapshot["robot_pose"] == {
        "x": 0.2,
        "y": 0.3,
        "theta_deg": 90.0,
        "source": "odom",
    }
    assert snapshot["target"] == {
        "x": 5.0,
        "y": 6.0,
        "theta_deg": 0.0,
        "source": "target",
    }


def test_live_map_adapter_snapshot_reports_waiting_without_topics() -> None:
    adapter = DogOpsLiveMapAdapter()
    adapter._started = True

    snapshot = adapter.snapshot()

    assert snapshot["ok"] is False
    assert snapshot["status"] == "waiting_for_topics"
    assert snapshot["costmap"] is None
    assert snapshot["path"] == []
    assert snapshot["robot_pose"] is None


def test_live_map_adapter_snapshot_expires_stale_topics() -> None:
    class Pose:
        x = 1.0
        y = 2.0
        yaw = 0.0

    adapter = DogOpsLiveMapAdapter()
    adapter._started = True
    adapter._latest["odom"] = (time.time() - LIVE_TOPIC_MAX_AGE_S - 1.0, Pose())

    snapshot = adapter.snapshot()

    assert snapshot["ok"] is False
    assert snapshot["topics"]["odom"]["received"] is False
    assert snapshot["topics"]["odom"]["stale"] is True
    assert snapshot["robot_pose"] is None


def test_live_costmap_downsampling_stays_within_source_bounds() -> None:
    class Position:
        x = 0.0
        y = 0.0

    class Origin:
        position = Position()

    class Costmap:
        width = 50
        height = 50
        resolution = 1.0
        origin = Origin()

    Costmap.grid = [[0 for _ in range(Costmap.width)] for _ in range(Costmap.height)]
    Costmap.grid[-1][-1] = 100

    costmap = _grid_to_costmap(Costmap(), max_columns=48, max_rows=32)

    assert len(costmap["cells"]) == 48 * 32
    assert all(cell["width"] > 0 for cell in costmap["cells"])  # type: ignore[index]
    assert all(cell["height"] > 0 for cell in costmap["cells"])  # type: ignore[index]
    assert max(cell["x"] + cell["width"] for cell in costmap["cells"]) <= 50  # type: ignore[index]
    assert max(cell["y"] + cell["height"] for cell in costmap["cells"]) <= 50  # type: ignore[index]
    assert max(cell["cost"] for cell in costmap["cells"]) == 1.0  # type: ignore[index]


def test_dashboard_server_close_stops_live_adapter_and_robot_sessions(monkeypatch) -> None:
    class Handler(dashboard.DogOpsDashboardHandler):
        run_dir = Path(".")
        robot_control_token = "test"
        robot_ip = "192.168.12.1"

    class FakeLiveAdapter:
        stopped = False

        def stop(self) -> None:
            self.stopped = True

    class FakeRobotSession:
        closed = False

        def close(self) -> None:
            self.closed = True

    adapter = FakeLiveAdapter()
    session = FakeRobotSession()
    monkeypatch.setitem(dashboard._ROBOT_SESSIONS, "192.168.12.1", session)
    server = dashboard.DogOpsDashboardServer(("127.0.0.1", 0), Handler, live_map_adapter=adapter)

    server.server_close()

    assert adapter.stopped is True
    assert session.closed is True
    assert dashboard._ROBOT_SESSIONS == {}


def test_dashboard_route_and_poi_data_project_evidence(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    state = run_offline_simulation(out=run_dir)
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))

    route = build_route_data(state.model_dump(mode="json"), report)
    poi = build_poi_data(state.model_dump(mode="json"), report)

    assert route["route_coverage"] == 1.0
    assert route["stops"][2]["target_id"] == "COOLING_1"  # type: ignore[index]
    assert route["stops"][2]["expected_tag_id"] == 41  # type: ignore[index]
    assert route["stops"][2]["tag_verified"] is True  # type: ignore[index]
    assert any(capture["id"] == "OBS-003" for capture in poi["captures"])  # type: ignore[index]
    assert any(
        reading["asset_id"] == "COOLING_1" and reading["clearance_clear"] is True
        for reading in poi["readings"]  # type: ignore[index]
    )
    assert any(
        reading["asset_id"] == "TEMP_1" and reading["within_threshold"] is True
        for reading in poi["readings"]  # type: ignore[index]
    )


def test_dashboard_robot_jog_sends_low_speed_bounded_pulse(tmp_path, monkeypatch) -> None:
    calls: list[tuple[float, float, float, float, str]] = []

    def fake_publish(
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_s: float,
        robot_ip: str,
    ) -> None:
        calls.append((linear_x, linear_y, angular_z, duration_s, robot_ip))

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fake_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "forward", "duration_s": 99},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["linear_x"] == 0.15
    assert result["duration_s"] == dashboard.MAX_JOG_DURATION_S
    assert result["profile"] == "nudge"
    assert calls == [(0.15, 0.0, 0.0, dashboard.MAX_JOG_DURATION_S, "192.168.12.1")]


def test_dashboard_robot_jog_applies_motion_profile(tmp_path, monkeypatch) -> None:
    calls: list[tuple[float, float, float, float, str]] = []

    def fake_publish(
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_s: float,
        robot_ip: str,
    ) -> None:
        calls.append((linear_x, linear_y, angular_z, duration_s, robot_ip))

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fake_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "forward", "profile": "walk"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["profile"] == "walk"
    assert result["linear_x"] == pytest.approx(0.6)
    assert result["duration_s"] == 2.0
    assert calls == [(0.6, 0.0, 0.0, 2.0, "192.168.12.1")]


def test_dashboard_robot_jog_ignores_payload_robot_ip(tmp_path, monkeypatch) -> None:
    calls: list[str] = []

    def fake_publish(
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_s: float,
        robot_ip: str,
    ) -> None:
        calls.append(robot_ip)

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fake_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "forward", "robot_ip": "10.0.0.99"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert "robot_ip" not in result
    assert calls == ["192.168.12.1"]


def test_dashboard_robot_control_requires_token(tmp_path, monkeypatch) -> None:
    def fail_publish(*_: object) -> None:
        raise AssertionError("unauthorized robot control must not publish")

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fail_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(f"{base_url}/api/robot/jog", {"command": "forward"})
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 403
    assert result["ok"] is False
    assert result["error"] == "robot_control_forbidden"


def test_dashboard_robot_control_rejects_non_loopback_host(tmp_path, monkeypatch) -> None:
    def fail_publish(*_: object) -> None:
        raise AssertionError("non-local robot control must not publish")

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fail_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "forward"},
            headers={
                **_robot_headers(server),
                "Host": "192.168.1.10:8765",
            },
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 403
    assert result["ok"] is False
    assert result["error"] == "robot_control_local_only"


def test_dashboard_robot_control_rejects_cross_origin(tmp_path, monkeypatch) -> None:
    def fail_publish(*_: object) -> None:
        raise AssertionError("cross-origin robot control must not publish")

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fail_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "forward"},
            headers={
                **_robot_headers(server),
                "Origin": "https://example.com",
            },
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 403
    assert result["ok"] is False
    assert result["error"] == "robot_control_bad_origin"


def test_motion_profile_falls_back_and_caps_speed() -> None:
    linear_x, linear_y, angular_z, duration_s, profile = dashboard._resolve_motion_request(
        "forward",
        {"profile": "too_fast", "duration_s": 99},
    )

    assert profile == "nudge"
    assert linear_x == 0.15
    assert linear_y == 0.0
    assert angular_z == 0.0
    assert duration_s == dashboard.MAX_JOG_DURATION_S


def test_dashboard_robot_hard_stop_uses_hard_stop_publisher(tmp_path, monkeypatch) -> None:
    jog_calls: list[tuple[float, float, float, float, str]] = []
    hard_stop_calls: list[str] = []

    def fake_publish(
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_s: float,
        robot_ip: str,
    ) -> None:
        jog_calls.append((linear_x, linear_y, angular_z, duration_s, robot_ip))

    def fake_hard_stop(robot_ip: str) -> None:
        hard_stop_calls.append(robot_ip)

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fake_publish)
    monkeypatch.setattr(dashboard, "_publish_robot_hard_stop", fake_hard_stop)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "hard_stop"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["duration_s"] == 0.0
    assert jog_calls == []
    assert hard_stop_calls == ["192.168.12.1"]


def test_dashboard_robot_jog_rejects_unknown_command(tmp_path, monkeypatch) -> None:
    def fail_publish(*_: object) -> None:
        raise AssertionError("unknown commands must not publish")

    monkeypatch.setattr(dashboard, "_publish_robot_jog", fail_publish)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/jog",
            {"command": "sprint"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 400
    assert result["ok"] is False
    assert result["error"] == "unknown_robot_command"


def test_dashboard_robot_posture_wake_calls_posture_runner(tmp_path, monkeypatch) -> None:
    calls: list[tuple[str, str]] = []

    def fake_posture(command: str, robot_ip: str) -> bool:
        calls.append((command, robot_ip))
        return True

    monkeypatch.setattr(dashboard, "_run_robot_posture", fake_posture)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/posture",
            {"command": "wake", "robot_ip": "192.168.12.1"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["command"] == "wake"
    assert "robot_ip" not in result
    assert calls == [("wake", "192.168.12.1")]


def test_dashboard_robot_posture_rejects_unknown_command(tmp_path, monkeypatch) -> None:
    def fail_posture(*_: object) -> bool:
        raise AssertionError("unknown posture commands must not run")

    monkeypatch.setattr(dashboard, "_run_robot_posture", fail_posture)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/posture",
            {"command": "dance"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 400
    assert result["ok"] is False
    assert result["error"] == "unknown_posture_command"


def test_dashboard_robot_go_to_calls_dimos_bridge(tmp_path, monkeypatch) -> None:
    calls: list[tuple[float, float]] = []

    def fake_go_to(x: float, y: float) -> dict[str, object]:
        calls.append((x, y))
        return {"transport": "dimos_mcp", "skill": "go_to"}

    monkeypatch.setattr(dashboard, "_run_robot_go_to", fake_go_to)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/go_to",
            {"command": "go_to", "x": 1.25, "y": -0.5, "source": "map_click"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["command"] == "go_to"
    assert result["source"] == "map_click"
    assert result["transport"] == "dimos_mcp"
    assert result["skill"] == "go_to"
    assert calls == [(1.25, -0.5)]


def test_dashboard_robot_scan_zone_calls_dimos_bridge(tmp_path, monkeypatch) -> None:
    calls: list[str] = []

    def fake_scan(zone_id: str) -> dict[str, object]:
        calls.append(zone_id)
        return {
            "transport": "dimos_mcp",
            "skill": "scan_zone",
            "mcp_result": {
                "ok": True,
                "source": "camera",
                "visible_tag_ids": [104],
                "package_ids": ["PKG-104"],
            },
        }

    monkeypatch.setattr(dashboard, "_run_robot_scan_zone", fake_scan)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/scan_zone",
            {"command": "scan_zone", "zone_id": "QA_HOLD"},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert result["ok"] is True
    assert result["command"] == "scan_zone"
    assert result["zone_id"] == "QA_HOLD"
    assert result["transport"] == "dimos_mcp"
    assert result["skill"] == "scan_zone"
    assert result["mcp_result"]["source"] == "camera"  # type: ignore[index]
    assert calls == ["QA_HOLD"]


def test_dashboard_robot_go_to_rejects_bad_target(tmp_path, monkeypatch) -> None:
    def fail_go_to(*_: object) -> dict[str, object]:
        raise AssertionError("bad go_to targets must not run")

    monkeypatch.setattr(dashboard, "_run_robot_go_to", fail_go_to)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/robot/go_to",
            {"command": "go_to", "x": "nan", "y": 0.0},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 400
    assert result["ok"] is False
    assert result["error"] == "invalid_go_to_target"


def test_dashboard_route_follow_stop_and_status_endpoints(tmp_path, monkeypatch) -> None:
    calls: list[tuple[str | None, bool]] = []
    timeouts: list[float] = []

    def fake_follow(route_id: str | None, dry_run: bool) -> dict[str, object]:
        calls.append((route_id, dry_run))
        return {
            "transport": "dimos_mcp",
            "skill": "follow_route",
            "mcp_result": {
                "ok": True,
                "route_id": route_id,
                "state": "completed",
                "route_execution": {"route_id": route_id, "state": "completed"},
            },
        }

    def fake_stop() -> dict[str, object]:
        return {
            "transport": "dimos_mcp",
            "skill": "stop_route",
            "mcp_result": {"ok": True, "state": "stopped", "route_execution": {"state": "stopped"}},
        }

    monkeypatch.setattr(dashboard, "_run_robot_follow_route", fake_follow)
    monkeypatch.setattr(dashboard, "_run_robot_stop_route", fake_stop)
    monkeypatch.setattr(dashboard, "_run_route_hard_stop", lambda robot_ip: {"robot_ip": robot_ip})
    original_run_robot_call = dashboard._run_robot_call

    def tracking_run_robot_call(fn: Any, *, timeout_s: float = dashboard.ROBOT_CALL_TIMEOUT_S) -> object:
        timeouts.append(timeout_s)
        return original_run_robot_call(fn, timeout_s=timeout_s)

    monkeypatch.setattr(dashboard, "_run_robot_call", tracking_run_robot_call)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="Waypoint 1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        )
                    ],
                )
            ],
        ),
    )
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_follow, follow_result = _post_json(
            f"{base_url}/api/map/routes/follow",
            {"route_id": "ROUTE_A", "dry_run": True},
            headers=_robot_headers(server),
        )
        status_status, status_result = _get_json_with_status(
            f"{base_url}/api/map/routes/status",
            headers=_robot_headers(server),
        )
        status_stop, stop_result = _post_json(
            f"{base_url}/api/map/routes/stop",
            {},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_follow == 200
    assert follow_result["ok"] is True
    assert follow_result["command"] == "follow_route"
    assert follow_result["route_execution"]["state"] == "completed"  # type: ignore[index]
    assert follow_result["authoring"]["selected_route_id"] == "ROUTE_A"  # type: ignore[index]
    assert calls == [("ROUTE_A", True)]
    assert timeouts[0] == dashboard.DIMOS_ROUTE_CALL_TIMEOUT_S
    assert status_status == 200
    assert status_result["ok"] is True
    assert status_result["route_execution"]["state"] == "idle"  # type: ignore[index]
    assert status_stop == 200
    assert stop_result["ok"] is True
    assert stop_result["command"] == "stop_route"
    assert stop_result["route_execution"]["state"] == "stopped"  # type: ignore[index]
    assert stop_result["hard_stop"]["ok"] is True  # type: ignore[index]
    assert stop_result["hard_stop"]["robot_ip"] == "192.168.12.1"  # type: ignore[index]


def test_dashboard_route_follow_reports_mcp_unavailable(tmp_path, monkeypatch) -> None:
    def fail_follow(route_id: str | None, dry_run: bool) -> dict[str, object]:
        raise RuntimeError("dimos mcp call follow_route failed: no running MCP server")

    monkeypatch.setattr(dashboard, "_run_robot_follow_route", fail_follow)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="Waypoint 1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        )
                    ],
                )
            ],
        ),
    )
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _post_json(
            f"{base_url}/api/map/routes/follow",
            {"route_id": "ROUTE_A", "dry_run": True},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 503
    assert result["ok"] is False
    assert result["error"] == "dimos_mcp_unavailable"
    assert "no running MCP server" in result["message"]


def test_dashboard_stop_syncs_route_history_when_mcp_stop_fails(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(dashboard, "_run_route_hard_stop", lambda robot_ip: {"robot_ip": robot_ip})
    monkeypatch.setattr(
        dashboard,
        "_run_robot_stop_route",
        lambda: (_ for _ in ()).throw(ModuleNotFoundError("no dimos")),
    )
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="Waypoint 1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        )
                    ],
                )
            ],
        ),
    )
    route_state = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    route_state.state = "running"
    route_state.stop_requested = False
    save_route_execution(run_dir, route_state)
    RouteRunStore(run_dir).sync_execution_state(route_state)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_stop, stop_result = _post_json(
            f"{base_url}/api/map/routes/stop",
            {},
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_stop == 503
    assert stop_result["error"] == "dimos_mcp_unavailable"
    route_run = RouteRunStore(run_dir).route_run_detail(route_state.route_run_id or "")
    assert route_run["state"] == "stopped"
    events = RouteRunStore(run_dir).route_run_events(route_state.route_run_id or "")
    assert events[-1]["state"] == "stopped"


def test_dashboard_route_status_requires_token(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status, result = _get_json_with_status(f"{base_url}/api/map/routes/status")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 403
    assert result["error"] == "map_authoring_forbidden"


def test_dashboard_route_run_history_endpoints(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="Waypoint 1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        )
                    ],
                )
            ],
        ),
    )
    route_state = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_list, route_runs = _get_json_with_status(
            f"{base_url}/api/route-runs",
            headers=_robot_headers(server),
        )
        status_current, current = _get_json_with_status(
            f"{base_url}/api/route-runs/current",
            headers=_robot_headers(server),
        )
        status_events, events = _get_json_with_status(
            f"{base_url}/api/route-runs/{route_state.route_run_id}/events",
            headers=_robot_headers(server),
        )
        status_detail, detail = _get_json_with_status(
            f"{base_url}/api/route-runs/{route_state.route_run_id}",
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_list == 200
    assert route_runs["route_runs"][0]["route_run_id"] == route_state.route_run_id  # type: ignore[index]
    assert status_current == 200
    assert current["route_run"]["route_run_id"] == route_state.route_run_id  # type: ignore[index]
    assert current["events"][0]["state"] == "queued"  # type: ignore[index]
    timeline_kinds = {row["kind"] for row in current["timeline"]}  # type: ignore[index]
    assert {"waypoint", "observation", "incident", "work_order", "verification"} <= timeline_kinds
    assert status_events == 200
    assert events["events"][0]["route_run_id"] == route_state.route_run_id  # type: ignore[index]
    assert status_detail == 200
    assert detail["map"]["route"][0]["target_id"] == "WP-1"  # type: ignore[index]
    assert detail["map"]["route"][0]["x"] == 1.0  # type: ignore[index]


def test_dashboard_route_run_images_api_serves_saved_image_files(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_IMAGE",
            routes=[
                EditableRoute(
                    id="ROUTE_IMAGE",
                    label="Route Image",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-IMAGE",
                            label="Waypoint Image",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                            actions=[
                                EditableRouteAction(
                                    id="CAPTURE",
                                    kind="capture_image",
                                    args={"target": "COOLING_1"},
                                )
                            ],
                        )
                    ],
                )
            ],
        ),
    )
    route_state = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_images, image_list = _get_json_with_status(
            f"{base_url}/api/route-runs/images",
            headers=_robot_headers(server),
        )
        image = image_list["images"][0]  # type: ignore[index]
        request = urllib.request.Request(
            f"{base_url}{image['url']}",  # type: ignore[index]
            headers=_robot_headers(server),
            method="GET",
        )
        with urllib.request.urlopen(request, timeout=5) as response:
            image_status = response.status
            image_type = response.headers["Content-Type"]
            image_bytes = response.read()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert route_state.route_run_id
    assert status_images == 200
    assert image["route_run_id"] == route_state.route_run_id  # type: ignore[index]
    assert image["dogops_run_id"] == "latest"  # type: ignore[index]
    assert image["metadata"]["waypoint_id"] == "WP-IMAGE"  # type: ignore[index]
    assert image_status == 200
    assert image_type == "image/png"
    assert image_bytes.startswith(b"\x89PNG\r\n\x1a\n")


def test_dashboard_route_run_detail_uses_historical_run_dir(tmp_path) -> None:
    first_run_dir = tmp_path / ".dogops" / "runs" / "first"
    second_run_dir = tmp_path / ".dogops" / "runs" / "second"
    for run_dir, route_id in ((first_run_dir, "ROUTE_FIRST"), (second_run_dir, "ROUTE_SECOND")):
        run_offline_simulation(out=run_dir)
        save_map_authoring(
            run_dir,
            MapAuthoringState(
                selected_route_id=route_id,
                routes=[
                    EditableRoute(
                        id=route_id,
                        label=route_id,
                        waypoints=[
                            EditableRouteWaypoint(
                                id=f"{route_id}-WP-1",
                                label="Waypoint 1",
                                pose=EditableMapPoint(x=1.0, y=2.0),
                            )
                        ],
                    )
                ],
            ),
        )
    second_report_path = second_run_dir / "report.json"
    second_report = json.loads(second_report_path.read_text(encoding="utf-8"))
    second_report["incidents"][0]["title"] = "second-run-only incident"
    second_report_path.write_text(json.dumps(second_report), encoding="utf-8")

    DogOpsRouteExecutor(first_run_dir).follow_route(dry_run=True)
    second_route_state = DogOpsRouteExecutor(second_run_dir).follow_route(dry_run=True)
    server = make_dashboard_server(first_run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_detail, detail = _get_json_with_status(
            f"{base_url}/api/route-runs/{second_route_state.route_run_id}",
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_detail == 200
    assert detail["route_run"]["dogops_run_id"] == "second"  # type: ignore[index]
    timeline_notes = {row["note"] for row in detail["timeline"]}  # type: ignore[index]
    assert "second-run-only incident" in timeline_notes


def test_dashboard_route_run_detail_survives_missing_run_files(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="Waypoint 1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        )
                    ],
                )
            ],
        ),
    )
    route_state = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    (run_dir / "state.json").unlink()
    (run_dir / "report.json").unlink()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        status_detail, detail = _get_json_with_status(
            f"{base_url}/api/route-runs/{route_state.route_run_id}",
            headers=_robot_headers(server),
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_detail == 200
    assert detail["route_run"]["route_run_id"] == route_state.route_run_id  # type: ignore[index]
    assert [row["kind"] for row in detail["timeline"]] == ["waypoint"]  # type: ignore[index]


def test_dimos_mcp_call_command_prefers_configured_prefix(monkeypatch) -> None:
    monkeypatch.setenv("DOGOPS_DIMOS_MCP_CALL", "python -m dimos mcp call")

    command = dashboard._dimos_mcp_call_command("go_to", {"x": 1.0, "y": 2.0})

    assert command == [
        "python",
        "-m",
        "dimos",
        "mcp",
        "call",
        "go_to",
        "--json-args",
        '{"x":1.0,"y":2.0}',
    ]


def test_dimos_mcp_call_skill_treats_tool_error_as_failure(monkeypatch) -> None:
    class _Result:
        returncode = 0
        stdout = '{"ok":false,"error":"navigation_stream_unavailable"}'
        stderr = ""

    monkeypatch.setattr(dashboard.subprocess, "run", lambda *_, **__: _Result())
    monkeypatch.setattr(dashboard, "_dimos_mcp_call_command", lambda *_: ["dimos", "mcp"])

    with pytest.raises(RuntimeError, match="navigation_stream_unavailable"):
        dashboard._call_dimos_mcp_skill("go_to", {"x": 1.0, "y": 2.0})


@pytest.mark.parametrize(
    ("command", "linear_x", "linear_y", "angular_z"),
    [
        ("forward", 0.15, 0.0, 0.0),
        ("backward", -0.15, 0.0, 0.0),
        ("left", 0.0, 0.15, 0.0),
        ("right", 0.0, -0.15, 0.0),
        ("yaw_left", 0.0, 0.0, 0.35),
        ("yaw_right", 0.0, 0.0, -0.35),
        ("hard_stop", 0.0, 0.0, 0.0),
        ("stop", 0.0, 0.0, 0.0),
    ],
)
def test_dashboard_robot_jog_command_caps(
    command: str,
    linear_x: float,
    linear_y: float,
    angular_z: float,
) -> None:
    assert dashboard.ROBOT_JOG_COMMANDS[command] == (linear_x, linear_y, angular_z)


def test_robot_motion_session_uses_sport_move_for_linear_jog(monkeypatch) -> None:
    monkeypatch.setattr(dashboard, "HARD_STOP_REPEATS", 1)
    session = object.__new__(dashboard._RobotMotionSession)
    session.lock = threading.RLock()
    session.mode = "connected"
    sport_calls: list[str] = []
    move_calls: list[tuple[float, float, float]] = []
    joystick_calls: list[dict[str, float | int]] = []
    obstacle_calls: list[bool] = []
    session.connection = type(
        "FakeConnection",
        (),
        {"set_obstacle_avoidance": lambda self, enabled: obstacle_calls.append(enabled)},
    )()

    session._sport = sport_calls.append
    session._sport_move = lambda x, y, z: move_calls.append((x, y, z))
    session._send_joystick = lambda data: joystick_calls.append(data)
    session._wait_pose = lambda: (0.0, 0.0, 0.0)

    result = session.jog(0.15, 0.0, 0.0, 0.0)

    assert obstacle_calls == [False, True]
    assert sport_calls == ["BalanceStand", "StopMove"]
    assert session.mode == "balance"
    assert move_calls == [(0.15, 0.0, 0.0)]
    assert joystick_calls == [{"lx": 0, "ly": 0, "rx": 0, "ry": 0}]
    assert result["observed"] is True


def test_robot_motion_session_uses_sport_move_for_yaw_jog(monkeypatch) -> None:
    monkeypatch.setattr(dashboard, "HARD_STOP_REPEATS", 1)
    session = object.__new__(dashboard._RobotMotionSession)
    session.lock = threading.RLock()
    session.mode = "connected"
    sport_calls: list[str] = []
    move_calls: list[tuple[float, float, float]] = []
    joystick_calls: list[dict[str, float | int]] = []

    session._sport = sport_calls.append
    session._sport_move = lambda x, y, z: move_calls.append((x, y, z))
    session._send_joystick = lambda data: joystick_calls.append(data)
    session._wait_pose = lambda: (0.0, 0.0, 0.0)

    session.jog(0.0, 0.0, 0.30, 0.0)

    assert sport_calls == ["BalanceStand", "StopMove"]
    assert session.mode == "balance"
    assert move_calls == [(0.0, 0.0, 0.30)]
    assert joystick_calls == [{"lx": 0, "ly": 0, "rx": 0, "ry": 0}]


def test_robot_motion_session_sport_move_builds_native_go2_request() -> None:
    session = object.__new__(dashboard._RobotMotionSession)
    session.rtc_topic = {"SPORT_MOD": "rt/api/sport/request"}
    session.sport_cmd = {"Move": 1008}
    requests: list[tuple[str, dict[str, object]]] = []
    session._request = lambda topic, data: requests.append((topic, data))

    session._sport_move(0.15, -0.1, 0.3)

    assert requests == [
        (
            "rt/api/sport/request",
            {"api_id": 1008, "parameter": {"x": 0.15, "y": -0.1, "z": 0.3}},
        )
    ]


def test_robot_motion_session_hard_stop_uses_stopmove_and_zero_joystick(monkeypatch) -> None:
    monkeypatch.setattr(dashboard, "HARD_STOP_REPEATS", 1)
    session = object.__new__(dashboard._RobotMotionSession)
    session.lock = threading.RLock()
    sport_calls: list[str] = []
    joystick_calls: list[dict[str, float | int]] = []
    session._sport = sport_calls.append
    session._send_joystick = lambda data: joystick_calls.append(data)

    session.hard_stop()

    assert sport_calls == ["StopMove"]
    assert joystick_calls == [{"lx": 0, "ly": 0, "rx": 0, "ry": 0}]


def test_response_status_code_extracts_go2_sport_response() -> None:
    response = {
        "type": "res",
        "topic": "rt/api/sport/response",
        "data": {"header": {"identity": {"api_id": 1008}}, "status": {"code": 0}},
    }

    assert dashboard._response_status_code(response) == 0
    assert dashboard._response_status_code({"data": {"status": {"code": 3203}}}) == 3203
    assert dashboard._response_status_code({}) is None



def _sample_dashboard_qr_event(location_node_id: str = "COOLING_1") -> dict[str, Any]:
    event = json.loads(
        Path("examples/dogops/qr_cargo_event_sample.json").read_text(encoding="utf-8")
    )
    payload = dict(event["qr_payload"])
    payload["location_node_id"] = location_node_id
    event["qr_payload"] = payload
    event["qr_payload_raw"] = json.dumps(payload, separators=(",", ":"))
    event["robot_pose_at_detection"] = {
        "frame": "map",
        "x": 3.25,
        "y": 0.25,
        "yaw": 0.1,
    }
    return event


def test_dashboard_qr_event_api_persists_and_composes_overlay(
    tmp_path, monkeypatch
) -> None:
    def fail_robot_call(*_: object, **__: object) -> dict[str, object]:
        raise AssertionError("QR event ingestion must not trigger robot control")

    monkeypatch.setattr(dashboard, "_run_robot_go_to", fail_robot_call)
    monkeypatch.setattr(dashboard, "_publish_robot_jog", fail_robot_call)
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    state_before = (run_dir / "state.json").read_text(encoding="utf-8")
    report_before = (run_dir / "report.json").read_text(encoding="utf-8")
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        missing_status, missing_result = _post_json(
            f"{base_url}/api/qr/events",
            _sample_dashboard_qr_event(),
        )
        status, result = _post_json(
            f"{base_url}/api/qr/events",
            _sample_dashboard_qr_event(),
            headers=_robot_headers(server),
        )
        events = _get_json(f"{base_url}/api/qr/events")
        latest = _get_json(f"{base_url}/api/qr/events/latest?limit=1")
        event_id = result["event"]["event_id"]  # type: ignore[index]
        single = _get_json(f"{base_url}/api/qr/events/{event_id}")
        map_data = _get_json(f"{base_url}/api/map")
        html = (run_dir / "dashboard.html").read_text(encoding="utf-8")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert missing_status == 403
    assert missing_result["error"] == "map_authoring_forbidden"
    assert status == 201
    assert result["event"]["action_policy"] == "report_only"  # type: ignore[index]
    assert (run_dir / "qr_events.jsonl").is_file()
    assert events["count"] == 1
    assert latest["events"][0]["event_id"] == event_id  # type: ignore[index]
    assert single["event"]["event_id"] == event_id  # type: ignore[index]
    overlay = map_data["qr_cargo_events"][0]  # type: ignore[index]
    assert overlay["cargo_id"] == "BOX-20260527-018"
    assert overlay["location_node_id"] == "COOLING_1"
    assert overlay["map_position"] == {
        "frame": "map",
        "x": 3.25,
        "y": 0.25,
        "yaw": 0.1,
        "source": "robot_pose_at_detection",
    }
    assert overlay["static_location_node_pose"]["source"] == "site_or_authoring"
    assert overlay["pose_delta"]["distance_m"] > 0
    assert map_data["layers"]["qr"] is True  # type: ignore[index]
    assert "QR Cargo" in html
    assert "BOX-20260527-018" in html
    assert (run_dir / "state.json").read_text(encoding="utf-8") == state_before
    assert (run_dir / "report.json").read_text(encoding="utf-8") == report_before
    assert not (run_dir / "map_authoring.json").exists()


def test_dashboard_qr_promotion_stays_run_local_authoring(tmp_path) -> None:
    run_dir = tmp_path / "latest"
    run_offline_simulation(out=run_dir)
    server = make_dashboard_server(run_dir, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    try:
        _, result = _post_json(
            f"{base_url}/api/qr/events",
            _sample_dashboard_qr_event(location_node_id="WH03-A12-SHELF05"),
            headers=_robot_headers(server),
        )
        event_id = result["event"]["event_id"]  # type: ignore[index]
        status, promoted = _post_json(
            f"{base_url}/api/qr/events/{event_id}/promote_to_package",
            {},
            headers=_robot_headers(server),
        )
        authoring = _get_json(f"{base_url}/api/map/authoring")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 200
    assert promoted["ok"] is True
    entity = authoring["entities"][0]  # type: ignore[index]
    assert entity["id"] == "BOX-20260527-018"
    assert entity["kind"] == "package"
    assert entity["source_id"] == event_id
    assert entity["pose"]["source"] == "qr_cargo_event"
    assert authoring["routes"] == []
