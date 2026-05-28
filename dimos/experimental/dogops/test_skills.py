from __future__ import annotations

import inspect
import json
import time
from typing import Any

import pytest

from dimos.experimental.dogops.map_authoring import (
    EditableMapPoint,
    EditableRoute,
    EditableRouteWaypoint,
    MapAuthoringState,
    save_map_authoring,
)
from dimos.experimental.dogops.route_actions import EditableRouteAction
from dimos.experimental.dogops.route_executor import load_route_execution, save_route_execution
from dimos.experimental.dogops.skills import DogOpsSkillContainer


def _payload(raw: str) -> dict[str, object]:
    return json.loads(raw)


class _ImageLike:
    def __init__(self, image, *, frame_id: str = "camera_optical", encoding: str = "bgr8") -> None:
        self._image = image
        self.frame_id = frame_id
        self.encoding = encoding

    def to_opencv(self):
        return self._image


class _ArrayLike:
    def __init__(self, rows: list[list[list[int]]]) -> None:
        self._rows = rows
        self.shape = (len(rows), len(rows[0]), len(rows[0][0]))

    def __getitem__(self, index: int) -> list[list[int]]:
        return self._rows[index]


class _FakePointPublisher:
    def __init__(self) -> None:
        self.points: list[Any] = []

    def publish(self, point: Any) -> None:
        self.points.append(point)


class _FakeHeatmapAdapter:
    def snapshot(self) -> dict[str, object]:
        return {
            "ok": True,
            "source": "DimOS live LCM topics",
            "status": "receiving",
            "topics": {"navigation_costmap": {"received": True}},
            "costmap": {
                "source": "DimOS live costmap",
                "columns": 1,
                "rows": 1,
                "cells": [{"x": 0.0, "y": 0.0, "width": 0.5, "height": 0.5, "cost": 0.25}],
            },
            "path": [],
            "route": [],
            "robot_pose": None,
            "target": None,
        }


def test_skill_container_runs_closed_loop_and_reports_state(tmp_path) -> None:
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    assert _payload(skills.load_site_config())["packages"] == 4
    assert _payload(skills.load_manifest())["packages"] == 4
    assert _payload(skills.load_mission())["mission_id"] == "receiving_sre_demo"

    run = _payload(skills.run_mission())
    assert run["ok"] is True
    assert run["state"] == "done"

    scan = _payload(skills.scan_zone("INBOUND_DOCK"))
    assert scan["visible_tag_ids"] == [20, 101, 102]

    manifest_scan = _payload(skills.scan_receiving_manifest("INBOUND_DOCK"))
    assert manifest_scan["expected_package_ids"] == ["PKG-101", "PKG-102", "PKG-103"]
    assert manifest_scan["observed_package_ids"] == ["PKG-101", "PKG-102"]
    assert manifest_scan["missing_package_ids"] == ["PKG-103"]
    assert manifest_scan["manifest_exceptions"] == 1

    asset = _payload(skills.inspect_asset("COOLING_1"))
    assert asset["ok"] is True
    assert asset["expected_clear"] is True

    clearance = _payload(skills.check_clearance("COOLING_1"))
    assert clearance["clearance_clear"] is True
    assert clearance["evidence_observation_id"] == "OBS-004"

    gauge = _payload(skills.read_gauge("TEMP_1"))
    assert gauge["within_threshold"] is True
    assert gauge["reading_celsius"] == 28.0

    aisle = _payload(skills.detect_blocked_aisle("AISLE_1"))
    assert aisle["blocked"] is False
    assert aisle["clearance_clear"] is True

    reconciliation = _payload(skills.reconcile_manifest())
    assert reconciliation["manifest_exceptions"] == 2

    changes = _payload(skills.what_changed())
    assert "PKG-104 moved" in str(changes["changes"])

    nav = _payload(skills.nav_eval_report())
    assert nav["nav_summary"]["waypoints_reached"] == 4  # type: ignore[index]


def test_skill_container_gather_heatmap_records_costmap_run(tmp_path) -> None:
    skills = DogOpsSkillContainer(
        run_dir=tmp_path / ".dogops" / "runs" / "latest",
        live_map_adapter=_FakeHeatmapAdapter(),  # type: ignore[arg-type]
    )

    result = _payload(skills.gather_heatmap(area_id="AISLE_1", duration_s=0.0))

    assert result["ok"] is True
    assert result["skill"] == "gather_heatmap"
    assert result["run_kind"] == "gather_heatmap"
    assert result["heatmap"]["area_id"] == "AISLE_1"  # type: ignore[index]
    assert (tmp_path / ".dogops" / "runs" / "latest" / "heatmaps" / "latest_heatmap.json").is_file()


def test_skill_container_go_to_publishes_clicked_point(tmp_path) -> None:
    publisher = _FakePointPublisher()
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")
    skills.clicked_point = publisher  # type: ignore[attr-defined]

    result = _payload(skills.go_to(1.25, -0.5))

    assert result["ok"] is True
    assert result["skill"] == "go_to"
    assert result["transport"] == "clicked_point"
    assert result["x"] == 1.25
    assert result["y"] == -0.5
    assert result["z"] == 0.0
    assert result["frame_id"] == "map"
    assert len(publisher.points) == 1
    point = publisher.points[0]
    assert point.x == 1.25
    assert point.y == -0.5
    assert point.z == 0.0
    assert point.frame_id == "map"


def test_skill_container_go_to_rejects_invalid_target(tmp_path) -> None:
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.go_to(float("nan"), 0.0))

    assert result["ok"] is False
    assert result["skill"] == "go_to"
    assert result["error"] == "invalid_go_to_target"


def test_skill_container_go_to_reports_missing_navigation_stream(tmp_path) -> None:
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.go_to(1.0, 2.0))

    assert result["ok"] is False
    assert result["skill"] == "go_to"
    assert result["error"] == "navigation_stream_unavailable"


def test_skill_container_reports_missing_camera_frame(tmp_path) -> None:
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.camera_stream_status())

    assert result["ok"] is False
    assert result["mode"] == "not_subscribed"


def test_skill_container_scan_zone_uses_latest_camera_frame(tmp_path) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV aruco is unavailable")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, 104, 240)
    else:
        marker = aruco.drawMarker(dictionary, 104, 240)
    canvas = np.full((320, 320), 255, dtype=marker.dtype)
    canvas[40:280, 40:280] = marker
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")
    skills.ingest_camera_image(_ImageLike(canvas))

    status = _payload(skills.camera_stream_status())
    result = _payload(skills.scan_zone("QA_HOLD"))

    assert status["ok"] is True
    assert result["ok"] is True
    assert result["source"] == "camera"
    assert result["visible_tag_ids"] == [104]
    assert result["package_ids"] == ["PKG-104"]
    assert result["evidence_observation_ids"] == []


def test_skill_container_capture_image_uses_latest_camera_frame(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_CAMERA_CAPTURE",
            routes=[
                EditableRoute(
                    id="ROUTE_CAMERA_CAPTURE",
                    label="Route Camera Capture",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-CAMERA",
                            label="Camera",
                            target_id="COOLING_1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                            actions=[
                                EditableRouteAction(
                                    id="CAPTURE",
                                    kind="capture_image",
                                    args={"target": "COOLING_1"},
                                )
                            ],
                        ),
                    ],
                )
            ],
        ),
    )
    frame = _ArrayLike(
        [
            [[0, 0, 255], [0, 255, 0]],
            [[255, 0, 0], [255, 255, 255]],
        ]
    )
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")
    skills.ingest_camera_image(_ImageLike(frame, frame_id="front-camera"))

    result = _payload(skills.follow_route(dry_run=True))

    assert result["ok"] is True
    events = result["route_execution"]["events"]  # type: ignore[index]
    completed = [event for event in events if event.get("action_id") == "CAPTURE"][-1]
    assert completed["payload"]["source"] == "go2_camera_live"
    evidence = completed["payload"]["evidence"][0]
    assert evidence["metadata"]["source"] == "go2_camera_live"
    assert evidence["metadata"]["camera_frame_id"] == "front-camera"
    image_path = tmp_path / "latest" / "route_runs" / result["route_execution"]["route_run_id"] / "evidence" / "WP-CAMERA-CAPTURE.png"  # type: ignore[index]
    assert image_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_skill_container_capture_image_fails_without_live_camera_frame(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_CAMERA_CAPTURE_MISSING",
            routes=[
                EditableRoute(
                    id="ROUTE_CAMERA_CAPTURE_MISSING",
                    label="Route Camera Capture Missing",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-CAMERA",
                            label="Camera",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                            actions=[
                                EditableRouteAction(
                                    id="CAPTURE",
                                    kind="capture_image",
                                )
                            ],
                        ),
                    ],
                )
            ],
        ),
    )
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.follow_route(dry_run=True))

    assert result["ok"] is False
    assert result["state"] == "failed"
    assert "no live Go2 camera frame is available" in str(result["last_error"])


def test_skill_container_capture_image_rejects_stale_camera_frame(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_CAMERA_CAPTURE_STALE",
            routes=[
                EditableRoute(
                    id="ROUTE_CAMERA_CAPTURE_STALE",
                    label="Route Camera Capture Stale",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-CAMERA",
                            label="Camera",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                            actions=[
                                EditableRouteAction(
                                    id="CAPTURE",
                                    kind="capture_image",
                                    args={"max_camera_frame_age_s": 0.5},
                                )
                            ],
                        ),
                    ],
                )
            ],
        ),
    )
    frame = _ArrayLike([[[0, 0, 255]]])
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")
    skills.ingest_camera_image(_ImageLike(frame, frame_id="front-camera"))
    skills._latest_camera_received_at = time.time() - 2.0

    result = _payload(skills.follow_route(dry_run=True))

    assert result["ok"] is False
    assert result["state"] == "failed"
    assert "latest Go2 camera frame is stale" in str(result["last_error"])


def test_skill_container_route_skills_validate_and_report_dry_run(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="One",
                            target_id="CHECKPOINT_1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        ),
                    ],
                )
            ],
        ),
    )
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.follow_route(dry_run=True))
    status = _payload(skills.route_status())

    assert result["ok"] is True
    assert result["skill"] == "follow_route"
    assert result["route_id"] == "ROUTE_A"
    assert result["state"] == "completed"
    assert result["transport"] == "dry_run"
    assert status["state"] == "completed"


def test_skill_container_qr_route_action_uses_scan_zone_flow(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_SCAN_QR",
            routes=[
                EditableRoute(
                    id="ROUTE_SCAN_QR",
                    label="Route Scan QR",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-INBOUND",
                            label="Inbound",
                            target_id="INBOUND_DOCK",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                            actions=[
                                EditableRouteAction(
                                    id="SCAN-QR",
                                    kind="scan_qr",
                                    args={},
                                )
                            ],
                        ),
                    ],
                )
            ],
        ),
    )
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.follow_route(dry_run=True))

    assert result["ok"] is True
    events = result["route_execution"]["events"]  # type: ignore[index]
    completed = [event for event in events if event.get("action_id") == "SCAN-QR"][-1]
    assert completed["state"] == "completed"
    assert completed["payload"]["source"] == "scan_zone"
    assert completed["payload"]["scan_zone_source"] == "simulation"
    assert completed["payload"]["detected_payloads"] == ["PKG-101", "PKG-102"]


def test_skill_container_follow_route_requires_navigation_stream_for_live_run(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="One",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        ),
                    ],
                )
            ],
        ),
    )
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    result = _payload(skills.follow_route(dry_run=False))

    assert result["ok"] is False
    assert result["skill"] == "follow_route"
    assert result["error"] == "navigation_stream_unavailable"


def test_skill_container_stop_route_marks_execution_stopped(tmp_path) -> None:
    save_map_authoring(
        tmp_path / "latest",
        MapAuthoringState(
            selected_route_id="ROUTE_A",
            routes=[
                EditableRoute(
                    id="ROUTE_A",
                    label="Route A",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-1",
                            label="One",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        ),
                    ],
                )
            ],
        ),
    )
    stop_calls: list[str] = []
    skills = DogOpsSkillContainer(
        run_dir=tmp_path / "latest",
        route_stop_handler=lambda: stop_calls.append("stop"),
    )
    _payload(skills.follow_route(dry_run=True))
    state = load_route_execution(tmp_path / "latest")
    state.state = "running"
    save_route_execution(tmp_path / "latest", state)

    result = _payload(skills.stop_route())

    assert result["ok"] is True
    assert result["state"] == "stopped"
    assert result["route_execution"]["stop_requested"] is True  # type: ignore[index]
    assert stop_calls == ["stop"]


def test_skill_container_work_order_methods_are_idempotent(tmp_path) -> None:
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")
    skills.run_mission()

    existing = _payload(skills.open_work_order("COOLING_1", "blocked_cooling"))
    assert existing["incident_id"] == "INC-001"
    assert existing["work_order_id"] == "WO-001"

    ready = _payload(skills.mark_ready_to_verify("WO-001"))
    assert ready["ok"] is True

    verified = _payload(skills.verify_work_order("WO-001"))
    assert verified["state"] == "verified_closed"


def test_skill_container_stretch_skills_are_simulated_without_cloud_keys(tmp_path) -> None:
    skills = DogOpsSkillContainer(run_dir=tmp_path / "latest")

    dock = _payload(skills.dock_align())
    assert dock["ok"] is True
    assert dock["simulated"] is True

    portal = _payload(skills.portal_entry())
    assert portal["ok"] is True
    assert portal["door_open"] is True

    stopped = _payload(skills.stop_mission())
    assert stopped["state"] == "not_started"


def test_skill_container_mcp_skills_have_docstrings() -> None:
    skill_methods = [
        method
        for _, method in inspect.getmembers(DogOpsSkillContainer, predicate=callable)
        if getattr(method, "__dogops_skill__", False) or getattr(method, "__skill__", False)
    ]

    assert skill_methods
    assert all(inspect.getdoc(method) for method in skill_methods)
