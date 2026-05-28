from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.experimental.dogops.map_authoring import (
    EditableMapPoint,
    EditableRoute,
    EditableRouteWaypoint,
    MapAuthoringState,
    save_map_authoring,
)
from dimos.experimental.dogops.gemini_vision import GeminiImageInspection, GeminiVisionResult
from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.route_actions import EditableRouteAction
from dimos.experimental.dogops.route_executor import (
    CallableGoalPublisher,
    DogOpsRouteExecutor,
    RouteExecutionError,
    load_route_execution,
    request_route_stop,
    route_feedback_from_snapshot,
    route_execution_lock,
    save_route_execution,
)
from dimos.experimental.dogops.route_run_store import RouteRunStore


def _route(route_id: str = "ROUTE_A") -> EditableRoute:
    return EditableRoute(
        id=route_id,
        label="Route A",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-1",
                label="One",
                target_id="CHECKPOINT_1",
                pose=EditableMapPoint(x=1.0, y=2.0),
            ),
            EditableRouteWaypoint(
                id="WP-2",
                label="Two",
                target_id="CHECKPOINT_2",
                pose=EditableMapPoint(x=2.0, y=2.5),
            ),
        ],
    )


def _save_authoring(tmp_path, *, selected_route_id: str | None = "ROUTE_A") -> None:
    save_map_authoring(
        tmp_path,
        MapAuthoringState(
            selected_route_id=selected_route_id,
            routes=[_route("ROUTE_A"), _route("ROUTE_B")],
        ),
    )


def test_dry_run_resolves_selected_route_without_publishing(tmp_path) -> None:
    _save_authoring(tmp_path, selected_route_id="ROUTE_A")
    published: list[tuple[float, float]] = []
    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(
            lambda x, y, z, frame: published.append((x, y))
        ),
    )

    state = executor.follow_route(dry_run=True)

    assert state.state == "completed"
    assert state.route_id == "ROUTE_A"
    assert state.transport == "dry_run"
    assert [event.waypoint_id for event in state.events] == ["WP-1", "WP-2"]
    assert [event.state for event in state.events] == ["queued", "queued"]
    assert published == []
    assert load_route_execution(tmp_path).route_id == "ROUTE_A"


def test_explicit_route_id_overrides_selected_route(tmp_path) -> None:
    _save_authoring(tmp_path, selected_route_id="ROUTE_A")

    state = DogOpsRouteExecutor(tmp_path).follow_route("ROUTE_B", dry_run=True)

    assert state.route_id == "ROUTE_B"


def test_missing_and_empty_routes_are_rejected(tmp_path) -> None:
    save_map_authoring(tmp_path, MapAuthoringState(routes=[]))
    executor = DogOpsRouteExecutor(tmp_path)

    with pytest.raises(RouteExecutionError, match="no route_id"):
        executor.follow_route(dry_run=True)

    save_map_authoring(
        tmp_path,
        MapAuthoringState(
            selected_route_id="EMPTY",
            routes=[EditableRoute(id="EMPTY", label="Empty")],
        ),
    )
    with pytest.raises(RouteExecutionError, match="no waypoints"):
        executor.follow_route(dry_run=True)

    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id=None, routes=[_route("ROUTE_A")]),
    )
    with pytest.raises(RouteExecutionError, match="no route_id"):
        executor.follow_route(dry_run=True)


def test_fake_publisher_receives_goals_in_order_and_waits_for_odom(tmp_path) -> None:
    _save_authoring(tmp_path)
    published: list[tuple[float, float, str]] = []
    current_goal = {"x": 0.0, "y": 0.0}

    def publish(x: float, y: float, z: float, frame: str) -> dict[str, object]:
        published.append((x, y, frame))
        current_goal.update({"x": x, "y": y})
        return {"accepted": True}

    def snapshot() -> dict[str, object]:
        return {
            "robot_pose": {"x": current_goal["x"], "y": current_goal["y"]},
            "target": {"x": current_goal["x"], "y": current_goal["y"]},
            "topics": {"odom": {"age_s": 0.1}},
        }

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(publish, transport_name="fake_nav"),
        live_snapshot_reader=snapshot,
        waypoint_timeout_s=0.1,
        sleep_fn=lambda _: None,
    )

    state = executor.follow_route()

    assert state.state == "completed"
    assert state.waypoints_reached == 2
    assert published == [(1.0, 2.0, "world"), (2.0, 2.5, "world")]
    assert [event.state for event in state.events] == [
        "queued",
        "sent",
        "reached",
        "queued",
        "sent",
        "reached",
    ]


def test_route_actions_record_timeline_and_placeholder_evidence(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_ACTIONS",
        label="Route Actions",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-ACTION",
                label="Waypoint",
                target_id="COOLING_1",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="SCAN-TAGS",
                        kind="scan_tags",
                        args={"expected": [41]},
                    ),
                    EditableRouteAction(
                        id="CAPTURE",
                        kind="capture_image",
                        args={"target": "COOLING_1"},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_ACTIONS", routes=[route]),
    )
    current_goal = {"x": 0.0, "y": 0.0}

    def publish(x: float, y: float, z: float, frame: str) -> dict[str, object]:
        current_goal.update({"x": x, "y": y})
        return {"accepted": True}

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(publish, transport_name="fake_nav"),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": current_goal["x"], "y": current_goal["y"]},
            "target": {"x": current_goal["x"], "y": current_goal["y"]},
            "topics": {"odom": {"age_s": 0.1}},
        },
        sleep_fn=lambda _: None,
    )

    state = executor.follow_route()

    assert state.state == "completed"
    assert state.route_run_id
    action_events = [event for event in state.events if event.kind == "action"]
    assert [event.action_id for event in action_events] == [
        "SCAN-TAGS",
        "SCAN-TAGS",
        "CAPTURE",
        "CAPTURE",
    ]
    assert action_events[-1].payload["source"] == "demo_placeholder"
    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id)
    evidence_kinds = {item["kind"] for item in evidence}
    assert {"tag_detection", "image"} <= evidence_kinds
    image_evidence = [item for item in evidence if item["kind"] == "image"][0]
    assert image_evidence["metadata"]["source"] == "demo_placeholder"
    assert image_evidence["mime_type"] == "image/png"
    assert (tmp_path / "route_runs" / state.route_run_id / "evidence" / "WP-ACTION-CAPTURE.png").exists()


def test_capture_image_uses_configured_go2_image(tmp_path) -> None:
    source_image = tmp_path / "go2-frame.jpg"
    source_image.write_bytes(b"fake-jpeg")
    route = EditableRoute(
        id="ROUTE_IMAGE",
        label="Route Image",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-IMAGE",
                label="Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="CAPTURE",
                        kind="capture_image",
                        args={"image_path": str(source_image)},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_IMAGE", routes=[route]),
    )
    current_goal = {"x": 0.0, "y": 0.0}

    def publish(x: float, y: float, z: float, frame: str) -> dict[str, object]:
        current_goal.update({"x": x, "y": y})
        return {"accepted": True}

    state = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(publish, transport_name="fake_nav"),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": current_goal["x"], "y": current_goal["y"]},
            "target": {"x": current_goal["x"], "y": current_goal["y"]},
            "topics": {"odom": {"age_s": 0.1}},
        },
        sleep_fn=lambda _: None,
    ).follow_route()

    assert state.route_run_id
    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id)
    image_evidence = [item for item in evidence if item["kind"] == "image"][0]
    assert image_evidence["metadata"]["source"] == "go2_camera_configured"
    copied_path = tmp_path / "route_runs" / state.route_run_id / "evidence" / "WP-IMAGE-CAPTURE.jpg"
    assert copied_path.read_bytes() == b"fake-jpeg"


def test_capture_image_uses_live_camera_handler_when_available(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_LIVE_IMAGE",
        label="Route Live Image",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-LIVE-IMAGE",
                label="Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="CAPTURE",
                        kind="capture_image",
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_LIVE_IMAGE", routes=[route]),
    )

    def capture_image(context: dict[str, object]) -> dict[str, object]:
        evidence_dir = context["evidence_dir"]
        assert isinstance(evidence_dir, Path)
        path = evidence_dir / "live-camera.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"live-camera")
        return {
            "path": str(path),
            "source": "go2_camera_live",
            "mime_type": "image/png",
            "metadata": {
                "camera_frame_id": "front-1",
                "camera_frame_age_s": 0.25,
            },
        }

    state = DogOpsRouteExecutor(
        tmp_path,
        capture_image_handler=capture_image,
    ).follow_route(dry_run=True)

    assert state.route_run_id
    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id)
    image_evidence = [item for item in evidence if item["kind"] == "image"][0]
    assert image_evidence["metadata"]["source"] == "go2_camera_live"
    assert image_evidence["metadata"]["camera_frame_id"] == "front-1"
    assert image_evidence["metadata"]["camera_frame_age_s"] == 0.25
    assert Path(image_evidence["path"]).read_bytes() == b"live-camera"


def test_qr_route_action_uses_scan_zone_handler_when_target_is_zone(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_CAMERA_QR",
        label="Route Camera QR",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-QA",
                label="QA",
                target_id="QA_HOLD",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="SCAN-QR",
                        kind="scan_qr",
                        args={"expected": ["PKG-104"]},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_CAMERA_QR", routes=[route]),
    )
    scanned_zones: list[str] = []

    def scan_zone(zone_id: str) -> str:
        scanned_zones.append(zone_id)
        return json.dumps(
            {
                "ok": True,
                "skill": "scan_zone",
                "zone_id": zone_id,
                "visible_tag_ids": [104],
                "package_ids": ["PKG-104"],
                "source": "camera",
                "evidence_observation_ids": ["CAM-1"],
            }
        )

    state = DogOpsRouteExecutor(tmp_path, scan_zone_handler=scan_zone).follow_route(dry_run=True)

    assert state.state == "completed"
    assert scanned_zones == ["QA_HOLD"]
    completed = [event for event in state.events if event.action_id == "SCAN-QR"][-1]
    assert completed.payload["source"] == "scan_zone"
    assert completed.payload["scan_zone_source"] == "camera"
    assert completed.payload["detected_payloads"] == ["PKG-104"]
    assert completed.payload["detected_tag_ids"] == [104]
    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id or "")
    assert evidence[0]["metadata"]["source"] == "scan_zone"
    assert evidence[0]["metadata"]["scan_zone_source"] == "camera"


def test_qr_route_action_fails_when_scan_zone_handler_fails(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_CAMERA_QR_FAIL",
        label="Route Camera QR Fail",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-QA",
                label="QA",
                target_id="QA_HOLD",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="SCAN-QR",
                        kind="scan_qr",
                        args={"expected": ["PKG-104"]},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_CAMERA_QR_FAIL", routes=[route]),
    )

    def scan_zone(zone_id: str) -> dict[str, object]:
        return {
            "ok": False,
            "skill": "scan_zone",
            "zone_id": zone_id,
            "error": "camera_detector_unavailable",
        }

    state = DogOpsRouteExecutor(tmp_path, scan_zone_handler=scan_zone).follow_route(dry_run=True)

    assert state.state == "failed"
    assert state.last_error == "QR scan zone failed: camera_detector_unavailable"
    failed = [event for event in state.events if event.action_id == "SCAN-QR"][-1]
    assert failed.state == "failed"
    assert failed.payload["source"] == "scan_zone"
    assert failed.payload["scan_zone"]["error"] == "camera_detector_unavailable"


def test_gemini_inspect_without_api_key_skips_without_analysis_evidence(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    source_image = tmp_path / "go2-frame.jpg"
    source_image.write_bytes(b"fake-jpeg")
    route = EditableRoute(
        id="ROUTE_GEMINI_SKIP",
        label="Route Gemini Skip",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-GEMINI",
                label="Gemini Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="CAPTURE",
                        kind="capture_image",
                        args={"image_path": str(source_image)},
                    ),
                    EditableRouteAction(
                        id="GEMINI",
                        kind="gemini_inspect_image",
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_GEMINI_SKIP", routes=[route]),
    )

    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    assert state.state == "completed"
    gemini_events = [event for event in state.events if event.action_id == "GEMINI"]
    assert [event.state for event in gemini_events] == ["started", "skipped"]
    assert gemini_events[-1].payload["status"] == "gemini_unavailable"
    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id or "")
    assert {item["kind"] for item in evidence} == {"image"}


def test_gemini_inspect_records_analysis_evidence_with_baseline(tmp_path, monkeypatch) -> None:
    source_image = tmp_path / "go2-frame.jpg"
    source_image.write_bytes(b"fake-jpeg")
    baseline_route = EditableRoute(
        id="ROUTE_GEMINI",
        label="Route Gemini",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-GEMINI",
                label="Gemini Waypoint",
                target_id="COOLING_1",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="CAPTURE",
                        kind="capture_image",
                        args={"image_path": str(source_image), "target": "COOLING_1"},
                    )
                ],
            )
        ],
    )
    inspect_route = baseline_route.model_copy(deep=True)
    inspect_route.waypoints[0].actions.append(
        EditableRouteAction(
            id="GEMINI",
            kind="gemini_inspect_image",
            args={"target": "COOLING_1"},
        )
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_GEMINI", routes=[baseline_route]),
    )
    first = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_GEMINI", routes=[inspect_route]),
    )

    def fake_inspect(**kwargs):
        assert kwargs["baseline_image_path"]
        assert kwargs["baseline_evidence_id"]
        return GeminiVisionResult(
            ok=True,
            status="completed",
            message="No visible change",
            model=kwargs["model"],
            inspection=GeminiImageInspection(
                ok=True,
                summary="No visible change",
                current_description="Cooling area is clear.",
                baseline_description="Cooling area was clear.",
                changed=False,
                change_summary="No material change.",
                change_type="no_change",
                severity="info",
                confidence=0.91,
                observations=["clearance visible"],
                possible_incident=False,
                recommended_action="Continue route.",
            ),
        )

    monkeypatch.setattr(
        "dimos.experimental.dogops.gemini_vision.inspect_images_with_gemini",
        fake_inspect,
    )

    second = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    assert first.route_run_id
    assert second.route_run_id
    evidence = RouteRunStore(tmp_path).route_run_evidence(second.route_run_id)
    analysis = [item for item in evidence if item["kind"] == "gemini_vision_analysis"]
    assert len(analysis) == 1
    assert analysis[0]["metadata"]["baseline_route_run_id"] == first.route_run_id
    assert analysis[0]["metadata"]["baseline_match"] == "same_route_waypoint"
    assert analysis[0]["metadata"]["changed"] is False
    assert (tmp_path / "route_runs" / second.route_run_id / "evidence" / "WP-GEMINI-GEMINI-gemini.json").exists()
    gemini_events = [event for event in second.events if event.action_id == "GEMINI"]
    assert gemini_events[-1].payload["analysis_evidence_id"] == analysis[0]["evidence_id"]


def test_gemini_inspect_can_analyze_default_placeholder_capture(tmp_path, monkeypatch) -> None:
    route = EditableRoute(
        id="ROUTE_GEMINI_PLACEHOLDER",
        label="Route Gemini Placeholder",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-GEMINI-PLACEHOLDER",
                label="Gemini Waypoint",
                target_id="PLACEHOLDER_TARGET",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="CAPTURE",
                        kind="capture_image",
                        args={"target": "PLACEHOLDER_TARGET"},
                    ),
                    EditableRouteAction(
                        id="GEMINI",
                        kind="gemini_inspect_image",
                        args={"target": "PLACEHOLDER_TARGET"},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_GEMINI_PLACEHOLDER", routes=[route]),
    )

    gemini_calls: list[dict[str, object]] = []

    def fake_inspect(**kwargs):
        gemini_calls.append(kwargs)
        return GeminiVisionResult(
            ok=True,
            status="completed",
            message="Placeholder inspected",
            model=kwargs["model"],
            inspection=GeminiImageInspection(
                ok=True,
                summary="Placeholder inspected",
                current_description="Demo placeholder image.",
                changed=False,
                change_summary="No baseline available.",
                change_type="unclear",
                severity="info",
                confidence=0.5,
                observations=["demo placeholder"],
                possible_incident=False,
                recommended_action="Use real camera evidence for final validation.",
            ),
        )

    monkeypatch.setattr(
        "dimos.experimental.dogops.gemini_vision.inspect_images_with_gemini",
        fake_inspect,
    )

    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id or "")
    image = [item for item in evidence if item["kind"] == "image"][0]
    analysis = [item for item in evidence if item["kind"] == "gemini_vision_analysis"][0]
    assert gemini_calls
    assert gemini_calls[0]["current_mime_type"] == "image/png"
    assert str(gemini_calls[0]["current_image_path"]).endswith(".png")
    assert image["mime_type"] == "image/png"
    assert image["metadata"]["source"] == "demo_placeholder"
    assert analysis["metadata"]["summary"] == "Placeholder inspected"


def test_dry_run_executes_route_actions_and_records_qr_evidence(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_QR",
        label="Route QR",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-QR",
                label="QR Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="SCAN-QR",
                        kind="scan_qr",
                        args={"expected": ["PKG-101"]},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_QR", routes=[route]),
    )

    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    assert state.state == "completed"
    action_events = [event for event in state.events if event.kind == "action"]
    assert [event.state for event in action_events] == ["started", "completed"]
    assert action_events[-1].payload["detected_payloads"] == ["PKG-101"]
    assert RouteRunStore(tmp_path).route_run_detail(state.route_run_id or "")["actions_completed"] == 1
    evidence = RouteRunStore(tmp_path).route_run_evidence(state.route_run_id or "")
    assert evidence[0]["kind"] == "qr_detection"
    assert evidence[0]["metadata"]["detected_payloads"] == ["PKG-101"]


def test_optional_action_failure_records_and_continues(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_OPTIONAL",
        label="Route Optional",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-OPTIONAL",
                label="Optional Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="OPTIONAL-QR",
                        kind="scan_qr",
                        required=False,
                    ),
                    EditableRouteAction(
                        id="SCAN-TAGS",
                        kind="scan_tags",
                        args={"expected": [41]},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_OPTIONAL", routes=[route]),
    )

    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    assert state.state == "completed"
    action_events = [event for event in state.events if event.kind == "action"]
    assert [(event.action_id, event.state) for event in action_events] == [
        ("OPTIONAL-QR", "started"),
        ("OPTIONAL-QR", "failed"),
        ("SCAN-TAGS", "started"),
        ("SCAN-TAGS", "completed"),
    ]
    route_run = RouteRunStore(tmp_path).route_run_detail(state.route_run_id or "")
    assert route_run["state"] == "completed"
    assert route_run["actions_completed"] == 1


def test_dry_run_required_action_failure_stays_failed(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_REQUIRED_FAIL",
        label="Route Required Fail",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-REQUIRED",
                label="Required Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[EditableRouteAction(id="REQUIRED-QR", kind="scan_qr")],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_REQUIRED_FAIL", routes=[route]),
    )

    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    assert state.state == "failed"
    assert state.last_error == "QR scan configured without expected payloads"
    assert load_route_execution(tmp_path).state == "failed"
    route_run = RouteRunStore(tmp_path).route_run_detail(state.route_run_id or "")
    assert route_run["state"] == "failed"
    assert route_run["actions_completed"] == 0


def test_skipped_action_state_is_preserved(tmp_path, monkeypatch) -> None:
    route = EditableRoute(
        id="ROUTE_SKIPPED",
        label="Route Skipped",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-SKIPPED",
                label="Skipped Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[EditableRouteAction(id="WAIT", kind="wait")],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_SKIPPED", routes=[route]),
    )

    from dimos.experimental.dogops.route_actions import RouteActionResult
    import dimos.experimental.dogops.route_executor as route_executor_module

    monkeypatch.setattr(
        route_executor_module,
        "execute_route_action",
        lambda *_, **__: RouteActionResult(ok=True, state="skipped", note="not needed"),
    )

    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    skipped_event = [event for event in state.events if event.action_id == "WAIT"][-1]
    assert skipped_event.state == "skipped"
    route_events = RouteRunStore(tmp_path).route_run_events(state.route_run_id or "")
    assert route_events[-1]["state"] == "skipped"


def test_route_action_exception_marks_route_failed(tmp_path) -> None:
    route = EditableRoute(
        id="ROUTE_BAD_ACTION",
        label="Route Bad Action",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-BAD",
                label="Waypoint",
                pose=EditableMapPoint(x=1.0, y=2.0),
                actions=[
                    EditableRouteAction(
                        id="BAD-SCAN",
                        kind="scan_tags",
                        args={"expected": ["not-an-int"]},
                    ),
                ],
            )
        ],
    )
    save_map_authoring(
        tmp_path,
        MapAuthoringState(selected_route_id="ROUTE_BAD_ACTION", routes=[route]),
    )
    current_goal = {"x": 0.0, "y": 0.0}

    def publish(x: float, y: float, z: float, frame: str) -> dict[str, object]:
        current_goal.update({"x": x, "y": y})
        return {"accepted": True}

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(publish, transport_name="fake_nav"),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": current_goal["x"], "y": current_goal["y"]},
            "target": {"x": current_goal["x"], "y": current_goal["y"]},
            "topics": {"odom": {"age_s": 0.1}},
        },
        sleep_fn=lambda _: None,
    )

    state = executor.follow_route()

    assert state.state == "failed"
    assert state.route_run_id
    assert "invalid literal" in (state.last_error or "")
    assert load_route_execution(tmp_path).state == "failed"
    failed_action = [event for event in state.events if event.action_id == "BAD-SCAN"][-1]
    assert failed_action.state == "failed"
    assert failed_action.payload["error"] == "ValueError"
    route_run = RouteRunStore(tmp_path).route_run_detail(state.route_run_id)
    assert route_run is not None
    assert route_run["state"] == "failed"


def test_stale_odom_causes_timeout_failure(tmp_path) -> None:
    _save_authoring(tmp_path)
    now = 0.0

    def time_fn() -> float:
        return now

    def sleep_fn(seconds: float) -> None:
        nonlocal now
        now += seconds

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(lambda *_: {"accepted": True}),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": 0.0, "y": 0.0},
            "target": {"x": 1.0, "y": 2.0},
            "topics": {"odom": {"age_s": 99.0}},
        },
        waypoint_timeout_s=0.25,
        poll_interval_s=0.1,
        time_fn=time_fn,
        sleep_fn=sleep_fn,
    )

    state = executor.follow_route()

    assert state.state == "failed"
    assert state.last_error == "odom stale: 99.0s"
    assert state.events[-1].state == "timeout"


def test_stop_request_interrupts_execution(tmp_path) -> None:
    _save_authoring(tmp_path)
    now = 0.0
    stopped = False

    def time_fn() -> float:
        return now

    def sleep_fn(seconds: float) -> None:
        nonlocal now, stopped
        now += seconds
        if not stopped:
            stopped = True
            state = load_route_execution(tmp_path)
            state.stop_requested = True
            save_route_execution(tmp_path, state)

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(lambda *_: {"accepted": True}),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": -10.0, "y": -10.0},
            "target": {"x": 1.0, "y": 2.0},
            "topics": {"odom": {"age_s": 0.1}},
        },
        waypoint_timeout_s=2.0,
        poll_interval_s=0.1,
        time_fn=time_fn,
        sleep_fn=sleep_fn,
    )

    state = executor.follow_route()

    assert state.state == "stopped"
    assert state.stop_requested is True
    assert state.events[-1].state == "stopped"


def test_completed_route_appends_nav_events_and_report(tmp_path) -> None:
    run_offline_simulation(out=tmp_path)
    _save_authoring(tmp_path)
    current_goal = {"x": 0.0, "y": 0.0}

    def publish(x: float, y: float, z: float, frame: str) -> dict[str, object]:
        current_goal.update({"x": x, "y": y})
        return {"accepted": True}

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(publish, transport_name="fake_nav"),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": current_goal["x"], "y": current_goal["y"]},
            "target": {"x": current_goal["x"], "y": current_goal["y"]},
            "topics": {"odom": {"age_s": 0.1}},
        },
        sleep_fn=lambda _: None,
    )

    state = executor.follow_route()

    assert state.state == "completed"
    report = (tmp_path / "report.md").read_text(encoding="utf-8")
    assert "live route ROUTE_A: reached CHECKPOINT_1 via fake_nav" in report
    stored = (tmp_path / "nav_events.jsonl").read_text(encoding="utf-8")
    assert "CHECKPOINT_1" in stored
    assert "CHECKPOINT_2" in stored
    assert '"error_m": 0.0' in stored
    assert '"guided": false' in stored
    assert '"retries": 0' in stored


def test_repeated_route_runs_append_distinct_nav_evidence(tmp_path) -> None:
    run_offline_simulation(out=tmp_path)
    _save_authoring(tmp_path)
    current_goal = {"x": 0.0, "y": 0.0}

    def publish(x: float, y: float, z: float, frame: str) -> dict[str, object]:
        current_goal.update({"x": x, "y": y})
        return {"accepted": True}

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(publish, transport_name="fake_nav"),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": current_goal["x"], "y": current_goal["y"]},
            "target": {"x": current_goal["x"], "y": current_goal["y"]},
            "topics": {"odom": {"age_s": 0.1}},
        },
        sleep_fn=lambda _: None,
    )

    first = executor.follow_route()
    second = executor.follow_route()

    assert first.state == "completed"
    assert second.state == "completed"
    stored = (tmp_path / "nav_events.jsonl").read_text(encoding="utf-8")
    assert stored.count("live route ROUTE_A: reached CHECKPOINT_1 via fake_nav") == 2


def test_no_progress_is_recorded_before_timeout(tmp_path) -> None:
    _save_authoring(tmp_path)
    now = 0.0

    def sleep_fn(seconds: float) -> None:
        nonlocal now
        now += seconds

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(lambda *_: {"accepted": True}),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": -10.0, "y": -10.0},
            "target": {"x": 1.0, "y": 2.0},
            "topics": {"odom": {"age_s": 0.1}},
        },
        waypoint_timeout_s=5.0,
        no_progress_timeout_s=0.3,
        poll_interval_s=0.1,
        time_fn=lambda: now,
        sleep_fn=sleep_fn,
    )

    state = executor.follow_route()

    assert state.state == "failed"
    assert state.last_error == "no progress toward waypoint"
    assert state.events[-1].note == "no progress toward waypoint"


def test_unconfirmed_goal_does_not_mark_waypoint_reached(tmp_path) -> None:
    _save_authoring(tmp_path)

    executor = DogOpsRouteExecutor(
        tmp_path,
        goal_publisher=CallableGoalPublisher(lambda *_: {"accepted": True}),
        live_snapshot_reader=lambda: {
            "robot_pose": {"x": 1.0, "y": 2.0},
            "topics": {"odom": {"age_s": 0.1}},
        },
        waypoint_timeout_s=0.1,
        sleep_fn=lambda _: None,
    )

    state = executor.follow_route()

    assert state.state == "failed"
    assert "unconfirmed" in state.last_error


def test_concurrent_route_start_is_rejected(tmp_path) -> None:
    _save_authoring(tmp_path)
    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)
    state.state = "running"
    save_route_execution(tmp_path, state)

    with route_execution_lock(tmp_path):
        with pytest.raises(RouteExecutionError, match="already running"):
            DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)

    # The failed attempt must not leave behind a stale lock.
    state.state = "completed"
    save_route_execution(tmp_path, state)
    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)
    assert state.state == "completed"


def test_route_feedback_from_snapshot_handles_odom_age() -> None:
    odom, age_s = route_feedback_from_snapshot(
        {"robot_pose": {"x": "1.5", "y": 2}, "topics": {"odom": {"age_s": 0.4}}}
    )

    assert odom == {"x": 1.5, "y": 2.0}
    assert age_s == 0.4


def test_request_route_stop_sets_server_side_stop_flag(tmp_path) -> None:
    _save_authoring(tmp_path)
    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)
    state.state = "running"
    state.stop_requested = False
    save_route_execution(tmp_path, state)

    stopped = request_route_stop(tmp_path, now=lambda: 123.0)

    assert stopped.state == "stopped"
    assert stopped.stop_requested is True
    assert stopped.completed_at == 123.0
    assert load_route_execution(tmp_path).stop_requested is True


def test_stop_route_invokes_transport_stop_handler(tmp_path) -> None:
    _save_authoring(tmp_path)
    calls: list[str] = []
    state = DogOpsRouteExecutor(tmp_path).follow_route(dry_run=True)
    state.state = "running"
    save_route_execution(tmp_path, state)

    stopped = DogOpsRouteExecutor(tmp_path, stop_handler=lambda: calls.append("stop")).stop_route()

    assert stopped.state == "stopped"
    assert calls == ["stop"]
    events = RouteRunStore(tmp_path).route_run_events(stopped.route_run_id or "")
    assert events[-1]["state"] == "stopped"
    assert events[-1]["kind"] == "system"
