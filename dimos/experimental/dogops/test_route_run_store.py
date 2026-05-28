from __future__ import annotations

import sqlite3
import time

from dimos.experimental.dogops.map_authoring import (
    EditableMapPoint,
    EditableRoute,
    EditableRouteWaypoint,
    MapAuthoringState,
    save_map_authoring,
)
from dimos.experimental.dogops.route_executor import DogOpsRouteExecutor
from dimos.experimental.dogops.route_actions import EditableRouteAction
from dimos.experimental.dogops.route_run_store import RouteRunStore, route_run_db_path


def _save_route(run_dir, route_id: str) -> None:
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
                            label="Waypoint",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        )
                    ],
                )
            ],
        ),
    )


def test_route_run_history_is_global_across_run_dirs(tmp_path) -> None:
    first = tmp_path / ".dogops" / "runs" / "first"
    second = tmp_path / ".dogops" / "runs" / "second"
    _save_route(first, "ROUTE_A")
    _save_route(second, "ROUTE_B")

    DogOpsRouteExecutor(first).follow_route(dry_run=True)
    DogOpsRouteExecutor(second).follow_route(dry_run=True)

    assert route_run_db_path(first) == tmp_path / ".dogops" / "dogops.sqlite"
    runs = RouteRunStore(first).list_route_runs()
    assert {run["dogops_run_id"] for run in runs} == {"first", "second"}
    assert {run["route_id"] for run in runs} == {"ROUTE_A", "ROUTE_B"}


def test_every_route_run_gets_distinct_history_record(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    _save_route(run_dir, "ROUTE_A")

    first = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    second = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)

    assert first.route_run_id
    assert second.route_run_id
    assert first.route_run_id != second.route_run_id
    runs = RouteRunStore(run_dir).list_route_runs()
    assert [run["route_run_id"] for run in runs] == [second.route_run_id, first.route_run_id]
    assert (run_dir / "route_runs" / first.route_run_id / "events.jsonl").exists()
    assert (run_dir / "route_runs" / second.route_run_id / "route_run.json").exists()


def test_latest_image_evidence_for_waypoint_ignores_current_route_run(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    source_image = tmp_path / "frame.jpg"
    source_image.write_bytes(b"fake-jpeg")
    route = EditableRoute(
        id="ROUTE_IMAGES",
        label="Route Images",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-IMAGE",
                label="Image Waypoint",
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
    save_map_authoring(run_dir, MapAuthoringState(selected_route_id="ROUTE_IMAGES", routes=[route]))

    first = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    second = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)

    assert first.route_run_id
    assert second.route_run_id
    store = RouteRunStore(run_dir)
    current = store.image_evidence_for_route_run_waypoint(
        route_run_id=second.route_run_id,
        waypoint_id="WP-IMAGE",
    )
    baseline = store.latest_image_evidence_for_waypoint(
        waypoint_id="WP-IMAGE",
        exclude_route_run_id=second.route_run_id,
        route_id="ROUTE_IMAGES",
        target_id="COOLING_1",
    )

    assert current is not None
    assert baseline is not None
    assert current["route_run_id"] == second.route_run_id
    assert baseline["route_run_id"] == first.route_run_id
    assert baseline["baseline_match"] == "same_route_waypoint"


def test_latest_image_evidence_for_waypoint_ignores_future_route_runs(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    source_image = tmp_path / "frame.jpg"
    source_image.write_bytes(b"fake-jpeg")
    route = EditableRoute(
        id="ROUTE_IMAGES",
        label="Route Images",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-IMAGE",
                label="Image Waypoint",
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
    save_map_authoring(run_dir, MapAuthoringState(selected_route_id="ROUTE_IMAGES", routes=[route]))
    previous = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    current = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    future = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    assert previous.route_run_id
    assert current.route_run_id
    assert future.route_run_id

    now = time.time()
    with sqlite3.connect(route_run_db_path(run_dir)) as conn:
        conn.execute(
            "UPDATE route_runs SET started_at = ? WHERE route_run_id = ?",
            (now - 60.0, previous.route_run_id),
        )
        conn.execute(
            "UPDATE route_runs SET started_at = ? WHERE route_run_id = ?",
            (now, current.route_run_id),
        )
        conn.execute(
            "UPDATE route_runs SET started_at = ? WHERE route_run_id = ?",
            (now + 60.0, future.route_run_id),
        )

    baseline = RouteRunStore(run_dir).latest_image_evidence_for_waypoint(
        waypoint_id="WP-IMAGE",
        exclude_route_run_id=current.route_run_id,
        route_id="ROUTE_IMAGES",
        target_id="COOLING_1",
    )

    assert baseline is not None
    assert baseline["route_run_id"] == previous.route_run_id


def test_latest_image_evidence_prefers_same_route_for_reused_waypoint_ids(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    source_image = tmp_path / "frame.jpg"
    source_image.write_bytes(b"fake-jpeg")

    def route(route_id: str) -> EditableRoute:
        return EditableRoute(
            id=route_id,
            label=route_id,
            waypoints=[
                EditableRouteWaypoint(
                    id="WP-SHARED",
                    label="Shared Waypoint",
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

    save_map_authoring(run_dir, MapAuthoringState(selected_route_id="ROUTE_A", routes=[route("ROUTE_A")]))
    route_a = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    save_map_authoring(run_dir, MapAuthoringState(selected_route_id="ROUTE_B", routes=[route("ROUTE_B")]))
    route_b = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    current = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    assert route_a.route_run_id
    assert route_b.route_run_id
    assert current.route_run_id

    baseline = RouteRunStore(run_dir).latest_image_evidence_for_waypoint(
        waypoint_id="WP-SHARED",
        exclude_route_run_id=current.route_run_id,
        route_id="ROUTE_B",
        target_id="COOLING_1",
    )

    assert baseline is not None
    assert baseline["route_run_id"] == route_b.route_run_id
    assert baseline["baseline_match"] == "same_route_waypoint"


def test_yesterday_baseline_policy_prefers_previous_calendar_day(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    source_image = tmp_path / "frame.jpg"
    source_image.write_bytes(b"fake-jpeg")
    route = EditableRoute(
        id="ROUTE_IMAGES",
        label="Route Images",
        waypoints=[
            EditableRouteWaypoint(
                id="WP-IMAGE",
                label="Image Waypoint",
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
    save_map_authoring(run_dir, MapAuthoringState(selected_route_id="ROUTE_IMAGES", routes=[route]))
    old = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    previous_day = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    current = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    assert old.route_run_id
    assert previous_day.route_run_id
    assert current.route_run_id

    current_started_at = time.time()
    with sqlite3.connect(route_run_db_path(run_dir)) as conn:
        conn.execute(
            "UPDATE route_runs SET started_at = ? WHERE route_run_id = ?",
            (current_started_at - 3 * 24 * 60 * 60, old.route_run_id),
        )
        conn.execute(
            "UPDATE route_runs SET started_at = ? WHERE route_run_id = ?",
            (current_started_at - 24 * 60 * 60, previous_day.route_run_id),
        )
        conn.execute(
            "UPDATE route_runs SET started_at = ? WHERE route_run_id = ?",
            (current_started_at, current.route_run_id),
        )

    baseline = RouteRunStore(run_dir).latest_image_evidence_for_waypoint(
        waypoint_id="WP-IMAGE",
        exclude_route_run_id=current.route_run_id,
        route_id="ROUTE_IMAGES",
        baseline_policy="yesterday",
    )

    assert baseline is not None
    assert baseline["route_run_id"] == previous_day.route_run_id
    assert baseline["baseline_match"] == "previous_day_same_route_waypoint"


def test_mission_steps_provide_default_route_actions_for_full_demo_route(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    from dimos.experimental.dogops.mission_engine import run_offline_simulation

    run_offline_simulation(out=run_dir)
    save_map_authoring(
        run_dir,
        MapAuthoringState(
            selected_route_id="ROUTE_DEFAULTS",
            routes=[
                EditableRoute(
                    id="ROUTE_DEFAULTS",
                    label="Route Defaults",
                    waypoints=[
                        EditableRouteWaypoint(
                            id="WP-INBOUND",
                            label="Inbound",
                            target_id="INBOUND_DOCK",
                            pose=EditableMapPoint(x=0.0, y=1.0),
                        ),
                        EditableRouteWaypoint(
                            id="WP-COOLING",
                            label="Cooling",
                            target_id="COOLING_1",
                            pose=EditableMapPoint(x=1.0, y=2.0),
                        ),
                        EditableRouteWaypoint(
                            id="WP-QA",
                            label="QA Hold",
                            target_id="QA_HOLD",
                            pose=EditableMapPoint(x=2.0, y=3.0),
                        )
                    ],
                )
            ],
        ),
    )

    state = DogOpsRouteExecutor(run_dir).follow_route(dry_run=True)
    route_run = RouteRunStore(run_dir).route_run_detail(state.route_run_id or "")

    assert route_run is not None
    assert route_run["actions_total"] >= 2
    actions_by_target = {
        waypoint["target_id"]: [(action["id"], action["kind"], action["args"]) for action in waypoint["actions"]]
        for waypoint in route_run["selected_route_snapshot"]["waypoints"]
    }
    assert [(action_id, kind) for action_id, kind, _ in actions_by_target["INBOUND_DOCK"]] == [
        ("scan_inbound_tags", "scan_tags"),
        ("scan_inbound_qr", "scan_qr"),
    ]
    assert actions_by_target["INBOUND_DOCK"][0][2]["expected"] == [20, 101, 102]
    assert actions_by_target["INBOUND_DOCK"][1][2]["expected"] == ["PKG-101", "PKG-102"]
    assert [(action_id, kind) for action_id, kind, _ in actions_by_target["COOLING_1"]] == [
        ("inspect_cooling_image", "capture_image"),
        ("inspect_cooling", "inspect_asset"),
        ("wait_for_human_fix", "operator_prompt"),
        ("verify_cooling_image", "capture_image"),
        ("verify_cooling", "verify_work_order"),
    ]
    assert [(action_id, kind) for action_id, kind, _ in actions_by_target["QA_HOLD"]] == [
        ("scan_qa_hold_tags", "scan_tags"),
        ("scan_qa_hold_qr", "scan_qr"),
    ]
    assert actions_by_target["QA_HOLD"][0][2]["expected"] == [30, 104]
    assert actions_by_target["QA_HOLD"][1][2]["expected"] == ["PKG-104"]


def test_timeline_events_are_persisted_in_sqlite(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    store = RouteRunStore(run_dir)

    store.replace_timeline_events(
        "latest",
        [
            {
                "event_id": "INC-001",
                "ts": 1.0,
                "kind": "incident",
                "state": "open",
                "target_id": "COOLING_1",
                "note": "blocked cooling",
            },
            {
                "event_id": "WO-001",
                "ts": 2.0,
                "kind": "work_order",
                "state": "verified_closed",
                "target_id": "INC-001",
                "note": "move package",
            },
        ],
    )

    timeline = store.timeline_events(dogops_run_id="latest")
    assert [event["kind"] for event in timeline] == ["incident", "work_order"]
    assert timeline[0]["payload"] == {}


def test_timeline_event_ids_are_scoped_by_run_and_route_run(tmp_path) -> None:
    run_dir = tmp_path / ".dogops" / "runs" / "latest"
    store = RouteRunStore(run_dir)

    store.replace_timeline_events(
        "first",
        [
            {
                "event_id": "INC-001",
                "route_run_id": "RR-FIRST",
                "ts": 1.0,
                "kind": "incident",
                "state": "open",
                "note": "first incident",
            },
        ],
        route_run_id="RR-FIRST",
    )
    store.replace_timeline_events(
        "second",
        [
            {
                "event_id": "INC-001",
                "route_run_id": "RR-SECOND",
                "ts": 1.0,
                "kind": "incident",
                "state": "open",
                "note": "second incident",
            },
        ],
        route_run_id="RR-SECOND",
    )

    first = store.timeline_events(dogops_run_id="first", route_run_id="RR-FIRST")
    second = store.timeline_events(dogops_run_id="second", route_run_id="RR-SECOND")
    assert [event["note"] for event in first] == ["first incident"]
    assert [event["note"] for event in second] == ["second incident"]
