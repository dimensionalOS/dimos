from __future__ import annotations

from collections.abc import Callable
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Literal, Protocol

from pydantic import Field, field_validator

from dimos.experimental.dogops.live_map import LIVE_TOPIC_MAX_AGE_S
from dimos.experimental.dogops.map_authoring import (
    EditableRoute,
    EditableRouteWaypoint,
    load_map_authoring,
)
from dimos.experimental.dogops.models import DogOpsModel, NavAction, NavEvent
from dimos.experimental.dogops.nav_eval import summarize_nav_events
from dimos.experimental.dogops.route_actions import (
    CaptureImageHandler,
    EditableRouteAction,
    ScanZoneHandler,
    execute_route_action,
)
from dimos.experimental.dogops.route_run_store import RouteRunStore, new_route_run_id
from dimos.experimental.dogops.store import DogOpsStore


ROUTE_EXECUTION_FILENAME = "route_execution.json"
ROUTE_EXECUTION_LOCK_FILENAME = "route_execution.lock"
GOAL_CONFIRM_RADIUS_M = 0.75
PROGRESS_EPSILON_M = 0.05
RouteExecutionEventState = Literal[
    "queued",
    "sent",
    "accepted",
    "started",
    "completed",
    "reached",
    "timeout",
    "failed",
    "skipped",
    "stopped",
]


class RouteExecutionError(ValueError):
    pass


class GoalPublisher(Protocol):
    transport_name: str

    def publish_goal(self, *, x: float, y: float, z: float, frame_id: str) -> dict[str, Any]:
        ...


StopHandler = Callable[[], Any]


class CallableGoalPublisher:
    def __init__(
        self,
        handler: Callable[[float, float, float, str], Any],
        *,
        transport_name: str = "handler",
    ) -> None:
        self._handler = handler
        self.transport_name = transport_name

    def publish_goal(self, *, x: float, y: float, z: float, frame_id: str) -> dict[str, Any]:
        result = self._handler(x, y, z, frame_id)
        return {"transport": self.transport_name, "result": result}


class ClickedPointGoalPublisher:
    transport_name = "clicked_point"

    def __init__(self, publisher: Any, point_type: type[Any]) -> None:
        self._publisher = publisher
        self._point_type = point_type

    def publish_goal(self, *, x: float, y: float, z: float, frame_id: str) -> dict[str, Any]:
        if self._publisher is None or not hasattr(self._publisher, "publish"):
            raise RouteExecutionError("DogOps follow_route needs the DimOS clicked_point stream.")
        point = self._point_type(ts=time.time(), frame_id=frame_id, x=x, y=y, z=z)
        self._publisher.publish(point)
        return {"transport": self.transport_name}


class RouteExecutionEvent(DogOpsModel):
    id: str
    ts: float
    route_id: str
    waypoint_id: str
    target_id: str | None = None
    x: float
    y: float
    state: RouteExecutionEventState
    elapsed_s: float = 0.0
    error_m: float | None = None
    retries: int = 0
    guided: bool = False
    note: str = ""
    kind: Literal["route", "waypoint", "navigation", "action", "observation", "evidence", "incident", "system"] = "waypoint"
    action_id: str | None = None
    payload: dict[str, Any] = Field(default_factory=dict)


class RouteExecutionState(DogOpsModel):
    run_id: str
    route_run_id: str | None = None
    route_id: str = ""
    state: Literal["idle", "running", "paused", "completed", "failed", "stopped"] = "idle"
    started_at: float | None = None
    completed_at: float | None = None
    active_waypoint_id: str | None = None
    active_action_id: str | None = None
    active_index: int = 0
    stop_requested: bool = False
    frame: str = "map"
    reach_radius_m: float = 0.35
    waypoint_timeout_s: float = 20.0
    max_retries: int = 1
    transport: str = "unconfigured"
    last_error: str | None = None
    waypoints_total: int = 0
    waypoints_reached: int = 0
    events: list[RouteExecutionEvent] = Field(default_factory=list)

    @field_validator("reach_radius_m", "waypoint_timeout_s")
    @classmethod
    def positive_float(cls, value: float) -> float:
        result = float(value)
        if not math.isfinite(result) or result <= 0:
            raise ValueError("value must be a positive finite number")
        return result


class DogOpsRouteExecutor:
    def __init__(
        self,
        run_dir: str | Path,
        *,
        goal_publisher: GoalPublisher | None = None,
        live_snapshot_reader: Callable[[], dict[str, Any]] | None = None,
        stop_handler: StopHandler | None = None,
        scan_zone_handler: ScanZoneHandler | None = None,
        capture_image_handler: CaptureImageHandler | None = None,
        frame: str = "map",
        reach_radius_m: float = 0.35,
        waypoint_timeout_s: float = 20.0,
        max_retries: int = 1,
        no_progress_timeout_s: float | None = None,
        poll_interval_s: float = 0.2,
        time_fn: Callable[[], float] = time.time,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self.run_dir = Path(run_dir)
        self.goal_publisher = goal_publisher
        self.live_snapshot_reader = live_snapshot_reader
        self.stop_handler = stop_handler
        self.scan_zone_handler = scan_zone_handler
        self.capture_image_handler = capture_image_handler
        self.frame = frame or "map"
        self.reach_radius_m = reach_radius_m
        self.waypoint_timeout_s = waypoint_timeout_s
        self.max_retries = max_retries
        self.no_progress_timeout_s = (
            no_progress_timeout_s
            if no_progress_timeout_s is not None
            else min(5.0, max(1.0, waypoint_timeout_s / 2.0))
        )
        self.poll_interval_s = poll_interval_s
        self.time_fn = time_fn
        self.sleep_fn = sleep_fn

    def follow_route(self, route_id: str | None = None, *, dry_run: bool = False) -> RouteExecutionState:
        with route_execution_lock(self.run_dir):
            route, frame = self._resolve_route(route_id)
            route = self._route_with_default_actions(route)
            self._validate_route(route)
            started_at = self.time_fn()
            route_run_id = new_route_run_id(route.id, now=started_at)
            state = RouteExecutionState(
                run_id=self.run_id,
                route_run_id=route_run_id,
                route_id=route.id,
                state="running",
                started_at=started_at,
                frame=frame,
                reach_radius_m=self.reach_radius_m,
                waypoint_timeout_s=self.waypoint_timeout_s,
                max_retries=self.max_retries,
                transport="dry_run" if dry_run else self._transport_name,
                waypoints_total=len(route.waypoints),
            )
            route_run_store = RouteRunStore(self.run_dir)
            route_run_store.create_route_run(
                route_run_id=route_run_id,
                dogops_run_id=self.run_id,
                route=route,
                state=state,
                dry_run=dry_run,
                route_snapshot=route.model_dump(mode="json"),
            )
            save_route_execution(self.run_dir, state)
            route_run_store.sync_execution_state(state)

            for index, waypoint in enumerate(route.waypoints):
                if load_route_execution(self.run_dir, run_id=self.run_id).stop_requested:
                    state = self._stop_state(state, waypoint, started_at)
                    break
                state.active_index = index
                state.active_waypoint_id = waypoint.id
                queued = self._event(state, waypoint, "queued", started_at, note="waypoint queued")
                state.events.append(queued)
                save_route_execution(self.run_dir, state)
                route_run_store.sync_execution_state(state)
                if dry_run:
                    state = self._execute_waypoint_actions(state, waypoint, started_at, route_run_store)
                    route_run_store.sync_execution_state(state)
                    if state.state in {"failed", "stopped"}:
                        break
                    continue
                state = self._execute_waypoint(state, waypoint, started_at)
                route_run_store.sync_execution_state(state)
                if state.state in {"failed", "stopped"}:
                    break
                state = self._execute_waypoint_actions(state, waypoint, started_at, route_run_store)
                route_run_store.sync_execution_state(state)
                if state.state in {"failed", "stopped"}:
                    break

            if dry_run:
                if state.state == "running":
                    state.state = "completed"
                    state.active_waypoint_id = None
                    state.completed_at = self.time_fn()
                    save_route_execution(self.run_dir, state)
                    route_run_store.sync_execution_state(state)
                return state

            if state.state == "running":
                state.state = "completed"
                state.active_waypoint_id = None
                state.completed_at = self.time_fn()
                save_route_execution(self.run_dir, state)
                route_run_store.sync_execution_state(state)
            self._append_nav_events(state)
            return state

    def stop_route(self) -> RouteExecutionState:
        state = request_route_stop(self.run_dir, run_id=self.run_id, now=self.time_fn)
        if self.stop_handler is not None:
            try:
                self.stop_handler()
            except Exception as exc:
                state.last_error = f"route stop requested; stop handler failed: {exc}"
                save_route_execution(self.run_dir, state)
        elif state.state == "stopped":
            state.last_error = "route stop requested; no navigation stop handler configured"
            save_route_execution(self.run_dir, state)
        RouteRunStore(self.run_dir).sync_execution_state(state)
        self._append_nav_events(state)
        return state

    def status(self) -> RouteExecutionState:
        return load_route_execution(self.run_dir, run_id=self.run_id)

    @property
    def run_id(self) -> str:
        return self.run_dir.name

    @property
    def _transport_name(self) -> str:
        if self.goal_publisher is None:
            return "unconfigured"
        return getattr(self.goal_publisher, "transport_name", self.goal_publisher.__class__.__name__)

    def _resolve_route(self, route_id: str | None) -> tuple[EditableRoute, str]:
        authoring = load_map_authoring(self.run_dir)
        resolved_id = route_id or authoring.selected_route_id
        if not resolved_id:
            raise RouteExecutionError("no route_id supplied and no authored route is selected")
        for route in authoring.routes:
            if route.id == resolved_id:
                return route, authoring.frame or self.frame
        raise RouteExecutionError(f"unknown route_id: {resolved_id}")

    def _validate_route(self, route: EditableRoute) -> None:
        if not route.waypoints:
            raise RouteExecutionError(f"route {route.id} has no waypoints")
        for waypoint in route.waypoints:
            if not math.isfinite(waypoint.pose.x) or not math.isfinite(waypoint.pose.y):
                raise RouteExecutionError(f"waypoint {waypoint.id} has invalid coordinates")

    def _route_with_default_actions(self, route: EditableRoute) -> EditableRoute:
        state_path = self.run_dir / "state.json"
        if not state_path.exists():
            return route
        store = DogOpsStore.load_existing(self.run_dir)
        state = store.state
        assert state is not None
        steps_by_target: dict[str, list[Any]] = {}
        for step in state.mission.steps:
            steps_by_target.setdefault(step.target_id, []).append(step)
        route_with_actions = route.model_copy(deep=True)
        for waypoint in route_with_actions.waypoints:
            if waypoint.actions:
                continue
            target_id = waypoint.target_id or waypoint.id
            actions = []
            for step in steps_by_target.get(target_id, []):
                actions.extend(_mission_step_actions(state, step))
            waypoint.actions = actions
        return route_with_actions

    def _execute_waypoint(
        self,
        state: RouteExecutionState,
        waypoint: EditableRouteWaypoint,
        started_at: float,
    ) -> RouteExecutionState:
        if self.goal_publisher is None:
            state.state = "failed"
            state.last_error = "navigation publisher unavailable"
            state.events.append(
                self._event(state, waypoint, "failed", started_at, note=state.last_error)
            )
            state.completed_at = self.time_fn()
            save_route_execution(self.run_dir, state)
            return state

        best_error_m: float | None = None
        for retry in range(self.max_retries + 1):
            waypoint_started = self.time_fn()
            try:
                publish_result = self.goal_publisher.publish_goal(
                    x=waypoint.pose.x,
                    y=waypoint.pose.y,
                    z=0.0,
                    frame_id=state.frame,
                )
            except Exception as exc:
                state.state = "failed"
                state.last_error = f"goal publish failed: {exc}"
                state.events.append(
                    self._event(
                        state,
                        waypoint,
                        "failed",
                        started_at,
                        retries=retry,
                        note=state.last_error,
                    )
                )
                state.completed_at = self.time_fn()
                save_route_execution(self.run_dir, state)
                return state

            state.events.append(
                self._event(
                    state,
                    waypoint,
                    "sent",
                    started_at,
                    retries=retry,
                    note=f"sent via {publish_result.get('transport') or self._transport_name}",
                )
            )
            save_route_execution(self.run_dir, state)

            reached, error_m, note = self._wait_until_reached(waypoint, waypoint_started)
            best_error_m = error_m if best_error_m is None else min(best_error_m, error_m or best_error_m)
            if reached:
                state.waypoints_reached += 1
                state.events.append(
                    self._event(
                        state,
                        waypoint,
                        "reached",
                        started_at,
                        retries=retry,
                        error_m=error_m,
                        note=note,
                    )
                )
                save_route_execution(self.run_dir, state)
                return state
            if load_route_execution(self.run_dir, run_id=self.run_id).stop_requested:
                return self._stop_state(state, waypoint, started_at)

        state.state = "failed"
        state.last_error = note
        state.events.append(
            self._event(
                state,
                waypoint,
                "timeout",
                started_at,
                retries=self.max_retries,
                error_m=best_error_m,
                note=note,
            )
        )
        state.completed_at = self.time_fn()
        save_route_execution(self.run_dir, state)
        return state

    def _execute_waypoint_actions(
        self,
        state: RouteExecutionState,
        waypoint: EditableRouteWaypoint,
        started_at: float,
        route_run_store: RouteRunStore,
    ) -> RouteExecutionState:
        for action in waypoint.actions:
            if load_route_execution(self.run_dir, run_id=self.run_id).stop_requested:
                return self._stop_state(state, waypoint, started_at)
            state.active_action_id = action.id
            state.events.append(
                self._action_event(state, waypoint, action, "started", started_at, note="action started")
            )
            save_route_execution(self.run_dir, state)
            route_run_store.sync_execution_state(state)
            try:
                result = execute_route_action(
                    action,
                    run_dir=self.run_dir,
                    route_run_id=state.route_run_id or "",
                    waypoint_id=waypoint.id,
                    route_id=state.route_id,
                    target_id=waypoint.target_id or waypoint.id,
                    pose=waypoint.pose.model_dump(mode="json"),
                    scan_zone_handler=self.scan_zone_handler,
                    capture_image_handler=self.capture_image_handler,
                )
                result_ok = result.ok
                result_note = result.note
                result_payload = result.payload
                result_evidence = result.evidence
            except Exception as exc:
                result_ok = False
                result_note = f"action {action.id} failed: {exc}"
                result_payload = {"error": exc.__class__.__name__, "source": "exception"}
                result_evidence = []
            event_state: RouteExecutionEventState = result.state if result_ok else "failed"
            state.events.append(
                self._action_event(
                    state,
                    waypoint,
                    action,
                    event_state,
                    started_at,
                    note=result_note,
                    payload=result_payload,
                )
            )
            save_route_execution(self.run_dir, state)
            route_run_store.sync_execution_state(state)
            action_event_id = f"{state.route_run_id}-{state.events[-1].id}" if state.route_run_id else state.events[-1].id
            recorded_evidence: list[dict[str, Any]] = []
            for evidence in result_evidence:
                recorded_evidence.append(route_run_store.record_evidence(
                    route_run_id=state.route_run_id or "",
                    event_id=action_event_id,
                    observation_id=evidence.get("observation_id"),
                    kind=str(evidence.get("kind") or "evidence"),
                    path=evidence.get("path"),
                    metadata=evidence.get("metadata") or {},
                    mime_type=evidence.get("mime_type"),
                ))
            if recorded_evidence:
                state.events[-1].payload["evidence"] = recorded_evidence
                analysis_evidence = [
                    item for item in recorded_evidence if item.get("kind") == "gemini_vision_analysis"
                ]
                if analysis_evidence:
                    state.events[-1].payload["analysis_evidence_id"] = analysis_evidence[0]["evidence_id"]
                save_route_execution(self.run_dir, state)
                route_run_store.sync_execution_state(state)
            if not result_ok and action.required:
                state.state = "failed"
                state.last_error = result_note or f"required action failed: {action.id}"
                state.completed_at = self.time_fn()
                save_route_execution(self.run_dir, state)
                route_run_store.sync_execution_state(state)
                return state
        state.active_action_id = None
        save_route_execution(self.run_dir, state)
        return state

    def _wait_until_reached(
        self,
        waypoint: EditableRouteWaypoint,
        waypoint_started: float,
    ) -> tuple[bool, float | None, str]:
        if self.live_snapshot_reader is None:
            return False, None, "no live odom reader configured"
        best_error_m: float | None = None
        last_progress_at = waypoint_started
        goal_confirmed = False
        note = "timeout waiting for odom"
        while self.time_fn() - waypoint_started <= self.waypoint_timeout_s:
            snapshot = self.live_snapshot_reader()
            odom, odom_age_s = route_feedback_from_snapshot(snapshot)
            goal_confirmed = goal_confirmed or route_goal_confirmed(snapshot, waypoint)
            if odom is None:
                note = "no odom received"
            elif odom_age_s is not None and odom_age_s > LIVE_TOPIC_MAX_AGE_S:
                note = f"odom stale: {odom_age_s:.1f}s"
            else:
                error_m = math.hypot(odom["x"] - waypoint.pose.x, odom["y"] - waypoint.pose.y)
                if best_error_m is None or error_m < best_error_m - PROGRESS_EPSILON_M:
                    best_error_m = error_m
                    last_progress_at = self.time_fn()
                note = (
                    f"odom error {error_m:.2f}m; "
                    f"goal {'confirmed' if goal_confirmed else 'unconfirmed'}"
                )
                if error_m <= self.reach_radius_m and goal_confirmed:
                    return True, error_m, note
                if self.time_fn() - last_progress_at >= self.no_progress_timeout_s:
                    return False, best_error_m, "no progress toward waypoint"
            if load_route_execution(self.run_dir, run_id=self.run_id).stop_requested:
                return False, best_error_m, "stop requested"
            self.sleep_fn(self.poll_interval_s)
        return False, best_error_m, note

    def _event(
        self,
        state: RouteExecutionState,
        waypoint: EditableRouteWaypoint,
        event_state: RouteExecutionEventState,
        started_at: float,
        *,
        retries: int = 0,
        error_m: float | None = None,
        note: str = "",
    ) -> RouteExecutionEvent:
        return RouteExecutionEvent(
            id=f"RTE-{len(state.events) + 1:03d}",
            ts=self.time_fn(),
            route_id=state.route_id,
            waypoint_id=waypoint.id,
            target_id=waypoint.target_id or waypoint.id,
            x=waypoint.pose.x,
            y=waypoint.pose.y,
            state=event_state,
            elapsed_s=max(0.0, self.time_fn() - started_at),
            error_m=error_m,
            retries=retries,
            guided=False,
            note=note,
        )

    def _action_event(
        self,
        state: RouteExecutionState,
        waypoint: EditableRouteWaypoint,
        action: EditableRouteAction,
        event_state: RouteExecutionEventState,
        started_at: float,
        *,
        note: str = "",
        payload: dict[str, Any] | None = None,
    ) -> RouteExecutionEvent:
        return RouteExecutionEvent(
            id=f"RTE-{len(state.events) + 1:03d}",
            ts=self.time_fn(),
            route_id=state.route_id,
            waypoint_id=waypoint.id,
            target_id=waypoint.target_id or waypoint.id,
            x=waypoint.pose.x,
            y=waypoint.pose.y,
            state=event_state,
            elapsed_s=max(0.0, self.time_fn() - started_at),
            guided=action.kind == "operator_prompt",
            note=note,
            kind="action",
            action_id=action.id,
            payload={"kind": action.kind, **(payload or {})},
        )

    def _stop_state(
        self,
        state: RouteExecutionState,
        waypoint: EditableRouteWaypoint,
        started_at: float,
    ) -> RouteExecutionState:
        state.state = "stopped"
        state.stop_requested = True
        state.last_error = "stop requested"
        state.events.append(self._event(state, waypoint, "stopped", started_at, note="stop requested"))
        state.completed_at = self.time_fn()
        save_route_execution(self.run_dir, state)
        RouteRunStore(self.run_dir).sync_execution_state(state)
        self._append_nav_events(state)
        return state

    def _append_nav_events(self, route_state: RouteExecutionState) -> None:
        state_path = self.run_dir / "state.json"
        if not state_path.exists():
            return
        store = DogOpsStore.load_existing(self.run_dir)
        state = store.state
        assert state is not None
        existing_events = {(event.ts, event.note) for event in state.nav_events}
        for event in route_state.events:
            if event.state not in {"reached", "timeout", "failed", "stopped"}:
                continue
            note = (
                f"live route {route_state.route_id}: {event.state} "
                f"{event.target_id or event.waypoint_id} via {route_state.transport}; {event.note}"
            )
            existing_key = (event.ts, note)
            if existing_key in existing_events:
                continue
            nav_event = NavEvent(
                id=f"NAV-{len(state.nav_events) + 1:03d}",
                run_id=state.run.id,
                ts=event.ts,
                action=NavAction.goto,
                target_id=event.target_id,
                success=event.state == "reached",
                elapsed_s=event.elapsed_s,
                retries=event.retries,
                guided=event.guided,
                error_m=event.error_m,
                note=note,
            )
            store.append_nav_event(nav_event)
            existing_events.add(existing_key)
        state.nav_summary = summarize_nav_events(state.run.id, state.nav_events)
        store.write_state(state.run.id)
        store.write_report(state.run.id)


def route_execution_path(run_dir: str | Path) -> Path:
    return Path(run_dir) / ROUTE_EXECUTION_FILENAME


def route_execution_lock_path(run_dir: str | Path) -> Path:
    return Path(run_dir) / ROUTE_EXECUTION_LOCK_FILENAME


class route_execution_lock:
    def __init__(self, run_dir: str | Path) -> None:
        self.run_dir = Path(run_dir)
        self.path = route_execution_lock_path(run_dir)

    def __enter__(self) -> None:
        self.run_dir.mkdir(parents=True, exist_ok=True)
        try:
            fd = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        except FileExistsError as exc:
            state = load_route_execution(self.run_dir)
            if state.state == "running":
                raise RouteExecutionError(f"route {state.route_id} is already running") from exc
            self.path.unlink(missing_ok=True)
            fd = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(f"{os.getpid()} {threading.get_ident()} {time.time()}\n")

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.path.unlink(missing_ok=True)


def load_route_execution(run_dir: str | Path, *, run_id: str | None = None) -> RouteExecutionState:
    path = route_execution_path(run_dir)
    if not path.exists():
        return RouteExecutionState(run_id=run_id or Path(run_dir).name)
    payload = json.loads(path.read_text(encoding="utf-8"))
    state = RouteExecutionState.model_validate(payload)
    if run_id is not None and state.run_id != run_id:
        raise RouteExecutionError(f"loaded route execution for {state.run_id}, expected {run_id}")
    return state


def save_route_execution(run_dir: str | Path, state: RouteExecutionState) -> Path:
    path = route_execution_path(run_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(state.model_dump(mode="json"), indent=2, sort_keys=True)
    tmp_path = path.with_name(
        f"{path.name}.{os.getpid()}.{threading.get_ident()}.{time.time_ns()}.tmp"
    )
    tmp_path.write_text(raw + "\n", encoding="utf-8")
    tmp_path.replace(path)
    return path


def route_goal_confirmed(snapshot: dict[str, Any], waypoint: EditableRouteWaypoint) -> bool:
    for key in ("target", "goal_request", "clicked_point"):
        pose = snapshot.get(key)
        if _pose_matches_waypoint(pose, waypoint):
            return True
    path = snapshot.get("path")
    return isinstance(path, list) and bool(path)


def request_route_stop(
    run_dir: str | Path,
    *,
    run_id: str | None = None,
    now: Callable[[], float] = time.time,
) -> RouteExecutionState:
    state = load_route_execution(run_dir, run_id=run_id)
    if state.state not in {"completed", "failed", "stopped"}:
        state.stop_requested = True
        state.state = "stopped"
        state.completed_at = now()
        _append_stop_event_to_state(state, now=now)
        save_route_execution(run_dir, state)
        RouteRunStore(run_dir).sync_execution_state(state)
    return state


def _append_stop_event_to_state(state: RouteExecutionState, *, now: Callable[[], float]) -> None:
    if state.state != "stopped" or any(event.state == "stopped" for event in state.events):
        return
    previous = state.events[-1] if state.events else None
    if previous is None:
        return
    state.events.append(
        RouteExecutionEvent(
            id=f"RTE-{len(state.events) + 1:03d}",
            ts=now(),
            route_id=state.route_id,
            waypoint_id=state.active_waypoint_id or previous.waypoint_id,
            target_id=previous.target_id,
            x=previous.x,
            y=previous.y,
            state="stopped",
            elapsed_s=previous.elapsed_s,
            retries=previous.retries,
            guided=previous.guided,
            note="stop requested",
            kind="system",
        )
    )


def route_feedback_from_snapshot(snapshot: dict[str, Any]) -> tuple[dict[str, float] | None, float | None]:
    pose = snapshot.get("robot_pose") or snapshot.get("pose")
    if not isinstance(pose, dict):
        return None, None
    try:
        x = float(pose["x"])
        y = float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return None, None
    if not math.isfinite(x) or not math.isfinite(y):
        return None, None
    age_s = None
    topics = snapshot.get("topics")
    if isinstance(topics, dict):
        odom = topics.get("odom")
        if isinstance(odom, dict) and odom.get("age_s") is not None:
            try:
                age_s = float(odom["age_s"])
            except (TypeError, ValueError):
                    age_s = None
    return {"x": x, "y": y}, age_s


def _mission_step_actions(state: Any, step: Any) -> list[EditableRouteAction]:
    base_args = {"target_id": step.target_id, "mission_action": step.action}
    sim_obs = state.mission.simulation_observations.get(step.id)
    if step.action == "scan_zone":
        visible_tags = list(sim_obs.visible_tag_ids) if sim_obs is not None else []
        qr_payloads = _qr_payloads_for_observation(sim_obs)
        actions = [
            EditableRouteAction(
                id=f"{step.id}_tags",
                kind="scan_tags",
                label="scan_tags",
                required=step.required,
                timeout_s=step.timeout_s,
                args={**base_args, "expected": visible_tags},
            )
        ]
        if qr_payloads:
            actions.append(
                EditableRouteAction(
                    id=f"{step.id}_qr",
                    kind="scan_qr",
                    label="scan_qr",
                    required=False,
                    timeout_s=step.timeout_s,
                    args={**base_args, "expected": qr_payloads},
                )
            )
        return actions
    if step.action == "inspect_asset":
        return [
            EditableRouteAction(
                id=f"{step.id}_image",
                kind="capture_image",
                label="capture_image",
                required=False,
                timeout_s=step.timeout_s,
                args={**base_args, "target": step.target_id},
            ),
            EditableRouteAction(
                id=step.id,
                kind="inspect_asset",
                label=step.action,
                required=step.required,
                timeout_s=step.timeout_s,
                args=base_args,
            ),
        ]
    if step.action == "verify_work_order":
        return [
            EditableRouteAction(
                id=f"{step.id}_image",
                kind="capture_image",
                label="capture_image",
                required=False,
                timeout_s=step.timeout_s,
                args={**base_args, "target": step.target_id},
            ),
            EditableRouteAction(
                id=step.id,
                kind="verify_work_order",
                label=step.action,
                required=step.required,
                timeout_s=step.timeout_s,
                args=base_args,
            ),
        ]
    if step.action == "wait_for_human_fix":
        return [
            EditableRouteAction(
                id=step.id,
                kind="operator_prompt",
                label=step.action,
                required=step.required,
                timeout_s=step.timeout_s,
                args=base_args,
            )
        ]
    return []


def _qr_payloads_for_observation(sim_obs: Any) -> list[str]:
    if sim_obs is None:
        return []
    payloads: list[str] = []
    for key in sim_obs.facts:
        if key.startswith("PKG-") and key.endswith(".zone_id"):
            payloads.append(key.removesuffix(".zone_id"))
    return sorted(set(payloads))


def _pose_matches_waypoint(pose: Any, waypoint: EditableRouteWaypoint) -> bool:
    if not isinstance(pose, dict):
        return False
    try:
        x = float(pose["x"])
        y = float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return False
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    return math.hypot(x - waypoint.pose.x, y - waypoint.pose.y) <= GOAL_CONFIRM_RADIUS_M
