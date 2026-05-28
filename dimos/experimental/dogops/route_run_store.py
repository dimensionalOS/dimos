from __future__ import annotations

from pathlib import Path
import json
import sqlite3
import time
from typing import Any
import uuid


ROUTE_RUN_DB_FILENAME = "dogops.sqlite"


def dogops_history_root(run_dir: str | Path) -> Path:
    path = Path(run_dir).resolve()
    if path.parent.name == "runs" and path.parent.parent.name == ".dogops":
        return path.parent.parent
    return path.parent


def route_run_db_path(run_dir: str | Path) -> Path:
    return dogops_history_root(run_dir) / ROUTE_RUN_DB_FILENAME


def new_route_run_id(route_id: str, *, now: float | None = None) -> str:
    timestamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now or time.time()))
    suffix = uuid.uuid4().hex[:8]
    slug = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "-" for ch in route_id).strip("-")
    return f"RR-{timestamp}-{slug or 'route'}-{suffix}"


class RouteRunStore:
    def __init__(self, run_dir: str | Path, *, db_path: str | Path | None = None) -> None:
        self.run_dir = Path(run_dir)
        self.db_path = Path(db_path) if db_path is not None else route_run_db_path(self.run_dir)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._ensure_schema()

    def create_route_run(
        self,
        *,
        route_run_id: str,
        dogops_run_id: str,
        route: Any,
        state: Any,
        dry_run: bool,
        route_snapshot: dict[str, Any],
    ) -> None:
        with self._connect() as conn:
            conn.execute(
                """
                INSERT INTO route_runs (
                  route_run_id, dogops_run_id, run_dir, route_id, route_label, mission_id,
                  state, started_at, completed_at, dry_run, transport, frame,
                  selected_route_snapshot_json, active_waypoint_id, active_action_id,
                  waypoints_total, waypoints_reached, actions_total, actions_completed,
                  last_error, summary
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    route_run_id,
                    dogops_run_id,
                    str(self.run_dir),
                    route.id,
                    route.label,
                    route.mission_id,
                    state.state,
                    state.started_at,
                    state.completed_at,
                    1 if dry_run else 0,
                    state.transport,
                    state.frame,
                    json.dumps(route_snapshot, sort_keys=True),
                    state.active_waypoint_id,
                    getattr(state, "active_action_id", None),
                    state.waypoints_total,
                    state.waypoints_reached,
                    _route_action_count(route_snapshot),
                    0,
                    state.last_error,
                    "",
                ),
            )

    def sync_execution_state(self, state: Any) -> None:
        route_run_id = getattr(state, "route_run_id", None)
        if not route_run_id:
            return
        events = [event.model_dump(mode="json") for event in state.events]
        actions_completed = sum(
            1 for event in events if event.get("kind") == "action" and event.get("state") == "completed"
        )
        with self._connect() as conn:
            conn.execute(
                """
                UPDATE route_runs
                SET state = ?, completed_at = ?, active_waypoint_id = ?, active_action_id = ?,
                    waypoints_total = ?, waypoints_reached = ?, actions_completed = ?,
                    last_error = ?, summary = ?
                WHERE route_run_id = ?
                """,
                (
                    state.state,
                    state.completed_at,
                    state.active_waypoint_id,
                    getattr(state, "active_action_id", None),
                    state.waypoints_total,
                    state.waypoints_reached,
                    actions_completed,
                    state.last_error,
                    _summary_for_state(state),
                    route_run_id,
                ),
            )
            for sequence, event in enumerate(events, 1):
                event_id = f"{route_run_id}-{event['id']}"
                conn.execute(
                    """
                    INSERT OR REPLACE INTO route_run_events (
                      event_id, route_run_id, ts, sequence, kind, state, waypoint_id,
                      action_id, target_id, x, y, error_m, retries, guided, payload_json, note
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event_id,
                        route_run_id,
                        event["ts"],
                        sequence,
                        event.get("kind") or "waypoint",
                        event["state"],
                        event.get("waypoint_id"),
                        event.get("action_id"),
                        event.get("target_id"),
                        event.get("x"),
                        event.get("y"),
                        event.get("error_m"),
                        event.get("retries") or 0,
                        1 if event.get("guided") else 0,
                        json.dumps(event.get("payload") or {}, sort_keys=True),
                        event.get("note") or "",
                    ),
                )
        self._write_route_run_exports(route_run_id, state)

    def record_evidence(
        self,
        *,
        route_run_id: str,
        event_id: str | None,
        observation_id: str | None,
        kind: str,
        path: str | Path | None,
        metadata: dict[str, Any] | None = None,
        mime_type: str | None = None,
    ) -> dict[str, Any]:
        evidence_id = f"EVD-{uuid.uuid4().hex[:10]}"
        created_at = time.time()
        evidence_path = str(path) if path is not None else None
        with self._connect() as conn:
            conn.execute(
                """
                INSERT INTO route_run_evidence (
                  evidence_id, route_run_id, event_id, observation_id, kind, path, uri,
                  sha256, mime_type, metadata_json, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    evidence_id,
                    route_run_id,
                    event_id,
                    observation_id,
                    kind,
                    evidence_path,
                    None,
                    None,
                    mime_type,
                    json.dumps(metadata or {}, sort_keys=True),
                    created_at,
                ),
            )
        self._append_evidence_export(route_run_id, {
            "evidence_id": evidence_id,
            "route_run_id": route_run_id,
            "event_id": event_id,
            "observation_id": observation_id,
            "kind": kind,
            "path": evidence_path,
            "mime_type": mime_type,
            "metadata": metadata or {},
            "created_at": created_at,
        })
        return {
            "evidence_id": evidence_id,
            "route_run_id": route_run_id,
            "event_id": event_id,
            "observation_id": observation_id,
            "kind": kind,
            "path": evidence_path,
            "mime_type": mime_type,
            "metadata": metadata or {},
            "created_at": created_at,
        }

    def replace_timeline_events(
        self,
        dogops_run_id: str,
        rows: list[dict[str, Any]],
        *,
        route_run_id: str | None = None,
    ) -> None:
        with self._connect() as conn:
            if route_run_id is None:
                conn.execute("DELETE FROM dogops_timeline_events WHERE dogops_run_id = ?", (dogops_run_id,))
            else:
                conn.execute(
                    "DELETE FROM dogops_timeline_events WHERE dogops_run_id = ? AND route_run_id = ?",
                    (dogops_run_id, route_run_id),
                )
            for sequence, row in enumerate(rows, 1):
                row_route_run_id = row.get("route_run_id") or route_run_id or ""
                source_event_id = str(row.get("event_id") or f"TL-{sequence:04d}")
                event_id = f"{dogops_run_id}:{row_route_run_id or 'run'}:{source_event_id}"
                conn.execute(
                    """
                    INSERT OR REPLACE INTO dogops_timeline_events (
                      event_id, dogops_run_id, route_run_id, ts, sequence, kind, state,
                      target_id, note, payload_json
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event_id,
                        dogops_run_id,
                        row_route_run_id or None,
                        float(row.get("ts") or 0.0),
                        int(row.get("sequence") or sequence),
                        str(row.get("kind") or "system"),
                        str(row.get("state") or ""),
                        row.get("target_id"),
                        str(row.get("note") or ""),
                        json.dumps(row.get("payload") or {}, sort_keys=True),
                    ),
                )

    def timeline_events(
        self,
        *,
        dogops_run_id: str | None = None,
        route_run_id: str | None = None,
    ) -> list[dict[str, Any]]:
        filters = []
        args: list[Any] = []
        if dogops_run_id is not None:
            filters.append("dogops_run_id = ?")
            args.append(dogops_run_id)
        if route_run_id is not None:
            filters.append("(route_run_id = ? OR route_run_id IS NULL)")
            args.append(route_run_id)
        where = f"WHERE {' AND '.join(filters)}" if filters else ""
        with self._connect() as conn:
            rows = conn.execute(
                f"""
                SELECT * FROM dogops_timeline_events
                {where}
                ORDER BY ts ASC, sequence ASC, event_id ASC
                """,
                args,
            ).fetchall()
        return [_row_to_dict(row) for row in rows]

    def list_route_runs(self, *, limit: int = 50) -> list[dict[str, Any]]:
        with self._connect() as conn:
            rows = conn.execute(
                """
                SELECT * FROM route_runs
                ORDER BY started_at DESC, route_run_id DESC
                LIMIT ?
                """,
                (limit,),
            ).fetchall()
        return [_row_to_dict(row) for row in rows]

    def current_route_run(self) -> dict[str, Any] | None:
        with self._connect() as conn:
            row = conn.execute(
                """
                SELECT * FROM route_runs
                WHERE dogops_run_id = ?
                ORDER BY
                  CASE state WHEN 'running' THEN 0 WHEN 'queued' THEN 1 ELSE 2 END,
                  started_at DESC
                LIMIT 1
                """,
                (self.run_dir.name,),
            ).fetchone()
        return _row_to_dict(row) if row is not None else None

    def route_run_detail(self, route_run_id: str) -> dict[str, Any] | None:
        with self._connect() as conn:
            row = conn.execute(
                "SELECT * FROM route_runs WHERE route_run_id = ?",
                (route_run_id,),
            ).fetchone()
        return _row_to_dict(row) if row is not None else None

    def route_run_events(self, route_run_id: str) -> list[dict[str, Any]]:
        with self._connect() as conn:
            rows = conn.execute(
                """
                SELECT * FROM route_run_events
                WHERE route_run_id = ?
                ORDER BY sequence ASC
                """,
                (route_run_id,),
            ).fetchall()
        return [_row_to_dict(row) for row in rows]

    def route_run_evidence(self, route_run_id: str) -> list[dict[str, Any]]:
        with self._connect() as conn:
            rows = conn.execute(
                """
                SELECT * FROM route_run_evidence
                WHERE route_run_id = ?
                ORDER BY created_at ASC
                """,
                (route_run_id,),
            ).fetchall()
        return [_row_to_dict(row) for row in rows]

    def image_evidence_for_route_run_waypoint(
        self,
        *,
        route_run_id: str,
        waypoint_id: str,
    ) -> dict[str, Any] | None:
        rows = [
            row
            for row in self._image_evidence_rows()
            if row["route_run_id"] == route_run_id
            and row.get("metadata", {}).get("waypoint_id") == waypoint_id
        ]
        if not rows:
            return None
        return max(rows, key=lambda row: float(row.get("created_at") or 0.0))

    def latest_image_evidence_for_waypoint(
        self,
        *,
        waypoint_id: str,
        exclude_route_run_id: str,
        route_id: str | None = None,
        target_id: str | None = None,
        pose: dict[str, Any] | None = None,
        pose_tolerance_m: float = 0.3,
        baseline_policy: str = "same_waypoint_latest_previous",
    ) -> dict[str, Any] | None:
        current_started_at = self._route_run_started_at(exclude_route_run_id)
        rows = [
            row
            for row in self._image_evidence_rows()
            if row["route_run_id"] != exclude_route_run_id
            and (
                current_started_at is None
                or float(row.get("started_at") or 0.0) < current_started_at
            )
        ]
        same_waypoint = [
            row for row in rows if row.get("metadata", {}).get("waypoint_id") == waypoint_id
        ]
        if same_waypoint:
            same_route_waypoint = [
                row for row in same_waypoint if route_id and row.get("route_id") == route_id
            ]
            result = _select_same_waypoint_baseline(
                same_route_waypoint or same_waypoint,
                exclude_route_run_id=exclude_route_run_id,
                route_id=route_id,
                baseline_policy=baseline_policy,
                current_started_at=current_started_at,
            )
            if not result.get("baseline_match"):
                result["baseline_match"] = (
                    "same_route_waypoint"
                    if route_id and result.get("route_id") == route_id
                    else "same_waypoint"
                )
            return result

        if target_id:
            same_target = [
                row for row in rows if row.get("metadata", {}).get("target") == target_id
            ]
            if same_target:
                result = max(same_target, key=_evidence_sort_key)
                result["baseline_match"] = "same_target"
                return result

        if pose:
            nearest = _nearest_pose_evidence(rows, pose=pose, tolerance_m=pose_tolerance_m)
            if nearest is not None:
                nearest["baseline_match"] = "nearest_pose"
                return nearest
        return None

    def _image_evidence_rows(self) -> list[dict[str, Any]]:
        with self._connect() as conn:
            rows = conn.execute(
                """
                SELECT route_run_evidence.*, route_runs.route_id, route_runs.started_at
                FROM route_run_evidence
                JOIN route_runs ON route_runs.route_run_id = route_run_evidence.route_run_id
                WHERE route_run_evidence.kind = 'image'
                ORDER BY route_runs.started_at DESC, route_run_evidence.created_at DESC
                """
            ).fetchall()
        return [_row_to_dict(row) for row in rows]

    def _route_run_started_at(self, route_run_id: str) -> float | None:
        with self._connect() as conn:
            row = conn.execute(
                "SELECT started_at FROM route_runs WHERE route_run_id = ?",
                (route_run_id,),
            ).fetchone()
        if row is None:
            return None
        try:
            return float(row["started_at"])
        except (KeyError, TypeError, ValueError):
            return None

    def _write_route_run_exports(self, route_run_id: str, state: Any) -> None:
        run_dir = self.run_dir / "route_runs" / route_run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        run_payload = state.model_dump(mode="json")
        (run_dir / "route_run.json").write_text(
            json.dumps(run_payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        with (run_dir / "events.jsonl").open("w", encoding="utf-8") as handle:
            for event in run_payload.get("events") or []:
                handle.write(json.dumps(event, sort_keys=True) + "\n")
        (run_dir / "evidence").mkdir(exist_ok=True)

    def _append_evidence_export(self, route_run_id: str, payload: dict[str, Any]) -> None:
        run_dir = self.run_dir / "route_runs" / route_run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        with (run_dir / "evidence.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _ensure_schema(self) -> None:
        with self._connect() as conn:
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS route_runs (
                  route_run_id TEXT PRIMARY KEY,
                  dogops_run_id TEXT NOT NULL,
                  run_dir TEXT NOT NULL,
                  route_id TEXT NOT NULL,
                  route_label TEXT,
                  mission_id TEXT,
                  state TEXT NOT NULL,
                  started_at REAL NOT NULL,
                  completed_at REAL,
                  operator_id TEXT,
                  dry_run INTEGER NOT NULL DEFAULT 0,
                  transport TEXT NOT NULL,
                  frame TEXT NOT NULL,
                  selected_route_snapshot_json TEXT NOT NULL,
                  active_waypoint_id TEXT,
                  active_action_id TEXT,
                  waypoints_total INTEGER NOT NULL DEFAULT 0,
                  waypoints_reached INTEGER NOT NULL DEFAULT 0,
                  actions_total INTEGER NOT NULL DEFAULT 0,
                  actions_completed INTEGER NOT NULL DEFAULT 0,
                  last_error TEXT,
                  summary TEXT NOT NULL DEFAULT ''
                );

                CREATE TABLE IF NOT EXISTS route_run_events (
                  event_id TEXT PRIMARY KEY,
                  route_run_id TEXT NOT NULL,
                  ts REAL NOT NULL,
                  sequence INTEGER NOT NULL,
                  kind TEXT NOT NULL,
                  state TEXT NOT NULL,
                  waypoint_id TEXT,
                  action_id TEXT,
                  target_id TEXT,
                  x REAL,
                  y REAL,
                  error_m REAL,
                  retries INTEGER NOT NULL DEFAULT 0,
                  guided INTEGER NOT NULL DEFAULT 0,
                  payload_json TEXT NOT NULL DEFAULT '{}',
                  note TEXT NOT NULL DEFAULT '',
                  FOREIGN KEY(route_run_id) REFERENCES route_runs(route_run_id)
                );

                CREATE TABLE IF NOT EXISTS route_run_evidence (
                  evidence_id TEXT PRIMARY KEY,
                  route_run_id TEXT NOT NULL,
                  event_id TEXT,
                  observation_id TEXT,
                  kind TEXT NOT NULL,
                  path TEXT,
                  uri TEXT,
                  sha256 TEXT,
                  mime_type TEXT,
                  metadata_json TEXT NOT NULL DEFAULT '{}',
                  created_at REAL NOT NULL,
                  FOREIGN KEY(route_run_id) REFERENCES route_runs(route_run_id),
                  FOREIGN KEY(event_id) REFERENCES route_run_events(event_id)
                );

                CREATE TABLE IF NOT EXISTS dogops_timeline_events (
                  event_id TEXT PRIMARY KEY,
                  dogops_run_id TEXT NOT NULL,
                  route_run_id TEXT,
                  ts REAL NOT NULL,
                  sequence INTEGER NOT NULL,
                  kind TEXT NOT NULL,
                  state TEXT NOT NULL,
                  target_id TEXT,
                  note TEXT NOT NULL DEFAULT '',
                  payload_json TEXT NOT NULL DEFAULT '{}'
                );

                CREATE INDEX IF NOT EXISTS idx_route_runs_started_at
                  ON route_runs(started_at DESC);
                CREATE INDEX IF NOT EXISTS idx_route_runs_dogops_run_id
                  ON route_runs(dogops_run_id, started_at DESC);
                CREATE INDEX IF NOT EXISTS idx_route_runs_route_id
                  ON route_runs(route_id, started_at DESC);
                CREATE INDEX IF NOT EXISTS idx_route_run_events_run_sequence
                  ON route_run_events(route_run_id, sequence);
                CREATE INDEX IF NOT EXISTS idx_route_run_evidence_run
                  ON route_run_evidence(route_run_id, created_at);
                CREATE INDEX IF NOT EXISTS idx_dogops_timeline_run
                  ON dogops_timeline_events(dogops_run_id, ts, sequence);
                CREATE INDEX IF NOT EXISTS idx_dogops_timeline_route_run
                  ON dogops_timeline_events(route_run_id, ts, sequence);
                """
            )


def _route_action_count(route_snapshot: dict[str, Any]) -> int:
    return sum(len(waypoint.get("actions") or []) for waypoint in route_snapshot.get("waypoints") or [])


def _summary_for_state(state: Any) -> str:
    return (
        f"{state.state}: {state.waypoints_reached}/{state.waypoints_total} waypoints"
        + (f"; {state.last_error}" if state.last_error else "")
    )


def _row_to_dict(row: sqlite3.Row | None) -> dict[str, Any]:
    if row is None:
        return {}
    result = dict(row)
    for key in ("selected_route_snapshot_json", "payload_json", "metadata_json"):
        if key in result:
            raw = result.pop(key)
            target = key.removesuffix("_json")
            try:
                result[target] = json.loads(raw or "{}")
            except json.JSONDecodeError:
                result[target] = {}
    return result


def _evidence_sort_key(row: dict[str, Any]) -> tuple[float, float, str]:
    return (
        float(row.get("started_at") or 0.0),
        float(row.get("created_at") or 0.0),
        str(row.get("evidence_id") or ""),
    )


def _nearest_pose_evidence(
    rows: list[dict[str, Any]],
    *,
    pose: dict[str, Any],
    tolerance_m: float,
) -> dict[str, Any] | None:
    try:
        x = float(pose["x"])
        y = float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return None
    candidates: list[tuple[float, dict[str, Any]]] = []
    for row in rows:
        metadata_pose = row.get("metadata", {}).get("pose")
        if not isinstance(metadata_pose, dict):
            continue
        try:
            dx = float(metadata_pose["x"]) - x
            dy = float(metadata_pose["y"]) - y
        except (KeyError, TypeError, ValueError):
            continue
        distance = (dx * dx + dy * dy) ** 0.5
        if distance <= tolerance_m:
            candidates.append((distance, row))
    if not candidates:
        return None
    candidates.sort(key=lambda item: (item[0], -_evidence_sort_key(item[1])[0]))
    return candidates[0][1]


def _select_same_waypoint_baseline(
    rows: list[dict[str, Any]],
    *,
    exclude_route_run_id: str,
    route_id: str | None,
    baseline_policy: str,
    current_started_at: float | None,
) -> dict[str, Any]:
    if baseline_policy in {"yesterday", "previous_calendar_day"} and current_started_at is not None:
        previous_day = time.localtime(current_started_at - 24 * 60 * 60)
        previous_day_rows = [
            row
            for row in rows
            if _same_local_day(float(row.get("started_at") or 0.0), previous_day)
        ]
        if previous_day_rows:
            result = max(previous_day_rows, key=_evidence_sort_key)
            result["baseline_match"] = (
                "previous_day_same_route_waypoint"
                if route_id and result.get("route_id") == route_id
                else "previous_day_same_waypoint"
            )
            return result

        window_start = current_started_at - 36 * 60 * 60
        window_end = current_started_at - 12 * 60 * 60
        window_rows = [
            row
            for row in rows
            if window_start <= float(row.get("started_at") or 0.0) <= window_end
        ]
        if window_rows:
            result = max(window_rows, key=_evidence_sort_key)
            result["baseline_match"] = (
                "previous_day_window_same_route_waypoint"
                if route_id and result.get("route_id") == route_id
                else "previous_day_window_same_waypoint"
            )
            return result

    result = max(rows, key=_evidence_sort_key)
    if baseline_policy in {"yesterday", "previous_calendar_day"}:
        result["baseline_match"] = "latest_previous_same_waypoint"
    return result


def _same_local_day(timestamp: float, target_day: time.struct_time) -> bool:
    candidate = time.localtime(timestamp)
    return (
        candidate.tm_year == target_day.tm_year
        and candidate.tm_yday == target_day.tm_yday
    )
