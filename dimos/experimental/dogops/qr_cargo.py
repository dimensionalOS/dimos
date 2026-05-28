from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time
from typing import Any
import uuid


QR_EVENTS_FILENAME = "qr_events.jsonl"
QR_CARGO_STATE_FILENAME = "qr_cargo_state.json"
QR_CARGO_STATE_SCHEMA_VERSION = 1
QR_LATEST_LIMIT = 50
QR_REQUIRED_PAYLOAD_FIELDS = (
    "warehouse_id",
    "location_node_id",
    "cargo_id",
    "task",
)


def validate_qr_payload(payload: dict[str, Any]) -> tuple[bool, list[str]]:
    errors: list[str] = []
    if not isinstance(payload, dict):
        return False, ["qr_payload must be an object"]
    if payload.get("type") != "cargo":
        errors.append("qr_payload.type must be cargo")
    for field in QR_REQUIRED_PAYLOAD_FIELDS:
        if not _non_empty_text(payload.get(field)):
            errors.append(f"qr_payload.{field} is required")
    return not errors, errors


def normalize_qr_event(event: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(event, dict):
        raise ValueError("QR event must be an object")

    status = str(event.get("status") or "").strip()
    if not status:
        raise ValueError("event.status is required")

    action_policy = str(event.get("action_policy") or "report_only").strip() or "report_only"
    if action_policy != "report_only":
        raise ValueError("event.action_policy must be report_only")

    qr_payload, qr_payload_raw = _qr_payload_from_event(event)
    ok, errors = validate_qr_payload(qr_payload)
    if not ok:
        raise ValueError("; ".join(errors))

    timestamp = _float_or_none(event.get("timestamp"))
    if timestamp is None:
        timestamp = time.time()

    event_id = str(event.get("event_id") or "").strip()
    if not event_id:
        event_id = _new_event_id(timestamp, qr_payload)

    return {
        "event_id": event_id,
        "timestamp": timestamp,
        "source": str(event.get("source") or "unknown"),
        "status": status,
        "qr_payload_raw": qr_payload_raw,
        "qr_payload": qr_payload,
        "robot_pose_at_detection": _normalize_robot_pose(
            event.get("robot_pose_at_detection")
        ),
        "bbox_px": _normalize_bbox(event.get("bbox_px")),
        "action_policy": action_policy,
    }


def load_qr_events(run_dir: Path) -> list[dict[str, Any]]:
    path = qr_events_path(run_dir)
    if not path.exists():
        return []
    events: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        raw = line.strip()
        if not raw:
            continue
        try:
            event = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid QR event JSON on line {line_number}: {path}") from exc
        if not isinstance(event, dict):
            raise ValueError(f"QR event line {line_number} must contain an object: {path}")
        events.append(event)
    return events


def append_qr_event(run_dir: Path, event: dict[str, Any]) -> dict[str, Any]:
    root = Path(run_dir)
    root.mkdir(parents=True, exist_ok=True)
    normalized = normalize_qr_event(event)
    path = qr_events_path(root)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(normalized, sort_keys=True) + "\n")
    events = load_qr_events(root)
    _write_qr_cargo_state(root, events)
    return normalized


def get_latest_qr_events(run_dir: Path, limit: int = QR_LATEST_LIMIT) -> list[dict[str, Any]]:
    events = load_qr_events(run_dir)
    safe_limit = max(1, int(limit or QR_LATEST_LIMIT))
    return list(reversed(events[-safe_limit:]))


def get_qr_event(run_dir: Path, event_id: str) -> dict[str, Any] | None:
    for event in reversed(load_qr_events(run_dir)):
        if str(event.get("event_id") or "") == event_id:
            return event
    return None


def qr_events_path(run_dir: str | Path) -> Path:
    return Path(run_dir) / QR_EVENTS_FILENAME


def qr_cargo_state_path(run_dir: str | Path) -> Path:
    return Path(run_dir) / QR_CARGO_STATE_FILENAME


def _write_qr_cargo_state(run_dir: Path, events: list[dict[str, Any]]) -> None:
    latest_by_cargo_id: dict[str, dict[str, Any]] = {}
    for event in reversed(events):
        payload = event.get("qr_payload") if isinstance(event.get("qr_payload"), dict) else {}
        cargo_id = str(payload.get("cargo_id") or "")
        if cargo_id and cargo_id not in latest_by_cargo_id:
            latest_by_cargo_id[cargo_id] = event

    state = {
        "schema_version": QR_CARGO_STATE_SCHEMA_VERSION,
        "updated_at": time.time(),
        "events_total": len(events),
        "latest_events": list(reversed(events[-QR_LATEST_LIMIT:])),
        "latest_by_cargo_id": latest_by_cargo_id,
    }
    path = qr_cargo_state_path(run_dir)
    raw = json.dumps(state, indent=2, sort_keys=True)
    tmp_path = path.with_name(
        f"{path.name}.{os.getpid()}.{time.time_ns()}.tmp"
    )
    tmp_path.write_text(raw + "\n", encoding="utf-8")
    tmp_path.replace(path)


def _qr_payload_from_event(event: dict[str, Any]) -> tuple[dict[str, Any], str]:
    payload = event.get("qr_payload")
    raw = event.get("qr_payload_raw")

    if isinstance(payload, dict):
        payload_dict = dict(payload)
        payload_raw = raw if isinstance(raw, str) else json.dumps(payload_dict, sort_keys=True)
        return payload_dict, payload_raw

    if isinstance(payload, str):
        payload_dict = _decode_payload_string(payload, "qr_payload")
        return payload_dict, raw if isinstance(raw, str) else payload

    if isinstance(raw, str):
        return _decode_payload_string(raw, "qr_payload_raw"), raw

    raise ValueError("event.qr_payload or event.qr_payload_raw is required")


def _decode_payload_string(raw: str, field: str) -> dict[str, Any]:
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"event.{field} must be valid JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"event.{field} must decode to an object")
    return payload


def _normalize_robot_pose(value: Any) -> dict[str, Any]:
    pose = value if isinstance(value, dict) else {}
    return {
        "frame": str(pose.get("frame") or "map"),
        "x": _float_or_none(pose.get("x")),
        "y": _float_or_none(pose.get("y")),
        "yaw": _float_or_none(pose.get("yaw")),
    }


def _normalize_bbox(value: Any) -> list[list[float]] | None:
    if not isinstance(value, list):
        return None
    points: list[list[float]] = []
    for point in value:
        if not isinstance(point, list | tuple) or len(point) != 2:
            return None
        x = _float_or_none(point[0])
        y = _float_or_none(point[1])
        if x is None or y is None:
            return None
        points.append([x, y])
    return points or None


def _float_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _non_empty_text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _new_event_id(timestamp: float, payload: dict[str, Any]) -> str:
    cargo_id = str(payload.get("cargo_id") or "cargo").strip() or "cargo"
    cargo_slug = "".join(ch if ch.isalnum() else "-" for ch in cargo_id.lower()).strip("-")
    return f"qr-{cargo_slug}-{int(timestamp * 1000)}-{uuid.uuid4().hex[:8]}"
