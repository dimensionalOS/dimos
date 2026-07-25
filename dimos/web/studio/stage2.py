"""Read-only Stage 2 supervision plus bounded submit/cancel adapters."""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
from threading import RLock
from typing import Any, Protocol
import unicodedata

import requests

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError

DEFAULT_STAGE2_WRAPPER_URL = "http://127.0.0.1:9991/mcp"
DEFAULT_STAGE2_GATEWAY_URL = "http://127.0.0.1:8080/v1/instructions"
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{7,127}$")
_REPLY_FIELDS = {
    "event",
    "reply_id",
    "instruction_id",
    "text",
    "completed_at",
}


class StageTwoMcp(Protocol):
    """Minimal Wrapper surface consumed by Studio."""

    def call_tool_text(
        self,
        name: str,
        arguments: dict[str, Any] | None = None,
    ) -> str: ...


GatewaySubmitter = Callable[[str, str], dict[str, Any]]


class StageTwoControl:
    """Aggregate canonical Stage 2 state without owning a robot Runtime."""

    def __init__(
        self,
        *,
        state_path: Path,
        wrapper_url: str | None = None,
        gateway_url: str | None = None,
        mcp: StageTwoMcp | None = None,
        gateway_submitter: GatewaySubmitter | None = None,
    ) -> None:
        self.state_path = state_path
        self.wrapper_url = (
            wrapper_url
            or os.environ.get("DIMOS_STUDIO_STAGE2_WRAPPER_URL")
            or DEFAULT_STAGE2_WRAPPER_URL
        )
        self.gateway_url = (
            gateway_url
            or os.environ.get("DIMOS_STUDIO_STAGE2_GATEWAY_URL")
            or DEFAULT_STAGE2_GATEWAY_URL
        )
        self._mcp = mcp or McpAdapter(self.wrapper_url, timeout=3)
        self._gateway_submitter = gateway_submitter or self._submit_to_gateway
        self._lock = RLock()

    def status(self) -> dict[str, Any]:
        """Return partial state with explicit errors instead of fake idle state."""

        errors: list[str] = []
        semantic_world: dict[str, Any] = {
            "map_id": None,
            "map_version": None,
            "places": [],
        }
        task: dict[str, Any] = {
            "state": "unavailable",
            "active": False,
            "task": None,
        }
        telemetry: dict[str, Any] = {
            "status": "unavailable",
            "odometry": {
                "available": False,
                "fresh": False,
                "age_s": None,
            },
            "planned_path_frame_id": None,
            "planned_path": [],
            "actual_path": [],
            "recovery": None,
        }

        try:
            semantic_world = _normalize_semantic_world(
                self._call_tool_json("list_semantic_places")
            )
        except Exception as exc:
            errors.append(f"list_semantic_places: {exc}")
        try:
            task = _normalize_task_status(
                self._call_tool_json("get_task_status")
            )
        except Exception as exc:
            errors.append(f"get_task_status: {exc}")
        try:
            telemetry = _normalize_telemetry(
                self._call_tool_json("get_robot_summary")
            )
        except Exception as exc:
            errors.append(f"get_robot_summary: {exc}")

        local = self._load_local_state()
        return {
            "connected": not errors,
            "wrapper_url": self.wrapper_url,
            "gateway_url": self.gateway_url,
            "errors": errors,
            "semantic_world": semantic_world,
            "task": task,
            "telemetry": telemetry,
            "last_submission": local["last_submission"],
            "last_reply": local["last_reply"],
        }

    def navigate(self, instruction_id: str, destination: str) -> dict[str, Any]:
        """Submit one confirmed-place instruction through the Agent Gateway."""

        stable_id = _clean_id(instruction_id, "instruction_id")
        requested = _clean_label(destination)
        if not requested:
            raise ValueError("地点不能为空")

        semantic_world = _normalize_semantic_world(
            self._call_tool_json("list_semantic_places")
        )
        if not semantic_world["map_id"] or not semantic_world["map_version"]:
            raise ValueError("当前地图 ID/version 未配置")
        place = _find_place(semantic_world["places"], requested)
        if place is None:
            raise ValueError(f"地点“{requested}”是当前地图的未确认地点")

        canonical_destination = place["name"]
        text = f"去{canonical_destination}"
        result = self._gateway_submitter(stable_id, text)
        if (
            result.get("instruction_id") != stable_id
            or result.get("status") != "accepted"
        ):
            raise ValueError("Agent Gateway 返回了无效受理响应")

        submission = {
            "instruction_id": stable_id,
            "destination": canonical_destination,
            "text": text,
            "status": "accepted",
        }
        self._update_local_state(last_submission=submission)
        return {"accepted": True, **submission}

    def confirm_current_place(
        self,
        name: str,
        aliases: list[str],
    ) -> dict[str, Any]:
        """Persist the current fresh odometry pose as an operator-confirmed place."""

        canonical_name = _clean_label(name)
        if not canonical_name:
            raise ValueError("地点名称不能为空")
        if len(canonical_name) > 200:
            raise ValueError("地点名称过长")
        canonical_aliases = _normalize_aliases(canonical_name, aliases)

        telemetry = _normalize_telemetry(
            self._call_tool_json("get_robot_summary")
        )
        odometry = telemetry["odometry"]
        if odometry.get("fresh") is not True:
            raise ValueError("当前里程计不是 fresh; 不能确认当前位置")
        latest_pose = _normalize_current_pose(telemetry.get("latest_pose"))
        place_json = json.dumps(
            {
                "name": canonical_name,
                "aliases": canonical_aliases,
                "pose": latest_pose,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        result = self._call_tool_json(
            "confirm_semantic_place",
            {"place_json": place_json},
        )
        if result.get("accepted") is not True:
            raise ValueError(str(result.get("reason") or "语义地点确认被拒绝"))
        place = result.get("place")
        if not isinstance(place, dict) or place.get("name") != canonical_name:
            raise ValueError("Wrapper 返回了不同的语义地点")
        return result

    def cancel(self, task_id: str) -> dict[str, Any]:
        """Cancel exactly one canonical task through the product Wrapper."""

        canonical_task_id = _clean_id(task_id, "task_id")
        result = self._call_tool_json(
            "cancel_task",
            {"task_id": canonical_task_id},
        )
        task = result.get("task")
        if isinstance(task, dict) and task.get("task_id") != canonical_task_id:
            raise ValueError("Wrapper 返回了不同 task ID")
        if result.get("requested_task_id") == canonical_task_id:
            raise ValueError(str(result.get("reason") or "任务取消被拒绝"))
        return result

    def stop_all(self) -> dict[str, Any]:
        """Request the canonical MCP emergency stop without owning task state."""

        result = self._call_tool_json("stop_all")
        if result.get("status") != "stopped":
            raise ValueError(str(result.get("error") or "stop_all 未确认停止"))
        return result

    def record_reply(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Persist one exact Agent reply event with reply-ID idempotency."""

        reply = _normalize_reply(payload)
        with self._lock:
            local = self._load_local_state()
            replies = local["replies"]
            previous = replies.get(reply["reply_id"])
            if isinstance(previous, dict):
                if previous != reply:
                    raise ValueError("相同 reply_id 的回复内容冲突")
                return {"accepted": True, "duplicate": True}
            replies[reply["reply_id"]] = reply
            while len(replies) > 100:
                replies.pop(next(iter(replies)))
            local["replies"] = replies
            local["last_reply"] = reply
            self._write_local_state(local)
        return {"accepted": True, "duplicate": False}

    def _call_tool_json(
        self,
        name: str,
        arguments: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        try:
            raw = self._mcp.call_tool_text(name, arguments or {})
        except (requests.RequestException, McpError) as exc:
            raise ValueError(f"Wrapper MCP 不可用: {exc}") from exc
        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, TypeError) as exc:
            raise ValueError(f"{name} 返回的不是 JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError(f"{name} 返回的不是对象")
        if payload.get("status") == "error":
            raise ValueError(str(payload.get("error") or f"{name} 失败"))
        return payload

    def _submit_to_gateway(
        self,
        instruction_id: str,
        text: str,
    ) -> dict[str, Any]:
        session = requests.Session()
        if self.gateway_url.startswith(
            ("http://127.0.0.1", "http://localhost", "http://[::1]")
        ):
            session.trust_env = False
        try:
            response = session.post(
                self.gateway_url,
                json={"instruction_id": instruction_id, "text": text},
                timeout=5,
            )
            if response.status_code != 202:
                detail = response.text.strip()
                raise ValueError(
                    f"Agent Gateway 拒绝请求 ({response.status_code}): {detail}"
                )
            payload = response.json()
        except requests.RequestException as exc:
            raise ValueError(f"Agent Gateway 不可用: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("Agent Gateway 返回的不是对象")
        return payload

    def _load_local_state(self) -> dict[str, Any]:
        with self._lock:
            try:
                payload = json.loads(self.state_path.read_text(encoding="utf-8"))
            except (FileNotFoundError, json.JSONDecodeError, OSError):
                return {
                    "schema_version": 1,
                    "last_submission": None,
                    "last_reply": None,
                    "replies": {},
                }
            if not isinstance(payload, dict) or payload.get("schema_version") != 1:
                return {
                    "schema_version": 1,
                    "last_submission": None,
                    "last_reply": None,
                    "replies": {},
                }
            replies = payload.get("replies")
            return {
                "schema_version": 1,
                "last_submission": payload.get("last_submission"),
                "last_reply": payload.get("last_reply"),
                "replies": replies if isinstance(replies, dict) else {},
            }

    def _update_local_state(self, **changes: Any) -> None:
        with self._lock:
            payload = self._load_local_state()
            payload.update(changes)
            self._write_local_state(payload)

    def _write_local_state(self, payload: dict[str, Any]) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.state_path.with_suffix(".tmp")
        temporary_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(self.state_path)


def _normalize_semantic_world(payload: dict[str, Any]) -> dict[str, Any]:
    map_id = payload.get("map_id")
    map_version = payload.get("map_version")
    places = payload.get("places")
    if not isinstance(map_id, str) or not isinstance(map_version, str):
        raise ValueError("语义地图缺少 map_id/map_version")
    if not isinstance(places, list):
        raise ValueError("语义地图 places 不是数组")
    normalized_places: list[dict[str, Any]] = []
    for place in places:
        if not isinstance(place, dict):
            raise ValueError("语义地点不是对象")
        name = place.get("name")
        aliases = place.get("aliases", [])
        if not isinstance(name, str) or not _clean_label(name):
            raise ValueError("语义地点缺少名称")
        if not isinstance(aliases, (list, tuple)) or not all(
            isinstance(alias, str) and _clean_label(alias) for alias in aliases
        ):
            raise ValueError(f"语义地点 {name} 的 aliases 无效")
        normalized_places.append(
            {
                **place,
                "name": _clean_label(name),
                "aliases": [_clean_label(alias) for alias in aliases],
            }
        )
    return {
        "map_id": map_id,
        "map_version": map_version,
        "places": normalized_places,
    }


def _normalize_task_status(payload: dict[str, Any]) -> dict[str, Any]:
    state = payload.get("state")
    active = payload.get("active")
    if state == "idle" and active is False and payload.get("task") is None:
        return {"state": "idle", "active": False, "task": None}
    task = payload.get("task")
    if not isinstance(state, str) or not isinstance(active, bool):
        raise ValueError("任务状态缺少 state/active")
    if not isinstance(task, dict):
        raise ValueError("任务状态缺少 canonical task")
    _clean_id(str(task.get("task_id", "")), "task_id")
    destination = task.get("destination")
    if not isinstance(destination, str) or not _clean_label(destination):
        raise ValueError("任务状态缺少 destination")
    return payload


def _normalize_telemetry(payload: dict[str, Any]) -> dict[str, Any]:
    planned_path = _normalize_path(payload.get("planned_path"), "planned_path")
    actual_path = _normalize_path(payload.get("actual_path"), "actual_path")
    odometry = payload.get("odometry")
    if not isinstance(odometry, dict):
        raise ValueError("机器人摘要缺少 odometry")
    recovery = payload.get("recovery")
    if recovery is not None:
        if (
            not isinstance(recovery, dict)
            or not isinstance(recovery.get("attempt"), int)
            or not isinstance(recovery.get("cause"), str)
            or not isinstance(recovery.get("action"), str)
        ):
            raise ValueError("恢复事件格式无效")
    return {
        **payload,
        "planned_path": planned_path,
        "actual_path": actual_path,
        "recovery": recovery,
    }


def _normalize_path(value: Any, field_name: str) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError(f"{field_name} 不是数组")
    result: list[dict[str, Any]] = []
    for point in value:
        if not isinstance(point, dict):
            raise ValueError(f"{field_name} 包含非对象点")
        x = point.get("x")
        y = point.get("y")
        if (
            isinstance(x, bool)
            or isinstance(y, bool)
            or not isinstance(x, (int, float))
            or not isinstance(y, (int, float))
            or not math.isfinite(float(x))
            or not math.isfinite(float(y))
        ):
            raise ValueError(f"{field_name} 包含无效坐标")
        result.append({"x": float(x), "y": float(y)})
    return result


def _normalize_current_pose(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("机器人摘要缺少 latest_pose")
    frame_id = value.get("frame_id")
    if not isinstance(frame_id, str) or not _clean_label(frame_id):
        raise ValueError("latest_pose 缺少 frame_id")
    result: dict[str, Any] = {
        "frame_id": _clean_label(frame_id),
    }
    fields = {
        "ts": value.get("source_ts"),
        "x": value.get("x"),
        "y": value.get("y"),
        "z": value.get("z"),
        "qx": value.get("qx"),
        "qy": value.get("qy"),
        "qz": value.get("qz"),
        "qw": value.get("qw"),
    }
    for field_name, raw in fields.items():
        if (
            isinstance(raw, bool)
            or not isinstance(raw, (int, float))
            or not math.isfinite(float(raw))
        ):
            raise ValueError(f"latest_pose 缺少有效 {field_name}")
        result[field_name] = float(raw)
    quaternion_norm = math.sqrt(
        sum(float(result[field]) ** 2 for field in ("qx", "qy", "qz", "qw"))
    )
    if not 0.99 <= quaternion_norm <= 1.01:
        raise ValueError("latest_pose 四元数不是单位四元数")
    return result


def _normalize_aliases(name: str, aliases: list[str]) -> list[str]:
    if not isinstance(aliases, list) or len(aliases) > 20:
        raise ValueError("aliases 最多允许 20 个")
    seen = {_label_key(name)}
    result: list[str] = []
    for raw in aliases:
        if not isinstance(raw, str):
            raise ValueError("alias 必须是字符串")
        alias = _clean_label(raw)
        if not alias:
            raise ValueError("alias 不能为空")
        if len(alias) > 200:
            raise ValueError("alias 过长")
        key = _label_key(alias)
        if key in seen:
            continue
        seen.add(key)
        result.append(alias)
    return result


def _normalize_reply(payload: dict[str, Any]) -> dict[str, Any]:
    if set(payload) != _REPLY_FIELDS:
        raise ValueError("回复字段不符合 agent.reply.completed 契约")
    if payload.get("event") != "agent.reply.completed":
        raise ValueError("回复 event 无效")
    reply_id = _clean_id(str(payload.get("reply_id", "")), "reply_id")
    instruction_id = _clean_id(
        str(payload.get("instruction_id", "")),
        "instruction_id",
    )
    text = payload.get("text")
    completed_at = payload.get("completed_at")
    if not isinstance(text, str) or not text.strip():
        raise ValueError("回复 text 不能为空")
    if not isinstance(completed_at, str):
        raise ValueError("回复 completed_at 无效")
    try:
        datetime.fromisoformat(completed_at.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("回复 completed_at 无效") from exc
    return {
        "event": "agent.reply.completed",
        "reply_id": reply_id,
        "instruction_id": instruction_id,
        "text": text,
        "completed_at": completed_at,
    }


def _find_place(
    places: list[dict[str, Any]],
    destination: str,
) -> dict[str, Any] | None:
    key = _label_key(destination)
    for place in places:
        keys = {
            _label_key(place["name"]),
            *(_label_key(alias) for alias in place["aliases"]),
        }
        if key in keys:
            return place
    return None


def _clean_label(value: str) -> str:
    return " ".join(unicodedata.normalize("NFKC", value).strip().split())


def _label_key(value: str) -> str:
    return _clean_label(value).casefold()


def _clean_id(value: str, field_name: str) -> str:
    clean = unicodedata.normalize("NFKC", value).strip()
    if not _ID_PATTERN.fullmatch(clean):
        raise ValueError(f"{field_name} 格式无效")
    return clean
