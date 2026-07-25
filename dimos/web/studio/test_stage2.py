# ruff: noqa: RUF001
"""Tests for the Stage 2 Studio supervision boundary."""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
from typing import Any

import pytest

from dimos.web.studio.stage2 import StageTwoControl


class FakeMcp:
    def __init__(self, payloads: dict[str, dict[str, Any]]) -> None:
        self.payloads = payloads
        self.calls: list[tuple[str, dict[str, Any]]] = []

    def call_tool_text(
        self,
        name: str,
        arguments: dict[str, Any] | None = None,
    ) -> str:
        self.calls.append((name, arguments or {}))
        payload = self.payloads[name]
        return json.dumps(payload, ensure_ascii=False)


def stage_two_payloads() -> dict[str, dict[str, Any]]:
    return {
        "list_semantic_places": {
            "map_id": "venue-a",
            "map_version": "v1",
            "places": [
                {
                    "entity_id": "place-start-001",
                    "name": "测试起点",
                    "aliases": ["起点"],
                    "map_id": "venue-a",
                    "map_version": "v1",
                    "pose": {"frame_id": "map", "x": 0.0, "y": 0.0},
                },
                {
                    "entity_id": "place-door-001",
                    "name": "门口测试点",
                    "aliases": ["演示点"],
                    "map_id": "venue-a",
                    "map_version": "v1",
                    "pose": {"frame_id": "map", "x": 2.0, "y": 0.5},
                },
            ],
        },
        "get_task_status": {
            "task": {
                "task_id": "task-stage2-0001",
                "kind": "go_to_place",
                "destination": "门口测试点",
                "target_description": None,
                "question": None,
                "priority": "normal",
                "created_at": "2026-07-25T06:00:00Z",
            },
            "state": "navigating",
            "active": True,
        },
        "get_robot_summary": {
            "status": "ready",
            "odometry": {
                "available": True,
                "fresh": True,
                "age_s": 0.1,
                "frame_id": "map",
                "latest_sample_at": "2026-07-25T06:00:01Z",
            },
            "planned_path_frame_id": "map",
            "latest_pose": {
                "x": 0.4,
                "y": 0.1,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
                "source_ts": 100.5,
                "frame_id": "map",
            },
            "planned_path": [
                {"x": 0.0, "y": 0.0},
                {"x": 2.0, "y": 0.5},
            ],
            "actual_path": [
                {"x": 0.0, "y": 0.0},
                {"x": 0.4, "y": 0.1},
            ],
            "recovery": {
                "attempt": 1,
                "cause": "obstacle",
                "action": "rotate_rescan",
                "outcome": "dispatched",
                "reason": "temporary obstacle",
                "timestamp": 100.0,
            },
        },
    }


def make_control(
    tmp_path: Path,
    *,
    submitter: Callable[[str, str], dict[str, Any]] | None = None,
) -> tuple[StageTwoControl, FakeMcp]:
    mcp = FakeMcp(stage_two_payloads())
    return (
        StageTwoControl(
            state_path=tmp_path / "stage2.json",
            mcp=mcp,
            gateway_submitter=submitter,
        ),
        mcp,
    )


def test_status_keeps_planned_and_actual_paths_separate(tmp_path: Path) -> None:
    control, mcp = make_control(tmp_path)

    status = control.status()

    assert status["connected"] is True
    assert status["semantic_world"]["map_id"] == "venue-a"
    assert [place["name"] for place in status["semantic_world"]["places"]] == [
        "测试起点",
        "门口测试点",
    ]
    assert status["task"]["task"]["task_id"] == "task-stage2-0001"
    assert status["task"]["state"] == "navigating"
    assert status["telemetry"]["planned_path"][-1] == {"x": 2.0, "y": 0.5}
    assert status["telemetry"]["actual_path"][-1] == {"x": 0.4, "y": 0.1}
    assert status["telemetry"]["recovery"]["attempt"] == 1
    assert [name for name, _ in mcp.calls] == [
        "list_semantic_places",
        "get_task_status",
        "get_robot_summary",
    ]


def test_submit_requires_a_confirmed_current_map_place_and_keeps_id(
    tmp_path: Path,
) -> None:
    submissions: list[tuple[str, str]] = []

    def submitter(instruction_id: str, text: str) -> dict[str, Any]:
        submissions.append((instruction_id, text))
        return {"instruction_id": instruction_id, "status": "accepted"}

    control, _mcp = make_control(tmp_path, submitter=submitter)

    accepted = control.navigate("studio-instruction-0001", " 演示点 ")

    assert accepted["accepted"] is True
    assert submissions == [("studio-instruction-0001", "去门口测试点")]
    assert control.status()["last_submission"]["instruction_id"] == (
        "studio-instruction-0001"
    )
    local = json.loads((tmp_path / "stage2.json").read_text(encoding="utf-8"))
    assert set(local) == {
        "schema_version",
        "last_submission",
        "last_reply",
        "replies",
    }
    assert "task" not in local
    assert "robot" not in local

    with pytest.raises(ValueError, match="未确认"):
        control.navigate("studio-instruction-0002", "不存在的房间")
    assert len(submissions) == 1


def test_cancel_forwards_the_exact_canonical_task_id(tmp_path: Path) -> None:
    control, mcp = make_control(tmp_path)
    mcp.payloads["cancel_task"] = {
        "task": stage_two_payloads()["get_task_status"]["task"],
        "state": "cancelled",
        "active": False,
        "terminal_reason": "operator cancelled task",
        "navigation_idle": True,
    }

    cancelled = control.cancel("task-stage2-0001")

    assert cancelled["state"] == "cancelled"
    assert mcp.calls[-1] == (
        "cancel_task",
        {"task_id": "task-stage2-0001"},
    )


def test_stop_all_calls_the_canonical_wrapper_without_local_state(
    tmp_path: Path,
) -> None:
    control, mcp = make_control(tmp_path)
    mcp.payloads["stop_all"] = {
        "status": "stopped",
        "failed_components": [],
        "results": {"navigation": {"status": "stopped"}},
    }

    stopped = control.stop_all()

    assert stopped["status"] == "stopped"
    assert mcp.calls[-1] == ("stop_all", {})
    assert not (tmp_path / "stage2.json").exists()


def test_confirm_current_place_requires_fresh_odometry_and_forwards_strict_pose(
    tmp_path: Path,
) -> None:
    control, mcp = make_control(tmp_path)
    mcp.payloads["confirm_semantic_place"] = {
        "accepted": True,
        "place": {
            "entity_id": "place-current-001",
            "name": "测试起点",
            "aliases": ["起点"],
            "map_id": "venue-a",
            "map_version": "v1",
        },
    }

    result = control.confirm_current_place(" 测试起点 ", [" 起点 ", "起点"])

    assert result["accepted"] is True
    name, arguments = mcp.calls[-1]
    assert name == "confirm_semantic_place"
    place = json.loads(arguments["place_json"])
    assert place == {
        "name": "测试起点",
        "aliases": ["起点"],
        "pose": {
            "frame_id": "map",
            "ts": 100.5,
            "x": 0.4,
            "y": 0.1,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },
    }

    mcp.payloads["get_robot_summary"]["odometry"]["fresh"] = False
    with pytest.raises(ValueError, match="里程计不是 fresh"):
        control.confirm_current_place("门口", [])


def test_reply_is_persisted_idempotently_and_conflicts_fail_closed(
    tmp_path: Path,
) -> None:
    control, _mcp = make_control(tmp_path)
    reply = {
        "event": "agent.reply.completed",
        "reply_id": "reply-stage2-0001",
        "instruction_id": "studio-instruction-0001",
        "text": "任务已完成：已到达“门口测试点”。",
        "completed_at": "2026-07-25T06:00:02Z",
    }

    assert control.record_reply(reply)["duplicate"] is False
    assert control.record_reply(reply)["duplicate"] is True
    assert control.status()["last_reply"] == reply

    newer = {
        **reply,
        "reply_id": "reply-stage2-0002",
        "instruction_id": "studio-instruction-0002",
    }
    assert control.record_reply(newer)["duplicate"] is False
    assert control.record_reply(reply)["duplicate"] is True
    assert control.status()["last_reply"] == newer

    with pytest.raises(ValueError, match="冲突"):
        control.record_reply({**reply, "text": "different"})


def test_unavailable_wrapper_is_not_reported_as_idle(tmp_path: Path) -> None:
    control, mcp = make_control(tmp_path)

    def unavailable(
        name: str,
        arguments: dict[str, Any] | None = None,
    ) -> str:
        del name, arguments
        raise ConnectionError("wrapper offline")

    mcp.call_tool_text = unavailable

    status = control.status()

    assert status["connected"] is False
    assert status["task"]["state"] == "unavailable"
    assert status["errors"]
