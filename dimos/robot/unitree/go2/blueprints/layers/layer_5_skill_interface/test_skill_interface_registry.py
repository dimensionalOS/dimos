# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
from typing import Any

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.unitree.go2.blueprints.layers.layer_5_skill_interface.skill_interface_registry import (
    _Go2SkillInterfaceRegistry,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


def test_snapshot_lists_go2_skill_contracts() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        snapshot = registry.get_skill_interface_snapshot()

        skill_names = {skill["skill_name"] for skill in snapshot["skills"]}
        assert snapshot["available"] is True
        assert snapshot["version"] == "v2"
        assert "navigation" in snapshot["domains"]
        assert "robot_motion" in snapshot["domains"]
        assert "observe" in skill_names
        assert "navigate_with_text" in skill_names
        assert "follow_person" in skill_names
        assert "speak" in skill_names
    finally:
        _stop_modules(registry)


def test_compare_mcp_tools_accepts_matching_mcp_tool_payload() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        snapshot = registry.get_skill_interface_snapshot()
        tool_names = [skill["skill_name"] for skill in snapshot["skills"]]
        payload = {
            "result": {
                "tools": [{"name": name, "inputSchema": {"type": "object"}} for name in tool_names]
                + [{"name": "get_context"}, {"name": "route_task"}, {"name": "server_status"}]
            }
        }

        result = registry.compare_mcp_tools(json.dumps(payload))

        assert result["valid"] is True
        assert result["missing_contracts"] == []
        assert result["unregistered_mcp_tools"] == []
        assert "get_context" in result["known_non_layer5_mcp_tools"]
    finally:
        _stop_modules(registry)


def test_compare_mcp_tools_reports_missing_and_unregistered_tools() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        payload = {"tools": [{"name": "observe"}, {"name": "unexpected_tool"}]}

        result = registry.compare_mcp_tools(json.dumps(payload))

        assert result["valid"] is False
        assert "navigate_with_text" in result["missing_contracts"]
        assert result["unregistered_mcp_tools"] == ["unexpected_tool"]
    finally:
        _stop_modules(registry)


def test_go2_agentic_mcp_tools_match_layer5_contracts() -> None:
    try:
        from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import (
            unitree_go2_agentic,
        )
    except (ImportError, ModuleNotFoundError, OSError) as exc:
        pytest.skip(f"Full Go2 agentic blueprint dependencies are unavailable: {exc}")

    registry = _Go2SkillInterfaceRegistry()
    try:
        mcp_tool_names = _static_mcp_tool_names(unitree_go2_agentic)
        payload = {"tools": [{"name": name} for name in sorted(mcp_tool_names)]}

        result = registry.compare_mcp_tools(json.dumps(payload))

        assert result["valid"] is True
        assert result["missing_contracts"] == []
        assert result["unregistered_mcp_tools"] == []
        assert "observe" in result["mcp_tool_names"]
        assert "server_status" in result["known_non_layer5_mcp_tools"]
    finally:
        _stop_modules(registry)


def _static_mcp_tool_names(blueprint: Blueprint) -> set[str]:
    tool_names: set[str] = set()
    for atom in blueprint.active_blueprints:
        for name in dir(atom.module):
            attr: Any = getattr(atom.module, name)
            if callable(attr) and hasattr(attr, "__skill__"):
                tool_names.add(name)
    return tool_names


def test_snapshot_can_filter_by_domain() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        snapshot = registry.get_skill_interface_snapshot(domain="navigation")

        assert snapshot["domain_filter"] == "navigation"
        assert snapshot["domains"] == ["navigation"]
        assert {skill["domain"] for skill in snapshot["skills"]} == {"navigation"}
    finally:
        _stop_modules(registry)


def test_get_skill_contract_returns_motion_sensitive_metadata() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        contract = registry.get_skill_contract("follow_person")

        assert contract is not None
        assert contract["domain"] == "person_follow"
        assert contract["required_args"] == ["query"]
        assert contract["motion_sensitive"] is True
        assert "predict_skill_outcome" in contract["recommended_preflight"]
    finally:
        _stop_modules(registry)


def test_validate_skill_request_rejects_unknown_skill() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        result = registry.validate_skill_request("not_a_skill", "{}")

        assert result["valid"] is False
        assert result["contract"] is None
        assert result["errors"] == ["unknown skill: not_a_skill"]
    finally:
        _stop_modules(registry)


def test_validate_skill_request_checks_required_arguments() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        result = registry.validate_skill_request("navigate_with_text", "{}")

        assert result["valid"] is False
        assert "missing required argument: query" in result["errors"]
        assert any("get_context" in warning for warning in result["warnings"])
    finally:
        _stop_modules(registry)


def test_validate_skill_request_accepts_safe_speech_call() -> None:
    registry = _Go2SkillInterfaceRegistry()
    try:
        result = registry.validate_skill_request(
            "speak", '{"text": "hello", "blocking": false}'
        )

        assert result["valid"] is True
        assert result["errors"] == []
        assert result["contract"]["risk_class"] == "low"
    finally:
        _stop_modules(registry)
