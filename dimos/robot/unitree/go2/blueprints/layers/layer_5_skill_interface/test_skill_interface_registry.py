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
        assert snapshot["version"] == "v1"
        assert "navigation" in snapshot["domains"]
        assert "robot_motion" in snapshot["domains"]
        assert "navigate_with_text" in skill_names
        assert "follow_person" in skill_names
        assert "speak" in skill_names
    finally:
        _stop_modules(registry)


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
