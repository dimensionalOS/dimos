#!/usr/bin/env python3
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

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Literal

from dimos.core.core import rpc
from dimos.core.module import Module

RiskClass = Literal["stop", "low", "medium", "high"]


@dataclass(frozen=True)
class _SkillContract:
    skill_name: str
    domain: str
    module: str
    summary: str
    required_args: tuple[str, ...] = ()
    optional_args: tuple[str, ...] = ()
    arg_types: dict[str, str] | None = None
    motion_sensitive: bool = False
    requires_context: bool = False
    requires_world_state: bool = False
    requires_robot_state: bool = False
    async_updates: bool = False
    risk_class: RiskClass = "low"
    recommended_preflight: tuple[str, ...] = ()
    outcome_shape: str = "str"

    def to_dict(self) -> dict[str, Any]:
        return {
            "skill_name": self.skill_name,
            "domain": self.domain,
            "module": self.module,
            "summary": self.summary,
            "required_args": list(self.required_args),
            "optional_args": list(self.optional_args),
            "arg_types": dict(self.arg_types or {}),
            "motion_sensitive": self.motion_sensitive,
            "requires_context": self.requires_context,
            "requires_world_state": self.requires_world_state,
            "requires_robot_state": self.requires_robot_state,
            "async_updates": self.async_updates,
            "risk_class": self.risk_class,
            "recommended_preflight": list(self.recommended_preflight),
            "outcome_shape": self.outcome_shape,
        }


_CONTEXT_PREFLIGHT = ("route_task", "get_context", "predict_skill_outcome")
_LIGHT_PREFLIGHT = ("route_task",)
_KNOWN_NON_LAYER5_MCP_TOOLS = frozenset(
    {
        "agent_send",
        "get_context",
        "list_modules",
        "predict_skill_outcome",
        "record_causal_transition",
        "record_skill_outcome",
        "route_task",
        "server_status",
        "summarize_causal_patterns",
        "summarize_skill_outcomes",
    }
)

_SKILL_CONTRACTS = (
    _SkillContract(
        skill_name="observe",
        domain="perception",
        module="GO2Connection",
        summary="Return the latest robot camera frame for visual world queries.",
        requires_context=False,
        requires_robot_state=True,
        risk_class="low",
        recommended_preflight=_LIGHT_PREFLIGHT,
        outcome_shape="Image | None",
    ),
    _SkillContract(
        skill_name="tag_location",
        domain="memory",
        module="NavigationSkillContainer",
        summary="Tag the current robot location in spatial memory.",
        required_args=("location_name",),
        arg_types={"location_name": "str"},
        requires_context=True,
        requires_world_state=True,
        requires_robot_state=True,
        risk_class="low",
        recommended_preflight=("get_context",),
    ),
    _SkillContract(
        skill_name="navigate_with_text",
        domain="navigation",
        module="NavigationSkillContainer",
        summary="Navigate to a tagged, visible, or semantic-map location.",
        required_args=("query",),
        arg_types={"query": "str"},
        motion_sensitive=True,
        requires_context=True,
        requires_world_state=True,
        requires_robot_state=True,
        risk_class="medium",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="stop_navigation",
        domain="navigation",
        module="NavigationSkillContainer",
        summary="Cancel the current navigation goal.",
        motion_sensitive=False,
        requires_context=False,
        risk_class="stop",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="begin_exploration",
        domain="exploration",
        module="WavefrontFrontierExplorer",
        summary="Start autonomous frontier exploration of the surrounding area.",
        motion_sensitive=True,
        requires_context=True,
        requires_world_state=True,
        requires_robot_state=True,
        async_updates=True,
        risk_class="high",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="end_exploration",
        domain="exploration",
        module="WavefrontFrontierExplorer",
        summary="Stop active frontier exploration and leave the robot at its current pose.",
        risk_class="stop",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="start_patrol",
        domain="navigation",
        module="PatrollingModule",
        summary="Start autonomous patrolling across known reachable goals.",
        motion_sensitive=True,
        requires_context=True,
        requires_world_state=True,
        requires_robot_state=True,
        async_updates=True,
        risk_class="high",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="stop_patrol",
        domain="navigation",
        module="PatrollingModule",
        summary="Stop the active autonomous patrol.",
        risk_class="stop",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="follow_person",
        domain="person_follow",
        module="PersonFollowSkillContainer",
        summary="Detect and follow a person matching a text description.",
        required_args=("query",),
        optional_args=("initial_bbox", "initial_image"),
        arg_types={"query": "str", "initial_bbox": "list[float]", "initial_image": "str"},
        motion_sensitive=True,
        requires_context=True,
        requires_world_state=True,
        requires_robot_state=True,
        async_updates=True,
        risk_class="medium",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="stop_following",
        domain="person_follow",
        module="PersonFollowSkillContainer",
        summary="Stop the active person-follow behavior.",
        risk_class="stop",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="relative_move",
        domain="robot_motion",
        module="UnitreeSkillContainer",
        summary="Move or rotate the robot relative to its current pose.",
        optional_args=("forward", "left", "degrees"),
        arg_types={"forward": "float", "left": "float", "degrees": "float"},
        motion_sensitive=True,
        requires_context=True,
        requires_robot_state=True,
        risk_class="medium",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="execute_sport_command",
        domain="robot_motion",
        module="UnitreeSkillContainer",
        summary="Execute a named Unitree sport command.",
        required_args=("command_name",),
        arg_types={"command_name": "str"},
        motion_sensitive=True,
        requires_context=True,
        requires_robot_state=True,
        risk_class="high",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="wait",
        domain="utility",
        module="UnitreeSkillContainer",
        summary="Wait for a given number of seconds.",
        required_args=("seconds",),
        arg_types={"seconds": "float"},
        risk_class="low",
        recommended_preflight=_LIGHT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="current_time",
        domain="utility",
        module="UnitreeSkillContainer",
        summary="Return the current local time.",
        risk_class="low",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="look_out_for",
        domain="perception",
        module="PerceiveLoopSkill",
        summary="Continuously look for visual targets and optionally dispatch a follow-up tool.",
        required_args=("description_of_things",),
        optional_args=("then",),
        arg_types={"description_of_things": "list[str]", "then": "dict"},
        requires_context=True,
        requires_world_state=True,
        async_updates=True,
        risk_class="medium",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="stop_looking_out",
        domain="perception",
        module="PerceiveLoopSkill",
        summary="Stop the active visual lookout.",
        risk_class="stop",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="start_security_patrol",
        domain="security",
        module="SecurityModule",
        summary="Start patrol, person detection, and automatic following.",
        motion_sensitive=True,
        requires_context=True,
        requires_world_state=True,
        requires_robot_state=True,
        async_updates=True,
        risk_class="high",
        recommended_preflight=_CONTEXT_PREFLIGHT,
    ),
    _SkillContract(
        skill_name="stop_security_patrol",
        domain="security",
        module="SecurityModule",
        summary="Stop the active security patrol behavior.",
        risk_class="stop",
        recommended_preflight=(),
    ),
    _SkillContract(
        skill_name="speak",
        domain="speech",
        module="SpeakSkill",
        summary="Speak text through the robot speakers.",
        required_args=("text",),
        optional_args=("blocking",),
        arg_types={"text": "str", "blocking": "bool"},
        risk_class="low",
        recommended_preflight=_LIGHT_PREFLIGHT,
    ),
)


class _Go2SkillInterfaceRegistry(Module):
    """Layer 5 registry for the Go2 MCP-callable skill interface.

    This registry is static in V1. It documents the skills that the current Go2
    blueprint composes, and gives upper layers a structured contract for
    validation and planning. It does not execute skills or wrap the existing
    skill containers.
    """

    @rpc
    def get_skill_interface_snapshot(self, domain: str = "") -> dict[str, Any]:
        """Return the Go2 Layer 5 skill contracts, optionally filtered by domain."""
        domain = domain.strip()
        contracts = [
            contract.to_dict()
            for contract in _SKILL_CONTRACTS
            if not domain or contract.domain == domain
        ]
        return {
            "available": True,
            "version": "v2",
            "source": "static_go2_layer5_contracts",
            "domain_filter": domain,
            "domains": sorted({contract["domain"] for contract in contracts}),
            "skill_count": len(contracts),
            "skills": contracts,
        }

    @rpc
    def get_skill_contract(self, skill_name: str) -> dict[str, Any] | None:
        """Return one Go2 skill contract by MCP tool name."""
        name = skill_name.strip()
        contract = _contract_by_name(name)
        if contract is None:
            return None
        return contract.to_dict()

    @rpc
    def validate_skill_request(self, skill_name: str, args_json: str = "{}") -> dict[str, Any]:
        """Validate a planned skill call against the Layer 5 contract."""
        name = skill_name.strip()
        contract = _contract_by_name(name)
        if contract is None:
            return {
                "valid": False,
                "skill_name": name,
                "errors": [f"unknown skill: {name}"],
                "warnings": [],
                "contract": None,
            }

        args, parse_error = _parse_args(args_json)
        errors: list[str] = []
        warnings: list[str] = []
        if parse_error:
            errors.append(parse_error)

        if args is not None:
            for required in contract.required_args:
                if _is_missing(args.get(required)):
                    errors.append(f"missing required argument: {required}")

            arg_types = contract.arg_types or {}
            for arg_name, expected_type in arg_types.items():
                if arg_name in args and not _matches_type(args[arg_name], expected_type):
                    errors.append(f"argument {arg_name} should be {expected_type}")

        if contract.requires_context:
            warnings.append("call get_context before this skill when task context is non-trivial")
        if contract.motion_sensitive:
            warnings.append("call predict_skill_outcome before executing this motion-sensitive skill")

        return {
            "valid": not errors,
            "skill_name": name,
            "errors": errors,
            "warnings": warnings,
            "contract": contract.to_dict(),
        }

    @rpc
    def compare_mcp_tools(self, tools_json: str) -> dict[str, Any]:
        """Compare Layer 5 contracts with an MCP tools/list payload."""
        tool_names, parse_error = _parse_mcp_tool_names(tools_json)
        contract_names = {contract.skill_name for contract in _SKILL_CONTRACTS}
        errors: list[str] = []
        if parse_error:
            errors.append(parse_error)

        mcp_names = set(tool_names)
        known_internal = sorted(mcp_names & _KNOWN_NON_LAYER5_MCP_TOOLS)
        missing_contracts = sorted(contract_names - mcp_names) if tool_names else []
        unregistered_mcp_tools = sorted(
            mcp_names - contract_names - _KNOWN_NON_LAYER5_MCP_TOOLS
        )

        return {
            "valid": not errors and not missing_contracts and not unregistered_mcp_tools,
            "errors": errors,
            "contract_skill_count": len(contract_names),
            "mcp_tool_count": len(mcp_names),
            "contract_skill_names": sorted(contract_names),
            "mcp_tool_names": sorted(mcp_names),
            "known_non_layer5_mcp_tools": known_internal,
            "missing_contracts": missing_contracts,
            "unregistered_mcp_tools": unregistered_mcp_tools,
        }


def _contract_by_name(skill_name: str) -> _SkillContract | None:
    for contract in _SKILL_CONTRACTS:
        if contract.skill_name == skill_name:
            return contract
    return None


def _parse_args(args_json: str) -> tuple[dict[str, Any] | None, str | None]:
    if not args_json.strip():
        return {}, None
    try:
        parsed = json.loads(args_json)
    except json.JSONDecodeError as exc:
        return None, f"args_json is invalid JSON: {exc.msg}"
    if not isinstance(parsed, dict):
        return None, "args_json must be a JSON object"
    return parsed, None


def _parse_mcp_tool_names(tools_json: str) -> tuple[list[str], str | None]:
    if not tools_json.strip():
        return [], "tools_json is required"
    try:
        parsed = json.loads(tools_json)
    except json.JSONDecodeError as exc:
        return [], f"tools_json is invalid JSON: {exc.msg}"

    tools_payload: Any = parsed
    if isinstance(parsed, dict):
        if isinstance(parsed.get("result"), dict):
            tools_payload = parsed["result"].get("tools", [])
        elif "tools" in parsed:
            tools_payload = parsed["tools"]
        elif "skills" in parsed:
            tools_payload = parsed["skills"]

    names: list[str] = []
    if isinstance(tools_payload, list):
        for item in tools_payload:
            if isinstance(item, str):
                names.append(item)
            elif isinstance(item, dict) and isinstance(item.get("name"), str):
                names.append(item["name"])
            elif isinstance(item, dict) and isinstance(item.get("func_name"), str):
                names.append(item["func_name"])
    else:
        return [], "tools_json must contain a list of tool names or tool objects"

    return sorted(set(names)), None


def _is_missing(value: Any) -> bool:
    if value is None:
        return True
    if isinstance(value, str) and not value.strip():
        return True
    if isinstance(value, list) and not value:
        return True
    return False


def _matches_type(value: Any, expected_type: str) -> bool:
    if _is_missing(value):
        return True
    if expected_type == "str":
        return isinstance(value, str)
    if expected_type == "bool":
        return isinstance(value, bool)
    if expected_type == "float":
        return isinstance(value, int | float) and not isinstance(value, bool)
    if expected_type == "list[float]":
        return isinstance(value, list) and all(
            isinstance(item, int | float) and not isinstance(item, bool) for item in value
        )
    if expected_type == "list[str]":
        return isinstance(value, list) and all(isinstance(item, str) for item in value)
    if expected_type == "dict":
        return isinstance(value, dict)
    return True


__all__ = ["_Go2SkillInterfaceRegistry"]
