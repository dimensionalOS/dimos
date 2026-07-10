#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

from dimos.core.coordination.blueprints import Blueprint

__all__ = ["_go2_agent_brain", "_go2_agent_brain_with_client"]

_LAYER_BLUEPRINTS: dict[str, Any] = {}


def __getattr__(name: str) -> Any:
    if name not in __all__:
        raise AttributeError(name)
    if name == "_go2_agent_brain_with_client":
        return _go2_agent_brain_with_client
    _build_layer_blueprints()
    return _LAYER_BLUEPRINTS[name]


def _go2_agent_brain_with_client(**mcp_client_kwargs: Any) -> Blueprint:
    """Layer 3: expert routing, context, MCP tools, and the LLM/VLM agent."""
    from dimos.agents.mcp.mcp_client import McpClient
    from dimos.agents.mcp.mcp_server import McpServer
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.causal_world_model import (
        _Go2CausalWorldModel,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_feedback import (
        _Go2ContextFeedbackStore,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_provider import (
        _Go2ContextProvider,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_ledger import (
        _Go2EvolutionLedger,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.expert_router import (
        _Go2ExpertRouter,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.memory_backend_status import (
        _Go2MemoryBackendStatus,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.prompt_policy import (
        _go2_layer_3_system_prompt,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_predictor import (
        _Go2SkillOutcomePredictor,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
        _Go2SkillOutcomeStore,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_proposal import (
        _Go2SkillProposalGenerator,
    )
    from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.task_feasibility import (
        _Go2TaskFeasibilityEvaluator,
    )

    client_kwargs = dict(mcp_client_kwargs)
    client_kwargs["system_prompt"] = _go2_layer_3_system_prompt(client_kwargs.get("system_prompt"))
    return autoconnect(
        _Go2ExpertRouter.blueprint(),
        _Go2SkillOutcomeStore.blueprint(),
        _Go2CausalWorldModel.blueprint(),
        _Go2ContextFeedbackStore.blueprint(),
        _Go2SkillOutcomePredictor.blueprint(),
        _Go2ContextProvider.blueprint(),
        _Go2MemoryBackendStatus.blueprint(),
        _Go2EvolutionLedger.blueprint(),
        _Go2TaskFeasibilityEvaluator.blueprint(),
        _Go2SkillProposalGenerator.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(**client_kwargs),
    )


def _build_layer_blueprints() -> None:
    if _LAYER_BLUEPRINTS:
        return
    _LAYER_BLUEPRINTS["_go2_agent_brain"] = _go2_agent_brain_with_client()
    globals().update(_LAYER_BLUEPRINTS)
