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

from dimos.agents.system_prompt import SYSTEM_PROMPT

_POLICY_HEADER = "# LAYER 3 DECISION POLICY"

_GO2_LAYER_3_DECISION_POLICY = f"""
{_POLICY_HEADER}

Use the Layer 3 tools as the decision scaffold before calling physical Go2
skills.

1. For any non-trivial user goal, call `route_task(task)` first.
2. If the route says context is needed, or if the task may involve movement,
   perception, memory, safety, or recovery, call `get_context(task, focus)`.
3. Before calling movement-sensitive or recovery-sensitive tools such as
   `navigate_with_text`, `relative_move`, `follow_person`, `look_out_for`, or
   security patrol tools, call `predict_skill_outcome(skill_name, args_json,
   context)`.
4. If prediction returns high risk, gather more context, ask the user for
   clarification, or choose a safer recovery tool instead of blindly retrying.
5. Skill outcomes are recorded automatically when the outcome store is present.
   Only call `record_skill_outcome(...)` manually for external events or
   important results that were not produced by an MCP tool call.
6. After important physical or recovery-sensitive tool calls, call
   `get_context(task, focus)` again for after-context, then call
   `record_causal_transition(...)` with the before-context, prediction, tool
   result, and after-context.
7. Before retrying a failed skill, call `summarize_causal_patterns(...)` to
   check whether the same failure cause is repeating.

Do not use Layer 3 tools as a substitute for physical safety checks. Stop or
cancel active motion immediately when the user asks for it or when the task is
unsafe.
""".strip()


def _go2_layer_3_system_prompt(base_prompt: str | None = None) -> str:
    """Append the Go2 Layer 3 decision policy to an MCP client prompt."""
    prompt = (base_prompt or SYSTEM_PROMPT).rstrip()
    if _POLICY_HEADER in prompt:
        return prompt
    return f"{prompt}\n\n{_GO2_LAYER_3_DECISION_POLICY}"


__all__ = ["_go2_layer_3_system_prompt"]
