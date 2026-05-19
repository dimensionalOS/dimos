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

from dataclasses import dataclass

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module


@dataclass(frozen=True)
class _RouteRule:
    domain: str
    keywords: tuple[str, ...]
    tools: tuple[str, ...]
    reason: str
    needs_context: bool = True


_ROUTE_RULES: tuple[_RouteRule, ...] = (
    _RouteRule(
        domain="safety",
        keywords=(
            "stop",
            "halt",
            "cancel",
            "emergency",
            "danger",
            "unsafe",
            "\u505c\u6b62",
            "\u505c\u4e0b",
            "\u53d6\u6d88",
            "\u5371\u9669",
            "\u7d27\u6025",
            "\u907f\u9669",
        ),
        tools=("stop_navigation", "stop_following", "stop_looking_out", "stop_security_patrol"),
        reason="The task describes stopping, cancelling, or handling a safety condition.",
        needs_context=False,
    ),
    _RouteRule(
        domain="speech",
        keywords=(
            "say",
            "speak",
            "tell",
            "announce",
            "reply",
            "\u8bf4",
            "\u544a\u8bc9",
            "\u56de\u7b54",
            "\u64ad\u62a5",
        ),
        tools=("speak",),
        reason="The task asks the robot to communicate with a person.",
        needs_context=False,
    ),
    _RouteRule(
        domain="memory",
        keywords=(
            "remember",
            "memory",
            "recall",
            "where did",
            "previous",
            "last time",
            "\u8bb0\u4f4f",
            "\u8bb0\u5f97",
            "\u56de\u5fc6",
            "\u4e4b\u524d",
            "\u521a\u624d",
            "\u4e0a\u6b21",
            "\u54ea\u91cc",
        ),
        tools=("get_context", "query", "tag_location"),
        reason="The task depends on remembered locations, events, or prior observations.",
    ),
    _RouteRule(
        domain="person_follow",
        keywords=(
            "follow",
            "track person",
            "follow person",
            "person wearing",
            "\u8ddf\u968f",
            "\u8ddf\u7740",
            "\u8ffd\u8e2a\u4eba",
            "\u8ddf\u4eba",
            "\u7a7f",
        ),
        tools=("get_context", "look_out_for", "follow_person", "stop_following"),
        reason="The task asks the robot to visually identify and follow a person.",
    ),
    _RouteRule(
        domain="perception",
        keywords=(
            "look",
            "watch",
            "detect",
            "find object",
            "find person",
            "see",
            "\u770b\u5230",
            "\u5bfb\u627e",
            "\u68c0\u6d4b",
            "\u89c2\u5bdf",
            "\u8bc6\u522b",
            "\u627e\u4eba",
            "\u627e\u7269\u4f53",
        ),
        tools=("get_context", "look_out_for", "stop_looking_out"),
        reason="The task primarily requires visual perception or object/person detection.",
    ),
    _RouteRule(
        domain="navigation",
        keywords=(
            "go to",
            "navigate",
            "walk to",
            "move to",
            "go find",
            "room",
            "hallway",
            "kitchen",
            "\u5230",
            "\u53bb",
            "\u5bfc\u822a",
            "\u8d70\u5230",
            "\u79fb\u52a8\u5230",
            "\u623f\u95f4",
            "\u8d70\u5eca",
            "\u53a8\u623f",
        ),
        tools=("get_context", "navigate_with_text", "relative_move", "stop_navigation"),
        reason="The task asks the robot to reach a place or move toward a target.",
    ),
    _RouteRule(
        domain="security",
        keywords=(
            "patrol",
            "guard",
            "security",
            "\u5de1\u903b",
            "\u5b89\u4fdd",
            "\u770b\u5b88",
            "\u8b66\u6212",
        ),
        tools=("get_context", "start_security_patrol", "stop_security_patrol"),
        reason="The task maps to the security patrol workflow.",
    ),
    _RouteRule(
        domain="robot_motion",
        keywords=(
            "forward",
            "back",
            "left",
            "right",
            "turn",
            "rotate",
            "jump",
            "dance",
            "\u524d\u8fdb",
            "\u540e\u9000",
            "\u5de6",
            "\u53f3",
            "\u8f6c",
            "\u8df3",
            "\u8df3\u821e",
        ),
        tools=("relative_move", "execute_sport_command", "wait"),
        reason="The task is a direct robot-body movement or Unitree sport command.",
        needs_context=False,
    ),
    _RouteRule(
        domain="utility",
        keywords=(
            "time",
            "wait",
            "sleep",
            "\u73b0\u5728\u51e0\u70b9",
            "\u65f6\u95f4",
            "\u7b49\u5f85",
        ),
        tools=("current_time", "wait"),
        reason="The task asks for a utility action that does not require world context.",
        needs_context=False,
    ),
    _RouteRule(
        domain="human_help",
        keywords=(
            "ask human",
            "human help",
            "clarify",
            "confirm",
            "\u95ee\u4eba",
            "\u786e\u8ba4",
            "\u6f84\u6e05",
        ),
        tools=("speak",),
        reason="The task is underspecified or needs explicit human confirmation.",
        needs_context=False,
    ),
)


class _Go2ExpertRouter(Module):
    """Rule-based Layer 3 router for Go2 agent tasks."""

    @skill
    def route_task(self, task: str, context: str = "") -> SkillResult:
        """Route a task to the most relevant Go2 expert domain.

        This tool classifies the current task and recommends MCP tools. It does
        not execute robot actions. Use it before selecting navigation,
        perception, speech, safety, memory, or direct movement tools.

        Args:
            task: The current user goal or agent subtask.
            context: Optional context summary from get_context or the dialogue.
        """
        task = task.strip()
        context = context.strip()
        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")

        route, matches = _select_route(task, context)
        needs_context = route.needs_context and not context
        tools = _recommended_tools(route, needs_context)
        confidence = _confidence(matches)

        metadata = {
            "task": task,
            "context_used": bool(context),
            "domain": route.domain,
            "confidence": confidence,
            "matched_keywords": matches,
            "recommended_tools": tools,
            "needs_context": needs_context,
            "reason": route.reason,
        }

        message = (
            f"Route domain={route.domain}, confidence={confidence}. "
            f"Recommended tools: {', '.join(tools)}. {route.reason}"
        )
        return SkillResult(success=True, message=message, metadata=metadata)


def _select_route(task: str, context: str) -> tuple[_RouteRule, list[str]]:
    text = f"{task} {context}".casefold()
    best_rule = _default_route()
    best_matches: list[str] = []

    for rule in _ROUTE_RULES:
        matches = [keyword for keyword in rule.keywords if keyword.casefold() in text]
        if len(matches) > len(best_matches):
            best_rule = rule
            best_matches = matches

    return best_rule, best_matches


def _default_route() -> _RouteRule:
    return _RouteRule(
        domain="general",
        keywords=(),
        tools=("get_context",),
        reason="No domain-specific rule matched; gather context before choosing tools.",
        needs_context=True,
    )


def _recommended_tools(route: _RouteRule, needs_context: bool) -> list[str]:
    tools = list(route.tools)
    if needs_context and "get_context" not in tools:
        tools.insert(0, "get_context")
    return tools


def _confidence(matches: list[str]) -> str:
    if len(matches) >= 2:
        return "high"
    if len(matches) == 1:
        return "medium"
    return "low"


__all__ = ["_Go2ExpertRouter"]
