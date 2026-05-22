# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""LangChain agent wired to dimos.memory2 tools.

One system prompt, the memory2 tool surface, a single ``agent.invoke``
per question.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Annotated, Any

from langchain.agents import AgentState, create_agent
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, ToolMessage
from langgraph.graph.message import add_messages

from dimos.memory2.experimental.memory2_agent import skills_registry
from dimos.memory2.experimental.memory2_agent.llm import build_chat_model
from dimos.memory2.experimental.memory2_agent.tools import build_tools
from dimos.memory2.store.sqlite import SqliteStore
from dimos.models.embedding.clip import CLIPModel

SYSTEM_PROMPT = (
    "You're answering questions about a robot's memory store. The robot "
    "is a Unitree Go2 quadruped that walked for about 60 seconds, "
    "recording ego-centric camera frames, lidar, and odometry. There is "
    "no semantic map; everything you know about the place comes from "
    "the recorded sensors.\n\n"
    "Be honest. Only state what the tools actually return — don't "
    "invent counts, timestamps, or labels. If the data doesn't have "
    "what's needed, say so. Match answer length to the question: "
    "single-number questions get one number; 'describe' or 'where' "
    "questions get one or two sentences. When you're done, end with a "
    "plain text reply to the user — no tool call for that final reply."
)


@dataclass
class AgentRun:
    final_answer: str
    tool_calls: list[dict] = field(default_factory=list)
    iterations: int = 0
    error: str | None = None


def _extract_text(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts: list[str] = []
        for block in content:
            if isinstance(block, dict) and block.get("type") == "text":
                t = block.get("text", "")
                if isinstance(t, str):
                    parts.append(t)
            elif isinstance(block, str):
                parts.append(block)
        return "".join(parts)
    return str(content)


def _fix_parallel_tool_batches(messages: list[BaseMessage]) -> list[BaseMessage]:
    """After a state merge, find places where the agent's parallel tool
    calls produced interleaved [ToolMessage, HumanMessage, ToolMessage,
    HumanMessage, ...] and reorder them to [Tool, Tool, ..., Human,
    Human, ...] so OpenAI's "all parallel tool responses must be
    contiguous" rule is satisfied.

    Each image-carrying HumanMessage emitted by our tools is tagged
    with `additional_kwargs["tool_call_id"]` matching the originating
    tool call, so we can confidently identify which Humans belong to
    which parallel batch.
    """
    out = list(messages)
    i = 0
    while i < len(out):
        msg = out[i]
        tool_calls = getattr(msg, "tool_calls", None) or []
        if isinstance(msg, AIMessage) and len(tool_calls) >= 2:
            expected_ids = {tc.get("id") for tc in tool_calls if tc.get("id")}
            tool_msgs: list[BaseMessage] = []
            other_msgs: list[BaseMessage] = []
            j = i + 1
            while j < len(out):
                m = out[j]
                if isinstance(m, ToolMessage) and m.tool_call_id in expected_ids:
                    tool_msgs.append(m)
                    j += 1
                elif (
                    isinstance(m, HumanMessage)
                    and getattr(m, "additional_kwargs", {}).get("tool_call_id") in expected_ids
                ):
                    other_msgs.append(m)
                    j += 1
                else:
                    break
            if tool_msgs and other_msgs and {m.tool_call_id for m in tool_msgs} == expected_ids:
                out[i + 1 : j] = [*tool_msgs, *other_msgs]
        i += 1
    return out


def _reorder_tool_responses(
    left: list[BaseMessage], right: list[BaseMessage] | BaseMessage
) -> list[BaseMessage]:
    """Standard add_messages merge, then fix any parallel-tool batches
    that ended up interleaved."""
    merged = add_messages(left, right)
    return _fix_parallel_tool_batches(merged)


class _OrderedAgentState(AgentState):
    # Override the messages reducer to keep parallel ToolMessages contiguous.
    messages: Annotated[list[BaseMessage], _reorder_tool_responses]


_PRESEED_TOOL_CALL_ID = "preseed_list_skills"


def _skills_listing() -> str:
    """Render the same output `list_skills()` would produce."""
    skills = skills_registry.list_skills()
    if not skills:
        return "No skills available."
    body = "\n".join(f"  - {s.name}: {s.description}" for s in skills)
    return "Available skills:\n" + body


def run_question(
    store: SqliteStore,
    clip: CLIPModel,
    question: str,
    *,
    model: str = "gpt-4.1-mini",
    temperature: float = 0.0,
) -> AgentRun:
    """Run one question against the memory2 agent.

    The conversation is pre-seeded with a synthetic `list_skills` tool
    call so every run starts with the available skills already visible
    in context — no need for the agent to remember to call it.
    """
    tools, state = build_tools(store, clip)
    llm = build_chat_model(model, temperature)
    agent = create_agent(
        model=llm,
        tools=tools,
        system_prompt=SYSTEM_PROMPT,
        state_schema=_OrderedAgentState,
    )

    preseed = [
        HumanMessage(content=f"Question: {question}"),
        AIMessage(
            content="",
            tool_calls=[
                {
                    "name": "list_skills",
                    "args": {},
                    "id": _PRESEED_TOOL_CALL_ID,
                }
            ],
        ),
        ToolMessage(content=_skills_listing(), tool_call_id=_PRESEED_TOOL_CALL_ID),
    ]

    try:
        result = agent.invoke({"messages": preseed})
    except Exception as e:
        return AgentRun(final_answer="", error=f"agent error: {e}")

    # Collect tool calls from the conversation.
    tool_calls: list[dict] = []
    for msg in result.get("messages", []):
        for tc in getattr(msg, "tool_calls", None) or []:
            tool_calls.append({"name": tc.get("name"), "args": tc.get("args", {})})

    # Final answer = the text of the last AIMessage in the run.
    answer = ""
    for msg in reversed(result.get("messages", [])):
        if isinstance(msg, AIMessage):
            answer = _extract_text(msg.content).strip()
            break
    return AgentRun(
        final_answer=answer,
        tool_calls=tool_calls,
        iterations=len(tool_calls) + 1,
    )
