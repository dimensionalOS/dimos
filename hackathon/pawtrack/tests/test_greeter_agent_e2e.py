"""End-to-end agent test: does the LLM pick our greeting tools from a prompt?

This drives the *real* agent stack (MCP server + LLM client + the greeter skill
container) exactly like a live run: it injects a natural-language prompt and
checks the agent's tool calls. So it needs a real LLM + API credentials and an
LCM/MCP bus, and is therefore skipped unless ``PAWTRACK_LLM_E2E=1`` is set.

    PAWTRACK_LLM_E2E=1 OPENAI_API_KEY=... PAWTRACK_MODEL=openai:deepseek-chat \
      OPENAI_BASE_URL=https://api.deepseek.com \
      PYTHONPATH=src pytest tests/test_greeter_agent_e2e.py -s

It mirrors DimOS's own ``agent_setup`` harness (dimos/agents/conftest.py): build
``McpServer`` + ``McpClient`` + the skills + an ``AgentTestRunner`` that injects
messages, collect the agent's message history off ``/agent``, and assert on the
tool calls. The greeter loop's scan is disabled (``scan_interval_s`` huge) so
executing ``start_greeting`` does not load the vision models during the test.
"""

from __future__ import annotations

import os
from threading import Event
from typing import Any

import pytest

pytestmark = pytest.mark.skipif(
    not os.environ.get("PAWTRACK_LLM_E2E"),
    reason="needs a real LLM + creds; set PAWTRACK_LLM_E2E=1 to run.",
)

from langchain_core.messages import HumanMessage  # noqa: E402

from dimos.agents.agent_test_runner import AgentTestRunner  # noqa: E402
from dimos.agents.mcp.mcp_client import McpClient  # noqa: E402
from dimos.agents.mcp.mcp_server import McpServer  # noqa: E402
from dimos.core.coordination.blueprints import autoconnect  # noqa: E402
from dimos.core.coordination.module_coordinator import (  # noqa: E402
    ModuleCoordinator,
)
from dimos.core.core import rpc  # noqa: E402
from dimos.core.global_config import global_config  # noqa: E402
from dimos.core.module import Module  # noqa: E402
from dimos.core.transport import pLCMTransport  # noqa: E402

from pawtrack.greeter_container import GreeterSkillContainer  # noqa: E402

_MCP_URL = os.environ.get("MCP_URL", "http://localhost:9990/mcp")
_LCM_URL = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")


class _FakePatrol(Module):
    """Minimal stand-in for the PatrollingModule so the greeter's spec injects.

    The greeter container requires a patrolling module; on the robot that is the
    nav stack. For this agent test we only care about tool selection, so a no-op
    patrol that satisfies the spec is enough.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._patrolling = False

    @rpc
    def start_patrol(self) -> str:
        self._patrolling = True
        return "Patrol started."

    @rpc
    def is_patrolling(self) -> bool:
        return self._patrolling

    @rpc
    def stop_patrol(self) -> str:
        self._patrolling = False
        return "Patrol stopped."


def _run_agent(messages: list[HumanMessage]) -> list[Any]:
    """Run the greeter agent over messages; return the agent's message history."""
    history: list[Any] = []
    finished = Event()
    agent_t: pLCMTransport = pLCMTransport("/agent", url=_LCM_URL)
    finished_t: pLCMTransport = pLCMTransport("/finished", url=_LCM_URL)
    unsubs = [
        agent_t.subscribe(history.append),
        finished_t.subscribe(lambda _msg: finished.set()),
    ]

    # Omit system_prompt so the client uses its default general agent prompt --
    # exactly like the greeter-agentic blueprint, where greeting is just one tool.
    model = os.environ.get("PAWTRACK_MODEL")
    client_kwargs: dict[str, Any] = {"mcp_server_url": _MCP_URL}
    if model:
        client_kwargs["model"] = model

    blueprint = autoconnect(
        _FakePatrol.blueprint(),
        # Disable scanning so executing start_greeting does not load the VLM.
        GreeterSkillContainer.blueprint(scan_interval_s=1e9),
        McpServer.blueprint(),
        McpClient.blueprint(**client_kwargs),
        AgentTestRunner.blueprint(messages=messages),
    )
    global_config.update(viewer="none")
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        if not finished.wait(120):
            raise TimeoutError("agent did not finish processing the prompt")
    finally:
        coordinator.stop()
        agent_t.stop()
        finished_t.stop()
        for unsub in unsubs:
            unsub()
    return history


def _tool_calls(history: list[Any], name: str) -> list[Any]:
    return [
        call
        for message in history
        for call in (getattr(message, "tool_calls", None) or [])
        if call["name"] == name
    ]


def test_agent_selects_greeting_then_stop_from_prompts():
    history = _run_agent([
        HumanMessage(
            "Walk around the room and say hi to everyone who is sitting down. "
            "Don't ask me for confirmation, just start doing it."
        ),
        HumanMessage("Okay, that is enough for now -- stop greeting."),
    ])

    assert _tool_calls(history, "start_greeting"), (
        "the agent did not select start_greeting for a greeting request"
    )
    assert _tool_calls(history, "stop_greeting"), (
        "the agent did not select stop_greeting when asked to stop"
    )
