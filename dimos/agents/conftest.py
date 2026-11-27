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

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import sys
from threading import Event
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from langchain_core.messages.base import BaseMessage
import pytest

from dimos.agents.agent import Agent
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.testing.agent_test_runner import AgentTestRunner
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.module import SkillInfo
from dimos.core.transport import pLCMTransport

FIXTURE_DIR = Path(__file__).parent / "fixtures"


@pytest.fixture
def lone_mcp_server(mcp_port: int) -> Iterator[McpServer]:
    """An McpServer running on its own, with no coordinator to send it modules."""
    if sys.version_info < (3, 11):
        pytest.skip("McpServer needs Python 3.11")
    server = McpServer(port=mcp_port)
    server.start()
    try:
        yield server
    finally:
        server.stop()


@pytest.fixture
def serve_skills(
    lone_mcp_server: McpServer,
) -> Callable[[list[SkillInfo], dict[str, Callable[..., Any]]], McpAdapter]:
    """Give `lone_mcp_server` skills, each answered by a callable in place of its module's RPC."""

    def serve(skills: list[SkillInfo], rpc_calls: dict[str, Callable[..., Any]]) -> McpAdapter:
        module = SimpleNamespace(get_skills=lambda: skills, remote_name="TestSkills")
        with patch(
            "dimos.agents.mcp.mcp_server.RpcCall",
            side_effect=lambda _cls, _rpc, name, *_: rpc_calls[name],
        ):
            lone_mcp_server.on_system_modules([module])
        return McpAdapter(lone_mcp_server.mcp_url())

    return serve


@pytest.fixture
def agent_setup(request, mcp_port: int, lcm_url: str):
    if sys.version_info < (3, 11):
        pytest.skip("McpServer needs Python 3.11")
    coordinator = None
    transports: list[pLCMTransport] = []
    unsubs: list = []
    recording = bool(os.getenv("RECORD"))

    def fn(
        *,
        blueprints,
        messages: list[BaseMessage],
        system_prompt: str | None = None,
        fixture: str | None = None,
    ) -> list[BaseMessage]:
        history: list[BaseMessage] = []
        finished_event = Event()

        agent_transport: pLCMTransport = pLCMTransport("/agent", url=lcm_url)
        finished_transport: pLCMTransport = pLCMTransport("/finished", url=lcm_url)
        transports.extend([agent_transport, finished_transport])

        def on_message(msg: BaseMessage) -> None:
            history.append(msg)

        unsubs.append(agent_transport.subscribe(on_message))
        unsubs.append(finished_transport.subscribe(lambda _: finished_event.set()))

        # Derive fixture path from test name if not explicitly provided.
        if fixture is not None:
            fixture_path = FIXTURE_DIR / fixture
        else:
            fixture_path = FIXTURE_DIR / f"{request.node.name}.json"

        agent_kwargs: dict = {
            "system_prompt": system_prompt,
        }

        if recording or fixture_path.exists():
            # RECORD=1: use real LLM, save responses to fixture file.
            # No RECORD but fixture exists: play back recorded responses.
            # The MockModel checks RECORD internally to decide record vs playback.
            agent_kwargs["model_fixture"] = str(fixture_path)

        blueprint = autoconnect(
            *blueprints,
            McpServer.blueprint(port=mcp_port),
            Agent.blueprint(**agent_kwargs),
            AgentTestRunner.blueprint(messages=messages),
        )

        global_config.update(viewer="none", transport="lcm")  # fixture uses pLCMTransport sidecars

        nonlocal coordinator
        coordinator = ModuleCoordinator.build(blueprint)

        if not finished_event.wait(60):
            raise TimeoutError("Timed out waiting for agent to finish processing messages.")

        return history

    yield fn

    if coordinator is not None:
        coordinator.stop()

    for transport in transports:
        transport.stop()

    for unsub in unsubs:
        unsub()
