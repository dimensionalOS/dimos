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

import os
from pathlib import Path
from threading import Event

from dotenv import load_dotenv
from langchain_core.messages.base import BaseMessage
import pytest

from dimos.agents.agent import Agent
from dimos.agents.agent_test_runner import AgentTestRunner
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pLCMTransport

load_dotenv()

FIXTURE_DIR = Path(__file__).parent / "fixtures"


@pytest.fixture
def fixture_dir() -> Path:
    return FIXTURE_DIR


@pytest.fixture
def agent_setup(request):
    coordinator = None
    transports: list[pLCMTransport] = []
    unsubs: list = []
    recording = bool(os.getenv("RECORD"))

    def fn(
        *,
        blueprints,
        messages: list[BaseMessage],
        dask: bool = False,
        system_prompt: str | None = None,
        fixture: str | None = None,
    ) -> list[BaseMessage]:
        history: list[BaseMessage] = []
        finished_event = Event()

        agent_transport: pLCMTransport = pLCMTransport("/agent")
        finished_transport: pLCMTransport = pLCMTransport("/finished")
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

        agent_kwargs: dict = {"system_prompt": system_prompt}

        if recording or fixture_path.exists():
            # RECORD=1: use real LLM, save responses to fixture file.
            # No RECORD but fixture exists: play back recorded responses.
            # The MockModel checks RECORD internally to decide record vs playback.
            agent_kwargs["model_fixture"] = str(fixture_path)

        blueprint = autoconnect(
            *blueprints,
            Agent.blueprint(**agent_kwargs),
            AgentTestRunner.blueprint(messages=messages),
        )

        global_config.update(
            viewer_backend="none",
            dask=dask,
        )

        nonlocal coordinator
        coordinator = blueprint.build()

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
