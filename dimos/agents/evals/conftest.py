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

"""Fixture that runs one spatial question through the shipping agent path.

``dimos/agents/conftest.py::agent_setup`` is the model for this file, but it
cannot be reused as is: the eval needs a model per run, timeouts long enough
for a live LLM turn, one coordinator per question, and the goals and tool
queries read back out of the worker processes. Hence a separate fixture rather
than a change to the shared one.

The blueprint under test is the shipping stack -- ``SpatialMemory`` answering
from a pre-ingested store, the real ``NavigationSkillContainer`` skills, the
real ``McpServer``/``McpClient`` -- with observation-only modules at the edges
(see ``modules.py``). Two composition rules are load-bearing:

* ``SpatialMemory`` gets the ingested ``db_path``/``collection_name`` with
  ``new_memory=False``, but a **disposable per-run** ``visual_memory_path``
  and ``output_dir``. ``SpatialMemory.stop()`` saves-then-clears and runs twice
  per teardown, so a pinned visual-memory file gets overwritten with an empty
  one on the second pass. The ChromaDB store the eval queries is unaffected.
* No ``ObjectTrackingSpec`` provider is deployed. That keeps the blueprint
  minimal, and belt-and-braces closes the vision-language object path (an
  external API key and a 30 s tracking loop) that ``navigate_with_text`` tries
  before the semantic map -- but it is not what closes that path.
  ``FakeCamera`` publishes nothing, so ``_latest_image`` stays ``None`` and
  ``_get_bbox_for_current_frame`` returns before any external call, tracker or
  no tracker. The eval reaches the semantic-map path it scores either way.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from itertools import count
from pathlib import Path
from threading import Event
import time
from typing import Any

from dotenv import load_dotenv
from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.evals.modules import (
    FakeCamera,
    FakeOdom,
    RecordingNavigationSkillContainer,
    RecordingStubNavigation,
    TimedAgentTestRunner,
)
from dimos.agents.evals.scorer import GoalRecord, RunObservation, prompt_sha256
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import pLCMTransport
from dimos.utils.logging_config import setup_logger

load_dotenv()

logger = setup_logger()

# Defaults sized for a live turn: a real LLM round trip plus a CLIP-backed
# memory query. The outer wait must stay above the runner's inner idle wait,
# otherwise a slow turn always looks like a fixture timeout and the runner's
# own diagnostic never surfaces.
DEFAULT_ANSWER_TIMEOUT_S = 180.0
DEFAULT_IDLE_TIMEOUT_S = 120.0
DEFAULT_SUBSCRIPTION_TIMEOUT_S = 30.0


@pytest.fixture
def spatial_eval_setup(
    mcp_url: str, lcm_url: str, tmp_path: Path
) -> Iterator[Callable[..., RunObservation]]:
    """Yield a callable that answers one question and reports what the agent did.

    Each call builds a fresh coordinator (``McpClient`` accumulates history for
    the life of the instance, so questions cannot share one) and stops it in a
    ``finally`` before returning, so a loop over questions cannot leak workers.

    The callable takes:

    ``question_id`` / ``question_text``
        Identity for the shard, and the message actually sent to the agent.
    ``prompt_id`` / ``system_prompt``
        Name and text of the prompt arm. The hash of the text travels with the
        observation so a shard can prove which prompt produced it.
    ``model`` **xor** ``model_fixture``
        Exactly one. ``McpClient`` ignores ``config.model`` whenever
        ``model_fixture`` is set, so accepting both would silently score a
        recording instead of the requested model.
    ``db_path`` / ``collection_name``
        The ingested spatial memory to answer from.
    ``answer_timeout_s`` / ``idle_timeout_s`` / ``subscription_timeout_s``
        The fixture's outer wait and the runner's two inner waits.
    ``spatial_memory_blueprint``
        Optional replacement for the ``SpatialMemory`` blueprint. Defaults to
        the shipping module pinned to ``db_path``; the hermetic smoke lane
        substitutes a stub so it can exercise the harness without the ingested
        store. Everything else in the blueprint is fixed.

    Returns a :class:`~dimos.agents.evals.scorer.RunObservation`: raw signal
    only, no verdict. Harness failures are captured into
    ``harness_error`` rather than raised, so a sweep records an attributable
    outcome for every question instead of losing the run.
    """
    run_index = count()
    active: dict[str, Any] = {}

    def shutdown() -> str | None:
        """Tear down whatever the last run built; report a failure instead of raising.

        Safe to call twice. A teardown that fails leaves worker processes
        behind that could contaminate the next question, so the caller marks
        the run as a harness error rather than trusting its measurement.
        """
        coordinator: ModuleCoordinator | None = active.pop("coordinator", None)
        transports: list[pLCMTransport] = active.pop("transports", [])
        unsubs: list[Callable[[], None]] = active.pop("unsubs", [])
        error: str | None = None
        try:
            if coordinator is not None:
                coordinator.stop()
        except Exception as exc:
            logger.error("Spatial eval coordinator teardown failed", exc_info=True)
            error = f"teardown failed: {type(exc).__name__}: {exc}"
        for unsub in unsubs:
            unsub()
        for transport in transports:
            transport.stop()
        return error

    def run_question(
        *,
        question_id: str,
        question_text: str,
        prompt_id: str,
        db_path: str | Path,
        collection_name: str,
        system_prompt: str | None = None,
        model: str | None = None,
        model_fixture: str | Path | None = None,
        answer_timeout_s: float = DEFAULT_ANSWER_TIMEOUT_S,
        idle_timeout_s: float = DEFAULT_IDLE_TIMEOUT_S,
        subscription_timeout_s: float = DEFAULT_SUBSCRIPTION_TIMEOUT_S,
        spatial_memory_blueprint: Blueprint | None = None,
    ) -> RunObservation:
        if (model is None) == (model_fixture is None):
            raise ValueError(
                "pass exactly one of model= / model_fixture=: McpClient builds a "
                "MockModel and ignores config.model whenever model_fixture is set, "
                "so a run carrying both would report a model it never called"
            )
        if answer_timeout_s <= idle_timeout_s:
            raise ValueError(
                f"answer_timeout_s ({answer_timeout_s}) must exceed idle_timeout_s "
                f"({idle_timeout_s}), otherwise every slow turn is reported as a "
                "fixture timeout and the runner's own error is never seen"
            )

        client_kwargs: dict[str, Any] = {
            "system_prompt": system_prompt,
            "mcp_server_url": mcp_url,
        }
        if model is not None:
            client_kwargs["model"] = model
            model_id = model
        else:
            client_kwargs["model_fixture"] = str(model_fixture)
            model_id = f"model_fixture:{Path(str(model_fixture)).name}"

        # Disposable per-run memory artifacts (see module docstring).
        run_dir = tmp_path / f"run-{next(run_index):03d}"
        run_dir.mkdir(parents=True, exist_ok=True)
        memory_blueprint = spatial_memory_blueprint
        if memory_blueprint is None:
            # ~4s: deferred so collecting this directory doesn't pay for CLIP.
            from dimos.perception.spatial_perception import SpatialMemory

            memory_blueprint = SpatialMemory.blueprint(
                collection_name=collection_name,
                db_path=str(db_path),
                new_memory=False,
                visual_memory_path=str(run_dir / "visual_memory.pkl"),
                output_dir=str(run_dir / "images"),
            )

        finished = False
        goals: list[GoalRecord] = []
        tool_queries: list[str] = []
        tool_errors: list[str] = []
        harness_error: str | None = None
        agent_seconds = 0.0
        started = time.perf_counter()

        try:
            finished_event = Event()
            finished_transport = pLCMTransport("/finished", url=lcm_url)
            active["transports"] = [finished_transport]
            active["unsubs"] = [finished_transport.subscribe(lambda _: finished_event.set())]

            blueprint = autoconnect(
                FakeCamera.blueprint(),
                FakeOdom.blueprint(),
                memory_blueprint,
                RecordingNavigationSkillContainer.blueprint(),
                RecordingStubNavigation.blueprint(),
                McpServer.blueprint(),
                McpClient.blueprint(**client_kwargs),
                TimedAgentTestRunner.blueprint(
                    messages=[HumanMessage(question_text)],
                    idle_timeout=idle_timeout_s,
                    subscription_timeout=subscription_timeout_s,
                ),
            )
            global_config.update(viewer="none", transport="lcm")  # pLCMTransport sidecar above

            coordinator = ModuleCoordinator.build(blueprint)
            active["coordinator"] = coordinator

            turn_started = time.perf_counter()
            finished = finished_event.wait(answer_timeout_s)
            agent_seconds = time.perf_counter() - turn_started

            # Both recorders keep their state inside their worker process.
            navigation = coordinator.get_instance(RecordingStubNavigation)
            goals = [GoalRecord.from_mapping(goal) for goal in navigation.get_goals() or []]
            skills = coordinator.get_instance(RecordingNavigationSkillContainer)
            tool_queries = list(skills.get_queries() or [])
            tool_errors = list(skills.get_tool_errors() or [])
        except Exception as exc:
            logger.error(f"Spatial eval harness failed on question {question_id!r}", exc_info=True)
            harness_error = f"{type(exc).__name__}: {exc}"
        finally:
            teardown_error = shutdown()
            harness_error = harness_error or teardown_error

        return RunObservation(
            question_id=question_id,
            model_id=model_id,
            prompt_id=prompt_id,
            prompt_sha256=prompt_sha256(system_prompt),
            finished=finished,
            agent_wall_time_s=agent_seconds,
            total_wall_time_s=time.perf_counter() - started,
            goals=goals,
            tool_queries=tool_queries,
            tool_errors=tool_errors,
            harness_error=harness_error,
        )

    yield run_question

    # `run_question` cleans up after itself; this only catches a run that died
    # between building something and reaching its own `finally`.
    shutdown()
