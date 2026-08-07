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

After the turn, and before teardown, the fixture asks the *same* memory what
each of the agent's queries retrieves, so the shard records what was retrieved
and not only where the agent went (see :func:`_replay_retrievals`). That is a
read-only query against a store this run never writes to, it costs ~0.1 s per
query, and it is a diagnostic: every failure of it is swallowed, because a
measurement that already succeeded must not be invalidated by an observation
bolted onto it.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping, Sequence
from itertools import count
import os
from pathlib import Path
from threading import Event
import time
from typing import Any

from dotenv import load_dotenv
from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.evals.contracts import RetrievalRecord
from dimos.agents.evals.ingest import assert_store_ingested
from dimos.agents.evals.scorer import GoalRecord, RunObservation, prompt_sha256
from dimos.agents.evals.testing.modules import (
    FakeCamera,
    FakeOdom,
    RecordingNavigationSkillContainer,
    RecordingStubNavigation,
    TimedAgentTestRunner,
)
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
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

#: Substituted when a retrieval result carries no distance, matching what the
#: shipping skill does with the same hole (``1.0 - (result.get("distance") or 1)``
#: in ``dimos/agents/skills/navigation.py``): it reads as "no usable similarity"
#: rather than as a perfect match, which a ``0.0`` default would.
_NO_DISTANCE = 1.0


def _top_retrieval(query: str, results: Any) -> RetrievalRecord | None:
    """The top hit of one ``query_by_text`` return, or ``None`` if there is none.

    Worth extracting because the shape is not the obvious one.
    ``SpatialVectorDB._process_query_results`` enumerates chroma's **per-query**
    lists, so a single-text query comes back as *one* result object, not one per
    hit: its ``metadata`` is the list of hits and its ``distance``/``id`` belong
    to the top one. The pose therefore lives in ``metadata[0]``, which is exactly
    how ``navigate_with_text`` reads it (``_get_goal_pose_from_result``).

    Returns ``None`` -- never raises, never guesses -- for anything that is not
    that shape: an empty answer, a memory whose metadata carries no ``pos_x`` /
    ``pos_y``, or a stub that answers something else entirely. This is a
    diagnostic, so an unrecognized shape is recorded as "nothing", not as a
    coordinate somebody might later read as a measurement.

    >>> _top_retrieval("houseplant", [{"metadata": [{"pos_x": 1.0, "pos_y": -2.0}],
    ...                                "distance": 0.25}])
    RetrievalRecord(query='houseplant', x=1.0, y=-2.0, distance=0.25)
    >>> _top_retrieval("houseplant", []) is None
    True
    """
    if not results:
        return None
    first = results[0]
    metadata = first.get("metadata") if isinstance(first, Mapping) else None
    if isinstance(metadata, Mapping):
        top: Mapping[str, Any] = metadata
    elif isinstance(metadata, Sequence) and metadata and isinstance(metadata[0], Mapping):
        top = metadata[0]
    else:
        return None
    if "pos_x" not in top or "pos_y" not in top:
        return None
    distance = first.get("distance")
    return RetrievalRecord(
        query=query,
        x=float(top["pos_x"]),
        y=float(top["pos_y"]),
        distance=float(distance) if distance is not None else _NO_DISTANCE,
    )


def _queryable_module(blueprint: Blueprint) -> type[ModuleBase] | None:
    """The module of *blueprint* that answers ``query_by_text``, if one does.

    The blueprint is resolved rather than the shipping ``SpatialMemory`` being
    assumed, because the memory module is a parameter of the fixture: the
    hermetic lanes substitute their own. A substitute without the RPC is a
    legitimate configuration (the re-query is a diagnostic, not a requirement),
    so this reports ``None`` and the caller skips with a note.
    """
    for atom in blueprint.blueprints:
        if hasattr(atom.module, "query_by_text"):
            return atom.module
    return None


def _replay_retrievals(
    coordinator: ModuleCoordinator, blueprint: Blueprint, queries: Sequence[str]
) -> list[RetrievalRecord]:
    """Re-ask the spatial memory what each of *queries* retrieves.

    Deterministic rather than a second sample: the store is attached read-only
    and ``query_by_text`` is a CLIP embedding against it, so re-asking the same
    text after the turn reproduces what the agent's tool call got. Queries are
    de-duplicated with order preserved -- an agent that asks the same thing twice
    gets one record, and the list stays joinable against ``tool_queries``.

    **Every exception is swallowed.** It is observability bolted onto a
    measurement that is already complete, so each failure path returns an empty
    list and logs; the caller must not turn it into a ``harness_error``. On a
    partial failure the records collected so far are dropped too: a list that
    silently omits one query is worse than an absent one, because a reader
    joining it against ``tool_queries`` cannot tell the two apart.

    What it is *not* is unbounded. ``query_by_text`` is an RPC into the memory's
    worker process, so a worker that stops answering is bounded by the
    module-level RPC timeout rather than by anything here: no per-method
    override is registered for ``query_by_text``
    (``dimos/protocol/rpc/spec.py::DEFAULT_RPC_TIMEOUTS``), so it falls back to
    ``ModuleConfig.default_rpc_timeout``, currently 120 s, after which
    ``call_sync`` raises ``TimeoutError`` and the ``except`` below records
    nothing. The worst case is therefore that timeout once per *distinct* query
    -- typically one -- added to a turn that has already been measured.
    """
    started = time.perf_counter()
    try:
        module = _queryable_module(blueprint)
        if module is None:
            logger.info(
                "Spatial eval: the spatial memory blueprint in use exposes no "
                "query_by_text; recording no retrievals for this question"
            )
            return []
        memory = coordinator.get_instance(module)
        if memory is None:
            logger.info(f"Spatial eval: {module.__name__} is not deployed; no retrievals recorded")
            return []
        records: list[RetrievalRecord] = []
        for query in dict.fromkeys(queries):
            record = _top_retrieval(query, memory.query_by_text(query, limit=1))
            if record is not None:
                records.append(record)
    except Exception:
        logger.warning(
            "Spatial eval: retrieval read-back failed; recording no retrievals. "
            "This is a diagnostic and does not invalidate the run.",
            exc_info=True,
        )
        return []
    logger.info(
        f"Spatial eval: recorded {len(records)} retrieval(s) for {len(set(queries))} "
        f"distinct query/ies in {time.perf_counter() - started:.2f}s"
    )
    return records


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
        The ingested spatial memory to answer from. Checked with
        ``assert_store_ingested`` before anything is built, whenever the shipping
        ``SpatialMemory`` blueprint is the one being used.
    ``run_id``
        Which sweep this measurement belongs to; defaults to one stamp per
        fixture instantiation, i.e. per test, so every question of one sweep
        shares it and a repeat of that sweep does not collide with it.
    ``answer_timeout_s`` / ``idle_timeout_s`` / ``subscription_timeout_s``
        The fixture's outer wait and the runner's two inner waits.
    ``spatial_memory_blueprint``
        Optional replacement for the ``SpatialMemory`` blueprint. Defaults to
        the shipping module pinned to ``db_path``; the hermetic smoke lane
        substitutes a stub so it can exercise the harness without the ingested
        store. Everything else in the blueprint is fixed.

    Returns a :class:`~dimos.agents.evals.scorer.RunObservation`: raw signal
    only, no verdict. Harness failures are captured into ``harness_error``
    rather than raised, so a sweep records *something* for every question
    instead of dying part-way through and losing the questions it had already
    answered. What it records is a ``harness_error``, which scores as a BROKEN
    outcome -- explicitly **not** attributable to the agent, excluded from every
    rate, and a reason to repeat the run rather than to read it.

    The observation also carries ``retrievals``: after the turn, each distinct
    tool query is re-asked of the spatial memory so the shard records *what was
    retrieved*, not only where the agent went. That is diagnostic observability
    and nothing scores it -- see ``contracts.RetrievalRecord`` and
    :func:`_replay_retrievals`, whose exceptions are all swallowed.
    """
    run_index = count()
    active: dict[str, Any] = {}
    # One stamp per fixture instantiation, i.e. per test: the questions of one
    # sweep are one run and must share a run_id, while a second sweep -- of the
    # same configuration, deliberately repeated -- must get its own or the
    # renderer cannot tell a repeat from a duplicate.
    default_run_id = f"{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"

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
        run_id: str | None = None,
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
        if spatial_memory_blueprint is None:
            # Raised, not captured as a harness_error: a store that is not there
            # is a misconfiguration of the run, not a failed measurement of it.
            # Checked here, before the ~20 s coordinator build, so a whole sweep
            # against the wrong collection name costs seconds.
            assert_store_ingested(db_path, collection_name)

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
            from dimos.perception.experimental.spatial_perception import SpatialMemory

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
        retrievals: list[RetrievalRecord] = []
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

            # Diagnostic, and last: the measurement above is already complete, so
            # this must not be able to spoil it. `_replay_retrievals` swallows
            # everything, which is why it sits inside this `try` without being
            # able to reach the `except` below. It needs the coordinator alive, so
            # it runs before `shutdown()` in the `finally`.
            retrievals = _replay_retrievals(coordinator, memory_blueprint, tool_queries)
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
            run_id=run_id or default_run_id,
            finished=finished,
            agent_wall_time_s=agent_seconds,
            total_wall_time_s=time.perf_counter() - started,
            goals=goals,
            tool_queries=tool_queries,
            tool_errors=tool_errors,
            harness_error=harness_error,
            retrievals=retrievals,
        )

    yield run_question

    # `run_question` cleans up after itself; this only catches a run that died
    # between building something and reaching its own `finally`.
    shutdown()
