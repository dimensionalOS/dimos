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

"""The runs that need a robot recording: the 2x2 sweep and a full-chain smoke.

Both lanes are ``self_hosted``. Everything the eval decides -- classification,
scoring, question rules, the figure -- is covered by the synthetic unit tests
next to this file; what is left here is the part that can only be checked by
actually driving the shipping agent, and it needs an ingested spatial memory
(and, for the sweep, an API key).

**The sweep** (``test_goal_selection_sweep``) is the measurement: two system
prompts (the shipping default, and one that names the spatial memory) x two
models over the whole committed question set, one JSONL shard per case. It is
a *tool-routing and query-formulation comparison*, not a ranking of spatial
ability -- the coordinates come from ``SpatialMemory``'s deterministic CLIP
top-1, and the model only decides whether to call the tool, what query to
send, and how many times. What it asserts is correspondingly narrow: every
question produced a paired, attributable record, and none of them came back as
something other than a measurement of the agent. The numbers live in the shards.

**The smoke** (``test_full_chain_pipeline``) needs no key. It ingests a thin
slice of the replay, asks one question with a recorded model transcript, and
scores and plots the answer. It validates the *pipeline* -- ingest, harness,
memory query, goal capture, scoring, shard, figure -- and says nothing
whatsoever about model quality.

Re-recording the smoke's transcript. ``MockModel`` reads ``RECORD`` itself: with
it set, the fixture path is *written* from a live turn instead of replayed
(``dimos/agents/testing/mock_model.py``), so the transcript is whatever the
shipping client's own model path returned, rather than hand-written JSON. It has
to be re-recorded whenever :data:`SMOKE_QUESTION_ID` changes, because the query in
it is that question's display name and the assertion below checks the two agree::

    RECORD=1 OPENAI_API_KEY=... uv run pytest \\
        dimos/agents/evals/test_spatial_goal_eval.py -m self_hosted -k full_chain

Then run it again *without* ``RECORD`` to confirm the recording plays back
keyless, which is the state the lane ships in.

Running the sweep locally::

    # once: build the store the sweep answers from (~1 min)
    uv run python -m dimos.agents.evals.ingest \\
        --dataset go2_bigoffice --out-dir ~/.local/state/dimos/spatial-eval

    DIMOS_EVAL_DB_PATH=~/.local/state/dimos/spatial-eval/chroma \\
    DIMOS_EVAL_COLLECTION=go2_bigoffice \\
    DIMOS_EVAL_SHARD_DIR=.ignore.eval-shards \\
    uv run pytest dimos/agents/evals/test_spatial_goal_eval.py -m self_hosted

    # then turn the four shards into the one figure. Thresholds are per question
    # (questions.THRESHOLD_MARGIN_M); --threshold-m draws the cap as a reference
    # line, not the line any single dot was scored against.
    uv run python -m dimos.agents.evals.render \\
        --shards .ignore.eval-shards --out error_distribution.png --threshold-m 3.5

Note for a first run on a fresh machine: ``navigate_with_text`` tries tagged
locations before the semantic map, and that path makes ChromaDB instantiate
its default embedding function, which lazily downloads ~80 MB into
``~/.cache/chroma`` even for an empty collection. Warm that cache before
timing anything (a self-hosted runner needs egress to
``chroma-onnx-models.s3.amazonaws.com``).
"""

from __future__ import annotations

from collections.abc import Callable
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from dimos.agents.evals.contracts import QuestionSpec
from dimos.agents.evals.questions import slugify
from dimos.agents.evals.render import MAX_PNG_BYTES, render_figure
from dimos.agents.evals.scorer import (
    BROKEN_OUTCOMES,
    RunObservation,
    append_shard,
    broken_count,
    build_answer_record,
    errors_m,
    no_prediction_rate,
    pass_rate,
    prompt_sha256,
    read_shard,
    score,
)
from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

REFERENCE_DIR = Path(__file__).parent / "reference"
QUESTIONS_PATH = REFERENCE_DIR / "questions.jsonl"

#: Where the sweep reads its ingested store from, and writes its shards to.
#: ``DIMOS_EVAL_DB_PATH`` is the ``chroma/`` directory ``ingest.py`` produced,
#: not the directory above it.
DB_PATH_ENV = "DIMOS_EVAL_DB_PATH"
COLLECTION_ENV = "DIMOS_EVAL_COLLECTION"
SHARD_DIR_ENV = "DIMOS_EVAL_SHARD_DIR"

#: Replay the smoke test ingests its slice from (an LFS artifact in ``data/``).
DATASET_ENV = "DIMOS_EVAL_DATASET"
DEFAULT_DATASET = "go2_bigoffice"

#: Frozen model arms. ``gpt-5.6-luna`` is the shipping default and is routed to
#: the OpenAI *Responses* API with reasoning effort, while ``openai:gpt-5.6-sol`` is
#: routed to chat completions (``mcp_client._init_model``). The two arms
#: therefore differ in API surface as well as in weights: that is deliberate --
#: they are the two request paths the shipping client actually takes -- and it
#: is why ``model_id`` is recorded on every shard line rather than being
#: reconstructed from the filename.
MODEL_IDS = ("gpt-5.6-luna", "openai:gpt-5.6-sol")

#: Frozen prompt arms. Neither says anything about the shape of an answer: the
#: constant task instruction is part of the question template
#: (``questions.QUESTION_TEMPLATE``), because a swept format instruction would
#: measure format compliance instead of spatial behavior. What differs between
#: the arms is how strongly the agent is told it *has* a spatial memory -- i.e.
#: whether it routes to the tool or answers from world knowledge, which is
#: exactly what the sweep compares.
PROMPT_SPATIAL = (
    "You are a mobile robot assistant with a semantic spatial memory of the space you "
    "have explored. When asked where something is, or to go to something, use your "
    "navigation tools rather than answering from general knowledge."
)

#: The two arms. The baseline is the **shipping** prompt -- ``McpClientConfig``'s
#: default, what a deployed robot actually runs with -- rather than a synthetic
#: minimal one, because a baseline nobody ships measures a configuration nobody
#: has. The coupling is the point: a change to ``dimos/agents/system_prompt.py``
#: moves this eval, which is the signal it exists to give.
PROMPTS = (("shipping", SYSTEM_PROMPT), ("spatial", PROMPT_SPATIAL))

#: The smoke test's question, and the query in its recorded transcript. The
#: transcript is recorded against *this* question -- the query below is the
#: question's own ``display_name``, which is what a model answering it would send
#: to ``navigate_with_text`` -- so the playback exercises the shipping skill on
#: the question the shard says it ran. Re-record after changing either (see the
#: docstring above for the ``RECORD`` flow).
SMOKE_QUESTION_ID = "go2-bigoffice-organization"
SMOKE_TOOL_QUERY = "stack of storage boxes"
SMOKE_MODEL_FIXTURE = Path(__file__).parent / "fixtures" / "test_full_chain_pipeline.json"

#: Frames per second of *recorded* time for the smoke ingest: enough of the
#: replay to exercise the store, few enough to keep the lane honest about its
#: runtime. The sweep's own store is built at the ``ingest.py`` default.
SMOKE_SAMPLE_HZ = 0.2
SMOKE_INGEST_TIMEOUT_S = 600.0


def load_question_set(path: Path = QUESTIONS_PATH) -> list[QuestionSpec]:
    """Read the committed question set, in file order."""
    return [QuestionSpec.from_json(line) for line in path.read_text().splitlines() if line.strip()]


def shard_path(directory: Path, model_id: str, prompt_id: str, run_id: str) -> Path:
    """A shard filename no other case can produce.

    Contract: one file per pytest case. The configuration names the file so a
    directory of shards is readable without opening them, and the run stamp --
    the same ``run_id`` the records inside carry -- keeps repeated runs (and
    parallel workers) from appending to each other. Filename and content agree by
    construction, so a shard can be attributed to its run without opening it.
    """
    return directory / f"{slugify(model_id)}__{prompt_id}__{run_id}.jsonl"


@pytest.fixture(scope="module")
def question_set() -> list[QuestionSpec]:
    return load_question_set()


@pytest.fixture(scope="module")
def run_id() -> str:
    """One stamp for the whole sweep: its cases are arms of a single run.

    Module-scoped on purpose. The four (model, prompt) cases are one sweep, and
    the renderer aggregates by ``run_id`` -- giving each case its own would make
    a repeat of the sweep indistinguishable from the sweep itself.
    """
    return f"{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"


@pytest.fixture
def ingested_store() -> tuple[str, str]:
    """The ``(db_path, collection)`` of a store built by ``ingest.py``.

    Skipping when the environment does not name a store is deliberate, and is
    the same shape as the repository's other recording-backed self-hosted tests
    (``dimos/navigation/cmu_nav/tests/rosbag_fixtures.py``): the store is a
    machine-local artifact an operator produces once, not a dependency of the
    package. A *named* store that is missing is a different thing -- that is a
    misconfiguration, and it fails.
    """
    db_path = os.environ.get(DB_PATH_ENV)
    collection = os.environ.get(COLLECTION_ENV)
    if not db_path or not collection:
        pytest.skip(
            f"no ingested spatial memory: set {DB_PATH_ENV} (the chroma/ directory) and "
            f"{COLLECTION_ENV}. Build one with: uv run python -m dimos.agents.evals.ingest "
            f"--dataset {DEFAULT_DATASET} --out-dir <dir>"
        )
    if not Path(db_path).is_dir():
        pytest.fail(
            f"{DB_PATH_ENV}={db_path!r} does not exist; it must be the chroma/ directory "
            "written by dimos.agents.evals.ingest"
        )
    return db_path, collection


@pytest.fixture
def shard_dir(tmp_path: Path) -> Path:
    """Where cases write their shards: ``$DIMOS_EVAL_SHARD_DIR``, else throwaway."""
    directory = Path(os.environ.get(SHARD_DIR_ENV) or tmp_path / "shards")
    directory.mkdir(parents=True, exist_ok=True)
    return directory


@pytest.mark.self_hosted
@pytest.mark.skipif_no_openai
# Every question x (a real LLM turn + a coordinator build that loads CLIP in the
# memory worker) outlasts the default per-test timeout.
@pytest.mark.timeout(2400)
@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize(("prompt_id", "system_prompt"), PROMPTS, ids=[p[0] for p in PROMPTS])
def test_goal_selection_sweep(
    spatial_eval_setup: Callable[..., RunObservation],
    question_set: list[QuestionSpec],
    ingested_store: tuple[str, str],
    shard_dir: Path,
    run_id: str,
    model_id: str,
    prompt_id: str,
    system_prompt: str,
) -> None:
    """Ask every question under one (model, prompt) configuration and record it.

    This case is one row of the figure. It asserts only what would mean the
    measurement is invalid -- a missing question, a record attributed to the
    wrong configuration, or a failure of the *measurement* rather than of the
    agent. Whether the agent finds the elevator is the *result*, and lives in
    the shard, not in an assertion.
    """
    db_path, collection = ingested_store
    out = shard_path(shard_dir, model_id, prompt_id, run_id)

    for question in question_set:
        observation = spatial_eval_setup(
            question_id=question.question_id,
            question_text=question.question_text,
            prompt_id=prompt_id,
            system_prompt=system_prompt,
            model=model_id,
            run_id=run_id,
            db_path=db_path,
            collection_name=collection,
        )
        answer = build_answer_record(observation)
        append_shard(out, answer, score(question, answer))

    cases = read_shard(out)
    results = [case.result for case in cases]
    logger.info(
        f"[{model_id} / {prompt_id}] {out}: n_pred {len(errors_m(results))}/{len(results)}, "
        f"no-prediction {no_prediction_rate(results):.0%}, pass {pass_rate(results):.0%}, "
        f"broken {broken_count(results)}"
    )

    assert [case.answer.question_id for case in cases] == [q.question_id for q in question_set]
    assert {case.answer.model_id for case in cases} == {model_id}
    assert {case.answer.prompt_id for case in cases} == {prompt_id}
    assert {case.answer.prompt_sha256 for case in cases} == {prompt_sha256(system_prompt)}
    assert {case.answer.run_id for case in cases} == {run_id}
    # Every BROKEN_OUTCOMES member, not just harness_error: a turn that never
    # finished and a skill that raised are equally not measurements of the agent,
    # and a row assembled from them would be a row of missing data wearing a
    # pass rate. The rates already exclude them, which is exactly why the run has
    # to be repeated rather than reported.
    broken = [
        f"{case.answer.question_id}: {case.result.reason}"
        for case in cases
        if case.result.outcome in BROKEN_OUTCOMES
    ]
    assert broken == [], (
        f"{len(broken)} question(s) were never measured, so this run has to be "
        f"repeated rather than read: {broken}"
    )


@pytest.mark.self_hosted
# One ingest of a replay slice plus one coordinator build; see the runtime note
# in the docstring.
@pytest.mark.timeout(1200)
def test_full_chain_pipeline(
    spatial_eval_setup: Callable[..., RunObservation],
    question_set: list[QuestionSpec],
    run_id: str,
    tmp_path: Path,
) -> None:
    """Replay -> store -> agent -> goal -> score -> shard -> figure, with no API key.

    **This validates the pipeline, not the model.** The agent's replies are a
    recorded transcript played back by ``MockModel``, so the tool call and the
    final message are fixed; what is under test is that a store ingested from
    the replay answers a real ``navigate_with_text`` call, that the goal the
    skill sets is captured out of its worker process, and that the result
    survives scoring, the shard format and the renderer. It cannot fail because
    a model is bad, and it cannot pass because a model is good.

    It therefore validates the *mechanism*, not the answer, and deliberately does
    not assert ``passed``: the store is a thin slice of the replay, so the top-1
    frame CLIP retrieves for :data:`SMOKE_TOOL_QUERY` may well be metres from the
    reference, and a scoring FAIL is an acceptable outcome of a working chain.
    Asserting the verdict here would make the smoke lane fail whenever the slice
    got thinner, which is a property of the fixture, not a regression.

    The store is a thin slice of the replay (``SMOKE_SAMPLE_HZ``), ingested
    through the shipped ``ingest.py`` entry point in its own process: that is
    how an operator builds one, and it keeps CLIP and ChromaDB's thread pool
    out of the pytest process.
    """
    dataset = os.environ.get(DATASET_ENV) or DEFAULT_DATASET
    replay = Path(dataset) if Path(dataset).suffix else get_data_dir() / f"{dataset}.db"
    if not replay.exists():
        pytest.skip(
            f"replay {replay} is not present; pull it with: git lfs pull "
            f'--include="data/.lfs/{dataset}.db.tar.gz"'
        )

    store_dir = tmp_path / "store"
    ingest = subprocess.run(
        [
            sys.executable,
            "-m",
            "dimos.agents.evals.ingest",
            "--dataset",
            str(replay),
            "--out-dir",
            str(store_dir),
            "--collection",
            dataset,
            "--sample-hz",
            str(SMOKE_SAMPLE_HZ),
            "--quiet",
        ],
        capture_output=True,
        text=True,
        timeout=SMOKE_INGEST_TIMEOUT_S,
        check=False,
    )
    assert ingest.returncode == 0, f"ingest failed:\n{ingest.stdout}\n{ingest.stderr}"

    question = next(q for q in question_set if q.question_id == SMOKE_QUESTION_ID)
    observation = spatial_eval_setup(
        question_id=question.question_id,
        question_text=question.question_text,
        prompt_id="smoke",
        system_prompt=PROMPT_SPATIAL,
        model_fixture=SMOKE_MODEL_FIXTURE,
        run_id=run_id,
        db_path=store_dir / "chroma",
        collection_name=dataset,
    )
    answer = build_answer_record(observation)
    result = score(question, answer)

    shard = tmp_path / "shards" / "full-chain-smoke.jsonl"
    append_shard(shard, answer, result)
    cases = read_shard(shard)
    figure = render_figure(
        cases, tmp_path / "error_distribution.png", threshold_m=question.threshold_m
    )
    # The retrieval read-back is logged, never asserted: it is a diagnostic that
    # must not be able to fail a run (see ``conftest._replay_retrievals``), so
    # asserting it here would convert exactly the property it is built around
    # into a gate. What the shard holds is visible in the line below.
    logger.info(
        f"[full chain] {question.question_id}: {result.reason} "
        f"(ingested {SMOKE_SAMPLE_HZ} Hz slice, {figure.stat().st_size} byte figure, "
        f"retrievals {[(r.query, round(r.x, 3), round(r.y, 3)) for r in answer.retrievals]})"
    )

    assert observation.harness_error is None, observation.harness_error
    # The two outcomes a working harness can legitimately produce: a goal, or
    # the memory's similarity gate rejecting the query. Anything else means the
    # chain broke, not that the memory had nothing to offer.
    assert answer.outcome in {"predicted", "no_prediction"}, result.reason
    assert answer.tool_queries == [SMOKE_TOOL_QUERY], (
        "the recorded transcript did not reach the shipping navigation skill"
    )
    assert answer.model_id == f"model_fixture:{SMOKE_MODEL_FIXTURE.name}"
    assert cases[0].answer == answer
    assert cases[0].result == result
    assert figure.stat().st_size <= MAX_PNG_BYTES
