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

"""The runs that need a robot recording: the dataset sweep and a full-chain smoke.

Both lanes are ``self_hosted``. Everything the eval decides -- classification,
scoring, question rules, the figure -- is covered by the synthetic unit tests
next to this file; what is left here is the part that can only be checked by
actually driving the shipping agent, and it needs an ingested spatial memory
(and, for the sweep, an API key). The one exception is the store-provisioning
guard at the bottom of this file: it decides whether a machine skips or fails
*before* either lane runs, so a recording cannot be what checks it.

**The sweep** (``test_goal_selection_sweep``) is the measurement: every
recording the reference tree ships questions for x two system prompts (the
shipping default, and one that names the spatial memory) x two models, one JSONL
shard per case. It is a *tool-routing and query-formulation comparison*, not a
ranking of spatial ability -- the coordinates come from ``SpatialMemory``'s
deterministic CLIP top-1, and the model only decides whether to call the tool,
what query to send, and how many times. What it asserts is correspondingly
narrow: every question produced a paired, attributable record, and none of them
came back as something other than a measurement of the agent. The numbers live
in the shards.

The dataset arm is discovered, not listed: ``reference_sets.dataset_names()``
globs ``reference/*/questions.jsonl``, so a new recording enters the sweep by
having its artifact directory committed and nothing here changes. Each dataset's
cases answer from **that dataset's own store** and write into **that dataset's
own shard directory**, because a question set, a spatial memory and a figure are
only comparable within one recording.

**The smoke** (``test_full_chain_pipeline``) needs no key. It ingests a thin
slice of one replay, asks one question with a recorded model transcript, and
scores and plots the answer. It validates the *pipeline* -- ingest, harness,
memory query, goal capture, scoring, shard, figure -- and says nothing
whatsoever about model quality. It stays pinned to :data:`SMOKE_DATASET` while
the sweep fans out over every dataset: the transcript is a recording of one
model answering one question about one room, so a smoke that took its replay
from an environment variable would play a ``go2_bigoffice`` transcript against
whatever store it was pointed at and still pass.

The committed transcript. :data:`SMOKE_MODEL_FIXTURE` is a **hand-maintained
minimal transcript** in the repository's ``model_fixture`` format -- one tool
call and one closing message -- replayed by ``MockModel``. It is not a capture
of any model's output and is not evidence about one: it exists to drive the
pipeline through the shipping tool path deterministically and without a key.
It has to be updated whenever :data:`SMOKE_QUESTION_ID` changes, because the
query in it is that question's display name and the test asserts the two agree.

``MockModel`` also reads ``RECORD`` (``dimos/agents/testing/mock_model.py``):
with it set the fixture path is *written* from a live turn instead of replayed,
which is a convenient way to regenerate a starting point::

    RECORD=1 OPENAI_API_KEY=... uv run pytest \\
        dimos/agents/evals/test_spatial_goal_eval.py -m self_hosted -k full_chain

A transcript produced that way still has to be trimmed to the minimal shape
above and then replayed *without* ``RECORD`` to confirm it plays back keyless,
which is the state the lane ships in.

Running the sweep locally::

    # once per recording, into one root, one sub-directory per dataset (~1 min
    # each). --collection is left off deliberately: ingest.py defaults it to the
    # dataset name, which is the name the sweep then looks the collection up by.
    ROOT=~/.local/state/dimos/spatial-eval
    uv run python -m dimos.agents.evals.ingest \\
        --dataset go2_bigoffice --out-dir $ROOT/go2_bigoffice
    uv run python -m dimos.agents.evals.ingest \\
        --dataset go2_short --out-dir $ROOT/go2_short

    DIMOS_EVAL_STORE_ROOT=$ROOT \\
    DIMOS_EVAL_SHARD_DIR=.ignore.eval-shards \\
    uv run pytest dimos/agents/evals/test_spatial_goal_eval.py -m self_hosted

A root holding only *some* of the datasets is a supported state, not an error:
the cases of a dataset with no store skip with the command that builds it, and
the rest of the sweep runs. Two states do fail rather than skip, because both
mean the requested live run cannot happen at all: a
``DIMOS_EVAL_STORE_ROOT`` that does not exist (a typo), and one that exists but
holds no store for *any* discovered dataset (wrong root, or an ingest that never
ran). Skipping either would leave the lane green having measured nothing.

Shards land in ``$DIMOS_EVAL_SHARD_DIR/<dataset>/``, and **the renderer is
invoked once per dataset directory**::

    uv run python -m dimos.agents.evals.render \\
        --shards .ignore.eval-shards/go2_bigoffice \\
        --out error_distribution_go2_bigoffice.png --threshold-m 3.5

One figure per recording, because the x axis is a distance error against that
recording's own references: pooling two recordings' dots into one plot would
average over two different rooms, two question sets and two threshold ranges.
Thresholds are per question (``questions.THRESHOLD_MARGIN_M``); ``--threshold-m``
draws the cap as a reference line, not the line any single dot was scored
against.

Note for a first run on a fresh machine: ``navigate_with_text`` tries tagged
locations before the semantic map, and that path makes ChromaDB instantiate
its default embedding function, which lazily downloads ~80 MB into
``~/.cache/chroma`` even for an empty collection. Warm that cache before
timing anything (a self-hosted runner needs egress to
``chroma-onnx-models.s3.amazonaws.com``).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from dimos.agents.evals.ingest import CHROMA_DIR_NAME, dataset_store, ingest_command
from dimos.agents.evals.questions import slugify
from dimos.agents.evals.reference_sets import dataset_names, load_question_set
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

#: Root of the per-dataset stores the sweep answers from: one sub-directory per
#: dataset, each holding what ``ingest.py --out-dir <root>/<dataset>`` wrote
#: (``ingest.dataset_store`` resolves one). A single root rather than a
#: ``db_path``/``collection`` pair because with several recordings the pair
#: would have to be re-exported per dataset, and the two halves could be made
#: to disagree -- a ``go2_short`` collection name against a ``go2_bigoffice``
#: chroma directory attaches to an *empty* collection and scores every question
#: as no_prediction.
STORE_ROOT_ENV = "DIMOS_EVAL_STORE_ROOT"

#: Where shards are written: one sub-directory per dataset underneath.
SHARD_DIR_ENV = "DIMOS_EVAL_SHARD_DIR"

#: The dataset arm of the sweep: every recording the committed reference tree
#: ships a question set for, discovered at import so ``--collect-only`` shows
#: the real case list. Sorted, so case ids -- and the shard directories named
#: after them -- do not depend on directory iteration order.
DATASETS = dataset_names()

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

#: The smoke test's recording, question, and the query in its committed
#: transcript. The transcript is written against *this* question of *this*
#: dataset -- the query below is the question's own ``display_name``, which is
#: what a model answering it would send to ``navigate_with_text`` -- so the
#: playback exercises the shipping skill on the question the shard says it ran.
#: Pinned rather than swept or read from the environment: a transcript is not
#: transferable between recordings. Update it after changing any of the three
#: (see the docstring above).
SMOKE_DATASET = "go2_bigoffice"
SMOKE_QUESTION_ID = "go2-bigoffice-organization"
SMOKE_TOOL_QUERY = "stack of storage boxes"
SMOKE_MODEL_FIXTURE = Path(__file__).parent / "fixtures" / "test_full_chain_pipeline.json"

#: Frames per second of *recorded* time for the smoke ingest: enough of the
#: replay to exercise the store, few enough to keep the lane honest about its
#: runtime. The sweep's own store is built at the ``ingest.py`` default.
SMOKE_SAMPLE_HZ = 0.2
SMOKE_INGEST_TIMEOUT_S = 600.0


def shard_path(directory: Path, model_id: str, prompt_id: str, run_id: str) -> Path:
    """A shard filename no other case can produce.

    Contract: one file per pytest case. The configuration names the file so a
    directory of shards is readable without opening them, and the run stamp --
    the same ``run_id`` the records inside carry -- keeps repeated runs (and
    parallel workers) from appending to each other. Filename and content agree by
    construction, so a shard can be attributed to its run without opening it.
    The dataset is not in the name because it is the *directory*: the renderer
    consumes one dataset's directory at a time.
    """
    return directory / f"{slugify(model_id)}__{prompt_id}__{run_id}.jsonl"


@pytest.fixture(scope="module")
def run_id() -> str:
    """One stamp for the whole sweep: its cases are arms of a single run.

    Module-scoped on purpose. The (dataset, model, prompt) cases are one sweep,
    and the renderer aggregates by ``run_id`` -- giving each case its own would
    make a repeat of the sweep indistinguishable from the sweep itself. It spans
    datasets as well as arms: the same stamp appearing under two dataset
    directories is what says the two were measured together.
    """
    return f"{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"


def resolve_ingested_store(dataset: str, datasets: Sequence[str] = DATASETS) -> Path:
    """Resolve one dataset's chroma directory under ``$DIMOS_EVAL_STORE_ROOT``.

    *datasets* is every dataset the sweep is fanned out over; it is a parameter
    only so the guard below can be tested without a recording.

    The four outcomes are deliberately different:

    * **root unset** -- skip. The store is a machine-local artifact an operator
      produces once, not a dependency of the package; this is the same shape as
      the repository's other recording-backed self-hosted tests
      (``dimos/navigation/cmu_nav/tests/rosbag_fixtures.py``).
    * **root set but absent** -- fail. That is a typo, not an unprovisioned
      machine, and silently skipping a whole sweep because of one is how a
      "green" self-hosted lane ends up measuring nothing.
    * **root present and holding no store at all** -- fail. Setting the variable
      is a request for a live run, and a root under which *no* discovered
      dataset can answer cannot serve one: every case would skip and the lane
      would report green having measured nothing, which is the same failure the
      absent-root branch exists to prevent. Wrong root, or an ingest that never
      ran.
    * **root present, some dataset's store absent** -- skip *those* cases, with
      the command that builds them. Provisioning is per recording, so a machine
      that has ingested one dataset must still be able to sweep it, and a
      partly-provisioned root is a supported state rather than an error.

    The collection name is the dataset name (``ingest.py``'s own default), and
    ``conftest`` still runs ``assert_store_ingested`` against it before building
    anything -- a store directory that exists but holds the wrong or an empty
    collection is a misconfiguration and raises there.
    """
    root = os.environ.get(STORE_ROOT_ENV)
    if not root:
        pytest.skip(
            f"no ingested spatial memory: set {STORE_ROOT_ENV} to the directory "
            f"holding one store per dataset. Build this one with: "
            f"{ingest_command('<store root>', dataset)}"
        )
    store_root = Path(root).expanduser()
    if not store_root.is_dir():
        pytest.fail(
            f"{STORE_ROOT_ENV}={root!r} does not exist; it must be the directory "
            f"holding one dimos.agents.evals.ingest output per dataset"
        )
    db_path = dataset_store(store_root, dataset)
    if db_path.is_dir():
        return db_path
    provisioned = [name for name in datasets if dataset_store(store_root, name).is_dir()]
    if not provisioned:
        pytest.fail(
            f"{STORE_ROOT_ENV}={root!r} exists but holds no ingested store for any of "
            f"{list(datasets)}: every case of the sweep would skip and the lane would "
            f"report green having measured nothing. Build one with: "
            f"{ingest_command(store_root, dataset)}"
        )
    pytest.skip(
        f"no ingested store for dataset {dataset!r}: {db_path} does not exist "
        f"(the sweep still runs {provisioned}). Build it with: "
        f"{ingest_command(store_root, dataset)}"
    )


@pytest.fixture
def ingested_store() -> Callable[[str], Path]:
    """The store resolver, as a fixture: a callable, because the answer is per case.

    See :func:`resolve_ingested_store` for what each provisioning state does.
    """
    return resolve_ingested_store


@pytest.fixture
def shard_root(tmp_path: Path) -> Path:
    """Root of the shard tree: ``$DIMOS_EVAL_SHARD_DIR``, else throwaway.

    Cases write into ``<root>/<dataset>/``; the renderer is pointed at one of
    those sub-directories at a time.
    """
    return Path(os.environ.get(SHARD_DIR_ENV) or tmp_path / "shards").expanduser()


@pytest.mark.self_hosted
@pytest.mark.skipif_no_openai
# Every question x (a real LLM turn + a coordinator build that loads CLIP in the
# memory worker) outlasts the default per-test timeout.
@pytest.mark.timeout(2400)
@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize(("prompt_id", "system_prompt"), PROMPTS, ids=[p[0] for p in PROMPTS])
@pytest.mark.parametrize("dataset", DATASETS)
def test_goal_selection_sweep(
    spatial_eval_setup: Callable[..., RunObservation],
    ingested_store: Callable[[str], Path],
    shard_root: Path,
    run_id: str,
    dataset: str,
    model_id: str,
    prompt_id: str,
    system_prompt: str,
) -> None:
    """Ask one recording's questions under one (model, prompt) arm and record it.

    This case is one row of that dataset's figure. It asserts only what would
    mean the measurement is invalid -- a missing question, a record attributed
    to the wrong configuration, or a failure of the *measurement* rather than of
    the agent. Whether the agent finds the elevator is the *result*, and lives
    in the shard, not in an assertion.

    The timeout is per case, i.e. per (dataset, model, prompt): adding a
    recording adds cases rather than lengthening one, so the budget a single
    case needs does not move when the sweep grows.
    """
    db_path = ingested_store(dataset)
    question_set = load_question_set(dataset)
    shard_dir = shard_root / dataset
    shard_dir.mkdir(parents=True, exist_ok=True)
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
            collection_name=dataset,
        )
        answer = build_answer_record(observation)
        append_shard(out, answer, score(question, answer))

    cases = read_shard(out)
    results = [case.result for case in cases]
    logger.info(
        f"[{dataset} / {model_id} / {prompt_id}] {out}: "
        f"n_pred {len(errors_m(results))}/{len(results)}, "
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
    run_id: str,
    tmp_path: Path,
) -> None:
    """Replay -> store -> agent -> goal -> score -> shard -> figure, with no API key.

    **This validates the pipeline, not any model.** The agent's replies come
    from :data:`SMOKE_MODEL_FIXTURE` -- a hand-maintained minimal transcript in
    the repository's ``model_fixture`` format, replayed by ``MockModel`` -- so
    the tool call and the final message are fixed and are not evidence about a
    model. What is under test is that a store ingested from the replay answers a
    real ``navigate_with_text`` call, that the goal the skill sets is captured
    out of its worker process, and that the result survives scoring, the shard
    format and the renderer. It cannot fail because a model is bad, and it
    cannot pass because a model is good.

    It therefore validates the *mechanism*, not the answer, and deliberately does
    not assert ``passed``: the store is a thin slice of the replay, so the top-1
    frame CLIP retrieves for :data:`SMOKE_TOOL_QUERY` may well be metres from the
    reference, and a scoring FAIL is an acceptable outcome of a working chain.
    Asserting the verdict here would make the smoke lane fail whenever the slice
    got thinner, which is a property of the fixture, not a regression.

    The store is a thin slice of the replay (``SMOKE_SAMPLE_HZ``), ingested
    through the shipped ``ingest.py`` entry point in its own process: that is
    how an operator builds one, and it keeps CLIP and ChromaDB's thread pool
    out of the pytest process. It is :data:`SMOKE_DATASET`'s replay and only
    ever that one -- the transcript being replayed was recorded against a
    question of that recording -- so this lane does not fan out with the sweep,
    and it builds its own throwaway store rather than reading
    ``$DIMOS_EVAL_STORE_ROOT``.
    """
    replay = get_data_dir() / f"{SMOKE_DATASET}.db"
    if not replay.exists():
        pytest.skip(
            f"replay {replay} is not present; pull it with: git lfs pull "
            f'--include="data/.lfs/{SMOKE_DATASET}.db.tar.gz"'
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
            SMOKE_DATASET,
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

    question_set = load_question_set(SMOKE_DATASET)
    question = next(q for q in question_set if q.question_id == SMOKE_QUESTION_ID)
    # The committed transcript's query is this question's display name. Checked
    # before the run so a rename in review surfaces as "re-write the transcript"
    # rather than as a tool_queries mismatch after a full ingest.
    assert question.display_name == SMOKE_TOOL_QUERY, (
        f"{SMOKE_QUESTION_ID} is now named {question.display_name!r}; "
        f"{SMOKE_MODEL_FIXTURE.name} still queries {SMOKE_TOOL_QUERY!r}"
    )
    observation = spatial_eval_setup(
        question_id=question.question_id,
        question_text=question.question_text,
        prompt_id="smoke",
        system_prompt=PROMPT_SPATIAL,
        model_fixture=SMOKE_MODEL_FIXTURE,
        run_id=run_id,
        db_path=store_dir / CHROMA_DIR_NAME,
        collection_name=SMOKE_DATASET,
    )
    answer = build_answer_record(observation)
    result = score(question, answer)

    shard = tmp_path / "shards" / "full-chain-smoke.jsonl"
    append_shard(shard, answer, result)
    cases = read_shard(shard)
    figure = render_figure(
        cases, tmp_path / "error_distribution.png", threshold_m=question.threshold_m
    )
    # The retrieval read-back is logged, never asserted: it is a diagnostic
    # whose every exception is swallowed (see ``conftest._replay_retrievals``),
    # so asserting it here would convert exactly the property it is built around
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
        "the committed transcript did not reach the shipping navigation skill"
    )
    assert answer.model_id == f"model_fixture:{SMOKE_MODEL_FIXTURE.name}"
    assert cases[0].answer == answer
    assert cases[0].result == result
    assert figure.stat().st_size <= MAX_PNG_BYTES


def _provision(root: Path, *datasets: str) -> Path:
    """Lay out *root* as an operator's store root holding *datasets*' stores."""
    for dataset in datasets:
        dataset_store(root, dataset).mkdir(parents=True)
    return root


def test_a_store_root_with_no_store_at_all_fails_rather_than_skipping_everything(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Setting the variable requests a live run; an empty root cannot serve one.

    This is the state that used to produce a full sweep of clean skips: the
    lane went green having measured nothing, which is indistinguishable from a
    lane nobody had provisioned on purpose. It is a misconfiguration -- the
    wrong root, or an ingest that never ran -- so it fails, and the failure
    names the command that builds a store.
    """
    monkeypatch.setenv(STORE_ROOT_ENV, str(tmp_path))

    with pytest.raises(pytest.fail.Exception) as excinfo:
        resolve_ingested_store("go2_short", datasets=("go2_short", "go2_bigoffice"))

    message = str(excinfo.value)
    assert "holds no ingested store" in message
    assert "dimos.agents.evals.ingest" in message


def test_a_partly_provisioned_root_skips_only_the_datasets_it_cannot_answer(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Provisioning is per recording, so one ingested dataset still sweeps.

    The pair is the point: the same root that *skips* the dataset it has no
    store for must *resolve* the one it does, so the guard above cannot be
    satisfied by failing on every partly-provisioned machine.
    """
    monkeypatch.setenv(STORE_ROOT_ENV, str(_provision(tmp_path, "go2_short")))
    datasets = ("go2_bigoffice", "go2_short")

    assert resolve_ingested_store("go2_short", datasets=datasets) == dataset_store(
        tmp_path, "go2_short"
    )

    with pytest.raises(pytest.skip.Exception) as excinfo:
        resolve_ingested_store("go2_bigoffice", datasets=datasets)
    assert "['go2_short']" in str(excinfo.value)


def test_an_unset_root_skips_and_a_missing_one_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The two states that bracket the ones above, kept honest together.

    An unprovisioned machine is not a broken one -- the store is a local
    artifact, so no variable means skip -- while a variable pointing at nothing
    is a typo, and skipping on it is how a self-hosted lane silently stops
    measuring.
    """
    monkeypatch.delenv(STORE_ROOT_ENV, raising=False)
    with pytest.raises(pytest.skip.Exception):
        resolve_ingested_store("go2_short")

    monkeypatch.setenv(STORE_ROOT_ENV, str(tmp_path / "typo"))
    with pytest.raises(pytest.fail.Exception, match="does not exist"):
        resolve_ingested_store("go2_short")
