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

"""Scoring for the replay-backed spatial goal-selection evals.

Everything here is a pure function of its arguments (the JSONL shard helpers
at the bottom are the only I/O), so the whole scoring path is unit-testable
without a robot, a model, or a database.

The pipeline is::

    RunObservation  --classify_outcome-->  Outcome
    RunObservation  --build_answer_record-->  AnswerRecord
    QuestionSpec + AnswerRecord  --score-->  ScoreResult

``RunObservation`` is the raw, unclassified thing the eval fixture hands back:
what the agent did, with no judgement attached. Classification lives here so
the fixture cannot quietly decide what counts as a failure.

Scoring rules (frozen, see ``contracts.py``):

* Error is **map-frame XY distance** only -- ``navigate_with_text`` emits goals
  with ``z == 0``, so a 3-D distance would penalize elevated reference points.
* ``passed`` requires a single prediction within ``question.threshold_m``.
* Everything that is not a clean single prediction scores ``error_m=None`` and
  stays in the pass-rate denominator; only the error plot drops it.
* Outcomes separate agent behavior (``no_prediction``, ``multiple_predictions``)
  from broken measurements (``tool_error``, ``harness_error``) so a broken run
  cannot masquerade as a bad model.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from dimos.agents.evals.contracts import AnswerRecord, Outcome, QuestionSpec, ScoreResult


@dataclass(frozen=True)
class GoalRecord:
    """One navigation goal intercepted from the agent, in the map frame."""

    x: float
    y: float
    z: float
    yaw: float
    frame_id: str | None = None

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> GoalRecord:
        """Build from the plain dict `RecordingStubNavigation.get_goals()` returns.

        >>> GoalRecord.from_mapping({"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5})
        GoalRecord(x=1.0, y=2.0, z=0.0, yaw=0.5, frame_id=None)
        """
        return cls(
            x=float(raw["x"]),
            y=float(raw["y"]),
            z=float(raw["z"]),
            yaw=float(raw["yaw"]),
            frame_id=raw.get("frame_id"),
        )


@dataclass(frozen=True)
class RunObservation:
    """Raw, unclassified result of running one question through the harness.

    Carries enough signal to tell all six outcomes apart without the fixture
    having to decide anything:

    * ``harness_error`` -- the harness itself failed (build, RPC, teardown).
    * ``finished`` -- the runner published `finished` before the outer wait
      expired; ``False`` means the agent never completed its turn.
    * ``tool_errors`` -- exceptions raised inside ``navigate_with_text``.
    * ``goals`` -- goals intercepted by ``RecordingStubNavigation``.
    * ``tool_queries`` -- the exact query strings the agent sent, in order.

    ``agent_wall_time_s`` is the turn alone; ``total_wall_time_s`` includes
    building the coordinator (dominated by loading CLIP in the memory worker),
    which is harness cost and must not be reported as model latency.
    """

    question_id: str
    model_id: str
    prompt_id: str
    prompt_sha256: str
    finished: bool
    agent_wall_time_s: float
    total_wall_time_s: float
    goals: list[GoalRecord] = field(default_factory=list)
    tool_queries: list[str] = field(default_factory=list)
    tool_errors: list[str] = field(default_factory=list)
    harness_error: str | None = None


def prompt_sha256(system_prompt: str | None) -> str:
    """Hash a system prompt so a shard can prove which prompt text produced it.

    >>> prompt_sha256("you are a robot")[:12]
    '71bc24ae1366'
    """
    return hashlib.sha256((system_prompt or "").encode("utf-8")).hexdigest()


def classify_outcome(observation: RunObservation) -> Outcome:
    """Map a raw observation onto the six-state outcome enum.

    The ladder is ordered by how much it invalidates the measurement, so a
    broken measurement is never reported as agent behavior:

    1. ``harness_error`` -- the harness failed; nothing about the agent is known.
    2. ``answer_timeout`` -- the turn never completed.
    3. ``tool_error``    -- the navigation skill raised; the goal read-back is
       not a clean read of the agent's choice.
    4. ``multiple_predictions`` -- more than one goal was set.
    5. ``predicted``     -- exactly one goal.
    6. ``no_prediction`` -- no goal at all (e.g. the memory's similarity gate
       rejected the query). A legitimate, expected agent outcome.

    >>> classify_outcome(RunObservation("q", "m", "p", "s", True, 1.0, 2.0))
    'no_prediction'
    """
    if observation.harness_error is not None:
        return "harness_error"
    if not observation.finished:
        return "answer_timeout"
    if observation.tool_errors:
        return "tool_error"
    if len(observation.goals) > 1:
        return "multiple_predictions"
    if len(observation.goals) == 1:
        return "predicted"
    return "no_prediction"


def _observation_detail(observation: RunObservation, outcome: Outcome) -> str | None:
    """Free-text detail for `AnswerRecord.error`, or None when there is nothing to say."""
    if outcome == "harness_error":
        return observation.harness_error
    if outcome == "answer_timeout":
        return (
            f"agent did not finish its turn within the fixture's outer wait "
            f"({observation.agent_wall_time_s:.1f}s elapsed)"
        )
    if outcome == "tool_error":
        return "; ".join(observation.tool_errors)
    if outcome == "multiple_predictions":
        goals = "; ".join(f"({g.x:.3f}, {g.y:.3f})" for g in observation.goals)
        return f"{len(observation.goals)} goals recorded: {goals}"
    return None


def build_answer_record(observation: RunObservation) -> AnswerRecord:
    """Turn a raw observation into the persisted ``AnswerRecord``.

    The first goal is the scored one; when several were set they are all listed
    in ``error`` so the shard keeps the evidence for ``multiple_predictions``.
    ``wall_time_s`` is the turn only -- coordinator build time is harness cost.
    """
    outcome = classify_outcome(observation)
    first = observation.goals[0] if observation.goals else None
    return AnswerRecord(
        question_id=observation.question_id,
        outcome=outcome,
        goal_x=first.x if first else None,
        goal_y=first.y if first else None,
        goal_yaw=first.yaw if first else None,
        n_goals=len(observation.goals),
        tool_invoked=bool(observation.tool_queries),
        tool_queries=list(observation.tool_queries),
        model_id=observation.model_id,
        prompt_id=observation.prompt_id,
        prompt_sha256=observation.prompt_sha256,
        wall_time_s=observation.agent_wall_time_s,
        error=_observation_detail(observation, outcome),
    )


def score(question: QuestionSpec, answer: AnswerRecord) -> ScoreResult:
    """Score one answer against its reference location.

    ``predicted`` answers get the map-frame XY distance to the reference and
    pass when it is within ``question.threshold_m``. Every other outcome fails
    with ``error_m=None`` and keeps the detail from the answer record.

    >>> q = QuestionSpec("q1", "sofa", "couch", "Where is the sofa?",
    ...                  1.0, 2.0, 0.4, 3, 0.2, "renamed", threshold_m=1.5)
    >>> a = AnswerRecord("q1", "predicted", 1.5, 2.5, 0.0, 1, True, ["sofa"],
    ...                  "gpt-5.6-luna", "baseline", "0" * 64, 3.2)
    >>> r = score(q, a)
    >>> r.passed, round(r.error_m, 3)
    (True, 0.707)
    """
    if answer.question_id != question.question_id:
        raise ValueError(
            f"answer is for question {answer.question_id!r}, "
            f"not {question.question_id!r}; shards must be paired by question_id"
        )

    if answer.outcome != "predicted":
        detail = f": {answer.error}" if answer.error else ""
        return ScoreResult(
            question_id=question.question_id,
            passed=False,
            reason=f"{answer.outcome}{detail}",
            score=0.0,
            error_m=None,
            outcome=answer.outcome,
        )

    if answer.goal_x is None or answer.goal_y is None:
        raise ValueError(
            f"answer {answer.question_id!r} is 'predicted' but carries no goal; "
            "build_answer_record only emits 'predicted' with exactly one goal"
        )

    error_m = math.hypot(answer.goal_x - question.ref_x, answer.goal_y - question.ref_y)
    passed = error_m <= question.threshold_m
    return ScoreResult(
        question_id=question.question_id,
        passed=passed,
        reason=(f"map-frame XY error {error_m:.3f} m vs threshold {question.threshold_m:.3f} m"),
        score=1.0 if passed else 0.0,
        error_m=error_m,
        outcome="predicted",
    )


def pass_rate(results: Sequence[ScoreResult]) -> float:
    """Fraction of questions passed, over **all** questions.

    Every outcome stays in the denominator: an agent that answers nothing is
    not thereby perfect. Returns 0.0 for an empty sequence.

    >>> pass_rate([])
    0.0
    """
    if not results:
        return 0.0
    return sum(1 for r in results if r.passed) / len(results)


def no_prediction_rate(results: Sequence[ScoreResult]) -> float:
    """Fraction of questions where the agent set no goal at all.

    Returns 0.0 for an empty sequence.
    """
    if not results:
        return 0.0
    return sum(1 for r in results if r.outcome == "no_prediction") / len(results)


def errors_m(results: Iterable[ScoreResult]) -> list[float]:
    """The measured errors, in input order, skipping outcomes that have none.

    This is the sample the error-distribution plot draws; ``len()`` of it is
    the ``n_pred`` the plot reports next to ``n_total``.
    """
    return [r.error_m for r in results if r.error_m is not None]


# --- JSONL shards -----------------------------------------------------------
#
# One shard file per pytest case (contract: parallel cases must never share a
# file). Each line is a tagged envelope so answers and scores can live in the
# same file and be read back without a side-channel schema:
#
#     {"kind": "answer", "data": {...AnswerRecord...}}
#     {"kind": "score",  "data": {...ScoreResult...}}
#
# A configuration (one point series in the plot) is identified by the
# ``(model_id, prompt_id)`` pair carried on every answer record.

_KIND_ANSWER = "answer"
_KIND_SCORE = "score"


@dataclass(frozen=True)
class ScoredCase:
    """One question's answer and its score, as read back from a shard."""

    answer: AnswerRecord
    result: ScoreResult


def append_shard(path: str | Path, answer: AnswerRecord, result: ScoreResult) -> None:
    """Append one scored case to a shard file, creating it if needed."""
    if answer.question_id != result.question_id:
        raise ValueError(
            f"answer {answer.question_id!r} and result {result.question_id!r} "
            "belong to different questions"
        )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(
            json.dumps({"kind": _KIND_ANSWER, "data": json.loads(answer.to_json())}) + "\n"
        )
        handle.write(json.dumps({"kind": _KIND_SCORE, "data": json.loads(result.to_json())}) + "\n")


def read_shard(path: str | Path) -> list[ScoredCase]:
    """Read one shard file back into paired records, in file order.

    Raises ``ValueError`` on an unknown record kind or an unpaired record --
    a half-written shard is a bug worth failing on, not a plot with holes.
    """
    answers: dict[str, AnswerRecord] = {}
    results: dict[str, ScoreResult] = {}
    order: list[str] = []

    for lineno, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        envelope = json.loads(line)
        kind, data = envelope.get("kind"), envelope.get("data")
        if kind == _KIND_ANSWER:
            answer = AnswerRecord(**data)
            if answer.question_id in answers:
                raise ValueError(
                    f"{path}:{lineno}: duplicate answer for question "
                    f"{answer.question_id!r} — a shard holds one run of one "
                    "configuration, so a repeat means two runs were mixed into one file"
                )
            answers[answer.question_id] = answer
            if answer.question_id not in order:
                order.append(answer.question_id)
        elif kind == _KIND_SCORE:
            result = ScoreResult(**data)
            if result.question_id in results:
                raise ValueError(
                    f"{path}:{lineno}: duplicate score for question "
                    f"{result.question_id!r} — a shard holds one run of one "
                    "configuration, so a repeat means two runs were mixed into one file"
                )
            results[result.question_id] = result
            if result.question_id not in order:
                order.append(result.question_id)
        else:
            raise ValueError(f"{path}:{lineno}: unknown record kind {kind!r}")

    unpaired = sorted(set(answers) ^ set(results))
    if unpaired:
        raise ValueError(f"{path}: unpaired records for question(s) {unpaired}")

    return [ScoredCase(answer=answers[qid], result=results[qid]) for qid in order]


def read_shards(paths: Iterable[str | Path]) -> list[ScoredCase]:
    """Read several shard files into one flat list, in the order given."""
    cases: list[ScoredCase] = []
    for path in paths:
        cases.extend(read_shard(path))
    return cases
