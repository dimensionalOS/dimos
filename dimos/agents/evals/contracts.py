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

"""Data contracts for the replay-backed spatial goal-selection evals.

Three records flow through the eval pipeline:

``QuestionSpec``
    One "Where is X" question derived offline from a replay. Carries the
    reference location in the map frame plus the provenance needed to audit
    it (detector label vs. reviewed display name, view count, spread).

``AnswerRecord``
    What the agent under test actually did for one question: the navigation
    goal(s) it set, the tool queries it issued, and the run configuration
    (model, prompt, run) it ran under. It also carries a list of
    ``RetrievalRecord``\\ s -- what the memory returned for those queries --
    which is **observability, not scoring**: see :class:`RetrievalRecord`.

``ScoreResult``
    The scored outcome, plus the continuous ``error_m``. Its ``{passed, reason,
    score}`` *field shape* mirrors DimSim's ``EvalSuccess`` so a reader of one
    surface is not surprised by the other, but the semantics differ and the two
    do not aggregate: DimSim's ``score`` is a continuous lower-is-better
    distance that goes to ``Infinity`` on failure, while here ``score`` is
    binary (``1.0`` iff ``passed``) and the distance lives in ``error_m``, which
    is ``None`` -- not zero and not infinite -- whenever there is nothing to
    measure. Averaging the two ``score`` columns together would be meaningless.

Scoring semantics (fixed):

* Distance is **map-frame XY only** — ``navigate_with_text`` always emits
  goals with ``z == 0`` (``dimos/agents/skills/navigation.py``), so a 3-D
  distance would silently penalize elevated reference points.
* ``no_prediction`` (the agent never set a goal, e.g. the similarity gate
  rejected its query) is a legitimate, expected outcome: it scores
  ``passed=False`` with ``error_m=None``, stays in the denominator of pass
  rate, and is excluded only from the error-distribution plot.
* Outcomes distinguish agent failures from harness failures so a broken run
  cannot masquerade as a bad model: the rates in ``scorer.py`` divide by the
  three agent-attributable outcomes only, and the renderer prints how many
  results were excluded as broken.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
import json
from typing import Any, Literal

Outcome = Literal[
    "predicted",
    "no_prediction",
    "multiple_predictions",
    "answer_timeout",
    "tool_error",
    "harness_error",
]

OUTCOMES: tuple[Outcome, ...] = (
    "predicted",
    "no_prediction",
    "multiple_predictions",
    "answer_timeout",
    "tool_error",
    "harness_error",
)

HumanReview = Literal["verified", "renamed"]


@dataclass(frozen=True)
class QuestionSpec:
    """One position question with its replay-derived reference location.

    ``display_name`` is the reviewed name used in ``question_text``;
    ``raw_label`` preserves the original detector label so renames stay
    auditable, and is also what ``question_id`` is built from, so a rename in
    review cannot break a longitudinal join. Questions whose labels were
    rejected in review are dropped upstream and never become specs.

    ``threshold_m`` is an *observation envelope*, not a point-accuracy
    tolerance: see ``questions.THRESHOLD_MARGIN_M`` for why a viewpoint-shaped
    goal cannot be scored against a tighter one. It is **per question** -- this
    question's own nearest teacher viewpoint plus that margin, capped -- because
    the viewing distances are: one radius for the whole set is either too tight
    for the object seen from across the room or wide enough, around an object the
    robot nearly touched, to also accept its neighbours. There is deliberately no
    class default: a spec built without a threshold would silently be scored
    under a radius nobody derived for it.
    """

    question_id: str
    display_name: str
    raw_label: str
    question_text: str
    ref_x: float
    ref_y: float
    ref_z: float
    n_views: int
    spread_m: float
    human_review: HumanReview
    threshold_m: float

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, line: str) -> QuestionSpec:
        return cls(**json.loads(line))


@dataclass(frozen=True)
class RetrievalRecord:
    """What the spatial memory returned for one of the agent's tool queries.

    **Diagnostic observability, not a pass criterion.** Nothing in ``scorer.py``
    reads these; the scored answer channel is the goal pose alone. They exist
    because the shipping answer channel is only ``(x, y, yaw)``, which cannot
    distinguish "found the right object" from "found a neighbour that happens to
    sit inside the pass radius" -- exactly the failure the confusability gate in
    ``questions.py`` currently has to prevent by *dropping questions*. Recording
    what was retrieved is what would make identity-level scoring possible later,
    and it has to be in the shard schema before that schema goes public.

    They are recorded by a **post-hoc re-query**, after the agent's turn, not
    intercepted from it: ``SpatialMemory.query_by_text`` is a CLIP embedding
    against a fixed, read-only store, so re-asking the same text reproduces the
    retrieval the agent got. That is a measured property of a frozen store, and
    the reason a re-query is admissible evidence rather than a second sample.

    ``x``/``y`` are the map-frame pose of the retrieved *frame* (``pos_x`` /
    ``pos_y`` of the chroma metadata) -- the same viewpoint
    ``navigate_with_text`` turns into its goal, which is why a populated record
    matches the recorded goal's XY rather than the object's centroid.

    ``distance`` is chroma's raw distance for the top hit, as
    ``query_by_text`` returns it: **lower is closer**, and it is not a
    similarity. The shipping skill converts it (``similarity = 1.0 - distance``
    in ``dimos/agents/skills/navigation.py``) before comparing against its
    threshold; the raw number is stored so that conversion stays a property of
    the code under test rather than of the shard. A result carrying no distance
    at all is recorded as ``1.0``, which is the same substitution the shipping
    skill makes and reads as "no usable similarity".
    """

    query: str
    x: float
    y: float
    distance: float


@dataclass(frozen=True)
class AnswerRecord:
    """Observed agent behavior for one question under one configuration.

    ``goal_x``/``goal_y``/``goal_yaw`` are the first recorded goal when
    ``n_goals >= 1`` and ``None`` otherwise. ``tool_queries`` preserves the
    exact query strings the agent passed to the navigation skill, in order.

    ``run_id`` identifies the *sweep*, not the configuration: repeating the same
    ``(model_id, prompt_id)`` is how a stochastic agent gets measured, and
    without it two runs are indistinguishable records of one. It is required
    rather than defaulted -- an untagged record could not be told apart from a
    repeat of an existing one, which is exactly the mistake it exists to prevent.

    ``retrievals`` is diagnostic only and is **never scored** (see
    :class:`RetrievalRecord`). It defaults to empty, and empty is not evidence of
    anything: the agent may not have called the tool at all, or the re-query may
    have been unavailable.
    """

    question_id: str
    outcome: Outcome
    goal_x: float | None
    goal_y: float | None
    goal_yaw: float | None
    n_goals: int
    tool_invoked: bool
    tool_queries: list[str]
    model_id: str
    prompt_id: str
    prompt_sha256: str
    run_id: str
    wall_time_s: float
    error: str | None = None
    retrievals: list[RetrievalRecord] = field(default_factory=list)

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, line: str) -> AnswerRecord:
        return cls.from_dict(json.loads(line))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> AnswerRecord:
        """Rebuild from an already-decoded shard record.

        Needed because ``retrievals`` is nested: ``asdict`` flattens it to plain
        dicts, and a bare ``cls(**data)`` would hand those back as dicts -- a
        record that compares unequal to the one that was written and whose
        ``.query`` attribute does not exist. Every read path (``from_json`` and
        ``scorer.read_shard``) goes through here, so a shard round trip stays an
        identity rather than being one on only the path that has a test.

        A line written before ``retrievals`` existed has no such key and gets the
        empty default, so shards from earlier runs stay readable. That is
        deliberate rather than merely convenient: an absent field and an empty
        list mean the same thing here -- no retrieval was recorded -- so there is
        nothing for a stricter reader to catch.
        """
        fields = dict(data)
        fields["retrievals"] = [
            item if isinstance(item, RetrievalRecord) else RetrievalRecord(**item)
            for item in fields.get("retrievals") or []
        ]
        return cls(**fields)


@dataclass(frozen=True)
class ScoreResult:
    """Scored outcome for one question, aligned with the DimSim rubric shape.

    ``score`` is binary (``1.0`` iff ``passed``). ``error_m`` is the XY
    distance from the recorded goal to the reference, and is ``None``
    whenever there is no single usable prediction (every outcome except
    ``predicted``).
    """

    question_id: str
    passed: bool
    reason: str
    score: float
    error_m: float | None
    outcome: Outcome

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, line: str) -> ScoreResult:
        return cls(**json.loads(line))
