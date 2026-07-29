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
    (model, prompt) it ran under.

``ScoreResult``
    The scored outcome, shaped ``{passed, reason, score}`` to match the
    DimSim rubric contract, plus the continuous ``error_m``.

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

from dataclasses import asdict, dataclass
import json
from typing import Literal

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
    tolerance: see ``questions.DEFAULT_THRESHOLD_M`` for why a viewpoint-shaped
    goal cannot be scored against a tighter one.
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
    threshold_m: float = 3.5

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, line: str) -> QuestionSpec:
        return cls(**json.loads(line))


@dataclass(frozen=True)
class AnswerRecord:
    """Observed agent behavior for one question under one configuration.

    ``goal_x``/``goal_y``/``goal_yaw`` are the first recorded goal when
    ``n_goals >= 1`` and ``None`` otherwise. ``tool_queries`` preserves the
    exact query strings the agent passed to the navigation skill, in order.
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
    wall_time_s: float
    error: str | None = None

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, line: str) -> AnswerRecord:
        return cls(**json.loads(line))


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
