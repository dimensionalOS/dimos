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

# Copyright 2026 Dimensional Inc.
from pathlib import Path

import pytest

from dimos.benchmark.spatial.models import (
    Answer,
    AnswerType,
    BooleanAnswerValue,
    Predicate,
    ReviewAction,
    ReviewOverride,
)
from dimos.benchmark.spatial.pi_baseline.records import Prediction
from dimos.benchmark.spatial.pi_baseline.scoring import score_case
from dimos.benchmark.spatial.utilities import stable_opaque_id


def _fixture(
    tmp_path: Path, *, override: ReviewOverride | None = None
) -> tuple[dict[str, object], Path, str, str]:
    scene_id = stable_opaque_id("scene", {"test": 1})
    trajectory_id = stable_opaque_id("trajectory", {"test": 1})
    question_id = stable_opaque_id("question", {"test": 1})
    instance_id = stable_opaque_id("instance", {"test": 1})
    case: dict[str, object] = {
        "record_type": "case",
        "schema_version": "1.0",
        "scene": {"scene_id": scene_id},
        "trajectory": {"trajectory_id": trajectory_id},
        "question": {
            "question_id": question_id,
            "predicate": Predicate.POSE_OCCUPANCY.value,
            "answer_type": "boolean",
        },
        "instance": {"instance_id": instance_id},
    }
    oracle = tmp_path / "oracle" / "scenes" / scene_id / "trajectories" / trajectory_id
    oracle.mkdir(parents=True)
    answer = Answer(
        question_id=question_id,
        predicate=Predicate.POSE_OCCUPANCY,
        value=BooleanAnswerValue(value=True),
        oracle_policy_version="test",
    )
    (oracle / "answers.jsonl").write_text(answer.model_dump_json() + "\n")
    if override is not None:
        (oracle / "review_overrides.jsonl").write_text(override.model_dump_json() + "\n")
    return case, tmp_path / "oracle", instance_id, question_id


def test_scoring_uses_explicit_oracle_and_default_answer(tmp_path: Path) -> None:
    case, oracle, instance_id, _ = _fixture(tmp_path)
    score = score_case(
        case,
        Prediction.typed(instance_id, AnswerType.BOOLEAN, True),
        oracle_root=oracle,
        run_id="run",
        mode="eval",
        release_id="release",
        scorer_revision="scorer-v1",
    )
    assert score.outcome == "correct"
    assert score.expected_value is True


def test_correct_and_exclude_overrides_apply_after_session(tmp_path: Path) -> None:
    case, oracle, instance_id, question_id = _fixture(tmp_path)
    corrected = ReviewOverride(
        override_id=stable_opaque_id("override", {"correct": 1}),
        question_id=question_id,
        action=ReviewAction.CORRECT,
        reason="review",
        corrected_value=BooleanAnswerValue(value=False),
    )
    scene = next(oracle.glob("scenes/*"))
    trajectory = next(scene.glob("trajectories/*"))
    (trajectory / "review_overrides.jsonl").write_text(corrected.model_dump_json() + "\n")
    prediction = Prediction.typed(instance_id, AnswerType.BOOLEAN, False)
    assert (
        score_case(
            case,
            prediction,
            oracle_root=oracle,
            run_id="run",
            mode="eval",
            release_id="release",
            scorer_revision="v1",
        ).outcome
        == "correct"
    )
    excluded = corrected.model_copy(
        update={
            "override_id": stable_opaque_id("override", {"exclude": 1}),
            "action": ReviewAction.EXCLUDE,
            "corrected_value": None,
        }
    )
    (trajectory / "review_overrides.jsonl").write_text(excluded.model_dump_json() + "\n")
    assert (
        score_case(
            case,
            prediction,
            oracle_root=oracle,
            run_id="run2",
            mode="eval",
            release_id="release",
            scorer_revision="v1",
        ).outcome
        == "excluded"
    )


def test_score_ledger_is_append_only_and_keyed(tmp_path: Path) -> None:
    case, oracle, instance_id, _ = _fixture(tmp_path)
    ledger = tmp_path / "private" / "scores.jsonl"
    kwargs = dict(
        oracle_root=oracle,
        run_id="run",
        mode="eval",
        release_id="release",
        scorer_revision="v1",
        ledger_path=ledger,
    )
    score_case(case, Prediction.typed(instance_id, AnswerType.BOOLEAN, True), **kwargs)
    original = ledger.read_bytes()
    with pytest.raises(ValueError):
        score_case(case, Prediction.typed(instance_id, AnswerType.BOOLEAN, False), **kwargs)
    assert ledger.read_bytes() == original
