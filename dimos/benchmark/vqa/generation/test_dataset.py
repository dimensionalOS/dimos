# Copyright 2026 Dimensional Inc.

import json
from pathlib import Path

from dimos.benchmark.vqa.generation.dataset import _evaluation_rows, write_dataset_manifest
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    ChoiceAnswerContract,
    DeferredHeightChoiceContract,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    ToolTrace,
    VqaExample,
)


def test_constrained_results_export_simple_multiple_choice_rows() -> None:
    result = GroundTruthResult(
        intent=QuestionIntent("presence", "chair"),
        question=VqaExample(
            "frame-chair-presence",
            "Is there a chair?",
            "yes",
            "boolean",
            (),
            ("yes", "no"),
        ),
        status="answered",
        answer="yes",
        reason=None,
        evidence=(),
        trace=(ToolTrace("ground", "chair"),),
    )

    cases, labels = _evaluation_rows("frame", [result])

    assert cases == [
        {
            "id": "frame-chair-presence",
            "image": "image.jpg",
            "question": "Is there a chair?",
            "choices": ("yes", "no"),
        }
    ]
    assert labels == [{"id": "frame-chair-presence", "answer": "yes"}]


def test_deferred_height_result_exports_resolved_public_choices() -> None:
    choices = ("under 0.2 m", "0.2-0.6 m", "0.6-1.0 m", "over 1.0 m")
    result = AcceptedOracleResult(
        QuestionProposal("chair-height", "How tall is the chair?", DeferredHeightChoiceContract()),
        "0.2-0.6 m",
        ChoiceAnswerContract(choices),
        ("height-1",),
        (),
        (),
    )

    cases, labels = _evaluation_rows("frame", [result])

    assert cases[0]["choices"] == choices
    assert labels == [{"id": "frame-chair-height", "answer": "0.2-0.6 m"}]


def test_dataset_manifest_exports_public_cases_and_private_labels(tmp_path: Path) -> None:
    frame = tmp_path / "frame-000040"
    frame.mkdir()
    (frame / "frame.json").write_text(
        json.dumps(
            {
                "frame_id": "frame-40",
                "accepted_question_count": 1,
                "rejected_question_count": 2,
            }
        )
    )
    (frame / "cases.json").write_text(
        json.dumps(
            [
                {
                    "id": "case-1",
                    "image": "image.jpg",
                    "question": "Is it visible?",
                    "choices": ["yes", "no"],
                }
            ]
        )
    )
    (frame / "labels.json").write_text(json.dumps([{"id": "case-1", "answer": "yes"}]))

    summary = write_dataset_manifest(tmp_path)

    assert summary == {"frame_count": 1, "accepted_question_count": 1, "rejected_question_count": 2}
    assert json.loads((tmp_path / "cases.jsonl").read_text()) == {
        "id": "case-1",
        "image": "frame-000040/image.jpg",
        "question": "Is it visible?",
        "choices": ["yes", "no"],
    }
    assert json.loads((tmp_path / "labels.jsonl").read_text()) == {"id": "case-1", "answer": "yes"}
    assert not (tmp_path / "frames.jsonl").exists()
    assert not (tmp_path / "manifest.json").exists()
