# Copyright 2026 Dimensional Inc.

import json
from pathlib import Path

from dimos.benchmark.vqa.generation.dataset import _evaluation_rows, write_dataset_manifest
from dimos.benchmark.vqa.models import GroundTruthResult, QuestionIntent, ToolTrace, VqaExample


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
