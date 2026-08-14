# Copyright 2026 Dimensional Inc.

import json
from pathlib import Path

import numpy as np
import pytest

from dimos.benchmark.vqa.contracts import (
    AcceptedOracleResult,
    CalibratedFrame,
    ChoiceAnswerContract,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    ToolTrace,
    VqaExample,
)
from dimos.benchmark.vqa.generation.dataset import (
    _evaluation_rows,
    frame_audit_path,
    write_dataset_manifest,
    write_frame_record,
)
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


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


def test_agentic_result_exports_public_choices() -> None:
    choices = ("left", "right")
    result = AcceptedOracleResult(
        QuestionProposal("chair-side", "Which side is the chair?", ChoiceAnswerContract(choices)),
        "left",
        ChoiceAnswerContract(choices),
        ("grounding-1",),
        (),
        (),
    )

    cases, labels = _evaluation_rows("frame", [result])

    assert cases[0]["choices"] == choices
    assert labels == [{"id": "frame-chair-side", "answer": "left"}]


def test_dataset_manifest_exports_public_cases_and_private_labels(tmp_path: Path) -> None:
    frame = tmp_path / "audit" / "frame-000040"
    frame.mkdir(parents=True)
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
                    "image": "assets/frame-000040.jpg",
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
        "image": "assets/frame-000040.jpg",
        "question": "Is it visible?",
        "choices": ["yes", "no"],
    }
    assert json.loads((tmp_path / "labels.jsonl").read_text()) == {"id": "case-1", "answer": "yes"}
    assert not (tmp_path / "frames.jsonl").exists()
    assert not (tmp_path / "manifest.json").exists()


def test_evaluation_rows_reject_duplicate_case_ids() -> None:
    result = GroundTruthResult(
        QuestionIntent("presence", "chair"),
        VqaExample("duplicate", "Is there a chair?", "yes", "boolean", (), ("yes", "no")),
        "answered",
        "yes",
        None,
        (),
        (),
    )

    with pytest.raises(ValueError, match="duplicate VQA case ID: duplicate"):
        _evaluation_rows("frame", [result, result])


def test_dataset_manifest_rejects_case_label_id_mismatch(tmp_path: Path) -> None:
    frame = tmp_path / "audit" / "frame-000001"
    frame.mkdir(parents=True)
    (frame / "frame.json").write_text(
        json.dumps({"accepted_question_count": 1, "rejected_question_count": 0})
    )
    (frame / "cases.json").write_text(
        json.dumps(
            [{"id": "case-1", "image": "image.jpg", "question": "Q?", "choices": ["a", "b"]}]
        )
    )
    (frame / "labels.json").write_text(json.dumps([{"id": "case-2", "answer": "a"}]))

    with pytest.raises(ValueError, match="case/label ID mismatch"):
        write_dataset_manifest(tmp_path)


def test_write_frame_record_resumes_partial_directory_and_marks_completion_last(
    tmp_path: Path,
) -> None:
    output = tmp_path / "dataset"
    frame_output = frame_audit_path(output, 1)
    frame_output.mkdir(parents=True)
    (frame_output / "cases.json").write_text("stale")
    image = Image.from_numpy(np.zeros((2, 2, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        "frame-1",
        image,
        PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32)),
        CameraInfo.from_intrinsics(1.0, 1.0, 1.0, 1.0, 2, 2),
        Transform.identity(),
        True,
    )

    write_frame_record(output, frame, "recording.db", 1, [], [], {})

    assert json.loads((frame_output / "cases.json").read_text()) == []
    assert (frame_output / "frame.json").is_file()
    assert (output / "assets" / "frame-000001.jpg").is_file()
    assert not list((output / "audit").rglob("*.jpg"))
    assert not list(output.rglob("*.tmp"))
    with pytest.raises(FileExistsError, match="completed frame"):
        write_frame_record(output, frame, "recording.db", 1, [], [], {})
