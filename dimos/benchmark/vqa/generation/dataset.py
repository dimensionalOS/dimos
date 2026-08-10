# Copyright 2026 Dimensional Inc.
"""Persist VQA generation evidence and a simple multiple-choice evaluation export."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

import cv2

from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    BooleanAnswerContract,
    CalibratedFrame,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)


def write_frame_record(
    output: Path,
    frame: CalibratedFrame,
    recording: str,
    frame_index: int,
    intents: list[QuestionIntent | QuestionProposal],
    results: list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult],
    metadata: dict[str, Any],
) -> None:
    """Write one frame's public cases alongside its private generation audit record."""
    output.mkdir(parents=True, exist_ok=False)
    image_path = output / "image.jpg"
    if not cv2.imwrite(str(image_path), frame.image.data):
        raise RuntimeError(f"failed to write {image_path}")
    accepted = [result for result in results if _is_accepted(result)]
    cases, labels = _evaluation_rows(frame.id, accepted)
    _write_json(
        output / "frame.json",
        {
            "schema_version": "1.0",
            "frame_id": frame.id,
            "recording": recording,
            "frame_index": frame_index,
            "image": image_path.name,
            "question_count": len(intents),
            "accepted_question_count": len(accepted),
            "rejected_question_count": len(results) - len(accepted),
            **metadata,
        },
    )
    _write_json(output / "ground_truth.json", [_private_result(item) for item in results])
    _write_json(output / "cases.json", cases)
    _write_json(output / "labels.json", labels)


def write_dataset_manifest(output: Path) -> dict[str, int]:
    """Build aggregate public cases and private labels from completed frame records."""
    frames = sorted(path for path in output.glob("frame-*") if (path / "frame.json").is_file())
    case_rows: list[dict[str, Any]] = []
    label_rows: list[dict[str, Any]] = []
    accepted = 0
    rejected = 0
    for path in frames:
        frame = json.loads((path / "frame.json").read_text())
        case_rows.extend(
            {**case, "image": f"{path.name}/{case['image']}"}
            for case in json.loads((path / "cases.json").read_text())
        )
        label_rows.extend(json.loads((path / "labels.json").read_text()))
        accepted += frame["accepted_question_count"]
        rejected += frame["rejected_question_count"]
    _write_jsonl(output / "cases.jsonl", case_rows)
    _write_jsonl(output / "labels.jsonl", label_rows)
    return {
        "frame_count": len(frames),
        "accepted_question_count": accepted,
        "rejected_question_count": rejected,
    }


def _is_accepted(result: GroundTruthResult | AcceptedOracleResult | RejectedOracleResult) -> bool:
    return isinstance(result, AcceptedOracleResult) or (
        isinstance(result, GroundTruthResult) and result.status == "answered"
    )


def _evaluation_rows(
    frame_id: str, results: list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult]
) -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
    cases: list[dict[str, Any]] = []
    labels: list[dict[str, str]] = []
    for result in results:
        if isinstance(result, RejectedOracleResult):
            continue
        if isinstance(result, AcceptedOracleResult):
            contract = result.answer_contract
            choices = (
                ("yes", "no") if isinstance(contract, BooleanAnswerContract) else contract.choices
            )
            case_id = f"{frame_id}-{result.proposal.id}"
            question = result.proposal.question
            answer = result.answer
        else:
            case_id = result.question.id
            question = result.question.question
            choices = result.question.allowed_answers
            answer = result.answer
        if answer is None or answer not in choices:
            raise ValueError(f"accepted VQA case {case_id} must have a choice answer")
        cases.append(
            {"id": case_id, "image": "image.jpg", "question": question, "choices": choices}
        )
        labels.append({"id": case_id, "answer": answer})
    return cases, labels


def _private_result(
    result: GroundTruthResult | AcceptedOracleResult | RejectedOracleResult,
) -> dict[str, Any]:
    if isinstance(result, AcceptedOracleResult):
        return {
            "status": "answered",
            "answer": result.answer,
            "proposal": asdict(result.proposal),
            "answer_contract": asdict(result.answer_contract),
            "evidence_ids": result.evidence_ids,
            "tool_results": [asdict(item) for item in result.tool_results],
            "trace": [asdict(item) for item in result.trace],
        }
    if isinstance(result, RejectedOracleResult):
        return {
            "status": "rejected",
            "reason": result.reason,
            "proposal": asdict(result.proposal),
            "tool_results": [asdict(item) for item in result.tool_results],
            "trace": [asdict(item) for item in result.trace],
        }
    return asdict(result)


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n")


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text("".join(f"{json.dumps(row, sort_keys=True)}\n" for row in rows))
