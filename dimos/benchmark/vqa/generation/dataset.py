# Copyright 2026 Dimensional Inc.
"""Persist VQA generation evidence and a simple multiple-choice evaluation export."""

from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
from typing import Any

from dimos.benchmark.vqa.contracts import (
    AcceptedOracleResult,
    BooleanAnswerContract,
    CalibratedFrame,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)
from dimos.benchmark.vqa.generation.config import ORACLE_MODEL, QUESTION_MODEL, GenerationConfig


class GenerationDataset:
    """Own resumable frame state and publication for one generated dataset."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)

    def completed_frame(self, frame_index: int, generation: GenerationConfig) -> bool:
        """Return whether a compatible completed frame record already exists."""
        frame = frame_audit_path(self.root, frame_index)
        if not (frame / "frame.json").is_file():
            return False
        _validate_completed_frame(frame, generation, frame_index)
        return True

    def write_frame(
        self,
        frame: CalibratedFrame,
        recording: str,
        frame_index: int,
        intents: list[QuestionIntent | QuestionProposal],
        results: list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult],
        metadata: dict[str, Any],
    ) -> None:
        """Write one frame and its private audit record."""
        write_frame_record(self.root, frame, recording, frame_index, intents, results, metadata)

    def finalize(self, generation: GenerationConfig) -> dict[str, int]:
        """Publish aggregate manifests and the resolved generation record."""
        summary = write_dataset_manifest(self.root)
        _write_generation_run(self.root, generation, summary)
        return summary


def write_frame_record(
    output: Path,
    frame: CalibratedFrame,
    recording: str,
    frame_index: int,
    intents: list[QuestionIntent | QuestionProposal],
    results: list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult],
    metadata: dict[str, Any],
) -> None:
    """Write one public image and its private resumable frame audit record."""
    import cv2

    frame_output = frame_audit_path(output, frame_index)
    if (frame_output / "frame.json").exists():
        raise FileExistsError(f"completed frame record already exists: {frame_output}")
    frame_output.mkdir(parents=True, exist_ok=True)
    assets = output / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    image_name = f"frame-{frame_index:06d}.jpg"
    image_path = assets / image_name
    image_temp = assets / f".{image_name}.tmp.jpg"
    try:
        if not cv2.imwrite(str(image_temp), frame.image.data):
            raise RuntimeError(f"failed to write {image_path}")
        os.replace(image_temp, image_path)
    finally:
        image_temp.unlink(missing_ok=True)
    accepted = [result for result in results if _is_accepted(result)]
    cases, labels = _evaluation_rows(frame.id, accepted)
    cases = [{**case, "image": f"assets/{image_name}"} for case in cases]
    _write_json_atomic(
        frame_output / "ground_truth.json", [_private_result(item) for item in results]
    )
    _write_json_atomic(frame_output / "cases.json", cases)
    _write_json_atomic(frame_output / "labels.json", labels)
    _write_json_atomic(
        frame_output / "frame.json",
        {
            "schema_version": "1.0",
            "frame_id": frame.id,
            "recording": recording,
            "frame_index": frame_index,
            "image": f"assets/{image_name}",
            "question_count": len(intents),
            "accepted_question_count": len(accepted),
            "rejected_question_count": len(results) - len(accepted),
            **metadata,
        },
    )


def write_dataset_manifest(output: Path) -> dict[str, int]:
    """Build aggregate public cases and private labels from completed frame records."""
    frames = sorted(
        path for path in (output / "audit").glob("frame-*") if (path / "frame.json").is_file()
    )
    case_rows: list[dict[str, Any]] = []
    label_rows: list[dict[str, Any]] = []
    accepted = 0
    rejected = 0
    for path in frames:
        frame = json.loads((path / "frame.json").read_text())
        case_rows.extend(json.loads((path / "cases.json").read_text()))
        label_rows.extend(json.loads((path / "labels.json").read_text()))
        accepted += frame["accepted_question_count"]
        rejected += frame["rejected_question_count"]
    _validate_evaluation_rows(case_rows, label_rows)
    _write_jsonl(output / "cases.jsonl", case_rows)
    _write_jsonl(output / "labels.jsonl", label_rows)
    return {
        "frame_count": len(frames),
        "accepted_question_count": accepted,
        "rejected_question_count": rejected,
    }


def frame_audit_path(output: Path, frame_index: int) -> Path:
    """Return the private resumable audit directory for one sampled frame."""
    return output / "audit" / f"frame-{frame_index:06d}"


def _write_generation_run(
    output: Path,
    generation: GenerationConfig,
    summary: dict[str, int],
) -> None:
    """Record the resolved request that produced one generated dataset."""
    payload = {
        "schema_version": "1.0",
        "generation": {**generation.model_dump(mode="json"), "output": str(output)},
        "models": {
            "question_author": QUESTION_MODEL,
            "oracle": ORACLE_MODEL if generation.question_mode == "agentic" else None,
        },
        "summary": summary,
    }
    audit = output / "audit"
    audit.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(audit / "run.json", payload)


def _validate_completed_frame(
    output: Path,
    generation: GenerationConfig,
    frame_index: int,
) -> None:
    """Reject completed frame records created by a different generation request."""
    try:
        payload = json.loads((output / "frame.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid completed frame marker: {output}") from exc
    expected_source = (
        "agentic_image_author" if generation.question_mode == "agentic" else "openai_image_agent"
    )
    grounding = payload.get("grounding")
    if (
        payload.get("recording") != generation.recording
        or payload.get("frame_index") != frame_index
        or payload.get("question_source") != expected_source
        or payload.get("question_model") != QUESTION_MODEL
        or payload.get("oracle_model")
        != (ORACLE_MODEL if generation.question_mode == "agentic" else None)
        or grounding != generation.grounding.model_dump(mode="json")
    ):
        raise ValueError(f"completed frame {output.name} was generated with different settings")


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
        answer: str | None
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
    _validate_evaluation_rows(cases, labels)
    return cases, labels


def _validate_evaluation_rows(
    cases: list[dict[str, Any]], labels: list[dict[str, Any]] | list[dict[str, str]]
) -> None:
    case_ids = _unique_row_ids(cases, "case")
    label_ids = _unique_row_ids(labels, "label")
    missing = sorted(case_ids - label_ids)
    orphaned = sorted(label_ids - case_ids)
    if missing or orphaned:
        raise ValueError(f"VQA case/label ID mismatch: missing={missing}, orphaned={orphaned}")


def _unique_row_ids(rows: list[dict[str, Any]] | list[dict[str, str]], kind: str) -> set[str]:
    identifiers: set[str] = set()
    for row in rows:
        identifier = row.get("id")
        if not isinstance(identifier, str) or not identifier:
            raise ValueError(f"VQA {kind} requires a non-empty string ID")
        if identifier in identifiers:
            raise ValueError(f"duplicate VQA {kind} ID: {identifier}")
        identifiers.add(identifier)
    return identifiers


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


def _write_json_atomic(path: Path, payload: Any) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        _write_json(temporary, payload)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text("".join(f"{json.dumps(row, sort_keys=True)}\n" for row in rows))
