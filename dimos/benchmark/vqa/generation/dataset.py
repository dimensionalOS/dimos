# Copyright 2026 Dimensional Inc.
"""Persist generated single-frame VQA records and dataset manifests."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

import cv2

from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundTruthResult,
    QuestionIntent,
)


def write_frame_record(
    output: Path,
    frame: CalibratedFrame,
    recording: str,
    frame_index: int,
    intents: list[QuestionIntent],
    results: list[GroundTruthResult],
    ground_truth: VqaGroundTruthGenerator,
    metadata: dict[str, Any],
) -> None:
    """Write one self-contained frame record and its private evidence."""
    output.mkdir(parents=True, exist_ok=False)
    image_path = output / "image.jpg"
    if not cv2.imwrite(str(image_path), frame.image.data):
        raise RuntimeError(f"failed to write {image_path}")
    original_image_path = output / "original_image.jpg"
    if frame.original_image is not None and not cv2.imwrite(
        str(original_image_path), frame.original_image.data
    ):
        raise RuntimeError(f"failed to write {original_image_path}")
    overlay_path = output / "grounding_overlay.jpg"
    ground_truth.write_overlay(frame, str(overlay_path))
    examples = [result.question for result in results if result.status == "answered"]
    frame_meta = {
        "schema_version": "1.0",
        "frame_id": frame.id,
        "recording": recording,
        "frame_index": frame_index,
        "image": image_path.name,
        "original_image": original_image_path.name if frame.original_image is not None else None,
        "grounding_overlay": overlay_path.name,
        "question_count": len(intents),
        "accepted_question_count": len(examples),
        "rejected_question_count": len(results) - len(examples),
        **metadata,
    }
    _write_json(output / "frame.json", frame_meta)
    _write_json(output / "intents.json", [asdict(item) for item in intents])
    _write_json(output / "examples.json", [asdict(item) for item in examples])
    _write_json(output / "ground_truth.json", [asdict(item) for item in results])


def write_dataset_manifest(output: Path) -> dict[str, int]:
    """Rebuild aggregate manifests from completed frame record directories."""
    frames = sorted(path for path in output.glob("frame-*") if (path / "frame.json").is_file())
    accepted = 0
    rejected = 0
    with (
        (output / "frames.jsonl").open("w") as frame_file,
        (output / "ground_truth.jsonl").open("w") as gt_file,
    ):
        for path in frames:
            frame = json.loads((path / "frame.json").read_text())
            frame_file.write(json.dumps(frame) + "\n")
            for result in json.loads((path / "ground_truth.json").read_text()):
                gt_file.write(json.dumps({"frame_id": frame["frame_id"], **result}) + "\n")
                if result["status"] == "answered":
                    accepted += 1
                else:
                    rejected += 1
    summary = {
        "frame_count": len(frames),
        "accepted_question_count": accepted,
        "rejected_question_count": rejected,
    }
    _write_json(output / "manifest.json", summary)
    return summary


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n")
