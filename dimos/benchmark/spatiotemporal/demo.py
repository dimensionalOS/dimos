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

"""One-command real-video spatiotemporal QA demonstration."""

from __future__ import annotations

import argparse
from collections import Counter
from collections.abc import Sequence
from dataclasses import asdict
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
from typing import Any

import cv2

from dimos.benchmark.spatiotemporal.bundles import EvaluationBundle, load_bundle
from dimos.benchmark.spatiotemporal.evidence_viewer import write_evidence_viewer
from dimos.benchmark.spatiotemporal.models import ObjectObservation, Question, QuestionId
from dimos.benchmark.spatiotemporal.observation_io import write_observations
from dimos.benchmark.spatiotemporal.replay import replay_observations
from dimos.benchmark.spatiotemporal.runner import build_evaluation_report
from dimos.benchmark.spatiotemporal.temporal_memory_answerer import TemporalMemoryAnswerer
from dimos.benchmark.spatiotemporal.video_adapter import OpenCVVideoSampler
from dimos.benchmark.spatiotemporal.yoloe_adapter import (
    YoloeAdapterStatistics,
    YoloeObservationDetector,
)
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.experimental.temporal_memory.temporal_memory import TemporalMemory

DEFAULT_PROMPTS = (
    "quadruped robot",
    "chair",
    "whiteboard",
    "table",
    "trash can",
    "refrigerator",
    "door",
    "cart",
)


class TimestampScriptedPublicVlModel(VlModel):
    """Deterministic TemporalMemory plumbing fixture with no private inputs."""

    def query(self, image: Image, query: str, **kwargs: Any) -> str:
        if "new_entities" in query and "entities_present" in query:
            timestamp_s = float(image.ts or 0.0)
            entities = [{"id": "R1", "type": "appliance", "descriptor": "white refrigerator"}]
            if timestamp_s < 10.0:
                entities.append({"id": "Q1", "type": "robot", "descriptor": "quadruped robot"})
            else:
                entities.append({"id": "C1", "type": "furniture", "descriptor": "office chair"})
            moving_id = "Q1" if timestamp_s < 10.0 else "C1"
            relation_type = "left_of" if timestamp_s < 10.0 else "right_of"
            return json.dumps(
                {
                    "window": {"start_s": max(0.0, timestamp_s - 4.0), "end_s": timestamp_s},
                    "caption": "Robot lab scene with changing robot and chair visibility.",
                    "entities_present": [{"id": entity["id"]} for entity in entities],
                    "new_entities": entities,
                    "relations": [
                        {
                            "type": relation_type,
                            "subject": moving_id,
                            "object": "R1",
                            "confidence": 0.7,
                        }
                    ],
                    "on_screen_text": [],
                }
            )
        if "rolling summary" in query.lower():
            return "A quadruped robot appeared before an office chair near a refrigerator."
        marker = "**Question:** "
        public_question = query.split(marker, 1)[1].splitlines()[0] if marker in query else query
        return "yes" if int(sha256(public_question.encode()).hexdigest(), 16) % 2 == 0 else "no"

    def query_batch(self, images: list[Image], query: str, **kwargs: Any) -> list[str]:
        return [self.query(image, query, **kwargs) for image in images]

    def stop(self) -> None:
        return None


def _assert_no_symlink_components(path: Path) -> None:
    current = Path(path.anchor) if path.is_absolute() else Path.cwd()
    parts = path.parts[1:] if path.is_absolute() else path.parts
    for part in parts:
        current /= part
        if current.is_symlink():
            raise ValueError(f"output path contains a symlink: {current}")


def _prepare_output_root(output_root: Path) -> Path:
    _assert_no_symlink_components(output_root.absolute())
    if output_root.exists() and not output_root.is_dir():
        raise ValueError("output root must be a directory")
    output_root.mkdir(parents=True, exist_ok=True)
    return output_root.resolve()


def _safe_child(output_root: Path, name: str) -> Path:
    if Path(name).name != name:
        raise ValueError(f"unsafe artifact name: {name}")
    path = output_root / name
    _assert_no_symlink_components(path)
    return path


def _reset_directory(path: Path) -> None:
    if path.is_symlink():
        raise ValueError(f"refusing to replace symlinked directory: {path}")
    if path.exists():
        if not path.is_dir():
            raise ValueError(f"expected directory artifact: {path}")
        shutil.rmtree(path)
    path.mkdir(parents=True)


def _trim_video(source: Path, target: Path, duration_s: float) -> tuple[float, int]:
    if source.is_symlink() or not source.is_file():
        raise ValueError(f"source video must be a regular file: {source}")
    if target.is_symlink() or target.is_dir():
        raise ValueError(f"target video must be a regular file path: {target}")
    if source.resolve() == target.resolve(strict=False) or (
        target.exists() and os.path.samefile(source, target)
    ):
        raise ValueError("source and target video paths must differ")
    capture = cv2.VideoCapture(str(source))
    written = 0
    try:
        if not capture.isOpened():
            raise RuntimeError(f"failed to open source video: {source}")
        fps = capture.get(cv2.CAP_PROP_FPS)
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if not math.isfinite(fps) or fps <= 0.0 or width <= 0 or height <= 0:
            raise ValueError("source video has invalid metadata")
        frame_limit = round(duration_s * fps)
        writer = cv2.VideoWriter(
            str(target),
            cv2.VideoWriter_fourcc(*"mp4v"),  # type: ignore[attr-defined]
            fps,
            (width, height),
        )
        try:
            if not writer.isOpened():
                raise RuntimeError(f"failed to open target video writer: {target}")
            while written < frame_limit:
                decoded, frame = capture.read()
                if not decoded:
                    break
                writer.write(frame)
                written += 1
        finally:
            writer.release()
    except Exception:
        if target.exists() and not target.is_symlink() and target.is_file():
            target.unlink()
        raise
    finally:
        capture.release()
    if written != frame_limit:
        if target.exists() and not target.is_symlink() and target.is_file():
            target.unlink()
        raise RuntimeError(f"expected {frame_limit} frames, wrote {written}")
    verification = cv2.VideoCapture(str(target))
    decoded_frames = 0
    try:
        if not verification.isOpened():
            raise RuntimeError(f"failed to reopen encoded video: {target}")
        while True:
            decoded, _ = verification.read()
            if not decoded:
                break
            decoded_frames += 1
    except Exception:
        if target.exists() and not target.is_symlink() and target.is_file():
            target.unlink()
        raise
    finally:
        verification.release()
    if decoded_frames != frame_limit:
        if target.exists() and not target.is_symlink() and target.is_file():
            target.unlink()
        raise RuntimeError(
            f"encoded video contains {decoded_frames} decodable frames; expected {frame_limit}"
        )
    return fps, written


def _detector_factory() -> Yoloe2DDetector:
    return Yoloe2DDetector(
        prompt_mode=YoloePromptMode.PROMPT,
        conf=0.15,
        max_area_ratio=0.8,
        device="cpu",
    )


def _sample_observations(
    video: Path, frame_stride: int
) -> tuple[
    tuple[ObjectObservation, ...],
    YoloeAdapterStatistics,
    tuple[tuple[int, float], ...],
]:
    detector = YoloeObservationDetector(prompts=DEFAULT_PROMPTS, detector_factory=_detector_factory)
    sampler = OpenCVVideoSampler(detector=detector, frame_stride=frame_stride)
    try:
        observations = sampler.sample(video, "office_robot_25s")
        return observations, detector.statistics, sampler.sample_schedule
    finally:
        sampler.close()


def _teacher_run(
    video: Path, output_root: Path, frame_stride: int
) -> tuple[EvaluationBundle, dict[str, Any]]:
    observations_path = _safe_child(output_root, "observations.jsonl")
    if observations_path.is_dir():
        raise ValueError("observation artifact path must not be a directory")
    first_root = _safe_child(output_root, "bundle-a")
    second_root = _safe_child(output_root, "bundle-b")
    _reset_directory(first_root)
    _reset_directory(second_root)
    observations, detector_statistics, sample_schedule = _sample_observations(video, frame_stride)
    repeated_observations, repeated_statistics, repeated_schedule = _sample_observations(
        video, frame_stride
    )
    if (
        observations != repeated_observations
        or detector_statistics != repeated_statistics
        or sample_schedule != repeated_schedule
    ):
        raise RuntimeError("YOLO-E observations differ across independent detector runs")
    write_observations(observations_path, observations)
    source_video_sha256 = sha256(video.read_bytes()).hexdigest()
    first = replay_observations(
        observations_path,
        first_root,
        source_video_sha256,
        sample_schedule=sample_schedule,
    )
    second = replay_observations(
        observations_path,
        second_root,
        source_video_sha256,
        sample_schedule=sample_schedule,
    )
    if first.logical_sha256 != second.logical_sha256:
        raise RuntimeError("bundle logical hashes differ across output roots")
    first_bundle = load_bundle(first_root)
    second_bundle = load_bundle(second_root)
    if first_bundle.public_manifest != second_bundle.public_manifest:
        raise RuntimeError("public manifests differ across output roots")
    if first_bundle.oracle_manifest != second_bundle.oracle_manifest:
        raise RuntimeError("oracle manifests differ across output roots")
    summary = {
        "answers": len(first_bundle.answers),
        "bundle_id": first_bundle.public_manifest.bundle_id,
        "detector_repeat_equal": True,
        "detector_statistics": asdict(detector_statistics),
        "labels": dict(sorted(Counter(observation.label for observation in observations).items())),
        "logical_sha256": first.logical_sha256,
        "observations": len(observations),
        "questions": len(first_bundle.questions),
        "relation_facts": len(first_bundle.relation_facts),
        "relation_intervals": len(first_bundle.relation_intervals),
        "sampled_frames": len(sample_schedule),
        "source_video_sha256": source_video_sha256,
        "spatial_questions": sum(
            question.question_kind.value == "spatial" for question in first_bundle.questions
        ),
        "temporal_questions": sum(
            question.question_kind.value == "temporal" for question in first_bundle.questions
        ),
        "unique_object_ids": sorted({observation.object_id for observation in observations}),
    }
    return first_bundle, summary


def _run_temporal_memory_candidate(
    video: Path,
    questions: Sequence[Question],
    output_root: Path,
    duration_s: int,
) -> tuple[dict[QuestionId, str | bool | None], dict[str, Any]]:
    """Run a public-only candidate and return predictions without oracle access."""
    database_root = _safe_child(output_root, "temporal-memory")
    _reset_directory(database_root)
    temporal_memory = TemporalMemory(
        vlm=TimestampScriptedPublicVlModel(),
        db_dir=str(database_root),
        new_memory=True,
        fps=1.0,
        window_s=5.0,
        stride_s=5.0,
        max_frames_per_window=3,
        max_buffer_frames=max(30, duration_s),
        summary_interval_s=30.0,
        enable_distance_estimation=False,
        visualize=False,
        use_clip_filtering=False,
    )

    def ingest(path: Path) -> int:
        temporal_memory._accumulator.set_start_time(0.0)
        capture = cv2.VideoCapture(str(path))
        try:
            if not capture.isOpened():
                raise RuntimeError(f"TemporalMemory failed to open video: {path}")
            fps = capture.get(cv2.CAP_PROP_FPS)
            if not math.isfinite(fps) or fps <= 0.0:
                raise ValueError("TemporalMemory video FPS must be finite and positive")
            ingested = 0
            for second in range(duration_s):
                capture.set(cv2.CAP_PROP_POS_FRAMES, round(second * fps))
                decoded, frame = capture.read()
                if not decoded:
                    raise RuntimeError(f"TemporalMemory failed to decode second {second}")
                image = Image.from_numpy(
                    cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                    format=ImageFormat.RGB,
                    frame_id=str(round(second * fps)),
                    ts=float(second),
                )
                temporal_memory._accumulator.add_frame(image, float(second))
                ingested += 1
                if ingested % 5 == 0:
                    temporal_memory._analyze_window()
            return ingested
        finally:
            capture.release()

    answerer = TemporalMemoryAnswerer(
        ingest=ingest,
        query=temporal_memory.query,
        cleanup=temporal_memory.stop,
    )
    try:
        readiness = answerer.ingest_video(video)
        state = temporal_memory.get_state()
        candidate_answers = {
            question.question_id: answerer.answer(question) for question in questions
        }
        runtime = {
            "baseline_kind": "public_only_plumbing_smoke_test",
            "no_answers": sum(answer == "no" for answer in candidate_answers.values()),
            "questions_answered": len(candidate_answers),
            "readiness": readiness.model_dump(mode="json"),
            "score_interpretation": "not_model_quality",
            "temporal_memory_state": state,
            "visually_grounded": False,
            "vlm_mode": "timestamp_scripted_temporal_memory_plumbing",
            "yes_answers": sum(answer == "yes" for answer in candidate_answers.values()),
        }
        return candidate_answers, runtime
    finally:
        answerer.close()


def _score_candidate(
    bundle: EvaluationBundle,
    candidate_answers: dict[QuestionId, str | bool | None],
    runtime: dict[str, Any],
) -> dict[str, Any]:
    """Score public-only predictions in the evaluator's private boundary."""
    oracle_by_question = {answer.question_id: answer for answer in bundle.answers}
    report = build_evaluation_report(
        bundle.questions,
        oracle_by_question,
        candidate_answers,
        source_video_sha256=bundle.public_manifest.source_video_sha256,
    )
    return {
        **runtime,
        "by_family": {family.value: asdict(score) for family, score in report.by_family.items()},
        "candidate_used_oracle": False,
        "overall": asdict(report.overall),
        "status_counts": {status.value: count for status, count in report.status_counts.items()},
    }


def run_demo(
    source_video: Path,
    output_root: Path,
    *,
    duration_s: int = 25,
    frame_stride: int = 150,
) -> dict[str, Any]:
    """Run the real video, YOLO-E, bundle, replay, and TemporalMemory gates."""
    if not 15 <= duration_s <= 30:
        raise ValueError("duration must be between 15 and 30 seconds")
    if frame_stride < 1:
        raise ValueError("frame stride must be positive")
    if source_video.is_symlink() or not source_video.is_file():
        raise ValueError(f"source video must be a regular file: {source_video}")
    output_root = _prepare_output_root(output_root)
    video = _safe_child(output_root, "office_robot_25s.mp4")
    summary_path = _safe_child(output_root, "summary.json")
    if summary_path.is_dir():
        raise ValueError("summary artifact path must not be a directory")
    fps, frames = _trim_video(source_video, video, float(duration_s))
    bundle, teacher = _teacher_run(video, output_root, frame_stride)
    with TemporaryDirectory(prefix="dimos-stqa-candidate-") as candidate_directory:
        candidate_root = Path(candidate_directory)
        candidate_video = candidate_root / video.name
        shutil.copyfile(video, candidate_video)
        predictions, runtime = _run_temporal_memory_candidate(
            candidate_video,
            bundle.questions,
            candidate_root,
            duration_s,
        )
    candidate = _score_candidate(bundle, predictions, runtime)
    viewer_result = write_evidence_viewer(
        video,
        bundle,
        _safe_child(output_root, "evidence-viewer"),
    )
    summary = {
        "candidate": candidate,
        "review": {
            "evidence_frame_count": viewer_result.evidence_frame_count,
            "index_path": f"evidence-viewer/{viewer_result.index_path}",
            "question_count": viewer_result.question_count,
        },
        "teacher": teacher,
        "video": {
            "duration_s": duration_s,
            "fps": fps,
            "frames": frames,
            "path": str(video),
        },
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-video", type=Path, default=Path("assets/simple_demo.mp4"))
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(".artifacts/spatiotemporal-video-qa"),
    )
    parser.add_argument("--duration-s", type=int, default=25)
    parser.add_argument("--frame-stride", type=int, default=150)
    args = parser.parse_args()
    print(
        json.dumps(
            run_demo(
                args.source_video,
                args.output_root,
                duration_s=args.duration_s,
                frame_stride=args.frame_stride,
            ),
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
