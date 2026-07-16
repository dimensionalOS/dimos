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

"""Evaluator-only HTML and annotated-frame evidence viewer."""

from collections import defaultdict
from dataclasses import dataclass
from hashlib import sha256
from html import escape
from pathlib import Path
import shutil
from tempfile import mkdtemp
from typing import Protocol

import cv2
import numpy as np

from dimos.benchmark.spatiotemporal.models import (
    ObjectObservation,
    OracleAnswer,
    Question,
    RelationInterval,
)


class EvidenceBundle(Protocol):
    """Private evaluator records required to render evidence."""

    @property
    def questions(self) -> tuple[Question, ...]: ...

    @property
    def answers(self) -> tuple[OracleAnswer, ...]: ...

    @property
    def observations(self) -> tuple[ObjectObservation, ...]: ...

    @property
    def relation_intervals(self) -> tuple[RelationInterval, ...]: ...


@dataclass(frozen=True)
class EvidenceViewerResult:
    """Root-independent description of generated viewer artifacts."""

    index_path: str
    evidence_frame_count: int
    question_count: int


def _assert_no_symlink_components(path: Path) -> None:
    absolute = path.absolute()
    for component in reversed((absolute, *absolute.parents)):
        if component.is_symlink():
            raise ValueError(f"viewer path contains a symlink: {component}")


def _validate_output_root(root: Path) -> None:
    _assert_no_symlink_components(root)
    if root.exists() and not root.is_dir():
        raise ValueError(f"viewer output must be a directory: {root}")
    if root.exists():
        for path in root.rglob("*"):
            if path.is_symlink():
                raise ValueError(f"viewer output contains a symlink: {path}")


def _create_staging_root(root: Path) -> Path:
    parent = root.parent
    _assert_no_symlink_components(parent)
    parent.mkdir(parents=True, exist_ok=True)
    _assert_no_symlink_components(parent)
    staging = Path(mkdtemp(prefix=f".{root.name}.tmp-", dir=parent))
    (staging / "frames").mkdir()
    return staging


def _replace_output_atomically(staging: Path, output_root: Path) -> None:
    backup: Path | None = None
    if output_root.exists():
        backup = Path(mkdtemp(prefix=f".{output_root.name}.old-", dir=output_root.parent))
        backup.rmdir()
        output_root.rename(backup)
    try:
        staging.rename(output_root)
    except Exception:
        if backup is not None and backup.exists() and not output_root.exists():
            backup.rename(output_root)
        raise
    if backup is not None:
        shutil.rmtree(backup)


def _frame_evidence(bundle: EvidenceBundle) -> tuple[dict[str, tuple[int, ...]], tuple[int, ...]]:
    intervals = {interval.interval_id: interval for interval in bundle.relation_intervals}
    evidence_by_question: dict[str, tuple[int, ...]] = {}
    all_frames: set[int] = set()
    for answer in bundle.answers:
        frames = set(answer.evidence_frame_ids)
        for interval_id in answer.evidence_interval_ids:
            try:
                frames.update(intervals[interval_id].evidence_frame_ids)
            except KeyError as error:
                raise ValueError(f"unknown evidence interval: {interval_id}") from error
        ordered = tuple(sorted(frames))
        evidence_by_question[answer.question_id] = ordered
        all_frames.update(ordered)
    return evidence_by_question, tuple(sorted(all_frames))


def _object_color(object_id: str) -> tuple[int, int, int]:
    digest = sha256(object_id.encode("utf-8")).digest()
    return tuple(64 + component % 160 for component in digest[:3])  # type: ignore[return-value]


def _annotate_frame(frame: np.ndarray, observations: tuple[ObjectObservation, ...]) -> np.ndarray:
    annotated = frame.copy()
    height, width = annotated.shape[:2]
    for observation in sorted(observations, key=lambda item: item.object_id):
        x_min = min(width - 1, max(0, round(observation.box.x_min * width)))
        y_min = min(height - 1, max(0, round(observation.box.y_min * height)))
        x_max = min(width - 1, max(0, round(observation.box.x_max * width)))
        y_max = min(height - 1, max(0, round(observation.box.y_max * height)))
        color = _object_color(observation.object_id)
        cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), color, 2)
        label = f"{observation.object_id}: {observation.label} {observation.confidence:.2f}"
        cv2.putText(
            annotated,
            label,
            (x_min, max(12, y_min - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            color,
            1,
            cv2.LINE_AA,
        )
    return annotated


def _write_evidence_frames(
    video_path: Path,
    output_root: Path,
    frame_ids: tuple[int, ...],
    observations: tuple[ObjectObservation, ...],
) -> None:
    if not frame_ids:
        return
    observations_by_frame: dict[int, list[ObjectObservation]] = defaultdict(list)
    for observation in observations:
        observations_by_frame[observation.frame_id].append(observation)

    wanted = set(frame_ids)
    capture = cv2.VideoCapture(str(video_path))
    written: set[int] = set()
    try:
        if not capture.isOpened():
            raise RuntimeError(f"failed to open evidence video: {video_path}")
        frame_id = 0
        while frame_id <= frame_ids[-1]:
            decoded, frame = capture.read()
            if not decoded or frame is None:
                raise RuntimeError(f"failed to decode evidence frame {frame_id}")
            if frame_id in wanted:
                annotated = _annotate_frame(frame, tuple(observations_by_frame.get(frame_id, ())))
                encoded, content = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not encoded:
                    raise RuntimeError(f"failed to encode evidence frame {frame_id}")
                (output_root / "frames" / f"frame_{frame_id:06d}.jpg").write_bytes(
                    content.tobytes()
                )
                written.add(frame_id)
            frame_id += 1
    finally:
        capture.release()
    if written != wanted:
        missing = sorted(wanted - written)
        raise RuntimeError(f"missing evidence frames: {missing}")


def _render_html(
    bundle: EvidenceBundle,
    evidence_by_question: dict[str, tuple[int, ...]],
) -> str:
    answers = {answer.question_id: answer for answer in bundle.answers}
    observations_by_frame: dict[int, list[ObjectObservation]] = defaultdict(list)
    for observation in bundle.observations:
        observations_by_frame[observation.frame_id].append(observation)

    rows: list[str] = []
    for question in sorted(bundle.questions, key=lambda item: item.question_id):
        try:
            answer = answers[question.question_id]
        except KeyError as error:
            raise ValueError(
                f"question is missing oracle answer: {question.question_id}"
            ) from error
        frame_ids = evidence_by_question.get(question.question_id, ())
        links = " ".join(
            f'<a href="frames/frame_{frame_id:06d}.jpg">frame {frame_id}</a>'
            for frame_id in frame_ids
        )
        rows.append(
            "<tr>"
            f"<td><code>{escape(question.question_id)}</code></td>"
            f"<td>{escape(question.text)}</td>"
            f"<td>{str(answer.expected).lower()}</td>"
            f"<td>{links or 'none'}</td>"
            "</tr>"
        )

    cards: list[str] = []
    evidence_frame_ids = sorted(
        {frame_id for frame_ids in evidence_by_question.values() for frame_id in frame_ids}
    )
    for frame_id in evidence_frame_ids:
        labels = ", ".join(
            f"{observation.object_id}: {observation.label}"
            for observation in sorted(
                observations_by_frame[frame_id], key=lambda item: item.object_id
            )
        )
        cards.append(
            "<figure>"
            f'<a href="frames/frame_{frame_id:06d}.jpg">'
            f'<img src="frames/frame_{frame_id:06d}.jpg" alt="Evidence frame {frame_id}"></a>'
            f"<figcaption>Frame {frame_id}: {escape(labels)}</figcaption>"
            "</figure>"
        )

    return (
        """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Spatiotemporal relationship evidence</title>
<style>
body{font:15px system-ui,sans-serif;margin:2rem;line-height:1.45;color:#172033}
.warning{padding:1rem;background:#fff3cd;border:1px solid #e3bd59;border-radius:8px}
table{border-collapse:collapse;width:100%;margin:1.5rem 0}th,td{border:1px solid #ccd2dc;padding:.55rem;text-align:left;vertical-align:top}th{background:#eef2f7}
code{font-size:11px;word-break:break-all}figure{display:inline-block;width:min(420px,100%);margin:0 1rem 1rem 0;vertical-align:top}img{width:100%;height:auto;border:1px solid #ccd2dc}figcaption{padding:.35rem 0}
</style>
</head>
<body>
<h1>Spatiotemporal relationship evidence</h1>
<p class="warning"><strong>Review required:</strong> These are teacher pseudo-labels, not ground truth. Inspect boxes, identities, labels, and relationships before using the episode for model-quality claims.</p>
<h2>Questions and private oracle</h2>
<table><thead><tr><th>ID</th><th>Question</th><th>Expected</th><th>Evidence</th></tr></thead><tbody>
"""
        + "\n".join(rows)
        + """
</tbody></table>
<h2>Annotated evidence frames</h2>
<div>"""
        + "\n".join(cards)
        + """</div>
</body>
</html>
"""
    )


def write_evidence_viewer(
    video_path: Path,
    bundle: EvidenceBundle,
    output_root: Path,
) -> EvidenceViewerResult:
    """Write an evaluator-only HTML report linked to annotated private evidence."""
    _assert_no_symlink_components(video_path)
    if not video_path.is_file():
        raise ValueError(f"evidence video must be a regular file: {video_path}")
    _validate_output_root(output_root)
    if video_path.resolve().is_relative_to(output_root.resolve(strict=False)):
        raise ValueError("evidence video cannot be inside viewer output")

    evidence_by_question, frame_ids = _frame_evidence(bundle)
    html = _render_html(bundle, evidence_by_question)
    staging = _create_staging_root(output_root)
    try:
        _write_evidence_frames(video_path, staging, frame_ids, bundle.observations)
        (staging / "index.html").write_text(html, encoding="utf-8")
        _replace_output_atomically(staging, output_root)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return EvidenceViewerResult(
        index_path="index.html",
        evidence_frame_count=len(frame_ids),
        question_count=len(bundle.questions),
    )
