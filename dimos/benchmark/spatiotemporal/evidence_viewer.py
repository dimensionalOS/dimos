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
from itertools import pairwise
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

    questions = sorted(bundle.questions, key=lambda item: item.question_id)
    evidence_frame_ids = sorted(
        {frame_id for frame_ids in evidence_by_question.values() for frame_id in frame_ids}
    )
    episode_id = questions[0].episode_id if questions else "unknown"
    object_ids = sorted({observation.object_id for observation in bundle.observations})
    spatial_count = sum(question.question_kind.value == "spatial" for question in questions)
    temporal_count = len(questions) - spatial_count
    positive_count = sum(answer.expected for answer in bundle.answers)

    timestamps_by_frame: dict[int, float] = {}
    for observation in bundle.observations:
        timestamps_by_frame.setdefault(observation.frame_id, observation.timestamp_s)
    for interval in bundle.relation_intervals:
        timestamps_by_frame.setdefault(interval.start_frame_id, interval.start_timestamp_s)
        timestamps_by_frame.setdefault(interval.end_frame_id, interval.end_timestamp_s)

    relations_by_frame: dict[int, list[str]] = defaultdict(list)
    for interval in bundle.relation_intervals:
        relation = (
            f"{interval.subject_id} {interval.predicate.value.replace('-', ' ')} "
            f"{interval.object_id}"
        )
        for frame_id in interval.evidence_frame_ids:
            relations_by_frame[frame_id].append(relation)

    robot_observations = [
        observation
        for observation in bundle.observations
        if "robot" in observation.label.casefold()
    ]
    robot_ids = sorted({observation.object_id for observation in robot_observations})
    robot_frames = sorted({observation.frame_id for observation in robot_observations})
    robot_coverage = len(robot_frames) / len(evidence_frame_ids) if evidence_frame_ids else 0.0
    referenced_interval_ids = {
        interval_id for answer in bundle.answers for interval_id in answer.evidence_interval_ids
    }
    displayed_frames = set(evidence_frame_ids)
    linked_intervals = [
        interval
        for interval in bundle.relation_intervals
        if interval.interval_id in referenced_interval_ids
        and bool(displayed_frames.intersection(interval.evidence_frame_ids))
    ]
    relation_status = (
        "pass"
        if linked_intervals and len(linked_intervals) == len(bundle.relation_intervals)
        else "review"
    )
    ordered_intervals = sorted(
        linked_intervals,
        key=lambda interval: (interval.start_timestamp_s, interval.interval_id),
    )
    strict_order = len(ordered_intervals) > 1 and all(
        left.end_frame_id < right.start_frame_id and left.end_timestamp_s < right.start_timestamp_s
        for left, right in pairwise(ordered_intervals)
    )
    proof_question = next(
        (
            question
            for question in questions
            if question.question_kind.value == "spatial"
            and answers[question.question_id].expected
            and evidence_by_question.get(question.question_id)
        ),
        None,
    )
    if proof_question is not None and len(proof_question.object_ids) == 2:
        proof_frame = evidence_by_question[proof_question.question_id][0]
        subject_id, object_id = proof_question.object_ids
        proof_relation = (
            f"{subject_id} {proof_question.predicate.value.replace('-', ' ')} {object_id}"
        )
        proof_chain = (
            f"<div><strong>01 · evidence</strong><span>frame {proof_frame}</span></div>"
            f"<div><strong>02 · derived fact</strong><span>{escape(proof_relation)}</span></div>"
            f"<div><strong>03 · public test</strong><span>{escape(proof_question.text)}</span></div>"
            f"<div><strong>04 · private oracle</strong><span>true · evidence f{proof_frame}</span></div>"
        )
    else:
        proof_chain = "<p class=muted>No positive spatial proof is available.</p>"

    rows: list[str] = []
    for question in questions:
        try:
            answer = answers[question.question_id]
        except KeyError as error:
            raise ValueError(
                f"question is missing oracle answer: {question.question_id}"
            ) from error
        frame_ids = evidence_by_question.get(question.question_id, ())
        links = " ".join(
            f'<a class="frame-link" href="#frame-{frame_id}">f{frame_id}</a>'
            for frame_id in frame_ids
        )
        expected = str(answer.expected).lower()
        rows.append(
            f'<tr data-kind="{question.question_kind.value}" data-expected="{expected}" '
            f'data-search="{escape(question.text.casefold())}">'
            f'<td><span class="kind kind-{question.question_kind.value}">'
            f"{question.question_kind.value}</span></td>"
            f"<td><strong>{escape(question.text)}</strong>"
            f'<code title="{escape(question.question_id)}">…{escape(question.question_id[-10:])}</code></td>'
            f'<td><span class="answer answer-{expected}">{expected}</span></td>'
            f"<td>{links or '<span class=muted>none</span>'}</td>"
            "</tr>"
        )

    timeline: list[str] = []
    cards: list[str] = []
    for frame_id in evidence_frame_ids:
        observations = sorted(observations_by_frame[frame_id], key=lambda item: item.object_id)
        timestamp_s = timestamps_by_frame.get(frame_id)
        timestamp = f"{timestamp_s:.3f}s" if timestamp_s is not None else "time unknown"
        relations = (
            "".join(
                f'<span class="relation-chip">{escape(relation)}</span>'
                for relation in sorted(relations_by_frame[frame_id])
            )
            or '<span class="muted">no accepted relation</span>'
        )
        timeline.append(
            f'<button class="timeline-stop" data-frame="{frame_id}" '
            f'aria-label="Show frame {frame_id}"><span class="timeline-dot"></span>'
            f"<strong>{timestamp}</strong><small>frame {frame_id}</small>{relations}</button>"
        )
        labels = (
            ", ".join(
                f"#{observation.object_id} {observation.label} ({observation.confidence:.2f})"
                for observation in observations
            )
            or "No accepted detections"
        )
        cards.append(
            f'<figure id="frame-{frame_id}" data-frame="{frame_id}">'
            f'<button class="image-button" data-image="frames/frame_{frame_id:06d}.jpg" '
            f'data-alt="Evidence frame {frame_id}">'
            f'<img src="frames/frame_{frame_id:06d}.jpg" alt="Evidence frame {frame_id}"></button>'
            f"<figcaption><div><strong>{timestamp}</strong><span>frame {frame_id}</span></div>"
            f"<p>{escape(labels)}</p><div class=chip-row>{relations}</div></figcaption>"
            "</figure>"
        )

    identity_status = "pass" if len(robot_ids) == 1 else "review"
    identity_detail = (
        f"one robot track ({robot_ids[0]})"
        if len(robot_ids) == 1
        else (
            f"robot label spans IDs {', '.join(robot_ids)}"
            if robot_ids
            else "no robot-labeled detection"
        )
    )
    coverage_status = "pass" if robot_coverage >= 0.8 else "review"
    if relation_status == "review" or not strict_order:
        motion_verdict = "Motion evidence incomplete; review required"
    elif identity_status == "review" or coverage_status == "review":
        motion_verdict = "Relation-order eval ready; motion claim needs review"
    else:
        motion_verdict = "Motion evidence ready for review"

    return (
        """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Video → Eval evidence</title>
<style>
:root{--bg:#09110f;--panel:#101b18;--panel2:#15231f;--ink:#edf7f2;--muted:#91a69e;--line:#294039;--mint:#62e6ae;--cyan:#6ccfe6;--amber:#f4c66a;--red:#ff8d7b;--radius:12px}*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 "Avenir Next","Segoe UI",sans-serif}body:before{content:"";position:fixed;inset:0;pointer-events:none;background-image:linear-gradient(rgba(98,230,174,.025) 1px,transparent 1px),linear-gradient(90deg,rgba(98,230,174,.025) 1px,transparent 1px);background-size:32px 32px}.shell{width:min(1440px,calc(100% - 40px));margin:auto;padding:28px 0 64px}.topbar{display:flex;justify-content:space-between;gap:24px;align-items:flex-start;border-bottom:1px solid var(--line);padding-bottom:22px}.eyebrow,.section-label{color:var(--mint);font:700 11px/1.2 ui-monospace,monospace;letter-spacing:.14em;text-transform:uppercase}h1{font-size:clamp(30px,5vw,58px);line-height:1;letter-spacing:-.045em;margin:8px 0 12px;max-width:760px}h2{font-size:24px;letter-spacing:-.025em;margin:6px 0}h3{font-size:16px;margin:0 0 8px}.lede{color:var(--muted);font-size:16px;max-width:760px;margin:0}.episode{font:12px ui-monospace,monospace;color:var(--muted);text-align:right}.episode strong{display:block;color:var(--ink);font-size:14px;margin-top:5px}.metrics{display:grid;grid-template-columns:repeat(5,1fr);border:1px solid var(--line);border-radius:var(--radius);margin:22px 0;overflow:hidden;background:var(--panel)}.metric{padding:16px 18px;border-right:1px solid var(--line)}.metric:last-child{border:0}.metric strong{font:700 24px ui-monospace,monospace;display:block}.metric span{color:var(--muted);font-size:12px}.grid{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(300px,.85fr);gap:18px;margin:18px 0}.panel{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:20px}.pipeline{display:grid;grid-template-columns:repeat(6,1fr);gap:0;margin-top:18px}.pipeline div{position:relative;padding:12px 10px;border:1px solid var(--line);background:var(--panel2);font-size:12px}.pipeline div:not(:last-child):after{content:"→";position:absolute;right:-9px;top:11px;color:var(--mint);z-index:2}.pipeline strong{display:block;font:700 11px ui-monospace,monospace;color:var(--mint);margin-bottom:4px}.warning{background:#281f10;border:1px solid #6d5526;color:#f9e6bc}.warning strong{color:var(--amber)}.warning p{margin:7px 0 0}.example-chain{display:grid;grid-template-columns:repeat(4,1fr);gap:0;margin-top:14px}.example-chain div{position:relative;padding:14px;border:1px solid var(--line);background:var(--panel2)}.example-chain div:not(:last-child):after{content:"→";position:absolute;right:-9px;top:24px;color:var(--mint);z-index:2}.example-chain strong,.example-chain span{display:block}.example-chain strong{font:700 10px ui-monospace,monospace;color:var(--mint);text-transform:uppercase;margin-bottom:6px}.example-chain span{font-size:12px}.proof-list{display:grid;gap:9px;margin-top:14px}.proof{display:grid;grid-template-columns:90px 1fr;gap:12px;align-items:start;border-top:1px solid var(--line);padding-top:9px}.status{font:700 10px ui-monospace,monospace;text-transform:uppercase;letter-spacing:.08em}.status-pass{color:var(--mint)}.status-review{color:var(--amber)}.timeline{display:grid;grid-template-columns:repeat(var(--timeline-count),minmax(150px,1fr));overflow:auto;padding:22px 4px 5px}.timeline-stop{position:relative;min-width:150px;text-align:left;color:var(--ink);background:transparent;border:0;border-top:2px solid var(--line);padding:20px 12px 8px;cursor:pointer}.timeline-stop:hover{background:var(--panel2)}.timeline-stop:focus-visible{background:var(--panel2);outline:2px solid var(--mint);outline-offset:2px}.timeline-dot{position:absolute;width:12px;height:12px;border:2px solid var(--mint);background:var(--bg);border-radius:50%;top:-7px;left:12px}.timeline-stop strong,.timeline-stop small{display:block}.timeline-stop small{color:var(--muted);margin-bottom:8px}.relation-chip,.object-chip,.frame-link{display:inline-flex;align-items:center;border:1px solid var(--line);border-radius:999px;padding:3px 7px;margin:2px;font:11px ui-monospace,monospace;color:var(--cyan);text-decoration:none}.object-chip{color:var(--ink)}.frame-link:hover{border-color:var(--mint);color:var(--mint)}.section-head{display:flex;align-items:end;justify-content:space-between;gap:20px;margin:38px 0 14px}.section-head p{color:var(--muted);margin:0;max-width:660px}.future-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.future{padding:16px;border-top:2px solid var(--mint);background:var(--panel)}.future strong{display:block;margin-bottom:6px}.future span{color:var(--muted);font-size:12px}.gallery{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:12px}.gallery figure{margin:0;background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);overflow:hidden;scroll-margin-top:20px}.gallery figure.highlight{border-color:var(--mint);box-shadow:0 0 0 2px rgba(98,230,174,.2)}.image-button{display:block;width:100%;border:0;padding:0;background:#000;cursor:zoom-in}.gallery img{display:block;width:100%;aspect-ratio:9/16;object-fit:cover}.gallery figcaption{padding:12px}.gallery figcaption>div:first-child{display:flex;justify-content:space-between}.gallery figcaption span,.gallery figcaption p{color:var(--muted);font-size:12px}.gallery figcaption p{margin:5px 0}.controls{display:flex;flex-wrap:wrap;gap:8px;align-items:center}.filter{border:1px solid var(--line);color:var(--muted);background:var(--panel);padding:8px 11px;border-radius:999px;cursor:pointer}.filter.active{border-color:var(--mint);color:var(--ink)}#search{margin-left:auto;min-width:260px;background:var(--panel);border:1px solid var(--line);color:var(--ink);padding:9px 12px;border-radius:8px}table{border-collapse:separate;border-spacing:0;width:100%;background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);overflow:hidden}th,td{text-align:left;padding:11px 12px;border-bottom:1px solid var(--line);vertical-align:top}th{font:700 10px ui-monospace,monospace;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);background:var(--panel2)}tr:last-child td{border:0}td code{display:block;color:var(--muted);font-size:10px;margin-top:3px}.kind,.answer{font:700 10px ui-monospace,monospace;text-transform:uppercase;border-radius:999px;padding:4px 7px}.kind-spatial{color:var(--mint);background:#143127}.kind-temporal{color:var(--cyan);background:#132c32}.answer-true{color:var(--mint)}.answer-false{color:var(--red)}.muted{color:var(--muted)}dialog{border:1px solid var(--line);background:var(--panel);padding:8px;border-radius:var(--radius);max-width:min(90vw,900px)}dialog::backdrop{background:rgba(0,0,0,.85)}dialog img{display:block;max-width:100%;max-height:85vh}.close{position:absolute;right:14px;top:14px;background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:999px;width:36px;height:36px;cursor:pointer}@media(max-width:1100px){.gallery{grid-template-columns:repeat(3,1fr)}}@media(max-width:900px){.grid{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}.metric{border-bottom:1px solid var(--line)}.pipeline{grid-template-columns:repeat(2,1fr)}.example-chain{grid-template-columns:1fr 1fr}.future-grid{grid-template-columns:1fr 1fr}.topbar{display:block}.episode{text-align:left;margin-top:18px}#search{margin-left:0;width:100%}}@media(max-width:650px){.gallery{grid-template-columns:1fr 1fr}}@media(max-width:560px){.shell{width:min(100% - 24px,1440px)}.metrics,.future-grid,.example-chain,.gallery{grid-template-columns:1fr}th:first-child,td:first-child{display:none}}
</style>
</head>
<body>
<main class="shell">
<header class="topbar"><div><div class="eyebrow">Evaluator-only review surface</div><h1>Video → relationship eval evidence</h1><p class="lede">Trace how sampled robot-video observations become spatial and temporal tests, then inspect the exact frames supporting each oracle answer.</p></div><div class="episode">EPISODE<strong>"""
        + escape(episode_id)
        + """</strong></div></header>
<section class="metrics" aria-label="Evaluation counts">
<div class="metric"><strong>"""
        + str(len(evidence_frame_ids))
        + """</strong><span>evidence frames</span></div><div class="metric"><strong>"""
        + str(len(object_ids))
        + """</strong><span>tracked IDs</span></div><div class="metric"><strong>"""
        + str(len(bundle.relation_intervals))
        + """</strong><span>relation intervals</span></div><div class="metric"><strong>"""
        + str(spatial_count)
        + """ / """
        + str(temporal_count)
        + """</strong><span>spatial / temporal</span></div><div class="metric"><strong>"""
        + str(positive_count)
        + """ / """
        + str(len(bundle.answers) - positive_count)
        + """</strong><span>true / false</span></div></section>
<section class="grid"><div class="panel"><div class="section-label">What the system does</div><h2>Turns observed relations into replayable tests</h2><div class="pipeline"><div><strong>01</strong>Video</div><div><strong>02</strong>Sample</div><div><strong>03</strong>Track</div><div><strong>04</strong>Relations</div><div><strong>05</strong>Questions</div><div><strong>06</strong>Score + evidence</div></div></div><aside class="panel warning"><div class="section-label">Evidence policy</div><h3>Teacher pseudo-labels, not ground truth</h3><p>These are teacher pseudo-labels, not ground truth. Inspect boxes, identity continuity, labels, and temporal ordering before making model-quality or robot-motion claims.</p></aside></section>
<section class="panel"><div class="section-label">One generated proof chain</div><h2>From a sampled frame to a scored evaluation</h2><div class="example-chain">"""
        + proof_chain
        + """</div></section>
<section class="panel"><div class="section-label">Robot-motion verification</div><h2>"""
        + escape(motion_verdict)
        + """</h2><div class="proof-list"><div class="proof"><span class="status status-"""
        + relation_status
        + """">"""
        + relation_status
        + """</span><div><strong>Relation events derived</strong><br><span class="muted">"""
        + f"{len(linked_intervals)}/{len(bundle.relation_intervals)}"
        + """ intervals are referenced by oracle answers and linked to displayed frames.</span></div></div><div class="proof"><span class="status status-"""
        + ("pass" if strict_order else "review")
        + """">"""
        + ("pass" if strict_order else "review")
        + """</span><div><strong>Temporal order</strong><br><span class="muted">Strict non-overlap across the displayed relation sequence.</span></div></div><div class="proof"><span class="status status-"""
        + identity_status
        + """">"""
        + identity_status
        + """</span><div><strong>Robot identity continuity</strong><br><span class="muted">"""
        + escape(identity_detail)
        + """.</span></div></div><div class="proof"><span class="status status-"""
        + coverage_status
        + """">"""
        + coverage_status
        + """</span><div><strong>Robot detection coverage</strong><br><span class="muted">"""
        + f"{len(robot_frames)}/{len(evidence_frame_ids)} evidence frames ({robot_coverage:.0%})"
        + """.</span></div></div></div></section>
<div class="section-head"><div><div class="section-label">Relation timeline</div><h2>What changed, and when</h2></div><p>Each stop is a sampled evidence frame. Relation labels come from private accepted intervals; gaps remain visible rather than interpolated.</p></div><section class="panel timeline" style="--timeline-count:"""
        + str(max(1, len(evidence_frame_ids)))
        + """">"""
        + "".join(timeline)
        + """</section>
<div class="section-head"><div><div class="section-label">Future-project leverage</div><h2>Reusable motion-evaluation patterns</h2></div><p>The same contracts can evaluate new videos without changing candidate/oracle separation.</p></div><section class="future-grid"><div class="future"><strong>Locomotion & patrol</strong><span>Verify before/after ordering around tracked landmarks and objects.</span></div><div class="future"><strong>Manipulation</strong><span>Turn above/below and left/right object transitions into regression cases.</span></div><div class="future"><strong>Long-horizon memory</strong><span>Test whether a candidate recalls relation events separated in time.</span></div><div class="future"><strong>Model releases</strong><span>Replay frozen bundles and block spatial or temporal regressions.</span></div></section>
<div class="section-head"><div><div class="section-label">Visual evidence</div><h2>Annotated sampled frames</h2></div><p>Click a frame to inspect full resolution. Colors are stable per object ID.</p></div><section class="gallery">"""
        + "".join(cards)
        + """</section>
<div class="section-head"><div><div class="section-label">Generated evaluation</div><h2>Questions and private oracle</h2></div><p>Filter by family or expected value; evidence links jump back to the supporting frames.</p></div><div class="controls" role="group" aria-label="Question filters"><button class="filter active" data-filter="all" aria-pressed="true">All</button><button class="filter" data-filter="spatial" aria-pressed="false">Spatial</button><button class="filter" data-filter="temporal" aria-pressed="false">Temporal</button><button class="filter" data-filter="true" aria-pressed="false">True</button><button class="filter" data-filter="false" aria-pressed="false">False</button><input id="search" type="search" placeholder="Search questions…" aria-label="Search questions"></div><div style="overflow:auto;margin-top:12px"><table><thead><tr><th>Family</th><th>Question</th><th>Expected</th><th>Evidence</th></tr></thead><tbody>"""
        + "".join(rows)
        + """</tbody></table></div>
</main><dialog id="lightbox"><button class="close" aria-label="Close">&times;</button><img alt=""></dialog>
<script>
const rows=[...document.querySelectorAll('tbody tr')],filters=[...document.querySelectorAll('.filter')],search=document.querySelector('#search');let active='all';function apply(){const q=search.value.toLowerCase();rows.forEach(r=>{const matchFilter=active==='all'||r.dataset.kind===active||r.dataset.expected===active;const matchSearch=r.dataset.search.includes(q);r.hidden=!(matchFilter&&matchSearch)})}filters.forEach(b=>b.addEventListener('click',()=>{filters.forEach(x=>{x.classList.remove('active');x.setAttribute('aria-pressed','false')});b.classList.add('active');b.setAttribute('aria-pressed','true');active=b.dataset.filter;apply()}));search.addEventListener('input',apply);document.querySelectorAll('.timeline-stop').forEach(b=>b.addEventListener('click',()=>{const card=document.querySelector(`#frame-${b.dataset.frame}`);document.querySelectorAll('.gallery figure').forEach(x=>x.classList.remove('highlight'));card.classList.add('highlight');card.tabIndex=-1;card.focus()}));const box=document.querySelector('#lightbox'),boxImg=box.querySelector('img');document.querySelectorAll('.image-button').forEach(b=>b.addEventListener('click',()=>{boxImg.src=b.dataset.image;boxImg.alt=b.dataset.alt;box.showModal()}));box.querySelector('.close').addEventListener('click',()=>box.close());box.addEventListener('click',e=>{if(e.target===box)box.close()});
</script></body></html>"""
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
