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

"""Tests for the evaluator-only, human-inspectable evidence viewer."""

from pathlib import Path
from types import SimpleNamespace
from typing import cast

import cv2
import numpy as np
import pytest

from dimos.benchmark.spatiotemporal.evidence_viewer import (
    EvidenceBundle,
    _annotate_frame,
    _object_color,
    write_evidence_viewer,
)
from dimos.benchmark.spatiotemporal.models import (
    BoundingBox2D,
    ObjectObservation,
    OracleAnswer,
    Question,
    QuestionKind,
    SpatialPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import stable_question_id


class _FakeCapture:
    def __init__(self, frame_count: int = 1) -> None:
        self._frame_count = frame_count
        self._frame_id = 0
        self.released = False

    def isOpened(self) -> bool:
        return True

    def read(self) -> tuple[bool, np.ndarray | None]:
        if self._frame_id >= self._frame_count:
            return False, None
        frame = np.full((20, 30, 3), self._frame_id, dtype=np.uint8)
        self._frame_id += 1
        return True, frame

    def release(self) -> None:
        self.released = True


def _bundle() -> EvidenceBundle:
    question_id = stable_question_id(
        episode_id="episode_1",
        object_ids=("oven", "robot"),
        predicate=SpatialPredicate.LEFT_OF.value,
        question_kind=QuestionKind.SPATIAL.value,
    )
    return cast(
        "EvidenceBundle",
        SimpleNamespace(
            questions=(
                Question(
                    question_id=question_id,
                    episode_id="episode_1",
                    text="Did <oven> ever appear left of robot?",
                    question_kind=QuestionKind.SPATIAL,
                    predicate=SpatialPredicate.LEFT_OF,
                    object_ids=("oven", "robot"),
                ),
            ),
            answers=(
                OracleAnswer(
                    question_id=question_id,
                    expected=True,
                    evidence_frame_ids=(0,),
                ),
            ),
            observations=(
                ObjectObservation(
                    episode_id="episode_1",
                    frame_id=0,
                    timestamp_s=0.0,
                    object_id="oven",
                    label="<oven>",
                    box=BoundingBox2D(x_min=0.1, y_min=0.1, x_max=0.4, y_max=0.6),
                    confidence=0.9,
                ),
                ObjectObservation(
                    episode_id="episode_1",
                    frame_id=1,
                    timestamp_s=0.1,
                    object_id="robot",
                    label="robot",
                    box=BoundingBox2D(x_min=0.5, y_min=0.1, x_max=0.8, y_max=0.6),
                    confidence=0.8,
                ),
            ),
            relation_intervals=(),
        ),
    )


def test_writes_deterministic_escaped_pseudo_label_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "video.mp4"
    source.write_bytes(b"decoded by fake capture")
    captures: list[_FakeCapture] = []

    def capture_factory(_: str) -> _FakeCapture:
        capture = _FakeCapture()
        captures.append(capture)
        return capture

    monkeypatch.setattr(cv2, "VideoCapture", capture_factory)
    roots = (tmp_path / "first", tmp_path / "second")

    results = tuple(write_evidence_viewer(source, _bundle(), root) for root in roots)

    assert results[0] == results[1]
    assert results[0].evidence_frame_count == 1
    assert results[0].question_count == 1
    outputs = [
        {path.relative_to(root): path.read_bytes() for path in root.rglob("*") if path.is_file()}
        for root in roots
    ]
    assert outputs[0] == outputs[1]
    html = (roots[0] / "index.html").read_text(encoding="utf-8")
    assert "teacher pseudo-labels, not ground truth" in html
    assert "One generated proof chain" in html
    assert "Robot-motion verification" in html
    assert "Motion evidence incomplete; review required" in html
    assert 'aria-pressed="true">All' in html
    assert 'aria-pressed="false">Spatial' in html
    assert "Relation timeline" in html
    assert "Future-project leverage" in html
    assert "Questions and private oracle" in html
    assert "Did &lt;oven&gt; ever appear left of robot?" in html
    assert "&lt;oven&gt;" in html
    assert "<oven>" not in html
    assert (roots[0] / "frames" / "frame_000000.jpg").is_file()
    assert "frame_000001.jpg" not in html
    assert not (roots[0] / "frames" / "frame_000001.jpg").exists()
    assert all(capture.released for capture in captures)


def test_annotation_maps_normalized_box_to_frame_pixels() -> None:
    observation = _bundle().observations[0]
    annotated = _annotate_frame(np.zeros((20, 30, 3), dtype=np.uint8), (observation,))

    assert tuple(int(component) for component in annotated[12, 12]) == _object_color(
        observation.object_id
    )


def test_resolves_temporal_interval_to_sparse_nonzero_frame(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "video.mp4"
    source.write_bytes(b"video")
    base = _bundle()
    interval_id = f"interval_{'1' * 64}"
    bundle = cast(
        "EvidenceBundle",
        SimpleNamespace(
            questions=base.questions,
            answers=(
                OracleAnswer(
                    question_id=base.questions[0].question_id,
                    expected=True,
                    evidence_interval_ids=(interval_id,),
                ),
            ),
            observations=base.observations,
            relation_intervals=(
                SimpleNamespace(
                    interval_id=interval_id,
                    relation_id=f"relation_{'2' * 64}",
                    episode_id="episode_1",
                    subject_id="oven",
                    predicate=SpatialPredicate.LEFT_OF,
                    object_id="robot",
                    start_frame_id=2,
                    end_frame_id=2,
                    start_timestamp_s=0.2,
                    end_timestamp_s=0.2,
                    evidence_frame_ids=(2,),
                ),
            ),
        ),
    )
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: _FakeCapture(frame_count=3))

    result = write_evidence_viewer(source, bundle, tmp_path / "viewer")

    assert result.evidence_frame_count == 1
    assert (tmp_path / "viewer" / "frames" / "frame_000002.jpg").is_file()
    html = (tmp_path / "viewer" / "index.html").read_text(encoding="utf-8")
    assert "frame_000002.jpg" in html


def test_rejects_symlinked_viewer_output(tmp_path: Path) -> None:
    source = tmp_path / "video.mp4"
    source.write_bytes(b"video")
    actual = tmp_path / "actual"
    actual.mkdir()
    linked = tmp_path / "linked"
    linked.symlink_to(actual, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        write_evidence_viewer(source, _bundle(), linked)

    assert not tuple(actual.iterdir())


def test_rejects_symlinked_output_ancestor(tmp_path: Path) -> None:
    source = tmp_path / "video.mp4"
    source.write_bytes(b"video")
    external = tmp_path / "external"
    external.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(external, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        write_evidence_viewer(source, _bundle(), linked_parent / "viewer")

    assert not tuple(external.iterdir())


def test_rejects_source_inside_output_without_deleting_it(tmp_path: Path) -> None:
    output = tmp_path / "viewer"
    output.mkdir()
    source = output / "video.mp4"
    source.write_bytes(b"must survive")

    with pytest.raises(ValueError, match="inside viewer output"):
        write_evidence_viewer(source, _bundle(), output)

    assert source.read_bytes() == b"must survive"


def test_failed_render_preserves_existing_viewer(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "video.mp4"
    source.write_bytes(b"video")
    output = tmp_path / "viewer"
    output.mkdir()
    marker = output / "existing.txt"
    marker.write_text("keep me", encoding="utf-8")

    class BrokenCapture(_FakeCapture):
        def read(self) -> tuple[bool, None]:
            return False, None

    monkeypatch.setattr(cv2, "VideoCapture", lambda _: BrokenCapture())

    with pytest.raises(RuntimeError, match="decode evidence frame"):
        write_evidence_viewer(source, _bundle(), output)

    assert marker.read_text(encoding="utf-8") == "keep me"
    assert tuple(output.iterdir()) == (marker,)
