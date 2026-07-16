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

"""Tests for dynamic robot-relationship video snapshots."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from dimos.benchmark.spatiotemporal.models import BoundingBox2D, SpatialPredicate
from dimos.benchmark.spatiotemporal.ports import DetectedObject
from dimos.benchmark.spatiotemporal.relationship_video import (
    derive_relationship_snapshot,
    main,
    render_relationship_video,
)
from dimos.msgs.sensor_msgs.Image import Image


def _detection(
    object_id: str,
    label: str,
    box: tuple[float, float, float, float],
    confidence: float,
) -> DetectedObject:
    return DetectedObject(
        object_id=object_id,
        label=label,
        box=BoundingBox2D(
            x_min=box[0],
            y_min=box[1],
            x_max=box[2],
            y_max=box[3],
        ),
        confidence=confidence,
    )


def test_relationship_snapshot_contract_is_deterministic_and_explicit() -> None:
    detections = (
        _detection("robot-low", "quadruped robot", (0.0, 0.0, 0.2, 0.2), 0.5),
        _detection("z-table", "table", (0.1, 0.6, 0.5, 0.9), 0.8),
        _detection("robot-high", "quadruped robot", (0.4, 0.4, 0.6, 0.6), 0.9),
        _detection("a-lamp", "lamp", (0.7, 0.1, 0.9, 0.3), 0.7),
    )

    snapshot = derive_relationship_snapshot(detections, robot_label="quadruped robot")

    assert snapshot.robot == detections[2]
    assert snapshot.message is None
    assert [relationship.object.object_id for relationship in snapshot.relationships] == [
        "a-lamp",
        "z-table",
    ]
    assert [
        (relationship.horizontal, relationship.vertical) for relationship in snapshot.relationships
    ] == [
        (SpatialPredicate.LEFT_OF, SpatialPredicate.BELOW),
        (SpatialPredicate.RIGHT_OF, SpatialPredicate.ABOVE),
    ]
    assert (
        derive_relationship_snapshot(detections[1:], robot_label="missing robot").message
        == "Robot not detected"
    )
    assert (
        derive_relationship_snapshot((detections[2],), robot_label="quadruped robot").message
        == "No other objects detected"
    )


class _ChangingDetector:
    def __init__(self) -> None:
        self.frame_ids: list[str] = []
        self.closed = False

    def detect(self, image: Image) -> tuple[DetectedObject, ...]:
        self.frame_ids.append(image.frame_id)
        object_box = (0.55, 0.55, 0.85, 0.85) if image.frame_id == "0" else (0.1, 0.1, 0.4, 0.4)
        return (
            _detection("robot", "quadruped robot", (0.4, 0.4, 0.6, 0.6), 0.9),
            _detection("chair", "chair", object_box, 0.8),
        )

    def close(self) -> None:
        self.closed = True


def _write_video(path: Path, *, fps: float, frame_count: int, size: tuple[int, int]) -> None:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),  # type: ignore[attr-defined]
        fps,
        size,
    )
    assert writer.isOpened()
    try:
        for _ in range(frame_count):
            writer.write(np.zeros((size[1], size[0], 3), dtype=np.uint8))
    finally:
        writer.release()


def _decode_video(path: Path) -> tuple[float, tuple[int, int], list[np.ndarray]]:
    capture = cv2.VideoCapture(str(path))
    assert capture.isOpened()
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        size = (
            int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        frames: list[np.ndarray] = []
        while True:
            decoded, frame = capture.read()
            if not decoded:
                break
            frames.append(frame)
    finally:
        capture.release()
    return fps, size, frames


def test_renderer_refreshes_once_per_second_and_preserves_full_rate_overlay(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.mp4"
    output = tmp_path / "relationships.mp4"
    _write_video(source, fps=2.0, frame_count=6, size=(128, 96))
    detector = _ChangingDetector()

    render_relationship_video(source, output, detector)

    output_fps, output_size, frames = _decode_video(output)
    assert detector.frame_ids == ["0", "2", "4"]
    assert detector.closed
    assert output_fps == 2.0
    assert output_size == (128, 96)
    assert len(frames) == 6
    assert np.mean(cv2.absdiff(frames[0], frames[1])) < 2.0
    assert np.mean(cv2.absdiff(frames[1], frames[2])) > 2.0
    assert np.mean(cv2.absdiff(frames[2], frames[3])) < 2.0


def test_renderer_attempts_all_cleanup_when_processing_and_release_fail(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FailingCapture:
        released = False

        def get(self, property_id: int) -> float:
            return {
                cv2.CAP_PROP_FPS: 1.0,
                cv2.CAP_PROP_FRAME_WIDTH: 16.0,
                cv2.CAP_PROP_FRAME_HEIGHT: 16.0,
            }[property_id]

        def read(self) -> tuple[bool, np.ndarray]:
            return True, np.zeros((16, 16, 3), dtype=np.uint8)

        def release(self) -> None:
            self.released = True
            raise RuntimeError("capture cleanup failed")

    class RecordingWriter:
        released = False

        def write(self, frame: np.ndarray) -> None:
            raise AssertionError("detector should fail before writing")

        def release(self) -> None:
            self.released = True

    class FailingDetector(_ChangingDetector):
        def detect(self, image: Image) -> tuple[DetectedObject, ...]:
            raise RuntimeError("detection failed")

    capture = FailingCapture()
    writer = RecordingWriter()
    detector = FailingDetector()
    monkeypatch.setattr(cv2, "VideoCapture", lambda _: capture)
    monkeypatch.setattr(cv2, "VideoWriter", lambda *_: writer)

    with pytest.raises(RuntimeError, match="capture cleanup failed"):
        render_relationship_video(Path("source.mp4"), Path("output.mp4"), detector)

    assert capture.released
    assert writer.released
    assert detector.closed


def test_cli_contract_is_configurable_safe_and_atomic(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = tmp_path / "source.mp4"
    output = tmp_path / "relationships.mp4"
    _write_video(source, fps=2.0, frame_count=2, size=(64, 48))
    detector = _ChangingDetector()
    configured_prompts: list[tuple[str, ...]] = []
    render_options: list[tuple[str, float]] = []

    def detector_factory(prompts: tuple[str, ...]) -> _ChangingDetector:
        configured_prompts.append(prompts)
        return detector

    def recording_render(
        source_path: Path,
        output_path: Path,
        configured_detector: _ChangingDetector,
        robot_label: str = "quadruped robot",
        update_period_s: float = 1.0,
    ) -> None:
        assert output_path.is_file()
        assert not output_path.is_symlink()
        render_options.append((robot_label, update_period_s))
        render_relationship_video(
            source_path,
            output_path,
            configured_detector,
            robot_label,
            update_period_s,
        )

    monkeypatch.setattr(
        "dimos.benchmark.spatiotemporal.relationship_video.render_relationship_video",
        recording_render,
    )

    main(
        [
            str(source),
            str(output),
            "--robot-label",
            "robot dog",
            "--update-period",
            "0.5",
            "--prompts",
            "robot dog",
            "refrigerator",
        ],
        detector_factory=detector_factory,
    )

    assert configured_prompts == [("robot dog", "refrigerator")]
    assert render_options == [("robot dog", 0.5)]
    assert output.is_file()
    assert "Wrote 2 frames" in capsys.readouterr().out

    with pytest.raises(ValueError, match="different files"):
        main([str(source), str(source)], detector_factory=detector_factory)

    symlink_output = tmp_path / "symlink.mp4"
    symlink_output.symlink_to(output)
    with pytest.raises(ValueError, match="symlink"):
        main([str(source), str(symlink_output)], detector_factory=detector_factory)

    with pytest.raises(SystemExit):
        main(
            [str(source), str(tmp_path / "invalid-period.mp4"), "--update-period", "0"],
            detector_factory=detector_factory,
        )

    unreadable = tmp_path / "not-video.mp4"
    unreadable.write_text("not a video", encoding="utf-8")
    unreadable_output = tmp_path / "unreadable-output.mp4"
    with pytest.raises(RuntimeError, match="failed to open source video"):
        main([str(unreadable), str(unreadable_output)], detector_factory=detector_factory)
    assert not unreadable_output.exists()

    class ClosedWriter:
        def isOpened(self) -> bool:
            return False

        def release(self) -> None:
            pass

    failed_output = tmp_path / "writer-failure.mp4"
    failed_output.write_bytes(b"existing output")
    monkeypatch.setattr(cv2, "VideoWriter", lambda *_args: ClosedWriter())
    with pytest.raises(RuntimeError, match="failed to open output video writer"):
        main([str(source), str(failed_output)], detector_factory=detector_factory)
    assert failed_output.read_bytes() == b"existing output"
    assert not list(tmp_path.glob(".*.tmp.mp4"))
