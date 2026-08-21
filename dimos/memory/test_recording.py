# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any

import pytest
from pytest_mock import MockerFixture
from reactivex.disposable import CompositeDisposable
from reactivex.subject import Subject

from dimos.memory.module import Recorder, pose_setter_for
from dimos.memory.recording import RecordingFailedError, RecordingPipeline
from dimos.memory.stream import Stream
from dimos.msgs.geometry_msgs.Pose import Pose


class _StampedMessage:
    def __init__(self) -> None:
        self.ts = 12.5
        self.frame_id = "camera"


class _PoseRecorder(Recorder):
    @pose_setter_for("camera")
    async def _camera_pose(self, _message: Any) -> Pose:
        return Pose(1.0, 2.0, 3.0)


def test_pipeline_preserves_global_fifo_and_drains_on_close() -> None:
    events: list[str] = []
    pipeline = RecordingPipeline(
        {
            "camera": lambda value: events.append(f"camera-{value}"),
            "imu": lambda value: events.append(f"imu-{value}"),
        }
    )

    pipeline.start()
    pipeline.submit("camera", 1)
    pipeline.submit("imu", 2)
    pipeline.submit("camera", 3)
    pipeline.submit("imu", 4)
    pipeline.close(timeout_s=1.0)

    assert events == ["camera-1", "imu-2", "camera-3", "imu-4"]


def test_pipeline_fails_instead_of_dropping_when_queue_is_full() -> None:
    started = threading.Event()
    release = threading.Event()

    def process(_value: int) -> None:
        started.set()
        if not release.wait(timeout=1.0):
            raise TimeoutError("test did not release processor")

    pipeline = RecordingPipeline({"depth": process}, queue_size=1)
    pipeline.start()
    try:
        pipeline.submit("depth", 1)
        assert started.wait(timeout=1.0)
        pipeline.submit("depth", 2)
        with pytest.raises(RecordingFailedError, match="reached capacity"):
            pipeline.submit("depth", 3)
    finally:
        release.set()

    with pytest.raises(RecordingFailedError, match="failed"):
        pipeline.close(timeout_s=1.0)


def test_pipeline_surfaces_processor_failure() -> None:
    def process(_value: int) -> None:
        raise OSError("database locked")

    pipeline = RecordingPipeline({"depth": process})
    pipeline.start()
    pipeline.submit("depth", 1)

    with pytest.raises(RecordingFailedError, match="failed") as exc:
        pipeline.close(timeout_s=1.0)

    assert isinstance(exc.value.__cause__, OSError)


def test_pipeline_rejects_messages_after_close() -> None:
    pipeline = RecordingPipeline({"camera": lambda _value: None})
    pipeline.start()
    pipeline.close(timeout_s=1.0)

    with pytest.raises(RecordingFailedError, match="closed"):
        pipeline.submit("camera", 1)


def test_pipeline_rejects_unknown_stream() -> None:
    pipeline = RecordingPipeline({"camera": lambda _value: None})
    pipeline.start()
    try:
        with pytest.raises(KeyError, match="Unknown recording stream"):
            pipeline.submit("depth", 1)
    finally:
        pipeline.close(timeout_s=1.0)


def test_pipeline_requires_positive_queue_size() -> None:
    with pytest.raises(ValueError, match="queue_size must be positive"):
        RecordingPipeline({}, queue_size=0)


def test_recorder_worker_preserves_payload_and_async_pose_setter(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    recorder = _PoseRecorder(db_path=tmp_path / "recording.db")
    recorder._pose_setters = recorder._collect_pose_setters()
    stream = mocker.Mock(spec=Stream)
    message = _StampedMessage()

    try:
        recorder._port_processor("camera", stream)((20.0, message))
    finally:
        recorder.stop()

    assert message.ts == 12.5
    stream.append.assert_called_once_with(
        message,
        ts=12.5,
        pose=Pose(1.0, 2.0, 3.0),
        tags={"reception_ts": 20.0},
    )


def test_recorder_stop_waits_for_in_flight_callback(tmp_path: Path, mocker: MockerFixture) -> None:
    recorder = _PoseRecorder(db_path=tmp_path / "recording.db")
    processor = mocker.Mock()
    pipeline = RecordingPipeline({"camera": processor})
    subscriptions = CompositeDisposable()
    topic = mocker.Mock()
    subject = Subject()
    topic.pure_observable.return_value = subject
    callback_started = threading.Event()
    release_callback = threading.Event()
    close_started = threading.Event()
    close_finished = threading.Event()
    stop_finished = threading.Event()
    thread_errors: list[Exception] = []
    message = _StampedMessage()

    def reception_time() -> float:
        callback_started.set()
        if not release_callback.wait(timeout=1.0):
            raise TimeoutError("test did not release callback")
        return 20.0

    original_close = pipeline.close

    def tracked_close() -> None:
        close_started.set()
        original_close(timeout_s=1.0)
        close_finished.set()

    def publish() -> None:
        try:
            subject.on_next(message)
        except Exception as error:
            thread_errors.append(error)

    def stop() -> None:
        try:
            recorder.stop()
        except Exception as error:
            thread_errors.append(error)
        finally:
            stop_finished.set()

    recorder._recording_pipeline = pipeline
    recorder._recording_subscriptions = subscriptions
    recorder._subscribe_port("camera", topic)
    pipeline.start()
    mocker.patch("dimos.memory.module.time.time", side_effect=reception_time)
    mocker.patch.object(pipeline, "close", side_effect=tracked_close)
    publisher = threading.Thread(target=publish)
    stopper = threading.Thread(target=stop)

    try:
        publisher.start()
        assert callback_started.wait(timeout=1.0)
        stopper.start()
        assert close_started.wait(timeout=1.0)
        assert not close_finished.is_set()
    finally:
        release_callback.set()
        publisher.join(timeout=1.0)
        stopper.join(timeout=1.0)

    assert stop_finished.is_set()
    assert thread_errors == []
    processor.assert_called_once_with((20.0, message))
