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

import threading

import pytest

from dimos.memory.module import _snapshot_recording_message
from dimos.memory.recorder_queue import RecorderFailedError, RecorderQueue


class _ReusedPointCloud:
    msg_name = "sensor_msgs.PointCloud2"

    def __init__(self) -> None:
        self.ts = 1.0
        self.points = [[1.0, 2.0, 3.0]]


def test_recorder_queue_preserves_fifo_during_stall() -> None:
    release = threading.Event()
    processed: list[int] = []

    accepted_times: list[float] = []

    def process(value: int, accepted_monotonic: float) -> None:
        if value == 0:
            assert release.wait(timeout=1.0)
        processed.append(value)
        accepted_times.append(accepted_monotonic)

    recorder_queue = RecorderQueue("camera", process, max_pending=16, max_backlog_s=1.0)
    for value in range(10):
        recorder_queue.submit(value)
    release.set()

    recorder_queue.close(timeout_s=1.0)

    assert processed == list(range(10))
    assert accepted_times == sorted(accepted_times)


def test_recorder_queue_reports_worker_failure() -> None:
    def process(_: int, __: float) -> None:
        raise OSError("disk failed")

    recorder_queue = RecorderQueue("imu", process, max_pending=4, max_backlog_s=1.0)
    recorder_queue.submit(1)

    with pytest.raises(RecorderFailedError, match="failed"):
        recorder_queue.flush(timeout_s=1.0)

    with pytest.raises(RecorderFailedError, match="failed"):
        recorder_queue.close(timeout_s=1.0)


def test_recorder_queue_fails_instead_of_dropping_when_full() -> None:
    release = threading.Event()
    started = threading.Event()

    def process(_: int, __: float) -> None:
        started.set()
        release.wait(timeout=1.0)

    recorder_queue = RecorderQueue("depth", process, max_pending=1, max_backlog_s=1.0)
    recorder_queue.submit(1)
    assert started.wait(timeout=1.0)
    recorder_queue.submit(2)

    with pytest.raises(RecorderFailedError, match="full"):
        recorder_queue.submit(3)

    release.set()
    with pytest.raises(RecorderFailedError, match="failed"):
        recorder_queue.close(timeout_s=1.0)


def test_pointcloud_snapshot_detaches_nested_mutable_state() -> None:
    source = _ReusedPointCloud()
    snapshot = _snapshot_recording_message(source)

    source.ts = 2.0
    source.points[0][0] = 9.0

    assert snapshot.ts == 1.0
    assert snapshot.points == [[1.0, 2.0, 3.0]]
