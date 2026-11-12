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

from collections.abc import Iterator
import threading
from unittest.mock import MagicMock

import cv2
import numpy as np
from numpy.typing import NDArray
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.experimental.object_tracker import ObjectTracking
from dimos.perception.experimental.object_tracker_2d import ObjectTracker2D
from dimos.perception.experimental.object_tracker_3d import ObjectTracker3D
from dimos.protocol.rpc.spec import RPCSpec

pytestmark = pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning")


class MockRPC(MagicMock, RPCSpec):
    """A mock that satisfies the module config's RPC transport type check."""


@pytest.fixture(params=[ObjectTracker2D, ObjectTracker3D, ObjectTracking])
def tracker_module(request: pytest.FixtureRequest) -> Iterator[ObjectTracker2D | ObjectTracking]:
    module = request.param(rpc_transport=MockRPC)
    module._latest_rgb_frame = np.random.default_rng(0).integers(
        0, 256, (96, 96, 3), dtype=np.uint8
    )
    if isinstance(module, ObjectTracking):
        module._latest_depth_frame = np.ones((96, 96), dtype=np.float32)
    yield module
    module.stop()


@pytest.fixture
def paused_worker(
    tracker_module: ObjectTracker2D | ObjectTracking, mocker: MockerFixture
) -> Iterator[tuple[threading.Thread, threading.Event]]:
    entered = threading.Event()
    release = threading.Event()

    def pause_frame() -> None:
        entered.set()
        assert release.wait(timeout=10), "Worker was never released"

    if isinstance(tracker_module, ObjectTracker2D):
        cvt_color = cv2.cvtColor

        def paused_conversion(frame: NDArray[np.uint8], code: int) -> NDArray[np.uint8]:
            if threading.current_thread() is tracker_module.tracking_thread:
                pause_frame()
            return cvt_color(frame, code)

        mocker.patch.object(cv2, "cvtColor", side_effect=paused_conversion)
    else:
        # This tracker updates RGB directly, so pause while obtaining the frame.
        frame_lock = mocker.patch.object(tracker_module, "_frame_lock")
        frame_lock.__enter__.side_effect = pause_frame

    worker = None
    try:
        assert tracker_module.track([24, 24, 64, 64])["status"] == "tracking_started"
        worker = tracker_module.tracking_thread
        assert worker is not None
        assert entered.wait(timeout=10), "Worker did not reach frame processing"
        yield worker, release
    finally:
        release.set()
        tracker_module.stop()
        if worker is not None:
            worker.join(timeout=10)


@pytest.mark.parametrize("stop_method, expected", [("stop_track", True), ("stop", None)])
def test_stop_waits_for_inflight_frame_before_clearing_state(
    tracker_module: ObjectTracker2D | ObjectTracking,
    paused_worker: tuple[threading.Thread, threading.Event],
    mocker: MockerFixture,
    stop_method: str,
    expected: bool | None,
) -> None:
    worker, release = paused_worker
    join = worker.join
    publish = mocker.spy(tracker_module.detection2darray, "publish")

    def release_on_join(timeout: float | None = None) -> None:
        release.set()
        join(timeout)

    mocker.patch.object(worker, "join", side_effect=release_on_join)

    assert getattr(tracker_module, stop_method)() is expected

    assert not worker.is_alive()
    assert tracker_module.tracking_thread is None
    assert tracker_module.tracker is None
    assert not tracker_module.tracking_initialized
    assert publish.call_args.args[0].detections == []


def test_stop_from_worker_defers_reset_until_external_join(
    tracker_module: ObjectTracker2D | ObjectTracking, mocker: MockerFixture
) -> None:
    stop_requested = threading.Event()
    results = []

    def stop_on_detection(detection: Detection2DArray) -> None:
        if threading.current_thread() is tracker_module.tracking_thread:
            results.append(tracker_module.stop_track())
            stop_requested.set()

    mocker.patch.object(tracker_module.detection2darray, "publish", side_effect=stop_on_detection)
    assert tracker_module.track([24, 24, 64, 64])["status"] == "tracking_started"
    assert stop_requested.wait(timeout=10)
    worker = tracker_module.tracking_thread
    assert worker is not None
    worker.join(timeout=10)

    assert not worker.is_alive()
    assert results == [False]
    assert tracker_module.tracker is not None
    assert tracker_module.stop_track() is True
    assert tracker_module.tracker is None
    assert tracker_module.tracking_thread is None


def test_stop_without_worker_is_repeatable(
    tracker_module: ObjectTracker2D | ObjectTracking,
) -> None:
    assert tracker_module.stop_track() is True
    assert tracker_module.stop_track() is True
    assert not tracker_module.is_tracking()


def test_stop_timeout_retains_state_until_worker_can_be_joined(
    tracker_module: ObjectTracker2D | ObjectTracking,
    paused_worker: tuple[threading.Thread, threading.Event],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    worker, release = paused_worker
    tracker = tracker_module.tracker
    bbox = tracker_module.tracking_bbox
    monkeypatch.setattr(f"{tracker_module.stop_track.__module__}.DEFAULT_THREAD_JOIN_TIMEOUT", 0.0)

    assert tracker_module.stop_track() is False

    assert worker.is_alive()
    assert tracker_module.tracking_thread is worker
    assert tracker_module.tracker is tracker
    assert tracker_module.tracking_bbox == bbox
    assert tracker_module.tracking_initialized

    release.set()
    worker.join(timeout=10)
    assert not worker.is_alive()
    assert tracker_module.stop_track() is True
    assert tracker_module.tracking_thread is None
    assert tracker_module.tracker is None
    assert not tracker_module.tracking_initialized
