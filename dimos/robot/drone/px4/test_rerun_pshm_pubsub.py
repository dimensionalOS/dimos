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

from collections.abc import Callable
from unittest.mock import call

import numpy as np
from pytest_mock import MockerFixture

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.drone.px4.rerun_pshm_pubsub import Px4FixedTopicPshm


class FakePickleSharedMemory:
    def __init__(self) -> None:
        self.callback: Callable[[Image, str], None] | None = None
        self.started = False
        self.stopped = False
        self.topic: str | None = None
        self.unsubscribed = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def subscribe(self, topic: str, callback: Callable[[Image, str], None]) -> Callable[[], None]:
        self.topic = topic
        self.callback = callback

        def unsubscribe() -> None:
            self.unsubscribed = True

        return unsubscribe


def test_fixed_topic_pshm_forwards_message_and_topic_and_owns_lifecycle() -> None:
    shared_memory = FakePickleSharedMemory()
    adapter = Px4FixedTopicPshm(shared_memory=shared_memory)
    image = Image.from_numpy(
        np.zeros((1, 1, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="camera_optical"
    )
    received: list[tuple[Image, str]] = []

    adapter.start()
    unsubscribe = adapter.subscribe_all(lambda message, topic: received.append((message, topic)))
    assert shared_memory.callback is not None
    shared_memory.callback(image, "/color_image")
    unsubscribe()
    adapter.stop()

    assert shared_memory.started
    assert shared_memory.topic == "/color_image"
    assert received == [(image, "/color_image")]
    assert shared_memory.unsubscribed
    assert shared_memory.stopped


def test_fixed_topic_pshm_rate_limits_boundary_callback_failures(mocker: MockerFixture) -> None:
    shared_memory = FakePickleSharedMemory()
    logger = mocker.Mock()
    times = iter((100.0, 100.1, 104.9, 105.0))
    adapter = Px4FixedTopicPshm(
        shared_memory=shared_memory,
        clock=lambda: next(times),
        logger=logger,
        error_interval_s=5.0,
    )
    image = Image.from_numpy(
        np.zeros((1, 1, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="camera_optical"
    )

    _ = adapter.subscribe_all(
        lambda _message, _topic: (_ for _ in ()).throw(RuntimeError("broken"))
    )
    assert shared_memory.callback is not None
    shared_memory.callback(image, "/color_image")
    shared_memory.callback(image, "/color_image")
    shared_memory.callback(image, "/color_image")
    shared_memory.callback(image, "/color_image")

    assert logger.exception.call_args_list == [
        call("PX4 pSHM bridge callback failed", topic="/color_image"),
        call("PX4 pSHM bridge callback failed", topic="/color_image"),
    ]
