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

from collections.abc import Iterator
import threading

from dimos_generated.sensor_msgs.msg import Image
import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.transport import LCMTransport
from dimos.msgs.image import image_from_array, image_view


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ObserveSkill]:
    # get_next routes through backpressure(), whose shared pool threads live for
    # the whole process; use a dedicated scheduler so the thread-leak detector
    # stays clean (same pattern as dimos/utils/test_reactive.py).
    scheduler = ThreadPoolScheduler(max_workers=2)
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    m = ObserveSkill()
    m.color_image.transport = LCMTransport("/test_observe/color_image", Image)
    try:
        yield m
    finally:
        m.stop()
        scheduler.executor.shutdown(wait=True)


def test_observe_returns_published_frame(module: ObserveSkill) -> None:
    frame = image_from_array(np.zeros((8, 8, 3), dtype=np.uint8), encoding="rgb8")
    stop = threading.Event()

    # get_next subscribes lazily, so keep publishing until observe picks a frame up.
    def pump() -> None:
        while not stop.is_set():
            module.color_image.transport.publish(frame)
            stop.wait(0.05)

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()
    try:
        result = module.observe()
    finally:
        stop.set()
        thread.join()

    assert isinstance(result, Image)
    assert image_view(result).shape[:2] == (8, 8)


def test_observe_without_frames_raises(
    module: ObserveSkill, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(ObserveSkill, "_frame_timeout", 0.2)
    with pytest.raises(TimeoutError, match="No camera frame received within 0.2 seconds"):
        module.observe()
