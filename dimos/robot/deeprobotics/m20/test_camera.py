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

from threading import Barrier, Lock, Thread
import time
from typing import Any

from pytest_mock import MockerFixture

from dimos.core.module import Module
from dimos.robot.deeprobotics.m20.camera import M20CameraRelay


def test_overlapping_starts_open_each_camera_once(mocker: MockerFixture) -> None:
    relay = M20CameraRelay()
    opened: list[str] = []
    opened_lock = Lock()

    def fake_relay(url: str, output: Any, camera_name: str) -> None:
        with opened_lock:
            opened.append(camera_name)
        relay._stop_event.wait()

    mocker.patch.object(relay, "_relay", side_effect=fake_relay)
    mocker.patch.object(relay, "_publish_camera_metadata", side_effect=relay._stop_event.wait)
    module_start = Module.start
    # widen the gap between the already-running check and the launch, so unguarded starts overlap every time
    mocker.patch.object(Module, "start", lambda self: (time.sleep(0.05), module_start(self))[-1])
    barrier = Barrier(8)

    def start() -> None:
        barrier.wait()
        relay.start()

    starters = [Thread(target=start) for _ in range(8)]
    for starter in starters:
        starter.start()
    for starter in starters:
        starter.join()
    relay.stop()

    assert sorted(opened) == ["m20_front_camera", "m20_rear_camera"]
