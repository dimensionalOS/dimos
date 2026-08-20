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

"""DimSimConnection forwards LCM /color_image frames onto video_stream()."""

from collections.abc import Callable, Iterator
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import GlobalConfig
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.unitree.dimsim_connection import DimSimConnection
from dimos.simulation.dimsim.dimsim_process import DimSimProcess


class _Bus:
    def __init__(self) -> None:
        self._callback: Callable[[Any], None] | None = None

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def subscribe(self, callback: Callable[[Any], None]) -> Callable[[], None]:
        self._callback = callback

        def unsubscribe() -> None:
            self._callback = None

        return unsubscribe

    def emit(self, msg: Any) -> None:
        if self._callback is not None:
            self._callback(msg)


def _rgb_frame(frame_id: str = "camera_optical") -> Image:
    return Image.from_numpy(
        np.zeros((4, 4, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id=frame_id,
        ts=1.0,
    )


@pytest.fixture
def dimsim_connection(mocker: MockerFixture) -> Iterator[tuple[DimSimConnection, dict[str, _Bus]]]:
    buses = {
        "/odom": _Bus(),
        "/tf": _Bus(),
        "/color_image": _Bus(),
    }

    def make_transport(name: str, msg_type: type | None = None, **kwargs: object) -> _Bus:
        return buses[name]

    mocker.patch(
        "dimos.robot.unitree.dimsim_connection.make_transport",
        side_effect=make_transport,
    )
    mocker.patch.object(DimSimProcess, "start")
    mocker.patch.object(DimSimProcess, "stop")
    conn = DimSimConnection(GlobalConfig())
    try:
        yield conn, buses
    finally:
        conn.stop()


def test_video_stream_forwards_color_image_from_bus(
    dimsim_connection: tuple[DimSimConnection, dict[str, _Bus]],
) -> None:
    conn, buses = dimsim_connection
    received: list[Image] = []
    conn.start()
    conn.video_stream().subscribe(received.append)

    frame = _rgb_frame()
    buses["/color_image"].emit(frame)

    assert received == [frame]


def test_stop_unsubscribes_from_color_image_bus(
    dimsim_connection: tuple[DimSimConnection, dict[str, _Bus]],
) -> None:
    conn, buses = dimsim_connection
    received: list[Image] = []
    conn.start()
    conn.video_stream().subscribe(received.append)
    conn.stop()

    buses["/color_image"].emit(_rgb_frame())

    assert received == []
