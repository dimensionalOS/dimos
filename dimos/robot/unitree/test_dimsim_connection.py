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
from typing import Any

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.dimsim_connection import DimSimConnection


class _FakeProcess:
    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class _FakeTransport:
    def __init__(self, topic: str, data_type: type[Any]) -> None:
        self.topic = topic
        self.data_type = data_type
        self.callback: Callable[[Any], None] | None = None
        self.published: list[Any] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def subscribe(self, callback: Callable[[Any], None]) -> Callable[[], None]:
        self.callback = callback
        return lambda: None

    def publish(self, value: Any) -> None:
        self.published.append(value)

    def emit(self, value: Any) -> None:
        assert self.callback is not None
        self.callback(value)


def test_dimsim_connection_forwards_sensor_samples_and_motion_commands(
    monkeypatch,
    mocker,
) -> None:
    transports: dict[str, _FakeTransport] = {}

    def make_transport(topic: str, data_type: type[Any]) -> _FakeTransport:
        transport = _FakeTransport(topic, data_type)
        transports[topic] = transport
        return transport

    monkeypatch.setattr(
        "dimos.robot.unitree.dimsim_connection.DimSimProcess",
        lambda _config: _FakeProcess(),
    )
    monkeypatch.setattr(
        "dimos.robot.unitree.dimsim_connection.make_transport",
        make_transport,
    )
    connection = DimSimConnection(GlobalConfig())
    lidar_samples: list[Any] = []
    video_samples: list[Any] = []
    odom_samples: list[Any] = []
    connection.lidar_stream().subscribe(lidar_samples.append)
    connection.video_stream().subscribe(video_samples.append)
    connection.odom_stream().subscribe(odom_samples.append)
    try:
        connection.start()
        lidar = mocker.Mock(ts=1.0)
        video = mocker.Mock(ts=2.0)
        odom = mocker.Mock(ts=3.0)
        transforms = [mocker.Mock()]
        monkeypatch.setattr(
            "dimos.robot.unitree.dimsim_connection._odom_to_tf",
            lambda _odom: transforms,
        )
        transports["/lidar"].emit(lidar)
        transports["/color_image"].emit(video)
        transports["/odom"].emit(odom)
        command = Twist.zero()

        assert connection.move(command)
        assert lidar_samples == [lidar]
        assert video_samples == [video]
        assert odom_samples == [odom]
        assert transports["/cmd_vel"].published == [command]
        assert transports["/tf"].published[0].transforms == transforms
    finally:
        connection.stop()


def test_dimsim_connection_rejects_duplicate_sensor_timestamps(
    monkeypatch,
    mocker,
) -> None:
    transports: dict[str, _FakeTransport] = {}

    def make_transport(topic: str, data_type: type[Any]) -> _FakeTransport:
        transport = _FakeTransport(topic, data_type)
        transports[topic] = transport
        return transport

    monkeypatch.setattr(
        "dimos.robot.unitree.dimsim_connection.DimSimProcess",
        lambda _config: _FakeProcess(),
    )
    monkeypatch.setattr(
        "dimos.robot.unitree.dimsim_connection.make_transport",
        make_transport,
    )
    connection = DimSimConnection(GlobalConfig())
    samples: list[Any] = []
    connection.video_stream().subscribe(samples.append)
    try:
        connection.start()
        first = mocker.Mock(ts=1.0)
        duplicate = mocker.Mock(ts=1.0)

        transports["/color_image"].emit(first)
        transports["/color_image"].emit(duplicate)

        assert samples == [first]
    finally:
        connection.stop()
