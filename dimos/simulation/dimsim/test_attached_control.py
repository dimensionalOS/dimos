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

import pytest

from dimos.benchmark.dimsim.models import Pose2
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.simulation.dimsim.attached_control import AttachedDimSimControl


class _FakeTransport:
    def __init__(self, topic: str, _data_type: type[Any]) -> None:
        self.topic = topic
        self.callback: Callable[[Any], None] | None = None
        self.published: list[Any] = []
        self.started = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def subscribe(self, callback: Callable[[Any], None]) -> Callable[[], None]:
        self.callback = callback
        return lambda: None

    def publish(self, value: Any) -> None:
        self.published.append(value)

    def emit(self, value: Any) -> None:
        assert self.callback is not None
        self.callback(value)


class _FakeScene:
    def __init__(self, **_kwargs: Any) -> None:
        self.started = False
        self.positions: list[tuple[float, float, float]] = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def get_agent_position(self) -> dict[str, float]:
        return {"x": 0.0, "y": 0.37, "z": 0.0}

    def set_agent_position(self, x: float, y: float, z: float) -> None:
        self.positions.append((x, y, z))


def _odom(ts: float, *, sim_x: float, sim_z: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        frame_id="world",
        position=Vector3(sim_z, sim_x, 0.37),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def test_attached_control_propagates_scene_timeout(mocker) -> None:
    scene_client = mocker.patch(
        "dimos.simulation.dimsim.attached_control.SceneClient",
        return_value=_FakeScene(),
    )

    AttachedDimSimControl(
        host="dimsim.internal",
        port=8090,
        scene_timeout_s=120.0,
    )

    scene_client.assert_called_once_with(
        host="dimsim.internal",
        port=8090,
        timeout=120.0,
    )


def test_attached_control_resets_via_zero_cmd_teleport_and_fresh_odom(
    monkeypatch,
) -> None:
    transports: dict[str, _FakeTransport] = {}
    scene = _FakeScene()

    monkeypatch.setattr(
        "dimos.simulation.dimsim.attached_control.LCMTransport",
        lambda topic, data_type: transports.setdefault(topic, _FakeTransport(topic, data_type)),
    )
    monkeypatch.setattr(
        "dimos.simulation.dimsim.attached_control.SceneClient",
        lambda **_kwargs: scene,
    )
    monkeypatch.setattr("dimos.simulation.dimsim.attached_control._SETTLE_TIME_S", 0.0)
    control = AttachedDimSimControl(host="localhost", port=8090)
    control.start()
    try:
        control.clear_motion()
        control.settle_motion()
        control.teleport(Pose2(x_m=2.0, z_m=3.0, yaw_rad=0.0))
        transports["/odom"].emit(_odom(10.0, sim_x=2.0, sim_z=3.0))
        transports["/odom"].emit(_odom(10.1, sim_x=2.0, sim_z=3.0))

        sample = control.wait_body_sample(0.1)

        assert transports["/cmd_vel"].published
        assert all(command.is_zero() for command in transports["/cmd_vel"].published)
        assert scene.positions == [(2.0, 0.37, 3.0)]
        assert sample.pose == Pose2(x_m=2.0, z_m=3.0, yaw_rad=0.0)
        assert sample.linear_speed_m_s == 0.0
        assert sample.angular_speed_rad_s == 0.0
    finally:
        control.stop()


def test_attached_control_derives_surface_plane_and_velocity_from_odom(
    monkeypatch,
) -> None:
    transports: dict[str, _FakeTransport] = {}
    monkeypatch.setattr(
        "dimos.simulation.dimsim.attached_control.LCMTransport",
        lambda topic, data_type: transports.setdefault(topic, _FakeTransport(topic, data_type)),
    )
    monkeypatch.setattr(
        "dimos.simulation.dimsim.attached_control.SceneClient",
        _FakeScene,
    )
    control = AttachedDimSimControl(host="localhost", port=8090)
    control.start()
    try:
        control.teleport(Pose2(x_m=2.0, z_m=3.0, yaw_rad=0.0))
        transports["/odom"].emit(_odom(10.0, sim_x=2.0, sim_z=3.0))
        transports["/odom"].emit(_odom(10.5, sim_x=2.3, sim_z=3.4, yaw=0.2))

        sample = control.wait_body_sample(0.1)

        assert sample.pose.x_m == pytest.approx(2.3)
        assert sample.pose.z_m == pytest.approx(3.4)
        assert sample.linear_speed_m_s == pytest.approx(1.0)
        assert sample.angular_speed_rad_s == pytest.approx(0.4)
    finally:
        control.stop()


def test_attached_control_times_out_without_two_post_teleport_samples(
    monkeypatch,
) -> None:
    transports: dict[str, _FakeTransport] = {}
    monkeypatch.setattr(
        "dimos.simulation.dimsim.attached_control.LCMTransport",
        lambda topic, data_type: transports.setdefault(topic, _FakeTransport(topic, data_type)),
    )
    monkeypatch.setattr(
        "dimos.simulation.dimsim.attached_control.SceneClient",
        _FakeScene,
    )
    control = AttachedDimSimControl(host="localhost", port=8090)
    control.start()
    try:
        control.teleport(Pose2(x_m=2.0, z_m=3.0, yaw_rad=0.0))
        transports["/odom"].emit(_odom(10.0, sim_x=2.0, sim_z=3.0))

        with pytest.raises(TimeoutError, match="post-reset odometry"):
            control.wait_body_sample(0.001)
    finally:
        control.stop()
