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

from pathlib import Path

import numpy as np
import pytest

from dimos.sim2.connections.whole_body import WholeBodyConnection
from dimos.sim2.ipc.channel import ChannelFrame, FrameMetadata
from dimos.sim2.sensors.spec import Imu
from dimos.sim2.spec import ControlInterface, Joint, RobotConfig


@pytest.fixture
def connection():
    definition = RobotConfig(
        model=Path("unused.xml"),
        root_body="base",
        control=ControlInterface.WHOLE_BODY,
        joints=(Joint("joint", "joint", "motor", home=0.25),),
        sensors=(Imu("imu", "imu"),),
    )
    module = WholeBodyConnection(definition=definition, address="test/robot", robot_id="robot")
    yield module
    module.stop()


def test_transient_contention_skips_sample_and_resumes_publication(connection, mocker):
    desc = connection._device.definition
    values = {
        "position": np.array([j.home for j in desc.joints]),
        "velocity": np.zeros(1),
        "effort": np.zeros(1),
        "wall_time": np.array([3.0]),
        "imu_quaternion": np.array([1.0, 0, 0, 0]),
        "imu_gyroscope": np.zeros(3),
        "imu_accelerometer": np.array([0.0, 0, 9.81]),
        "root_position": np.array([0.0, 0, 0.3]),
        "root_quaternion": np.array([1.0, 0, 0, 0]),
    }
    sample = mocker.patch.object(
        connection._device,
        "sample",
        side_effect=[
            BlockingIOError("writer busy"),
            ChannelFrame(FrameMetadata(1, 1, 1, 0, 1), values),
        ],
    )
    wait = mocker.patch.object(connection._stop, "wait", return_value=False)
    joints = mocker.patch.object(connection.motor_states, "publish")
    imu = mocker.patch.object(connection.imu, "publish")
    mocker.patch.object(connection.odom, "publish")
    mocker.patch.object(connection.tf, "publish", side_effect=lambda _: connection._stop.set())
    error = mocker.patch("dimos.sim2.connections.whole_body.logger.exception")

    connection._publish()

    assert sample.call_count == 2
    wait.assert_any_call(1.0 / connection.config.rate_hz)
    assert joints.call_count == imu.call_count == 1
    assert joints.call_args.args[0].ts == imu.call_args.args[0].ts == 3.0
    assert joints.call_args.args[0].position == values["position"].tolist()
    error.assert_not_called()


def test_non_contention_error_is_not_suppressed(connection, mocker):
    sample = mocker.patch.object(connection._device, "sample", side_effect=RuntimeError("closed"))
    publish = mocker.patch.object(connection.motor_states, "publish")
    error = mocker.patch("dimos.sim2.connections.whole_body.logger.exception")

    connection._publish()

    sample.assert_called_once_with()
    publish.assert_not_called()
    error.assert_called_once_with("sim2 whole-body connection stopped")
