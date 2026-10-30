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

"""The stock policy consumes the same physical observations on every engine."""

from dataclasses import replace

import mujoco
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.velocity_policy import G1_DRIFT_COMPENSATION, G1VelocityController
from dimos.sim2.physics import BodyState, InertialState, RobotState
from dimos.simulation.mujoco.policy import G1OnnxController


@pytest.fixture
def inference(mocker):
    predictor = mocker.patch(
        "dimos.robot.unitree.g1.velocity_policy.OnnxPolicy", autospec=True
    ).return_value
    predictor.predict.return_value = np.ones(29, dtype=np.float32)
    return predictor


@pytest.fixture
def state():
    return RobotState(
        joint_names=tuple(f"g1/joint_{i}" for i in range(29)),
        position=(0.0,) * 29,
        velocity=(0.0,) * 29,
        effort=(0.0,) * 29,
        base=BodyState((0, 0, 1), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0)),
        imu=InertialState((0, 0, 0, 1), (4, 5, 6), (1, 2, 3), (0, 0, 9.81)),
    )


def test_policy_observation_uses_sensor_coordinates_and_xyzw_orientation(inference, state):
    controller = G1VelocityController(0.002, policy_path="unused.onnx", default_angles=(0,) * 29)
    state = replace(
        state,
        position=(0.1,) + (0.0,) * 28,
        velocity=(0.2,) + (0.0,) * 28,
        imu=replace(state.imu, orientation=(0, 2**-0.5, 0, 2**-0.5)),
    )
    command = Twist(linear=(0.2, 0.1, 0), angular=(0, 0, 0.3))

    observation = controller.observation(state, command)

    assert observation[:12] == pytest.approx([1, 2, 3, 4, 5, 6, 1, 0, 0, 0.22, 0.2, 0.21])
    assert observation[12:41] == pytest.approx([0.1] + [0] * 28)
    assert observation[41:70] == pytest.approx([0.2] + [0] * 28)
    assert observation[70:99] == pytest.approx([0] * 29)
    assert observation[99:] == pytest.approx([1, -1, 0, 0], abs=1e-7)


def test_policy_cadence_and_reset_keep_the_original_home(inference, state):
    controller = G1VelocityController(0.002, policy_path="unused.onnx")
    for _ in range(9):
        assert controller.step(state, Twist()).q == [0] * 29
    inference.predict.assert_not_called()

    assert controller.step(state, Twist()).q == [0.5] * 29
    inference.predict.assert_called_once()
    controller.reset()

    moved = replace(state, position=(0.9,) * 29)
    assert controller.step(moved, Twist()).q == [0] * 29
    assert controller.observation(moved, Twist())[12:41] == pytest.approx([0.9] * 29)
    assert controller.observation(moved, Twist())[70:99] == pytest.approx([0] * 29)


def test_policy_requires_its_sensor_feedback(inference, state):
    controller = G1VelocityController(0.002, policy_path="unused.onnx")
    with pytest.raises(ValueError, match="pelvis IMU"):
        controller.step(replace(state, imu=None), Twist())


@pytest.fixture
def native_robot():
    bodies = "".join(
        f'<body name="link{i}" pos="0 0 .02"><joint name="joint{i}" axis="0 1 0"/><geom size=".01" mass=".1"/></body>'
        for i in range(29)
    )
    actuators = "".join(f'<motor joint="joint{i}"/>' for i in range(29))
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="pelvis" pos="0 0 1">'
        '<freejoint/><geom size=".1" mass="1"/><site name="imu_in_pelvis" pos=".1 0 0"/>'
        + bodies
        + "</body></worldbody><actuator>"
        + actuators
        + "</actuator><sensor>"
        '<velocimeter name="local_linvel_pelvis" site="imu_in_pelvis"/>'
        '<gyro name="gyro_pelvis" site="imu_in_pelvis"/>'
        '<accelerometer name="accelerometer_pelvis" site="imu_in_pelvis"/>'
        "</sensor></mujoco>"
    )
    data = mujoco.MjData(model)
    data.qvel[:6] = [1, 2, 3, 4, 5, 6]
    data.qpos[7] = 0.1
    data.qvel[6] = 0.2
    mujoco.mj_forward(model, data)
    return model, data


def test_legacy_adapter_keeps_site_velocity_and_policy_output(inference, native_robot, mocker):
    model, data = native_robot
    input_device = mocker.Mock()
    input_device.get_command.return_value = np.array([0.2, 0.1, 0.3])
    controller = G1OnnxController(
        "unused.onnx",
        np.zeros(29),
        0.02,
        1,
        0.5,
        input_device,
        drift_compensation=G1_DRIFT_COMPENSATION,
    )

    observation = controller.get_obs(model, data)
    controller.get_control(model, data)

    assert observation[:12] == pytest.approx([1, 2.6, 2.5, 4, 5, 6, 0, 0, -1, 0.22, 0.2, 0.21])
    assert observation[12:41] == pytest.approx([0.1] + [0] * 28)
    assert observation[41:70] == pytest.approx([0.2] + [0] * 28)
    assert data.ctrl == pytest.approx([0.5] * 29)
