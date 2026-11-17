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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.booster.k1.control_config import HOME, JOINTS, KD, KP
from dimos.robot.booster.k1.policy_module import K1PolicyModule
from dimos.robot.booster.k1.sim2 import motor_action


@pytest.fixture
def host(mocker):
    module = K1PolicyModule()
    inference = mocker.patch.object(module._policy, "step", return_value=np.asarray(HOME))
    publish = mocker.patch.object(module.motor_command, "publish")
    clock = mocker.patch("dimos.robot.booster.k1.policy_module.time.monotonic", return_value=10)
    yield module, inference, publish, clock
    module.dispose()


@pytest.fixture
def joints():
    return JointState(name=list(JOINTS), position=list(HOME), velocity=[0.0] * 22, ts=1)


async def test_sensor_join_ignores_unmatched_and_repeated_samples(host, joints):
    module, inference, publish, _ = host
    await module.handle_joint_state(joints)
    await module.handle_imu(Imu(ts=0.98))
    inference.assert_not_called()
    await module.handle_imu(Imu(ts=1))
    await module.handle_joint_state(joints)
    await module.handle_imu(Imu(ts=1))
    assert inference.call_count == publish.call_count == 1
    command = publish.call_args.args[0]
    assert command.q == list(HOME)
    assert command.timestamp == 1


async def test_policy_rate_does_not_drift_with_hardware_sample_jitter(host, joints):
    module, _, publish, _ = host
    for tick in range(501):
        joints.ts = 1 + tick * 0.002 + (0.0001 if tick % 2 else 0)
        await module.handle_joint_state(joints)
        await module.handle_imu(Imu(ts=joints.ts))
    assert publish.call_count == 51
    assert publish.call_args.args[0].timestamp == 2


def test_reordered_joints_and_expiring_velocity(host, joints):
    module, inference, _, clock = host
    joints.name.reverse()
    joints.position.reverse()
    module.move(Twist(linear=Vector3(0.2, -0.1, 0), angular=Vector3(0, 0, 0.3)), duration=0.3)
    module.compute(joints, Imu(ts=1))
    np.testing.assert_allclose(inference.call_args.args[0], HOME)
    np.testing.assert_allclose(inference.call_args.args[-1], [0.2, -0.1, 0.3])
    clock.return_value = 10.4
    module.compute(joints, Imu(ts=1))
    np.testing.assert_array_equal(inference.call_args.args[-1], [0, 0, 0])


def test_unknown_attitude_does_not_reach_actor(host, joints):
    module, inference, _, _ = host
    with pytest.raises(ValueError, match="attitude estimate"):
        module.compute(joints, Imu(ts=1, orientation_covariance=[-1] + [0] * 8))
    inference.assert_not_called()


def test_sim_action_preserves_joint_order_gains_and_acquisition_time():
    action = motor_action(
        MotorCommandArray(q=list(HOME), kp=list(KP), kd=list(KD), timestamp=1.02), "episode"
    )
    assert action.header.clock_id == "episode"
    assert action.header.time_ns == action.based_on_time_ns == action.apply_at_ns == 1_020_000_000
    assert action.valid_until_ns == 1_120_000_000
    rows = {row.interface_name: row for row in action.joint_commands}
    assert set(rows) == {"position", "velocity", "kp", "kd", "effort"}
    assert rows["position"].joint_names == list(JOINTS)
    assert rows["position"].values == list(HOME)
    assert rows["kp"].values == list(KP)
    assert rows["kd"].values == list(KD)
