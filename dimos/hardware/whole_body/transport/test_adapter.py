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

from dimos_generated.geometry_msgs.msg import Quaternion, Vector3
from dimos_generated.sensor_msgs.msg import Imu, JointState
import pytest

from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState
from dimos.hardware.whole_body.transport.adapter import TransportWholeBodyAdapter


def test_unconnected_adapter_does_not_send_commands():
    adapter = TransportWholeBodyAdapter(dof=2)
    assert not adapter.write_motor_commands([MotorCommand(), MotorCommand()])
    assert not adapter.has_motor_states()
    assert adapter.read_motor_states() == [MotorState(), MotorState()]


@pytest.mark.parametrize("field", ["position", "velocity", "effort"])
def test_short_feedback_preserves_last_complete_snapshot(field):
    adapter = TransportWholeBodyAdapter(dof=2)
    message = JointState(position=[1.0, 2.0], velocity=[3.0, 4.0], effort=[5.0, 6.0])
    adapter._on_motor_states(message)
    setattr(message, field, [0.0])
    adapter._on_motor_states(message)
    assert adapter.read_motor_states() == [
        MotorState(q=1.0, dq=3.0, tau=5.0),
        MotorState(q=2.0, dq=4.0, tau=6.0),
    ]


def test_generated_imu_converts_xyzw_to_hardware_wxyz():
    adapter = TransportWholeBodyAdapter(dof=2)
    adapter._on_imu(
        Imu(
            orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.4),
            angular_velocity=Vector3(x=1.0, y=2.0, z=3.0),
            linear_acceleration=Vector3(x=4.0, y=5.0, z=6.0),
        )
    )
    assert adapter.read_imu() == IMUState(
        quaternion=(0.4, 0.1, 0.2, 0.3), gyroscope=(1.0, 2.0, 3.0), accelerometer=(4.0, 5.0, 6.0)
    )
