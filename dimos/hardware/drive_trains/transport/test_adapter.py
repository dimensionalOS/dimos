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

import math

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
import pytest

from dimos.hardware.drive_trains.transport.adapter import TransportTwistAdapter


def test_nested_pose_becomes_planar_odometry_without_sharing_state():
    adapter = TransportTwistAdapter()
    assert adapter.read_odometry() is None
    adapter._on_odom(
        PoseStamped(
            pose=Pose(
                position=Point(x=2.0, y=-1.0),
                orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5)),
            )
        )
    )
    odometry = adapter.read_odometry()
    assert odometry == pytest.approx([2.0, -1.0, math.pi / 2])
    odometry[0] = 99.0
    assert adapter.read_odometry()[0] == 2.0


@pytest.mark.parametrize(
    "dof,values,expected",
    [
        (1, [0.1], Twist(linear=Vector3(x=0.1))),
        (2, [0.1, 0.2], Twist(linear=Vector3(x=0.1, y=0.2))),
        (3, [0.1, 0.2, 0.3], Twist(linear=Vector3(x=0.1, y=0.2), angular=Vector3(z=0.3))),
    ],
)
def test_generated_commands_preserve_enable_and_stop_contract(
    monkeypatch, mocker, dof, values, expected
):
    adapter = TransportTwistAdapter(dof=dof)
    transport = mocker.Mock()
    monkeypatch.setattr(adapter, "_cmd_vel_transport", transport)
    assert not adapter.write_velocities(values)
    transport.publish.assert_not_called()
    adapter.write_enable(True)
    assert adapter.write_velocities(values)
    message = transport.publish.call_args.args[0]
    assert Twist.decode(message.encode()) == expected
    assert adapter.read_velocities() == values
    adapter.write_enable(False)
    assert transport.publish.call_args.args[0] == Twist()
    assert adapter.read_velocities() == [0.0] * dof
