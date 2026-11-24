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

import pytest

from dimos.core.transport import PubSubTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer


@pytest.fixture
def explorer_transport(mocker):
    command = mocker.Mock(spec=PubSubTransport)
    odom = mocker.Mock(spec=PubSubTransport)
    mocker.patch(
        "dimos.simulation.mujoco.direct_cmd_vel_explorer.make_transport",
        side_effect=[command, odom],
    )
    explorer = DirectCmdVelExplorer()
    explorer.start()
    receive = odom.subscribe.call_args.args[0]
    wait = mocker.patch.object(explorer._new_pose, "wait")
    yield explorer, command, receive, wait
    explorer.stop()


@pytest.mark.parametrize("initial_poses", [[], [PoseStamped(position=(0, 0, 0))]])
def test_missing_fresh_odometry_fails_and_stops(explorer_transport, initial_poses):
    explorer, command, receive, wait = explorer_transport
    for pose in initial_poses:
        receive(pose)
    # A second wait is a bug: the first timeout must end exploration.
    wait.side_effect = [False, RuntimeError("Retried an expired odometry wait")]

    with pytest.raises(TimeoutError, match="No fresh /odom"):
        explorer.follow_points([(1, 0)])

    command.broadcast.assert_called_once()
    stop = command.broadcast.call_args.args[1]
    assert stop.linear.x == stop.angular.z == 0


def test_stalled_waypoint_fails_and_stops(explorer_transport, mocker):
    explorer, command, receive, wait = explorer_transport
    poses = iter([PoseStamped(position=(0, 0, 0))] * 3)

    def receive_pose(*, timeout):
        receive(next(poses))
        return True

    wait.side_effect = receive_pose
    mocker.patch(
        "dimos.simulation.mujoco.direct_cmd_vel_explorer.monotonic",
        side_effect=[0.0, 0.0, 60.0],
    )

    with pytest.raises(TimeoutError, match=r"waypoint \(1, 0\).*Last odom.*\(0.0, 0.0\)"):
        explorer.follow_points([(1, 0)])

    commands = [call.args[1] for call in command.broadcast.call_args_list]
    assert commands[0].linear.x == 0.8
    assert commands[-1].linear.x == commands[-1].angular.z == 0


def test_follow_points_reaches_each_waypoint_and_stops(explorer_transport):
    explorer, command, receive, wait = explorer_transport
    poses = iter(PoseStamped(position=(x, 0, 0)) for x in [0, 0, 1, 2])

    def receive_pose(*, timeout):
        receive(next(poses))
        return True

    wait.side_effect = receive_pose

    explorer.follow_points([(1, 0), (2, 0)])

    commands = [call.args[1] for call in command.broadcast.call_args_list]
    assert commands[0].linear.x == 0.8
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    assert sum(cmd.linear.x == cmd.angular.z == 0 for cmd in commands) == 2
