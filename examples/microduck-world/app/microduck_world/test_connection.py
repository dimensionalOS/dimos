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

from unittest.mock import Mock

import numpy as np
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from microduck_world.connection import SimRobotConnection
from microduck_world.robot_io import RobotCommand, RobotState, RobotVision


def test_previous_visitor_sensor_packets_cannot_enter_new_stack(module_factory, monkeypatch):
    connection = module_factory(SimRobotConnection, generation="new")
    published = {name: Mock() for name in connection.outputs}
    for name, callback in published.items():
        monkeypatch.setattr(getattr(connection, name), "publish", callback)
    image = Image(data=np.zeros((2, 2, 3), dtype=np.uint8), format=ImageFormat.RGB)
    connection.receive_state(RobotState("old", PoseStamped(), JointState(), "{}"))
    connection.receive_vision(
        RobotVision(
            "old",
            image,
            image,
            CameraInfo(),
            PoseStamped(),
            TFMessage(),
            np.empty((0, 3), dtype=np.float32),
        )
    )
    assert not any(callback.called for callback in published.values())
    state = RobotState("new", PoseStamped(), JointState(), '{"active":"walk"}')
    connection.receive_state(state)
    published["odom"].assert_called_once_with(state.odom)
    published["policy_state"].assert_called_once_with(state.policy)
    connection.drive(Twist())
    assert published["hardware_command"].call_args.args[0].generation == "new"


def test_respawn_command_is_scoped_to_current_visitor(module_factory, monkeypatch):
    connection = module_factory(SimRobotConnection, generation="current-visitor")
    publish = Mock()
    monkeypatch.setattr(connection.hardware_command, "publish", publish)
    connection.respawn(False)
    publish.assert_not_called()
    connection.respawn(True)
    publish.assert_called_once_with(RobotCommand("current-visitor", "respawn", ""))
