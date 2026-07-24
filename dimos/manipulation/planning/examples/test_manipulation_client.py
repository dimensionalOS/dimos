# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from unittest.mock import MagicMock

from pytest_mock import MockerFixture

from dimos.manipulation.planning.examples import manipulation_client
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_update_box_calls_complete_obstacle_rpc(mocker: MockerFixture) -> None:
    client = MagicMock()
    client.update_obstacle.return_value = True
    mocker.patch.object(manipulation_client, "_client", client)

    result = manipulation_client.update_box(
        "box",
        1.0,
        2.0,
        3.0,
        w=0.4,
        h=0.5,
        d=0.6,
        color=[0.1, 0.2, 0.3, 0.9],
    )

    assert result is True
    name, pose, shape, dimensions, mesh_path, color = client.update_obstacle.call_args.args
    assert name == "box"
    assert pose.position == Vector3(1.0, 2.0, 3.0)
    assert shape == "box"
    assert dimensions == [0.4, 0.5, 0.6]
    assert mesh_path is None
    assert color == [0.1, 0.2, 0.3, 0.9]


def test_update_obstacle_exposes_generic_mesh_replacement(mocker: MockerFixture) -> None:
    client = MagicMock()
    client.update_obstacle.return_value = True
    mocker.patch.object(manipulation_client, "_client", client)
    pose = Pose(
        position=Vector3(0.1, 0.2, 0.3),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    result = manipulation_client.update_obstacle(
        "mesh",
        pose,
        "mesh",
        mesh_path="replacement.obj",
    )

    assert result is True
    client.update_obstacle.assert_called_once_with(
        "mesh",
        pose,
        "mesh",
        None,
        "replacement.obj",
        None,
    )


def test_update_pose_calls_pose_only_rpc(mocker: MockerFixture) -> None:
    client = MagicMock()
    client.update_obstacle_pose.return_value = True
    mocker.patch.object(manipulation_client, "_client", client)

    result = manipulation_client.update_pose("box", 4.0, 5.0, 6.0)

    assert result is True
    name, pose = client.update_obstacle_pose.call_args.args
    assert name == "box"
    assert pose.position == Vector3(4.0, 5.0, 6.0)
    assert pose.orientation == Quaternion(0.0, 0.0, 0.0, 1.0)
