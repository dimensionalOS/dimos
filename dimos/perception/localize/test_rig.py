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

"""Canonical pose frames and camera extrinsics used by localization."""

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.memory.type.observation import Observation
from dimos.msgs.geometry import transform_matrix
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.perception.localize.rig import Rig
from dimos.robot.unitree.go2.camera_calibration import front_camera_calibration


@pytest.mark.parametrize("query, expected_x", [(1.0, 2.0), (1.5, 3.0)])
def test_pose_interpolation_preserves_world_frame_for_camera_composition(mocker, query, expected_x):
    poses = mocker.Mock()
    poses.at.return_value = [
        Observation(ts=1.0, pose=(2.0, 0.0, 0.0)),
        Observation(ts=2.0, pose=(4.0, 0.0, 0.0)),
    ]
    mount = TransformStamped(
        header=Header(stamp=time_from_seconds(0), frame_id="base_link"),
        child_frame_id="camera_optical",
        transform=Transform(
            translation=Vector3(x=0.5, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    rig = Rig(
        cameras={"camera_optical": front_camera_calibration()},
        color=None,
        world_frame="world",
        poses=poses,
        mounts={"camera_optical": mount},
    )

    pose = rig.pose_at(query)
    camera = rig.world_to_optical(query)

    assert pose.header.frame_id == "world"
    assert pose.pose.position.x == expected_x
    assert to_seconds(pose.header.stamp) == query
    assert camera.header.frame_id == "camera_optical"
    assert camera.child_frame_id == "world"
    np.testing.assert_allclose(
        transform_matrix(camera.transform)[:3, 3], [-expected_x - 0.5, 0.0, 0.0]
    )
    assert to_seconds(camera.header.stamp) == query
