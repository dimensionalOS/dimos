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

from types import SimpleNamespace

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.nav_3d.mls_planner.odom_body_frame import (
    OdomBodyFrame,
    OdomBodyFrameConfig,
)


def _level(mount_rotation, orientation):
    """Run one odometry message through the handler and return the output.

    Builds the module without its transport so no runtime threads spawn.
    """
    module = object.__new__(OdomBodyFrame)
    module.config = OdomBodyFrameConfig(
        mount_rotation=list(mount_rotation), body_frame_id="base_link"
    )
    module._mount_inv = Quaternion(*module.config.mount_rotation).inverse()
    captured = []
    module.body_odometry = SimpleNamespace(publish=captured.append)
    module._on_odometry(
        Odometry(
            ts=1.0,
            frame_id="odom",
            child_frame_id="mid360_link",
            pose=Pose(Vector3(1.0, 2.0, 3.0), orientation),
        )
    )
    return captured[0]


def test_composes_out_the_mount_pitch():
    # A level body reads its own mount tilt as the sensor's world orientation, so
    # composing the mount out returns identity.
    mount = Quaternion.from_euler(Vector3(0.0, 0.3, 0.0))
    out = _level(mount.to_tuple(), mount)
    assert out.orientation.angle_to(Quaternion(0.0, 0.0, 0.0, 1.0)) < 1e-5


def test_preserves_body_yaw_under_mount_tilt():
    # A body yawed by a known angle keeps that yaw after the mount is composed out.
    mount = Quaternion.from_euler(Vector3(0.0, 0.3, 0.0))
    body = Quaternion.from_euler(Vector3(0.0, 0.0, 0.7))
    out = _level(mount.to_tuple(), body * mount)
    assert out.orientation.angle_to(body) < 1e-5


def test_relabels_child_frame_and_passes_position_through():
    out = _level([0.0, 0.0, 0.0, 1.0], Quaternion(0.0, 0.0, 0.0, 1.0))
    assert out.child_frame_id == "base_link"
    assert out.position.to_tuple() == (1.0, 2.0, 3.0)
