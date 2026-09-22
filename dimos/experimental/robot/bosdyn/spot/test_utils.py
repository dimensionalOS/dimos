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
from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.experimental.robot.bosdyn.spot.utils import camera_mount_transforms, roll_optical_frame
from dimos.msgs.geometry import yaw


def test_camera_mount_transforms_uses_loaded_robot_topology(tmp_path: Path) -> None:
    urdf = tmp_path / "spot.urdf"
    urdf.write_text(
        """
        <robot name="spot">
          <link name="base"/>
          <link name="camera_mount"/>
          <link name="camera_optical"/>
          <joint name="mount" type="fixed">
            <origin xyz="1 0 0"/>
            <parent link="base"/>
            <child link="camera_mount"/>
          </joint>
          <joint name="optical" type="fixed">
            <origin xyz="0 2 0"/>
            <parent link="camera_mount"/>
            <child link="camera_optical"/>
          </joint>
        </robot>
        """
    )

    transforms = camera_mount_transforms(urdf, "body", ["camera_optical"])

    assert len(transforms) == 1
    assert transforms[0].header.frame_id == "body"
    assert transforms[0].child_frame_id == "camera_optical"
    translation = transforms[0].transform.translation
    assert (translation.x, translation.y, translation.z) == (1.0, 2.0, 0.0)


@pytest.mark.parametrize("turns", [-1, 0, 1])
def test_optical_roll_preserves_mount_and_exact_stamp(turns):
    edge = TransformStamped(
        header=Header(frame_id="body", stamp=Time(sec=1700000000, nanosec=123456789)),
        child_frame_id="optical",
        transform=Transform(translation=Vector3(x=1, y=2, z=3)),
    )
    rolled = roll_optical_frame(edge, turns)
    assert rolled.header == edge.header
    assert rolled.child_frame_id == edge.child_frame_id
    assert rolled.transform.translation == edge.transform.translation
    assert yaw(rolled.transform.rotation) == pytest.approx(turns * math.pi / 2)
    assert edge.transform.rotation.w == 1
