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

from pathlib import Path

from dimos.experimental.robot.bosdyn.spot.utils import camera_mount_transforms


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
    assert transforms[0].frame_id == "body"
    assert transforms[0].child_frame_id == "camera_optical"
    assert transforms[0].translation.to_list() == [1.0, 2.0, 0.0]
