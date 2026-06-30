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

import xml.etree.ElementTree as ET

from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake


def test_prepare_urdf_can_strip_fixed_world_joint_for_base_pose_placement(
    tmp_path,
) -> None:
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        """
<robot name="arm">
  <link name="world" />
  <link name="link_base" />
  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link="link_base" />
    <origin xyz="0 0.5 0" rpy="0 0 0" />
  </joint>
</robot>
""".strip()
    )

    prepared_path = prepare_urdf_for_drake(
        urdf_path,
        strip_world_joint_child_link="link_base",
    )

    root = ET.parse(prepared_path).getroot()
    assert [joint.attrib["name"] for joint in root.findall("joint")] == []
    assert [link.attrib["name"] for link in root.findall("link")] == ["link_base"]
