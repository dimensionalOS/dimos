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

"""URDF mount extraction emits generated transforms and excludes driver-owned frames."""

from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.robot.diy.alfred import mount_tf


def test_mounts_preserve_parent_and_translation_and_skip_imager_frames(tmp_path, monkeypatch):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="demo">
      <joint name="lidar" type="fixed">
        <parent link="base"/><child link="lidar_link"/>
        <origin xyz="1 2 3" rpy="0 0 0"/>
      </joint>
      <joint name="imager" type="fixed">
        <parent link="lidar_link"/><child link="camera_optical_frame"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </joint>
    </robot>""")
    monkeypatch.setattr(mount_tf, "ALFRED_URDF", urdf)

    edges = mount_tf.mount_transforms()
    decoded = TFMessage.decode(TFMessage(transforms=edges).encode())

    assert len(decoded.transforms) == 1
    edge = decoded.transforms[0]
    assert (edge.header.frame_id, edge.child_frame_id) == ("base", "lidar_link")
    translation = edge.transform.translation
    assert (translation.x, translation.y, translation.z) == (1, 2, 3)
    assert edge.transform.rotation.w == 1
