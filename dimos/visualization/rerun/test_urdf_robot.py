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

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.rerun.urdf_robot import (
    UrdfRobotJointStateRerunFactory,
    UrdfRobotTransformFilter,
)

_TEST_URDF = """\
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="fixed_link"/>
  <link name="moving_link"/>
  <joint name="fixed_joint" type="fixed">
    <parent link="base_link"/>
    <child link="fixed_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
  <joint name="moving_joint" type="revolute">
    <parent link="fixed_link"/>
    <child link="moving_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _write_test_urdf(path: Path) -> Path:
    path.write_text(_TEST_URDF, encoding="utf-8")
    return path


def _transform(child_frame_id: str, x: float = 0.0) -> Transform:
    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id=child_frame_id,
        ts=1.0,
    )


def test_urdf_transform_filter_drops_robot_links_and_unchanged_scene_tf(
    tmp_path: Path,
) -> None:
    transform_filter = UrdfRobotTransformFilter(
        urdf_path=_write_test_urdf(tmp_path / "robot.urdf"),
        child_frame_prefix="pimsim/g1",
    )
    initial = TFMessage(
        _transform("pimsim/g1/base_link"),
        _transform("pimsim/g1/moving_link"),
        _transform("pimsim/g1/cabinet_handle"),
        _transform("lidar"),
    )

    first = transform_filter(initial)
    repeated = transform_filter(initial)
    moved = transform_filter(TFMessage(_transform("pimsim/g1/cabinet_handle", x=0.2)))
    transform_filter.reset()
    after_reset = transform_filter(initial)

    assert [item.child_frame_id for item in first] == ["pimsim/g1/cabinet_handle", "lidar"]
    assert len(repeated) == 0
    assert [item.child_frame_id for item in moved] == ["pimsim/g1/cabinet_handle"]
    assert [item.child_frame_id for item in after_reset] == [
        "pimsim/g1/cabinet_handle",
        "lidar",
    ]


def test_urdf_joint_factory_logs_fixed_joints_once_per_recording(tmp_path: Path) -> None:
    factory = UrdfRobotJointStateRerunFactory(
        urdf_path=_write_test_urdf(tmp_path / "robot.urdf"),
        root_path="world/robot",
        joint_name_mapper=str,
    )
    state = JointState(name=["moving_joint"], position=[0.25])

    first = factory(state)
    second = factory(state)
    numerical_jitter = factory(JointState(name=["moving_joint"], position=[0.25000001]))
    meaningful_motion = factory(JointState(name=["moving_joint"], position=[0.3]))
    factory.reset()
    after_reset = factory(state)

    assert len(first) == 2
    assert second == []
    assert numerical_jitter == []
    assert len(meaningful_motion) == 1
    assert meaningful_motion[0][0].endswith("/moving_link")
    assert len(after_reset) == 2
