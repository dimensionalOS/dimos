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
import pickle
import xml.etree.ElementTree as ET

import pytest
from pytest_mock import MockerFixture

import dimos.robot.assets.model as robot_model


def test_model_load_is_lazy_and_memoized(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    xacro = tmp_path / "robot.xacro"
    xacro.write_text("<robot/>")
    expand_xacro = mocker.patch.object(
        robot_model,
        "expand_xacro",
        return_value="<robot name='expanded'><link name='base'/></robot>",
    )

    model = robot_model.RobotModel.from_file(xacro, xacro_args={"dof": "6"})

    expand_xacro.assert_not_called()
    first = model.load()
    second = model.load()

    assert first is second
    assert first.xml == "<robot name='expanded'><link name='base'/></robot>"
    expand_xacro.assert_called_once_with(xacro.resolve(), {}, {"dof": "6"})


def test_model_load_resolves_package_uris_without_writing_a_derived_urdf(
    tmp_path: Path,
) -> None:
    package = tmp_path / "description"
    mesh = package / "meshes" / "link.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("solid link")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        '<robot name="r"><link name="base"><visual><geometry>'
        '<mesh filename="package://description/meshes/link.stl"/>'
        "</geometry></visual></link></robot>"
    )

    loaded = robot_model.RobotModel.from_file(
        urdf,
        package_paths={"description": package},
    ).load()

    assert "package://" not in loaded.xml
    assert str(mesh) in loaded.xml
    assert set(tmp_path.iterdir()) == {package, urdf}


def test_loaded_model_exposes_cached_urdf_topology(tmp_path: Path) -> None:
    loaded = robot_model.LoadedRobotModel(
        """
        <robot name="test">
          <link name="base"/>
          <link name="tool"/>
          <joint name="tool_joint" type="fixed">
            <origin xyz="0.1 0.2 0.3" rpy="0.0 0.0 1.57"/>
            <parent link="base"/>
            <child link="tool"/>
          </joint>
        </robot>
        """,
        tmp_path / "robot.urdf",
        {},
    )

    joints = loaded.joints

    assert joints is loaded.joints
    assert joints == (
        robot_model.JointDescription(
            name="tool_joint",
            type="fixed",
            parent_link="base",
            child_link="tool",
            origin_xyz=(0.1, 0.2, 0.3),
            origin_rpy=(0.0, 0.0, 1.57),
        ),
    )
    assert loaded.root_link == "base"
    assert loaded.get_joint("tool_joint") == joints[0]
    assert loaded.get_joint("missing") is None


def test_with_fixed_frame_builds_ordered_model_chain_and_preserves_extensions(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'/><gazebo reference='base'/>"
        "<transmission name='drive'/><ros2_control name='control'/></robot>"
    )
    model = (
        robot_model.RobotModel.from_file(urdf)
        .with_fixed_frame("tool", "base", xyz=(0.1, 0.2, 0.3))
        .with_fixed_frame("camera", "tool", rpy=(0.0, 0.0, 1.57))
    )

    root = ET.fromstring(model.load().xml)

    assert [link.get("name") for link in root.findall("link")] == [
        "base",
        "tool",
        "camera",
    ]
    joints = root.findall("joint")
    assert [joint.get("name") for joint in joints] == ["tool_joint", "camera_joint"]
    assert joints[0].find("origin").attrib == {"xyz": "0.1 0.2 0.3", "rpy": "0.0 0.0 0.0"}
    assert joints[1].find("parent").attrib == {"link": "tool"}
    assert [element.tag for element in root if element.tag not in {"link", "joint"}] == [
        "gazebo",
        "transmission",
        "ros2_control",
    ]


@pytest.mark.parametrize(
    ("name", "parent", "message"),
    [
        ("base", "base", "link already exists"),
        ("tool", "missing", "unknown parent link"),
    ],
)
def test_with_fixed_frame_rejects_invalid_model(
    tmp_path: Path,
    name: str,
    parent: str,
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")
    model = robot_model.RobotModel.from_file(urdf).with_fixed_frame(name, parent)

    with pytest.raises(ValueError, match=message):
        model.load()


def test_pickle_keeps_model_edits_but_not_materialized_xml(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='before'><link name='base'/></robot>")
    model = robot_model.RobotModel.from_file(urdf).with_fixed_frame("tool", "base")
    assert ET.fromstring(model.load().xml).get("name") == "before"

    restored = pickle.loads(pickle.dumps(model))
    urdf.write_text("<robot name='after'><link name='base'/></robot>")
    root = ET.fromstring(restored.load().xml)

    assert root.get("name") == "after"
    assert root.find("link[@name='tool']") is not None


def test_model_rejects_non_urdf_source(tmp_path: Path) -> None:
    mjcf = tmp_path / "robot.xml"

    with pytest.raises(ValueError, match="must reference a .urdf or .xacro"):
        robot_model.RobotModel.from_file(mjcf)
