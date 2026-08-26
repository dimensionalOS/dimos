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
from dimos.utils.data import LfsPath


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


def test_model_construction_does_not_materialize_lazy_source(mocker: MockerFixture) -> None:
    source_path = LfsPath("robot_description/model.urdf")
    ensure_downloaded = mocker.patch.object(LfsPath, "_ensure_downloaded")

    model = robot_model.RobotModel.from_file(source_path)

    assert model.source_path is source_path
    ensure_downloaded.assert_not_called()


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


def test_with_joint_position_limits_preserves_model_content(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'/><link name='finger'/>"
        "<joint name='finger_joint' type='prismatic'>"
        "<parent link='base'/><child link='finger'/>"
        "<limit lower='0.018' upper='0.06' effort='10' velocity='2'/>"
        "<mimic joint='driver' multiplier='-1'/></joint>"
        "<gazebo reference='finger'/><transmission name='drive'/>"
        "<ros2_control name='control'/></robot>"
    )

    loaded = (
        robot_model.RobotModel.from_file(urdf)
        .with_joint_position_limits(
            "finger_joint",
            lower=0.0,
            upper=0.06,
        )
        .load()
    )
    root = ET.fromstring(loaded.xml)
    joint = root.find("joint[@name='finger_joint']")

    assert joint is not None
    assert joint.find("limit").attrib == {
        "lower": "0.0",
        "upper": "0.06",
        "effort": "10",
        "velocity": "2",
    }
    assert joint.find("mimic").attrib == {"joint": "driver", "multiplier": "-1"}
    assert [element.tag for element in root if element.tag not in {"link", "joint"}] == [
        "gazebo",
        "transmission",
        "ros2_control",
    ]
    assert "lower='0.018'" in urdf.read_text()


def test_with_fixed_joints_preserves_topology_and_removes_movable_elements(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'/><link name='finger'/>"
        "<joint name='finger_joint' type='revolute'>"
        "<origin xyz='0.1 0.2 0.3'/><parent link='base'/><child link='finger'/>"
        "<axis xyz='0 0 1'/><calibration rising='0'/><dynamics damping='0.1'/>"
        "<limit lower='0' upper='1' effort='10' velocity='2'/>"
        "<mimic joint='driver'/><safety_controller soft_lower_limit='0'/>"
        "</joint><gazebo reference='finger'/><ros2_control name='control'/></robot>"
    )

    loaded = robot_model.RobotModel.from_file(urdf).with_fixed_joints("finger_joint").load()
    root = ET.fromstring(loaded.xml)
    joint = root.find("joint[@name='finger_joint']")

    assert joint is not None
    assert joint.get("type") == "fixed"
    assert [element.tag for element in joint] == ["origin", "parent", "child"]
    assert joint.find("origin").attrib == {"xyz": "0.1 0.2 0.3"}
    assert joint.find("parent").attrib == {"link": "base"}
    assert joint.find("child").attrib == {"link": "finger"}
    assert [element.tag for element in root if element.tag not in {"link", "joint"}] == [
        "gazebo",
        "ros2_control",
    ]
    assert "type='revolute'" in urdf.read_text()


@pytest.mark.parametrize(
    ("joint_xml", "names", "message"),
    [
        ("", ("missing",), "Joint not found"),
        (
            "<joint name='tool_joint' type='fixed'>"
            "<parent link='base'/><child link='tool'/></joint>",
            ("tool_joint",),
            "already fixed",
        ),
        (
            "<joint name='tool_joint' type='revolute'>"
            "<parent link='base'/><child link='tool'/></joint>",
            ("tool_joint", "tool_joint"),
            "already requested",
        ),
    ],
)
def test_with_fixed_joints_rejects_invalid_model(
    tmp_path: Path,
    joint_xml: str,
    names: tuple[str, ...],
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(f"<robot name='r'><link name='base'/><link name='tool'/>{joint_xml}</robot>")
    model = robot_model.RobotModel.from_file(urdf).with_fixed_joints(*names)

    with pytest.raises(ValueError, match=message):
        model.load()


def test_with_fixed_joints_requires_a_joint(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")

    with pytest.raises(ValueError, match="At least one joint"):
        robot_model.RobotModel.from_file(urdf).with_fixed_joints()


@pytest.mark.parametrize(
    ("lower", "upper", "message"),
    [
        (1.0, 0.0, "inverted"),
        (float("nan"), 1.0, "finite"),
    ],
)
def test_with_joint_position_limits_rejects_invalid_range(
    tmp_path: Path,
    lower: float,
    upper: float,
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")

    with pytest.raises(ValueError, match=message):
        robot_model.RobotModel.from_file(urdf).with_joint_position_limits(
            "joint",
            lower=lower,
            upper=upper,
        )


@pytest.mark.parametrize(
    ("joint_xml", "name", "message"),
    [
        ("", "missing", "Joint not found"),
        (
            "<joint name='fixed' type='fixed'><parent link='base'/><child link='tool'/></joint>",
            "fixed",
            "has no position limits",
        ),
    ],
)
def test_with_joint_position_limits_rejects_unsupported_model(
    tmp_path: Path,
    joint_xml: str,
    name: str,
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(f"<robot name='r'><link name='base'/><link name='tool'/>{joint_xml}</robot>")
    model = robot_model.RobotModel.from_file(urdf).with_joint_position_limits(
        name,
        lower=0.0,
        upper=1.0,
    )

    with pytest.raises(ValueError, match=message):
        model.load()


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
    urdf.write_text(
        "<robot name='before'><link name='base'/><link name='slider'/><link name='camera'/>"
        "<joint name='slider_joint' type='prismatic'><parent link='base'/>"
        "<child link='slider'/><limit lower='0.1' upper='1' effort='1' velocity='1'/>"
        "</joint><joint name='camera_joint' type='revolute'><parent link='slider'/>"
        "<child link='camera'/><axis xyz='0 0 1'/>"
        "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>"
    )
    model = (
        robot_model.RobotModel.from_file(urdf)
        .with_joint_position_limits("slider_joint", lower=0.0, upper=1.0)
        .with_fixed_joints("camera_joint")
        .with_fixed_frame("tool", "camera")
    )
    assert ET.fromstring(model.load().xml).get("name") == "before"

    restored = pickle.loads(pickle.dumps(model))
    urdf.write_text(urdf.read_text().replace("name='before'", "name='after'"))
    root = ET.fromstring(restored.load().xml)

    assert root.get("name") == "after"
    assert root.find("link[@name='tool']") is not None
    assert root.find("joint[@name='slider_joint']/limit").get("lower") == "0.0"
    assert root.find("joint[@name='camera_joint']").get("type") == "fixed"


def test_model_rejects_non_urdf_source(tmp_path: Path) -> None:
    mjcf = tmp_path / "robot.xml"
    model = robot_model.RobotModel.from_file(mjcf)

    with pytest.raises(ValueError, match="must reference a .urdf or .xacro"):
        model.load()
