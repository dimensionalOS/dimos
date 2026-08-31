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

from collections.abc import Callable
from pathlib import Path
import pickle
import xml.etree.ElementTree as ET

import pytest
from pytest_mock import MockerFixture

import dimos.robot.assets.model as robot_model
from dimos.utils.data import LfsPath


def _planar_base() -> robot_model.PlanarBaseConfig:
    return robot_model.PlanarBaseConfig(
        position_lower=(-2.0, -3.0, -3.14),
        position_upper=(2.0, 3.0, 3.14),
        velocity_limits=(0.5, 0.6, 1.0),
        acceleration_limits=(1.0, 1.2, 2.0),
    )


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


def test_model_load_resolves_relative_assets_from_source_directory(tmp_path: Path) -> None:
    meshes = tmp_path / "meshes"
    meshes.mkdir()
    mesh = meshes / "link.stl"
    mesh.write_text("mesh")
    texture = tmp_path / "surface.png"
    texture.write_text("texture")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'><visual><geometry>"
        '<mesh filename="meshes/link.stl"/></geometry><material name="m">'
        '<texture filename="surface.png"/></material></visual></link></robot>'
    )

    loaded = robot_model.RobotModel.from_file(urdf).load()
    root = ET.fromstring(loaded.xml)
    loaded_mesh = root.find(".//mesh")
    loaded_texture = root.find(".//texture")

    assert loaded_mesh is not None
    assert loaded_texture is not None
    assert loaded_mesh.get("filename") == str(mesh)
    assert loaded_texture.get("filename") == str(texture)


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


def test_with_planar_base_prepends_three_one_dof_joints(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'/><link name='tool'/>"
        "<joint name='arm' type='revolute'><parent link='base'/><child link='tool'/>"
        "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>"
    )
    planar = _planar_base()

    model = robot_model.RobotModel.from_file(urdf).with_planar_base(planar)
    loaded = model.load()
    root = ET.fromstring(loaded.xml)

    assert model.planar_base is planar
    assert loaded.root_link == "planar_base_root"
    assert [loaded.get_joint(name).type for name in planar.joint_names] == [
        "prismatic",
        "prismatic",
        "revolute",
    ]
    assert [
        (
            loaded.get_joint(name).parent_link,
            loaded.get_joint(name).child_link,
        )
        for name in planar.joint_names
    ] == [
        ("planar_base_root", "planar_base_root_x"),
        ("planar_base_root_x", "planar_base_root_xy"),
        ("planar_base_root_xy", "base"),
    ]
    assert [root.find(f"joint[@name='{name}']/axis").get("xyz") for name in planar.joint_names] == [
        "1 0 0",
        "0 1 0",
        "0 0 1",
    ]
    assert root.find("joint[@name='base/x']/limit").attrib == {
        "lower": "-2.0",
        "upper": "2.0",
        "effort": "1",
        "velocity": "0.5",
        "acceleration": "1.0",
    }
    assert "planar_base_root" not in urdf.read_text()


def test_planar_base_can_be_extended_by_other_model_transforms(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")

    loaded = (
        robot_model.RobotModel.from_file(urdf)
        .with_planar_base(_planar_base())
        .with_fixed_frame("map_origin", "planar_base_root")
        .with_joint_position_limits("base/x", lower=-1.0, upper=1.0)
        .load()
    )
    root = ET.fromstring(loaded.xml)

    assert root.find("joint[@name='map_origin_joint']/parent").get("link") == "planar_base_root"
    assert root.find("joint[@name='base/x']/limit").get("lower") == "-1.0"


@pytest.mark.parametrize(
    ("update", "message"),
    [
        ({"position_lower": (-1.0, -1.0)}, "x, y, and yaw"),
        ({"position_upper": (-3.0, 3.0, 3.14)}, "strictly increasing"),
        ({"velocity_limits": (0.5, 0.0, 1.0)}, "positive"),
        ({"acceleration_limits": (1.0, float("nan"), 2.0)}, "finite"),
        ({"joint_names": ("base/x", "base/x", "base/yaw")}, "unique"),
    ],
)
def test_planar_base_rejects_invalid_configuration(
    update: dict[str, object],
    message: str,
) -> None:
    values: dict[str, object] = {
        "position_lower": (-2.0, -3.0, -3.14),
        "position_upper": (2.0, 3.0, 3.14),
        "velocity_limits": (0.5, 0.6, 1.0),
        "acceleration_limits": (1.0, 1.2, 2.0),
    }
    values.update(update)

    with pytest.raises(ValueError, match=message):
        robot_model.PlanarBaseConfig(**values)


@pytest.mark.parametrize(
    ("urdf_xml", "message"),
    [
        ("<robot name='r'><link name='one'/><link name='two'/></robot>", "one URDF root"),
        (
            "<robot name='r'><link name='base'/><link name='planar_base_root'/></robot>",
            "one URDF root",
        ),
        (
            "<robot name='r'><link name='base'/><joint name='base/x' type='fixed'>"
            "<parent link='base'/><child link='tool'/></joint><link name='tool'/></robot>",
            "joint names already exist",
        ),
    ],
)
def test_with_planar_base_rejects_incompatible_model(
    tmp_path: Path,
    urdf_xml: str,
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(urdf_xml)
    model = robot_model.RobotModel.from_file(urdf).with_planar_base(_planar_base())

    with pytest.raises(ValueError, match=message):
        model.load()


def test_with_planar_base_rejects_duplicate_application(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")
    model = robot_model.RobotModel.from_file(urdf).with_planar_base(_planar_base())

    with pytest.raises(ValueError, match="already has"):
        model.with_planar_base(_planar_base())


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


def test_with_subtree_rooted_at_selects_existing_descendants(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><material name='dark'/><link name='world'/><link name='pelvis'/>"
        "<link name='leg'/><link name='torso'/><joint name='floating' type='floating'>"
        "<parent link='world'/><child link='pelvis'/></joint>"
        "<joint name='hip' type='revolute'><parent link='pelvis'/><child link='leg'/></joint>"
        "<joint name='waist' type='revolute'><parent link='pelvis'/><child link='torso'/></joint>"
        "<gazebo reference='leg'/></robot>"
    )

    loaded = robot_model.RobotModel.from_file(urdf).with_subtree_rooted_at("pelvis").load()
    root = ET.fromstring(loaded.xml)

    assert loaded.root_link == "pelvis"
    assert [link.get("name") for link in root.findall("link")] == ["pelvis", "leg", "torso"]
    assert [joint.get("name") for joint in root.findall("joint")] == ["hip", "waist"]
    assert root.find("material[@name='dark']") is not None
    assert root.find("gazebo[@reference='leg']") is not None
    assert "<link name='world'" in urdf.read_text()


def test_without_joint_subtrees_removes_complete_descendant_branches(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='pelvis'/><link name='hip'/><link name='knee'/>"
        "<link name='torso'/><joint name='hip_joint' type='revolute'>"
        "<parent link='pelvis'/><child link='hip'/></joint>"
        "<joint name='knee_joint' type='revolute'><parent link='hip'/><child link='knee'/></joint>"
        "<joint name='waist_joint' type='revolute'>"
        "<parent link='pelvis'/><child link='torso'/></joint></robot>"
    )

    loaded = (
        robot_model.RobotModel.from_file(urdf)
        .with_subtree_rooted_at("pelvis")
        .without_joint_subtrees("hip_joint")
        .load()
    )
    root = ET.fromstring(loaded.xml)

    assert loaded.root_link == "pelvis"
    assert [link.get("name") for link in root.findall("link")] == ["pelvis", "torso"]
    assert [joint.get("name") for joint in root.findall("joint")] == ["waist_joint"]


@pytest.mark.parametrize(
    ("configure", "message"),
    [
        (lambda model: model.with_subtree_rooted_at("missing"), "root link not found"),
        (lambda model: model.without_joint_subtrees("missing"), "subtree not found"),
        (
            lambda model: model.with_subtree_rooted_at("torso").without_joint_subtrees("hip_joint"),
            "outside selected root",
        ),
    ],
)
def test_structural_model_views_reject_unknown_or_out_of_scope_topology(
    tmp_path: Path,
    configure: Callable[[robot_model.RobotModel], robot_model.RobotModel],
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='pelvis'/><link name='leg'/><link name='torso'/>"
        "<joint name='hip_joint' type='revolute'>"
        "<parent link='pelvis'/><child link='leg'/></joint>"
        "<joint name='waist_joint' type='revolute'>"
        "<parent link='pelvis'/><child link='torso'/></joint></robot>"
    )

    with pytest.raises(ValueError, match=message):
        configure(robot_model.RobotModel.from_file(urdf)).load()


def test_structural_model_view_validates_later_transformations(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='pelvis'/><link name='leg'/><link name='torso'/>"
        "<joint name='hip_joint' type='revolute'>"
        "<parent link='pelvis'/><child link='leg'/></joint>"
        "<joint name='waist_joint' type='revolute'>"
        "<parent link='pelvis'/><child link='torso'/></joint></robot>"
    )
    model = (
        robot_model.RobotModel.from_file(urdf)
        .without_joint_subtrees("hip_joint")
        .with_fixed_joints("hip_joint")
    )

    with pytest.raises(ValueError, match="Joint not found: hip_joint"):
        model.load()


def test_structural_model_view_rejects_duplicate_configuration(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='pelvis'/></robot>")
    model = robot_model.RobotModel.from_file(urdf)

    with pytest.raises(ValueError, match="already selected"):
        model.with_subtree_rooted_at("pelvis").with_subtree_rooted_at("pelvis")
    with pytest.raises(ValueError, match="already requested"):
        model.without_joint_subtrees("hip", "hip")


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


def test_with_renamed_joints_updates_model_references(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'/><link name='first'/><link name='second'/>"
        "<joint name='source' type='revolute'><parent link='base'/><child link='first'/></joint>"
        "<joint name='follower' type='revolute'><parent link='first'/><child link='second'/>"
        "<mimic joint='source'/></joint></robot>"
    )

    loaded = (
        robot_model.RobotModel.from_file(urdf)
        .with_renamed_joints({"source": "robot/source"})
        .load()
    )
    root = ET.fromstring(loaded.xml)

    assert [joint.get("name") for joint in root.findall("joint")] == [
        "robot/source",
        "follower",
    ]
    assert root.find("joint[@name='follower']/mimic").get("joint") == "robot/source"


@pytest.mark.parametrize(
    ("names", "message"),
    [
        ({"missing": "robot/missing"}, "Joint not found"),
        ({"source": "follower"}, "unique"),
        ({"source": "robot/joint", "follower": "robot/joint"}, "unique"),
    ],
)
def test_with_renamed_joints_rejects_invalid_model(
    tmp_path: Path,
    names: dict[str, str],
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'/><link name='first'/><link name='second'/>"
        "<joint name='source' type='revolute'><parent link='base'/><child link='first'/></joint>"
        "<joint name='follower' type='revolute'><parent link='first'/><child link='second'/></joint>"
        "</robot>"
    )

    with pytest.raises(ValueError, match=message):
        robot_model.RobotModel.from_file(urdf).with_renamed_joints(names).load()


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
