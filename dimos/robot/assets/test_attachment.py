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

"""Tests for lazy fixed attachment of one RobotModel tree to another."""

from dataclasses import replace
from pathlib import Path
import pickle
import xml.etree.ElementTree as ET

from pydantic import TypeAdapter
import pytest

from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel


def arm(tmp_path: Path, *, name: str = "arm", mesh: bool = False) -> RobotModel:
    folder = tmp_path / name
    folder.mkdir(exist_ok=True)
    if mesh:
        (folder / "shape.stl").write_text("mesh")
        (folder / "surface.png").write_text("texture")
    geometry = (
        '<visual><geometry><mesh filename="package://parts/shape.stl"/></geometry>'
        '<material name="paint"/></visual>'
        if mesh
        else ""
    )
    texture = '<texture filename="surface.png"/>' if mesh else '<color rgba="1 0 0 1"/>'
    source = folder / "arm.urdf"
    source.write_text(f"""<robot name="arm">
      <material name="paint">{texture}</material>
      <link name="base">{geometry}</link><link name="wrist"/><link name="tool"/>
      <joint name="joint1" type="revolute"><parent link="base"/><child link="wrist"/>
        <axis xyz="0 1 0"/><limit lower="-1" upper="2" velocity="3" effort="4"/></joint>
      <joint name="joint2" type="revolute"><parent link="wrist"/><child link="tool"/>
        <mimic joint="joint1" multiplier="-1" offset="0.2"/>
        <limit lower="-1" upper="2" velocity="3" effort="4"/></joint>
    </robot>""")
    return RobotModel.from_file(source, package_paths={"parts": folder})


def base(tmp_path: Path) -> RobotModel:
    source = tmp_path / "fixture.urdf"
    source.write_text(
        '<robot name="fixture"><link name="world"/><link name="mount_link"/>'
        '<joint name="stand" type="fixed"><parent link="world"/><child link="mount_link"/>'
        '<origin xyz="1 0 0"/></joint></robot>'
    )
    return RobotModel.from_file(source)


def test_attachment_construction_is_lazy_and_preserves_source_handle(tmp_path, mocker):
    host, child = base(tmp_path), arm(tmp_path)
    load = mocker.spy(RobotModel, "load")
    attached = host.with_attached_model(child, parent_link="mount_link", joint_name="mount")
    assert attached.source_path is host.source_path
    load.assert_not_called()


def test_attach_to_specific_nonroot_link_preserves_semantics_without_files(tmp_path):
    host, child = base(tmp_path), arm(tmp_path)
    original_host, original_child = host.load().xml, child.load().xml
    files = set(tmp_path.rglob("*"))
    attached = host.with_attached_model(
        child, parent_link="mount_link", joint_name="left_mount", prefix="left_", xyz=(0, 0.3, 0)
    ).with_attached_model(
        child, parent_link="world", joint_name="right_mount", prefix="right_", rpy=(0, 0, 1.2)
    )
    loaded = attached.load()
    assert loaded.root_link == "world"
    assert {joint.name for joint in loaded.joints} == {
        "stand",
        "left_joint1",
        "left_joint2",
        "left_mount",
        "right_joint1",
        "right_joint2",
        "right_mount",
    }
    joint = loaded.get_joint("left_joint1")
    assert (joint.parent_link, joint.child_link, joint.axis) == (
        "left_base",
        "left_wrist",
        (0, 1, 0),
    )
    assert (joint.lower, joint.upper, joint.velocity, joint.effort) == (-1, 2, 3, 4)
    assert loaded.get_joint("right_joint2").mimic == ("right_joint1", -1, 0.2)
    mount = loaded.get_joint("left_mount")
    assert (mount.type, mount.parent_link, mount.child_link) == ("fixed", "mount_link", "left_base")
    assert mount.origin_xyz == (0, 0.3, 0)
    assert loaded.get_joint("right_mount").origin_rpy == (0, 0, 1.2)
    assert loaded.source_path == host.load().source_path
    assert set(tmp_path.rglob("*")) == files
    assert host.load().xml == original_host
    assert child.load().xml == original_child
    assert attached.load() is loaded


def test_input_packages_and_textures_are_resolved_independently(tmp_path):
    host = arm(tmp_path, name="a", mesh=True)
    child = arm(tmp_path, name="b", mesh=True)
    loaded = host.with_attached_model(
        child, parent_link="tool", joint_name="attachment", prefix="other_"
    ).load()
    root = ET.fromstring(loaded.xml)
    assert {node.attrib["filename"] for node in root.findall(".//mesh")} == {
        str(tmp_path / "a/shape.stl"),
        str(tmp_path / "b/shape.stl"),
    }
    assert {node.attrib["filename"] for node in root.findall(".//texture")} == {
        str(tmp_path / "a/surface.png"),
        str(tmp_path / "b/surface.png"),
    }
    assert {node.attrib["name"] for node in root.findall("material")} == {"paint", "other_paint"}
    assert {node.attrib["name"] for node in root.findall("link/visual/material")} == {
        "paint",
        "other_paint",
    }
    assert loaded.package_paths == host.load().package_paths


def test_attachment_can_target_existing_fixed_frames_and_attached_links(tmp_path):
    host, child = base(tmp_path), arm(tmp_path)
    accessory = tmp_path / "accessory.urdf"
    accessory.write_text('<robot><link name="tip"/></robot>')
    model = (
        host.with_fixed_frame("socket", "mount_link")
        .with_attached_model(
            child.with_fixed_joints("joint2"),
            parent_link="socket",
            joint_name="left_mount",
            prefix="left_",
        )
        .with_renamed_joints({"left_joint1": "axis"})
        .with_attached_model(
            RobotModel.from_file(accessory),
            parent_link="left_tool",
            joint_name="tool_mount",
            prefix="tool_",
        )
        .with_fixed_frame("camera", "tool_tip")
        .with_joint_position_limits("axis", lower=-0.5, upper=0.5)
    )
    loaded = model.load()
    assert loaded.get_joint("left_mount").parent_link == "socket"
    assert loaded.get_joint("left_joint2").type == "fixed"
    assert loaded.get_joint("tool_mount").parent_link == "left_tool"
    assert loaded.get_joint("camera_joint").parent_link == "tool_tip"
    assert loaded.get_joint("axis").lower == -0.5


def test_no_prefix_is_required_for_unique_child_names(tmp_path):
    model = base(tmp_path).with_attached_model(
        arm(tmp_path), parent_link="world", joint_name="install"
    )
    assert model.load().get_joint("joint1") is not None
    assert model.load().get_joint("install").child_link == "base"


def test_pickle_and_configuration_keep_declarations_but_drop_materialized_state(tmp_path):
    model = base(tmp_path).with_attached_model(
        arm(tmp_path), parent_link="world", joint_name="left_mount", prefix="left_"
    )
    loaded = model.load()
    restored = pickle.loads(pickle.dumps(model))
    assert "_loaded" not in restored.__dict__
    assert "_loaded" not in restored._attached_model.host.__dict__
    assert "_loaded" not in restored._attached_model.child.__dict__
    assert restored.load() == loaded
    adapter = TypeAdapter(RobotModel)
    from_config = adapter.validate_python(adapter.dump_python(model))
    assert from_config.load() == loaded
    assert from_config.source_path == model.source_path


def test_host_planar_base_metadata_is_retained_and_not_applied_twice(tmp_path):
    definition = PlanarBaseDefinition(velocity_limits=(1, 1, 1), acceleration_limits=(2, 2, 2))
    host = base(tmp_path).with_planar_base(definition)
    attached = host.with_attached_model(
        arm(tmp_path), parent_link="mount_link", joint_name="install"
    )
    assert attached.planar_base is definition
    assert attached.load().root_link == definition.root_link
    with pytest.raises(ValueError, match="already has a planar base"):
        attached.with_planar_base(definition)


@pytest.mark.parametrize("vector", [(float("nan"), 0, 0), (0, float("inf"), 0), (0, 0)])
@pytest.mark.parametrize("field", ["xyz", "rpy"])
def test_rejects_invalid_transforms_without_loading(tmp_path, vector, field):
    with pytest.raises(ValueError, match="three finite"):
        base(tmp_path).with_attached_model(
            RobotModel.from_file(tmp_path / "missing.urdf"),
            parent_link="world",
            joint_name="mount",
            **{field: vector},
        )


@pytest.mark.parametrize(
    "field, value", [("parent_link", ""), ("joint_name", ""), ("prefix", "bad prefix")]
)
def test_rejects_invalid_identifiers(tmp_path, field, value):
    fields = {"parent_link": "world", "joint_name": "mount", "prefix": ""}
    fields[field] = value
    with pytest.raises(ValueError):
        base(tmp_path).with_attached_model(
            RobotModel.from_file(tmp_path / "missing.urdf"), **fields
        )


@pytest.mark.parametrize(
    "xml, error",
    [
        ('<robot><link name="a"/><link name="b"/></robot>', "exactly one root"),
        ('<robot><link name="base"/><gazebo/></robot>', "Unsupported"),
        (
            '<robot><link name="base"/><joint name="j"><parent link="base"/><child link="missing"/></joint></robot>',
            "unknown link",
        ),
        (
            '<robot><link name="base"/><link name="loop"/><joint name="j"><parent link="loop"/><child link="loop"/></joint></robot>',
            "connected tree",
        ),
        (
            '<robot><link name="base"/><link name="tip"/><joint name="j"><parent link="base"/><child link="tip"/><mimic joint="missing"/></joint></robot>',
            "mimic",
        ),
        ('<robot><link name="base"/><link name="base"/></robot>', "link names"),
    ],
)
def test_rejects_invalid_child_trees(tmp_path, xml, error):
    source = tmp_path / "bad.urdf"
    source.write_text(xml)
    with pytest.raises(ValueError, match=error):
        base(tmp_path).with_attached_model(
            RobotModel.from_file(source), parent_link="world", joint_name="mount"
        ).load()


def test_rejects_unknown_parent_and_joint_or_link_collisions(tmp_path):
    host, child = base(tmp_path), arm(tmp_path)
    with pytest.raises(ValueError, match="unknown parent link"):
        host.with_attached_model(child, parent_link="missing", joint_name="mount").load()
    with pytest.raises(ValueError, match="joint names"):
        host.with_attached_model(child, parent_link="world", joint_name="stand").load()
    with pytest.raises(ValueError, match="joint names"):
        host.with_attached_model(child, parent_link="world", joint_name="joint1").load()
    with pytest.raises(ValueError, match="link names"):
        child.with_attached_model(child, parent_link="tool", joint_name="mount").load()


def test_unresolved_child_assets_do_not_bind_to_host_packages(tmp_path):
    source = tmp_path / "child.urdf"
    source.write_text(
        '<robot><link name="base"><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link></robot>'
    )
    with pytest.raises(ValueError, match="Unresolved asset"):
        base(tmp_path).with_attached_model(
            RobotModel.from_file(source), parent_link="world", joint_name="mount"
        ).load()


def test_host_source_override_propagates_through_attachment_views(tmp_path):
    host = base(tmp_path)
    child = arm(tmp_path)
    model = host.with_attached_model(
        child, parent_link="mount_link", joint_name="left_mount", prefix="left_"
    ).with_attached_model(child, parent_link="world", joint_name="right_mount", prefix="right_")
    alternative = tmp_path / "other_fixture.urdf"
    alternative.write_text(Path(host.source_path).read_text().replace('xyz="1 0 0"', 'xyz="2 0 0"'))
    changed = replace(model, _source_path=alternative)
    assert changed.load().source_path == alternative
    assert changed.load().get_joint("stand").origin_xyz == (2, 0, 0)
    assert model.load().get_joint("stand").origin_xyz == (1, 0, 0)


def test_subtree_view_drops_removed_inherited_planar_base_metadata(tmp_path):
    definition = PlanarBaseDefinition(velocity_limits=(1, 1, 1), acceleration_limits=(2, 2, 2))
    model = (
        base(tmp_path)
        .with_planar_base(definition)
        .with_attached_model(
            arm(tmp_path), parent_link="mount_link", joint_name="mount", prefix="arm_"
        )
    )
    selected = model.with_subtree_rooted_at("world")
    assert selected.planar_base is None
    assert selected.load().root_link == "world"
    assert model.planar_base is definition
