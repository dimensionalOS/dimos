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

"""Behavioral tests for first-class lazy RobotModel composition."""

from pathlib import Path
import pickle
import xml.etree.ElementTree as ET

from pydantic import TypeAdapter
import pytest

from dimos.robot.assets import composition
from dimos.robot.assets.model import RobotModel, RobotModelInstance


@pytest.fixture(autouse=True)
def cache_root(tmp_path, monkeypatch):
    root = tmp_path / "cache"
    monkeypatch.setattr(composition, "DEFAULT_ROBOT_ASSET_CACHE_ROOT", root)
    return root


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


def test_composition_defers_source_loading_and_cache_writes(tmp_path, cache_root, mocker):
    model = arm(tmp_path)
    load = mocker.spy(model.__class__, "load")
    result = RobotModel.compose([RobotModelInstance(model, "left_")])
    assert result.source_path is None
    load.assert_not_called()
    assert not cache_root.exists()


def test_composition_preserves_joint_semantics_and_mount_transforms(tmp_path):
    model = arm(tmp_path)
    original = model.load().xml
    result = RobotModel.compose(
        [
            RobotModelInstance(model, "left_", xyz=(0, 0.3, 0)),
            RobotModelInstance(model, "right_", xyz=(0, -0.3, 0), rpy=(0, 0, 1.2)),
        ],
        name="two_arms",
        root_link="world",
    ).load()
    assert result.root_link == "world"
    assert {joint.name for joint in result.joints} == {
        "left_joint1",
        "left_joint2",
        "left_mount",
        "right_joint1",
        "right_joint2",
        "right_mount",
    }
    left = result.get_joint("left_joint1")
    assert (left.parent_link, left.child_link, left.axis) == ("left_base", "left_wrist", (0, 1, 0))
    assert (left.lower, left.upper, left.velocity, left.effort) == (-1, 2, 3, 4)
    assert result.get_joint("right_joint2").mimic == ("right_joint1", -1, 0.2)
    assert result.get_joint("left_mount").origin_xyz == (0, 0.3, 0)
    assert result.get_joint("right_mount").origin_rpy == (0, 0, 1.2)
    assert result.source_path.read_text() == result.xml
    assert model.load().xml == original


def test_same_package_names_resolve_per_input_including_textures(tmp_path):
    models = [arm(tmp_path, name=name, mesh=True) for name in ("a", "b")]
    result = RobotModel.compose(
        [RobotModelInstance(models[0], "left_"), RobotModelInstance(models[1], "right_")]
    ).load()
    root = ET.fromstring(result.xml)
    assert {node.attrib["filename"] for node in root.findall(".//mesh")} == {
        str(tmp_path / "a/shape.stl"),
        str(tmp_path / "b/shape.stl"),
    }
    assert {node.attrib["filename"] for node in root.findall(".//texture")} == {
        str(tmp_path / "a/surface.png"),
        str(tmp_path / "b/surface.png"),
    }
    assert {node.attrib["name"] for node in root.findall("material")} == {
        "left_paint",
        "right_paint",
    }
    assert {node.attrib["name"] for node in root.findall("link/visual/material")} == {
        "left_paint",
        "right_paint",
    }
    assert result.package_paths == {}


def test_composition_memoizes_and_hashes_final_transformed_xml(tmp_path):
    single = arm(tmp_path)
    instance = RobotModelInstance(single.with_fixed_joints("joint2"), "left_")
    model = RobotModel.compose([instance]).with_fixed_frame("camera", "left_tool")
    first = model.load()
    assert model.load() is first
    assert first.get_joint("left_joint2").type == "fixed"
    assert first.get_joint("camera_joint").parent_link == "left_tool"
    restored = pickle.loads(pickle.dumps(model))
    assert "_loaded" not in restored.__dict__
    assert "_loaded" not in restored._composition.instances[0].model.__dict__
    assert restored.load().source_path == first.source_path
    changed = RobotModel.compose([RobotModelInstance(single, "left_", xyz=(0, 1, 0))])
    assert changed.load().source_path != first.source_path


def test_composed_models_round_trip_through_configuration_and_allow_nesting(tmp_path):
    single = arm(tmp_path)
    inner = RobotModel.compose([RobotModelInstance(single, "left_")])
    model = RobotModel.compose([RobotModelInstance(inner, "rig_")], root_link="world")
    adapter = TypeAdapter(RobotModel)
    restored = adapter.validate_python(adapter.dump_python(model))
    assert restored.load().get_joint("rig_left_joint1") is not None
    assert restored.load().root_link == "world"


@pytest.mark.parametrize("prefix", ["", "left arm_"])
def test_rejects_invalid_prefixes_without_loading(tmp_path, prefix):
    with pytest.raises(ValueError, match="prefix"):
        RobotModelInstance(RobotModel.from_file(tmp_path / "missing.urdf"), prefix)


@pytest.mark.parametrize("vector", [(float("nan"), 0, 0), (0, float("inf"), 0), (0, 0)])
@pytest.mark.parametrize("field", ["xyz", "rpy"])
def test_rejects_invalid_mounts(tmp_path, vector, field):
    with pytest.raises(ValueError, match="three finite"):
        RobotModelInstance(
            RobotModel.from_file(tmp_path / "missing.urdf"), "left_", **{field: vector}
        )


def test_rejects_empty_composition_and_duplicate_prefixes(tmp_path):
    with pytest.raises(ValueError, match="at least one"):
        RobotModel.compose([])
    item = RobotModelInstance(arm(tmp_path), "left_")
    with pytest.raises(ValueError, match="prefixes must be unique"):
        RobotModel.compose([item, item])


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
            '<robot><link name="base"/><link name="tip"/><joint name="mount"><parent link="base"/><child link="tip"/></joint></robot>',
            "joint names",
        ),
        ('<robot><link name="base"/></robot>', "link names"),
    ],
)
def test_rejects_invalid_or_colliding_namespaces(tmp_path, xml, error):
    source = tmp_path / "bad.urdf"
    source.write_text(xml)
    instance = RobotModelInstance(RobotModel.from_file(source), "left_")
    with pytest.raises(ValueError, match=error):
        RobotModel.compose(
            [instance], root_link="left_base" if error == "link names" else "world"
        ).load()


def test_unresolved_relative_assets_do_not_silently_bind_to_another_input(tmp_path):
    source = tmp_path / "arm.urdf"
    source.write_text(
        '<robot><link name="base"><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link></robot>'
    )
    with pytest.raises(ValueError, match="Unresolved asset"):
        RobotModel.compose([RobotModelInstance(RobotModel.from_file(source), "arm_")]).load()
