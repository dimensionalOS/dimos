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

"""Loaded model transformations preserve source context without derived files."""

from pickle import dumps as encode, loads as decode

import pytest

from dimos.robot.assets.model import JointDescription, RobotModel


def test_added_joints_and_sequential_renames_modify_the_loaded_model(tmp_path):
    mesh = tmp_path / "link.stl"
    mesh.write_text("solid link")
    path = tmp_path / "robot.urdf"
    original = (
        '<robot name="test"><link name="base"><visual><geometry>'
        '<mesh filename="link.stl"/></geometry></visual></link><link name="tip"/>'
        '<joint name="joint1" type="revolute"><parent link="base"/><child link="tip"/>'
        '<limit lower="-1" upper="1" velocity="2" effort="3"/></joint></robot>'
    )
    path.write_text(original)
    model = (
        RobotModel.from_file(path)
        .with_renamed_joints({"joint1": "canonical_joint1"})
        .with_added_joints(
            JointDescription(
                name="gripper",
                type="revolute",
                parent_link="base",
                child_link="grip",
                lower=0.0,
                upper=1.0,
                velocity=0.0,
                effort=0.0,
            )
        )
        .with_renamed_joints({"canonical_joint1": "arm/joint1", "gripper": "arm/gripper"})
    )
    assert model.source_path == path
    assert path.read_text() == original
    loaded = model.load()
    assert loaded.source_path == path
    assert [joint.name for joint in loaded.joints] == ["arm/joint1", "arm/gripper"]
    assert [(joint.lower, joint.upper) for joint in loaded.joints] == [(-1.0, 1.0), (0.0, 1.0)]
    assert str(mesh) in loaded.xml
    assert decode(encode(model)).load().xml == loaded.xml
    assert set(tmp_path.iterdir()) == {path, mesh}


def test_model_view_construction_is_lazy(tmp_path, mocker):
    load = mocker.patch.object(RobotModel, "load", side_effect=AssertionError("must stay lazy"))
    model = RobotModel.from_file(tmp_path / "not-downloaded.urdf")
    model.with_added_joints(JointDescription("grip", "revolute", "base", "grip_link"))
    model.with_renamed_joints({"joint": "arm/joint"}).with_renamed_joints(
        {"arm/joint": "other/joint"}
    )
    load.assert_not_called()


@pytest.mark.parametrize(
    "name,parent,child,message",
    [
        ("joint", "base", "new", "Joint already exists"),
        ("new", "absent", "new", "parent link does not exist"),
        ("new", "base", "tip", "child link already exists"),
    ],
)
def test_added_joint_rejects_invalid_topology_after_loading(tmp_path, name, parent, child, message):
    path = tmp_path / "robot.urdf"
    path.write_text(
        '<robot name="test"><link name="base"/><link name="tip"/>'
        '<joint name="joint" type="fixed"><parent link="base"/><child link="tip"/></joint></robot>'
    )
    model = RobotModel.from_file(path).with_added_joints(
        JointDescription(name, "fixed", parent, child)
    )
    with pytest.raises(ValueError, match=message):
        model.load()
