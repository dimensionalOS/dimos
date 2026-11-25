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


from __future__ import annotations

from pathlib import Path

import pytest

from dimos.robot.assets.model import RobotModel


def test_adding_a_frame_again_replaces_it_in_joints_and_the_layer_alike(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    source = tmp_path / "arm.urdf"
    source.write_text('<robot name="arm"><link name="base_link"/></robot>')
    model = (
        RobotModel.from_file(source)
        .to_usd()
        .with_frames({"tool": ("arm/base_link", (0.1, 0.0, 0.2), (0.0, 0.0, 0.0))})
        .with_frames({"tool": ("arm/base_link", (0.3, 0.0, 0.0), (0.0, 0.0, 0.0))})
    )

    welded = [joint for joint in model.joints if joint.child_link == "arm/base_link/tool"]
    assert [joint.origin_xyz for joint in welded] == [(0.3, 0.0, 0.0)]
    assert model.usd.count('def Xform "tool"') == 1
    assert "(0.3, 0, 0)" in model.usd


def test_a_layer_with_a_source_path_resolves_its_assets_from_there(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from dimos.robot.assets.model import UsdModel

    layer = '#usda 1.0\ndef Xform "arm" (\n    references = @parts/arm.usda@\n)\n{\n}\n'
    model = UsdModel.from_layer(layer, tmp_path / "robot.usda")

    assert model.source_path == tmp_path / "robot.usda"
    assert UsdModel.from_layer(layer).source_path is None


def test_a_frame_pose_is_where_the_frame_sits_above_the_mount(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    import numpy as np

    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils.frames import usd_frame_pose

    source = tmp_path / "leg.urdf"
    source.write_text(
        '<robot name="leg"><link name="base_link"/><link name="foot"/>'
        '<joint name="ankle" type="fixed"><parent link="base_link"/><child link="foot"/>'
        '<origin xyz="0.1 0 -0.4" rpy="0 0 1.5707963267948966"/></joint></robot>'
    )
    usd = UsdModel.from_robot_model(RobotModel.from_file(source)).usd

    pose = usd_frame_pose(usd, "leg/base_link/foot")

    np.testing.assert_allclose(pose[:3, 3], [0.1, 0.0, -0.4], atol=1e-6)
    np.testing.assert_allclose(pose[:3, 0], [0.0, 1.0, 0.0], atol=1e-6)


def test_a_urdf_model_names_its_frames_as_the_layer_it_converts_to(tmp_path: Path) -> None:
    """Without reading the layer, a URDF-backed model's frames, joints and prim edges are the layer's own."""
    pytest.importorskip("pxr")
    from dimos.robot.assets.model import UsdModel

    source = tmp_path / "leg.urdf"
    source.write_text(
        '<robot name="leg"><link name="hip"/><link name="knee"/><link name="foot"/>'
        '<joint name="bend" type="revolute"><parent link="hip"/><child link="knee"/>'
        '<origin xyz="0 0 -0.3"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
        '<joint name="ankle" type="fixed"><parent link="knee"/><child link="foot"/>'
        '<origin xyz="0 0 -0.3"/></joint></robot>'
    )
    fast = (
        RobotModel.from_file(source)
        .to_usd()
        .with_frames({"toe": ("leg/hip/knee/foot", (0.1, 0.0, 0.0), (0.0, 0.0, 0.0))})
    )
    read = UsdModel.from_layer(fast.usd)

    assert (
        fast.frame_names()
        == read.frame_names()
        == {
            "leg",
            "leg/hip",
            "leg/hip/knee",
            "leg/hip/knee/foot",
            "leg/hip/knee/foot/toe",
        }
    )
    assert fast.root_link == read.root_link == "leg/hip"
    assert fast.top_frame == read.top_frame == "leg"
    links = lambda model: {(j.name, j.parent_link, j.child_link) for j in model.joints}  # noqa: E731
    assert links(fast) == links(read)
    edges = lambda model: {(j.parent_link, j.child_link) for j in model.prim_edges}  # noqa: E731
    assert edges(fast) == edges(read) == {("leg", "leg/hip")}


def test_a_urdf_model_refuses_a_frame_hung_off_a_frame_it_lacks(tmp_path: Path) -> None:
    source = tmp_path / "arm.urdf"
    source.write_text('<robot name="arm"><link name="base_link"/></robot>')
    model = RobotModel.from_file(source).to_usd()

    with pytest.raises(ValueError, match="no frame 'base_link'"):
        model.with_frames({"tool": ("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}).frame_names()
    with pytest.raises(ValueError, match="has no '/'"):
        model.with_frames({"a/tool": ("arm/base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})
