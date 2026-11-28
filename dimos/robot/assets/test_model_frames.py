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

from dimos.robot.assets.model import RobotModel, UsdModel


def test_adding_a_frame_again_replaces_it_in_joints_and_the_layer_alike(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    source = tmp_path / "arm.urdf"
    source.write_text('<robot name="arm"><link name="base_link"/></robot>')
    model = (
        UsdModel.from_robot_model(RobotModel.from_file(source))
        .with_frames({"tool": ("arm/base_link", (0.1, 0.0, 0.2), (0.0, 0.0, 0.0))})
        .with_frames({"tool": ("arm/base_link", (0.3, 0.0, 0.0), (0.0, 0.0, 0.0))})
    )

    welded = [joint for joint in model.joints if joint.child_link == "arm/base_link/tool"]
    assert [joint.origin_xyz for joint in welded] == [pytest.approx((0.3, 0.0, 0.0))]
    assert model.usd.count('def Xform "tool"') == 1
    assert "(0.3, 0, 0)" in model.usd


def test_a_layer_with_a_source_path_resolves_its_assets_from_there(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    layer = '#usda 1.0\ndef Xform "arm" (\n    references = @parts/arm.usda@\n)\n{\n}\n'
    model = UsdModel.from_layer(layer, tmp_path / "robot.usda")

    assert model.source_path == tmp_path / "robot.usda"
    assert UsdModel.from_layer(layer).source_path is None


def test_a_frame_pose_is_where_the_frame_sits_above_the_mount(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    import numpy as np

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


def test_a_model_refuses_a_frame_hung_off_a_frame_it_lacks(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    source = tmp_path / "arm.urdf"
    source.write_text('<robot name="arm"><link name="base_link"/></robot>')
    model = UsdModel.from_robot_model(RobotModel.from_file(source))

    with pytest.raises(ValueError, match="no frame 'base_link'"):
        model.with_frames({"tool": ("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}).frame_names()
    with pytest.raises(ValueError, match="has no '/'"):
        model.with_frames({"a/tool": ("arm/base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})


_LEG_URDF = (
    '<robot name="leg"><link name="world"/><link name="hip"/><link name="knee"/><link name="foot"/>'
    '<joint name="float" type="floating"><parent link="world"/><child link="hip"/></joint>'
    '<joint name="bend" type="revolute"><parent link="hip"/><child link="knee"/>'
    '<origin xyz="0 0 -0.3"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>'
    '<joint name="ankle" type="fixed"><parent link="knee"/><child link="foot"/>'
    '<origin xyz="0 0 -0.3"/></joint></robot>'
)


def test_a_usd_model_says_which_tf_edges_are_static_and_where_the_moving_ones_sit(
    tmp_path: Path,
) -> None:
    pytest.importorskip("pxr")
    """Fixed joints and prim edges never move, other single-position joints do; a floating joint's child keeps its rest pose."""
    import numpy as np

    source = tmp_path / "leg.urdf"
    source.write_text(_LEG_URDF)
    model = UsdModel.from_robot_model(RobotModel.from_file(source))

    static = {(edge.parent, edge.child): edge for edge in model.static_transforms()}
    assert set(static) == {
        ("leg", "leg/world"),
        ("leg/world", "leg/world/hip"),
        ("leg/world/hip/knee", "leg/world/hip/knee/foot"),
    }
    np.testing.assert_allclose(
        static["leg/world/hip/knee", "leg/world/hip/knee/foot"].translation, [0, 0, -0.3]
    )

    (bend,) = model.moving_transforms({"bend": np.pi / 2})
    assert (bend.parent, bend.child) == ("leg/world/hip", "leg/world/hip/knee")
    np.testing.assert_allclose(bend.rotation, [0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)], atol=1e-9)
    assert set(model.tf_edges) == {*static, (bend.parent, bend.child)}
    assert "leg/world/hip" in model.tf_frames
