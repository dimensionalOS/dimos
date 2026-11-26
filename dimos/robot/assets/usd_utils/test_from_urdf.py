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

from typing import Any

import numpy as np
import pytest

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd_utils.from_urdf import urdf_to_usd

pytest.importorskip("pxr")


def _stage(urdf: str) -> Any:
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(urdf_to_usd(RobotModel.from_urdf_string(urdf).load()))
    return Usd.Stage.Open(layer)


def _world_from(prim: Any) -> Any:
    from pxr import Usd, UsdGeom

    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))


@pytest.mark.parametrize(
    "shape",
    ['<box size="0.4 0.3 0.1"/>', '<mesh filename="meshes/part.stl" scale="0.4 0.3 0.1"/>'],
)
def test_a_scaled_shape_sits_at_its_urdf_origin_not_a_scaled_one(shape: str) -> None:
    stage = _stage(
        f'<robot name="toy"><link name="base_link"><visual>'
        f'<origin xyz="0.2 -0.1 0.05" rpy="0 0 0.3"/><geometry>{shape}</geometry>'
        f"</visual></link></robot>"
    )
    world = _world_from(stage.GetPrimAtPath("/toy/base_link/visual_0"))

    # USD matrices are row-vector: the translation is the last row, the axes scaled rows.
    np.testing.assert_allclose(world[3, :3], (0.2, -0.1, 0.05), atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(world[:3, :3], axis=1), (0.4, 0.3, 0.1), atol=1e-9)


def test_a_visual_that_names_a_robot_material_takes_its_colour() -> None:
    stage = _stage(
        '<robot name="toy">'
        '<material name="paint"><color rgba="0.9 0.1 0.2 1"/></material>'
        '<link name="base_link"><visual><geometry><sphere radius="0.1"/></geometry>'
        '<material name="paint"/></visual></link></robot>'
    )
    colors = stage.GetPrimAtPath("/toy/base_link/visual_0").GetAttribute("primvars:displayColor")

    assert colors.HasAuthoredValue(), "the named material's colour was dropped"
    np.testing.assert_allclose(colors.Get()[0], (0.9, 0.1, 0.2), atol=1e-6)


def test_an_inline_colour_wins_over_the_robot_material_of_the_same_name() -> None:
    stage = _stage(
        '<robot name="toy">'
        '<material name="paint"><color rgba="0.9 0.1 0.2 1"/></material>'
        '<link name="base_link"><visual><geometry><sphere radius="0.1"/></geometry>'
        '<material name="paint"><color rgba="0 0 1 1"/></material></visual></link></robot>'
    )
    colors = stage.GetPrimAtPath("/toy/base_link/visual_0").GetAttribute("primvars:displayColor")

    np.testing.assert_allclose(colors.Get()[0], (0.0, 0.0, 1.0), atol=1e-6)
