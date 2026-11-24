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
        .with_frames({"tool": ("base_link", (0.1, 0.0, 0.2), (0.0, 0.0, 0.0))})
        .with_frames({"tool": ("base_link", (0.3, 0.0, 0.0), (0.0, 0.0, 0.0))})
    )

    welded = [joint for joint in model.joints if joint.child_link == "tool"]
    assert [joint.origin_xyz for joint in welded] == [(0.3, 0.0, 0.0)]
    assert model.usd.count('displayName = "tool"') == 1
    assert "(0.3, 0, 0)" in model.usd


def test_a_layer_with_a_source_path_resolves_its_assets_from_there(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from dimos.robot.assets.model import UsdModel

    layer = '#usda 1.0\ndef Xform "arm" (\n    references = @parts/arm.usda@\n)\n{\n}\n'
    model = UsdModel.from_layer(layer, tmp_path / "robot.usda")

    assert model.source_path == tmp_path / "robot.usda"
    assert UsdModel.from_layer(layer).source_path is None
