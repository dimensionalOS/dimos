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

from dimos.robot.assets.processing import AddFixedFrame
from dimos.robot.model_parser import parse_model


def test_parse_model_applies_urdf_processors(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='robot'><link name='base'/></robot>")

    model = parse_model(
        urdf,
        urdf_processors=(AddFixedFrame(name="tool", parent="base"),),
    )

    assert model.links == ["base", "tool"]
    joint = model.get_joint("tool_joint")
    assert joint is not None
    assert joint.type == "fixed"
    assert joint.parent_link == "base"
    assert joint.child_link == "tool"
