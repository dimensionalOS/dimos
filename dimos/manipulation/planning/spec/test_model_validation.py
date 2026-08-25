# Copyright 2025-2026 Dimensional Inc.
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

"""Prepared robot-model validation and backend naming compatibility tests.

Canonical slash names remain native across the common parser, Viser/yourdfpy,
Pink/Pinocchio, Drake, and RoboPlan loaders; no private name encoding is needed.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from yourdfpy import URDF  # type: ignore[import-untyped]

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.robot.assets.model import RobotModel


def _write_slash_model(path: Path) -> None:
    path.write_text(
        """
<robot name="canonical">
  <link name="world"/>
  <link name="left/base"/>
  <link name="left/tool"/>
  <link name="right/base"/>
  <link name="right/tool"/>
  <joint name="left/mount" type="fixed">
    <parent link="world"/><child link="left/base"/>
  </joint>
  <joint name="left/j1" type="revolute">
    <parent link="left/base"/><child link="left/tool"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="right/mount" type="fixed">
    <parent link="world"/><child link="right/base"/>
  </joint>
  <joint name="right/j1" type="revolute">
    <parent link="right/base"/><child link="right/tool"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    )


def _config(path: Path) -> RobotModelConfig:
    return RobotModelConfig(
        name="robot",
        model=RobotModel.from_file(path),
        joint_names=["left/j1", "right/j1"],
        base_link="world",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=("left/j1",),
                base_link="left/base",
                tip_link="left/tool",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=("right/j1",),
                base_link="right/base",
                tip_link="right/tool",
            ),
        ],
    )


def test_canonical_slash_names_round_trip_through_common_model_parsers(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)

    parsed = RobotModel.from_file(urdf).load()
    visual = URDF.load(urdf)

    assert [joint.name for joint in parsed.joints if joint.type != "fixed"] == [
        "left/j1",
        "right/j1",
    ]
    assert set(visual.actuated_joint_names) == {"left/j1", "right/j1"}


def test_canonical_slash_names_load_natively_in_pinocchio(tmp_path: Path) -> None:
    pinocchio = pytest.importorskip("pinocchio")
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)

    model = pinocchio.buildModelFromUrdf(str(urdf))

    assert [str(name) for name in model.names if "/j" in str(name)] == [
        "left/j1",
        "right/j1",
    ]


def test_prepared_model_validation_accepts_canonical_bimanual_model(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)

    model = validate_robot_model_config(_config(urdf))

    assert model.root_link == "world"
    assert [joint.name for joint in model.joints if joint.type != "fixed"] == [
        "left/j1",
        "right/j1",
    ]


@pytest.mark.parametrize(
    ("replacement", "message"),
    [
        ({"joint_names": ["missing/j1"]}, "configured joints are missing"),
        ({"base_link": "missing/base"}, "base link 'missing/base' is missing"),
        (
            {
                "planning_groups": [
                    PlanningGroupDefinition(
                        name="left_arm",
                        joint_names=("left/j1",),
                        base_link="missing/base",
                        tip_link="left/tool",
                    )
                ]
            },
            "missing base link 'missing/base'",
        ),
    ],
)
def test_prepared_model_validation_identifies_invalid_configuration(
    tmp_path: Path,
    replacement: dict[str, object],
    message: str,
) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)
    with pytest.raises(ValueError, match=message):
        validate_robot_model_config(_config(urdf).model_copy(update=replacement))


def test_prepared_model_validation_wraps_malformed_asset(tmp_path: Path) -> None:
    urdf = tmp_path / "broken.urdf"
    urdf.write_text("<robot>")

    with pytest.raises(ValueError, match="invalid model asset"):
        validate_robot_model_config(_config(urdf))
