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

from io import BytesIO
from pathlib import Path

import pytest
from yourdfpy import URDF  # type: ignore[import-untyped]

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.robot.assets.model import PlanarBaseConfig, RobotModel


def _planar_base() -> PlanarBaseConfig:
    return PlanarBaseConfig(
        position_lower=(-2.0, -2.0, -3.14),
        position_upper=(2.0, 2.0, 3.14),
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )


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


def test_planar_base_joints_are_one_coordinate_in_pinocchio(tmp_path: Path) -> None:
    pinocchio = pytest.importorskip("pinocchio")
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)
    planar_base = _planar_base()
    loaded = RobotModel.from_file(urdf).with_planar_base(planar_base).load()

    model = pinocchio.buildModelFromXML(loaded.xml)

    for joint_name in planar_base.joint_names:
        joint = model.joints[model.getJointId(joint_name)]
        assert joint.nq == 1
        assert joint.nv == 1


def test_planar_base_loads_natively_in_visualization_parser(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)
    planar_base = _planar_base()
    loaded = RobotModel.from_file(urdf).with_planar_base(planar_base).load()

    visual = URDF.load(BytesIO(loaded.xml.encode()), build_scene_graph=True)

    assert set(planar_base.joint_names) <= set(visual.actuated_joint_names)


def test_prepared_model_validation_accepts_canonical_bimanual_model(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)

    model = validate_robot_model_config(_config(urdf))

    assert model.root_link == "world"
    assert [joint.name for joint in model.joints if joint.type != "fixed"] == [
        "left/j1",
        "right/j1",
    ]


def test_prepared_model_validation_accepts_prismatic_joint(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)
    urdf.write_text(
        urdf.read_text().replace(
            'name="left/j1" type="revolute"', 'name="left/j1" type="prismatic"'
        )
    )

    validate_robot_model_config(_config(urdf))


@pytest.mark.parametrize("joint_type", ["fixed", "floating", "planar"])
def test_prepared_model_validation_rejects_non_one_dof_controlled_joint(
    tmp_path: Path,
    joint_type: str,
) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)
    urdf.write_text(
        urdf.read_text().replace(
            'name="left/j1" type="revolute"',
            f'name="left/j1" type="{joint_type}"',
        )
    )

    with pytest.raises(ValueError, match="one-DoF revolute, continuous, or prismatic"):
        validate_robot_model_config(_config(urdf))


def test_planar_base_requires_synthetic_root_and_all_base_joints(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_slash_model(urdf)
    base = _planar_base()
    model = RobotModel.from_file(urdf).with_planar_base(base)
    values = _config(urdf).model_dump()
    values.update(
        model=model,
        joint_names=[*base.joint_names, "left/j1", "right/j1"],
        base_link=base.root_link,
    )
    valid = RobotModelConfig.model_validate(values)

    validate_robot_model_config(valid)
    with pytest.raises(ValueError, match="Planar robot base_link"):
        validate_robot_model_config(valid.model_copy(update={"base_link": "world"}))
    with pytest.raises(ValueError, match="Planar robot controllable joints are missing"):
        validate_robot_model_config(
            valid.model_copy(update={"joint_names": [base.joint_names[0], "left/j1"]})
        )


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
    values = _config(urdf).model_dump()
    values.update(replacement)

    with pytest.raises(ValueError, match=message):
        validate_robot_model_config(RobotModelConfig.model_validate(values))


def test_prepared_model_validation_wraps_malformed_asset(tmp_path: Path) -> None:
    urdf = tmp_path / "broken.urdf"
    urdf.write_text("<robot>")

    with pytest.raises(ValueError, match="invalid model asset"):
        validate_robot_model_config(_config(urdf))
