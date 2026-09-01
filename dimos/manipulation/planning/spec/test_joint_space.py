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

"""Behavior tests for canonical joint-coordinate preparation and operations."""

import math
from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.joint_space import CoordinateTopology
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.robot.assets.model import RobotModel


def _write_model(path: Path, joints: str) -> None:
    path.write_text(
        f"""
<robot name="joint-space">
  <link name="base"/>
  <link name="link1"/><link name="link2"/><link name="link3"/><link name="link4"/>
  {joints}
</robot>
"""
    )


def _joint(name: str, joint_type: str, parent: str, child: str, limit: str) -> str:
    return f"""
<joint name="{name}" type="{joint_type}">
  <parent link="{parent}"/><child link="{child}"/>
  <axis xyz="0 0 1"/><limit effort="1" velocity="2" acceleration="3" {limit}/>
</joint>
"""


def test_preparation_compiles_urdf_joint_types_into_topologies(tmp_path: Path) -> None:
    path = tmp_path / "model.urdf"
    _write_model(
        path,
        "".join(
            (
                _joint("bounded_angle", "revolute", "base", "link1", 'lower="-1" upper="1"'),
                _joint("bounded_slide", "prismatic", "link1", "link2", 'lower="-2" upper="2"'),
                _joint("free_slide", "prismatic", "link2", "link3", ""),
                _joint("free_angle", "continuous", "link3", "link4", ""),
            )
        ),
    )
    config = RobotModelConfig(
        model=RobotModel.from_file(path),
        joint_names=["bounded_angle", "bounded_slide", "free_slide", "free_angle"],
        base_link="base",
    )

    prepared = prepare_robot_model(config)

    assert tuple(coordinate.topology for coordinate in prepared.joint_space.coordinates) == (
        CoordinateTopology.INTERVAL,
        CoordinateTopology.INTERVAL,
        CoordinateTopology.LINE,
        CoordinateTopology.CIRCLE,
    )
    assert prepared.joint_space.velocity_limits == (2.0, 2.0, 2.0, 2.0)
    assert prepared.joint_space.acceleration_limits == (3.0, 3.0, 3.0, 3.0)


@pytest.mark.parametrize(
    ("joint_type", "limit", "message"),
    [
        ("revolute", "", "requires finite position limits"),
        ("prismatic", 'lower="-1"', "both lower and upper"),
        ("continuous", 'lower="-1" upper="1"', "must not define position limits"),
    ],
)
def test_preparation_rejects_ambiguous_coordinate_domains(
    tmp_path: Path, joint_type: str, limit: str, message: str
) -> None:
    path = tmp_path / "model.urdf"
    _write_model(path, _joint("joint", joint_type, "base", "link1", limit))
    config = RobotModelConfig(
        model=RobotModel.from_file(path), joint_names=["joint"], base_link="base"
    )

    with pytest.raises(ValueError, match=message):
        prepare_robot_model(config)


def test_acceleration_default_is_an_explicit_model_transformation(tmp_path: Path) -> None:
    path = tmp_path / "model.urdf"
    joints = _joint("joint", "revolute", "base", "link1", 'lower="-1" upper="1"')
    _write_model(path, joints.replace(' acceleration="3"', ""))
    config = RobotModelConfig(
        model=RobotModel.from_file(path), joint_names=["joint"], base_link="base"
    )

    with pytest.raises(ValueError, match="missing an acceleration limit"):
        prepare_robot_model(config)

    prepared = prepare_robot_model(
        config.model_copy(
            update={"model": config.model.with_default_joint_acceleration_limit(4.0)}
        )
    )
    assert prepared.joint_space.acceleration_limits == (4.0,)
    assert prepared.description.get_joint("joint").acceleration == 4.0  # type: ignore[union-attr]


def test_joint_space_wraps_interpolates_lifts_and_builds_finite_domains(
    tmp_path: Path,
) -> None:
    path = tmp_path / "model.urdf"
    _write_model(
        path,
        _joint("x", "prismatic", "base", "link1", "")
        + _joint("yaw", "continuous", "link1", "link2", ""),
    )
    space = prepare_robot_model(
        RobotModelConfig(
            model=RobotModel.from_file(path),
            joint_names=["x", "yaw"],
            base_link="base",
        )
    ).joint_space
    start = space.configuration([10.0, math.pi - 0.1])
    goal = space.configuration([-4.0, -math.pi + 0.1])

    assert space.delta(start, goal).values == pytest.approx((-14.0, 0.2))
    assert space.interpolate(start, goal, 0.5).positions == pytest.approx((3.0, -math.pi))
    assert space.distance(start, goal) == pytest.approx(np.linalg.norm([-7.0, 0.1]))
    lower, upper = space.finite_sampling_domain(start, goal, margin=2.0)
    assert lower == pytest.approx([-6.0, -math.pi])
    assert upper == pytest.approx([12.0, math.pi])
    assert space.lifted_positions([start, goal])[-1] == pytest.approx(
        (-4.0, math.pi + 0.1)
    )
