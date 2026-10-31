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

"""Canonical geometry, deployment coordinates and physical computations."""

from __future__ import annotations

from dataclasses import replace
import importlib
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.manipulators.common import damiao_connection as common
from dimos.robot.manipulators.common.damiao_model import PhysicalModel


def test_control_and_planning_namespace_are_lazy_consistent_and_portable(robot_spec, mocker):
    load = mocker.patch.object(
        common.RobotModel, "load", side_effect=AssertionError("must stay lazy")
    )
    deployed = common.deployment(robot_spec, "other")
    assert deployed.arm_joints == ("other/joint1", "other/joint2", "other/joint3")
    assert deployed.gripper_joints == ("other/left_gripper", "other/right_gripper")
    assert deployed.planning.planning_groups[0].joint_names == deployed.arm_joints
    assert deployed.planning.planning_groups[0].base_link == "link0"
    mocker.stop(load)
    assert [joint.name for joint in deployed.planning.model.load().joints] == list(
        deployed.arm_joints
    )
    loaded = deployed.control.load()
    assert [j.name for j in loaded.joints] == list(deployed.joints)
    assert [(j.lower, j.upper) for j in loaded.joints[-2:]] == [(0.0, 1.0), (0.0, 1.0)]
    assert decode(encode(deployed.control)).load().xml == loaded.xml


def test_physical_limits_clamp_margin_and_reject_gross_error(robot_spec):
    physical = PhysicalModel(robot_spec)
    upper = float(physical._upper[0])
    assert physical.normalize([upper + 0.05, 0.2, 0.2]) == [upper, 0.2, 0.2]
    with pytest.raises(RuntimeError, match="outside"):
        physical.normalize([upper + 0.051, 0.2, 0.2])
    metal = PhysicalModel(replace(robot_spec, feedback_margin_rad=0.06))
    assert metal.normalize([upper + 0.05050, 0.2, 0.2])[0] == upper


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_gravity_rejects_invalid_positions(robot_spec, bad):
    with pytest.raises(ValueError, match="positions"):
        PhysicalModel(robot_spec).gravity(np.array([bad, 0.2, 0.2]))


def test_physical_model_order_must_match_even_without_gravity(robot_spec):
    arm = robot_spec.arms[0]
    reordered = replace(
        robot_spec, arms=(replace(arm, joints=tuple(reversed(arm.joints))), robot_spec.arms[1])
    )
    with pytest.raises(ValueError, match="order"):
        PhysicalModel(reordered)


@pytest.mark.parametrize("invalid", ["dimensions", "lower", "upper", "inverted"])
def test_physical_model_rejects_invalid_ranges_and_dimensions(robot_spec, mocker, invalid):
    pin = importlib.import_module("pinocchio")
    model = SimpleNamespace(
        nq=3,
        nv=3,
        names=("universe", "joint1", "joint2", "joint3"),
        lowerPositionLimit=np.array([-1.0, -1.0, -1.0]),
        upperPositionLimit=np.array([1.0, 1.0, 1.0]),
    )
    if invalid == "dimensions":
        model.nq = 4
    elif invalid == "lower":
        model.lowerPositionLimit[0] = float("nan")
    elif invalid == "upper":
        model.upperPositionLimit = np.array([1.0, 1.0])
    else:
        model.lowerPositionLimit[0] = 2.0
    mocker.patch.object(pin, "buildModelFromXML", return_value=model)
    with pytest.raises(ValueError, match="dimensions|position limits"):
        PhysicalModel(robot_spec)


def test_gravity_rejects_nonfinite_calculation(robot_spec, mocker):
    physical = PhysicalModel(replace(robot_spec, gravity_comp=True))
    mocker.patch.object(
        physical._pinocchio,
        "computeGeneralizedGravity",
        return_value=np.array([float("nan"), 0.0, 0.0]),
    )
    with pytest.raises(ValueError, match="invalid torques"):
        physical.gravity(np.array([0.2, 0.2, 0.2]))


def test_models_accessor_preserves_framework_deployment(connection_class):
    assert connection_class.deployment == "python"
    assert connection_class.models(source="robot2").joints == (
        "robot2/joint1",
        "robot2/joint2",
        "robot2/joint3",
        "robot2/left_gripper",
        "robot2/right_gripper",
    )


def test_deployment_materializes_existing_canonical_joint_rename(robot_spec):
    renamed = robot_spec.planning.model.with_renamed_joints({"joint1": "yam_joint1"})
    first_arm = robot_spec.arms[0]
    robot = replace(
        robot_spec,
        arms=(
            replace(
                first_arm,
                joints=(replace(first_arm.joints[0], name="yam_joint1"), first_arm.joints[1]),
            ),
            robot_spec.arms[1],
        ),
        planning=robot_spec.planning.model_copy(
            update={
                "model": renamed,
                "joint_names": ["yam_joint1", "joint2", "joint3"],
                "planning_groups": [
                    PlanningGroupDefinition("both", ("yam_joint1", "joint2", "joint3"), "link0")
                ],
            }
        ),
    )
    deployed = common.deployment(robot, "arm")
    expected = ["arm/yam_joint1", "arm/joint2", "arm/joint3"]
    assert [joint.name for joint in deployed.planning.model.load().joints] == expected
    assert [joint.name for joint in deployed.control.load().joints] == [
        *expected,
        "arm/left_gripper",
        "arm/right_gripper",
    ]
    assert deployed.planning.planning_groups[0].joint_names == tuple(expected)
    assert decode(encode(deployed.planning.model)).load().xml == deployed.planning.model.load().xml
