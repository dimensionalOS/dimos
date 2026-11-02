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

import importlib
from pickle import dumps as encode, loads as decode
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.robot.manipulators.common import damiao_connection as common


def test_control_and_planning_namespace_are_lazy_consistent_and_portable(connection_class, mocker):
    load = mocker.patch.object(
        common.RobotModel, "load", side_effect=AssertionError("must stay lazy")
    )
    planning = connection_class.planning_model("other")
    control = connection_class.control_model("other")
    assert planning.joint_names == ["other/joint1", "other/joint2", "other/joint3"]
    assert planning.planning_groups[0].joint_names == tuple(planning.joint_names)
    assert planning.planning_groups[0].base_link == "link0"
    mocker.stop(load)
    assert [joint.name for joint in planning.model.load().joints] == planning.joint_names
    loaded = control.load()
    assert [joint.name for joint in loaded.joints] == [
        *planning.joint_names,
        "other/left_gripper",
        "other/right_gripper",
    ]
    assert [(j.lower, j.upper) for j in loaded.joints[-2:]] == [(0.0, 1.0), (0.0, 1.0)]
    assert decode(encode(control)).load().xml == loaded.xml


def test_physical_limits_clamp_margin_and_reject_gross_error(make_physical):
    physical = make_physical()
    upper = float(physical._upper[0])
    assert physical.normalize([upper + 0.05, 0.2, 0.2]) == [upper, 0.2, 0.2]
    with pytest.raises(RuntimeError, match="outside"):
        physical.normalize([upper + 0.051, 0.2, 0.2])
    metal = make_physical(feedback_margin_rad=0.06)
    assert metal.normalize([upper + 0.05050, 0.2, 0.2])[0] == upper


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_gravity_rejects_invalid_positions(make_physical, bad):
    with pytest.raises(ValueError, match="positions"):
        make_physical().gravity(np.array([bad, 0.2, 0.2]))


def test_physical_model_order_must_match_even_without_gravity(
    connection_class, make_physical, monkeypatch
):
    monkeypatch.setattr(connection_class, "kinematic_joint_names", ["joint2", "joint1", "joint3"])
    with pytest.raises(ValueError, match="order"):
        make_physical()


@pytest.mark.parametrize("invalid", ["dimensions", "lower", "upper", "inverted"])
def test_physical_model_rejects_invalid_ranges_and_dimensions(make_physical, mocker, invalid):
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
        make_physical()


def test_gravity_rejects_nonfinite_calculation(make_physical, mocker):
    physical = make_physical(gravity_comp=True)
    mocker.patch.object(
        physical._pinocchio,
        "computeGeneralizedGravity",
        return_value=np.array([float("nan"), 0.0, 0.0]),
    )
    with pytest.raises(ValueError, match="invalid torques"):
        physical.gravity(np.array([0.2, 0.2, 0.2]))


def test_model_views_preserve_the_framework_deployment(connection_class):
    assert connection_class.deployment == "python"
    assert connection_class.planning_model("robot2").joint_names == [
        "robot2/joint1",
        "robot2/joint2",
        "robot2/joint3",
    ]


def test_model_views_compose_an_existing_canonical_joint_rename(connection_class, monkeypatch):
    renamed = connection_class.planning.model.with_renamed_joints({"joint1": "yam_joint1"})
    planning = connection_class.planning.model_copy(
        update={
            "model": renamed,
            "joint_names": ["yam_joint1", "joint2", "joint3"],
            "planning_groups": [
                PlanningGroupDefinition("both", ("yam_joint1", "joint2", "joint3"), "link0")
            ],
        }
    )
    monkeypatch.setattr(connection_class, "planning", planning)
    deployed = connection_class.planning_model("arm")
    control = connection_class.control_model("arm")
    expected = ["arm/yam_joint1", "arm/joint2", "arm/joint3"]
    assert [joint.name for joint in deployed.model.load().joints] == expected
    assert [joint.name for joint in control.load().joints] == [
        *expected,
        "arm/left_gripper",
        "arm/right_gripper",
    ]
    assert deployed.planning_groups[0].joint_names == tuple(expected)
    assert decode(encode(deployed.model)).load().xml == deployed.model.load().xml
