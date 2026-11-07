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

"""Canonical control model, joint order and gravity."""

from __future__ import annotations

from pickle import dumps as encode, loads as decode
from types import SimpleNamespace

import numpy as np
import pinocchio
import pytest

from dimos.robot.manipulators.damiao import connection as connection_module
from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.makermods_metal.config import METAL


def test_control_model_is_lazy(connection_class, mocker):
    load = mocker.patch.object(
        connection_module.RobotModel, "load", side_effect=AssertionError("must stay lazy")
    )
    connection_class.control_model("other")
    load.assert_not_called()


def test_control_model_is_namespaced_and_portable(connection_class):
    control = connection_class.control_model("other")
    assert [joint.name for joint in control.load().joints] == [
        "other/joint1",
        "other/joint2",
        "other/joint3",
        "other/left_gripper",
        "other/right_gripper",
    ]
    assert [(joint.lower, joint.upper) for joint in control.load().joints[-2:]] == [
        (0.0, 1.0),
        (0.0, 1.0),
    ]
    assert decode(encode(control)).load().xml == control.load().xml


def test_control_model_without_source_keeps_vendor_names(connection_class):
    control = connection_class.control_model()
    assert [joint.name for joint in control.load().joints] == [
        *connection_class.robot.joint_names,
    ]


def test_metal_joint3_limit_is_widened_for_the_measured_stop():
    joint3 = METAL.model.load().get_joint("joint3")
    assert joint3 is not None
    assert joint3.lower is not None and joint3.lower < -0.05050


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_gravity_rejects_invalid_positions(make_physical, bad):
    with pytest.raises(ValueError, match="positions"):
        make_physical().gravity(np.array([bad, 0.2, 0.2]))


def test_physical_model_order_must_match_even_without_gravity():
    with pytest.raises(ValueError, match="order"):
        PhysicalModel(_robot_model(), ["joint2", "joint1", "joint3"], gravity_comp=False)


def _robot_model():
    from dimos.control._control_test_helpers import fake_arm_model

    return fake_arm_model(("joint1", "joint2", "joint3"))


def test_physical_model_reduces_joints_outside_the_arm():
    from dimos.control._control_test_helpers import fake_arm_model

    model = fake_arm_model(("joint1", "joint2", "joint3", "extra"))
    physical = PhysicalModel(model, ["joint1", "joint2", "joint3"], gravity_comp=False)
    assert physical._model.nq == 3


def test_physical_model_rejects_mismatched_dimensions(make_physical, mocker):
    model = SimpleNamespace(nq=4, nv=3, names=("universe", "joint1", "joint2", "joint3"))
    mocker.patch.object(pinocchio, "buildModelFromXML", return_value=model)
    with pytest.raises(ValueError, match="dimensions"):
        make_physical()


def test_gravity_rejects_nonfinite_calculation(make_physical, mocker):
    physical = make_physical(gravity_comp=True)
    mocker.patch.object(
        pinocchio,
        "computeGeneralizedGravity",
        return_value=np.array([float("nan"), 0.0, 0.0]),
    )
    with pytest.raises(ValueError, match="invalid torques"):
        physical.gravity(np.array([0.2, 0.2, 0.2]))


def test_model_views_preserve_the_framework_deployment(connection_class):
    assert connection_class.deployment == "python"
    assert connection_class.control_model("robot2").load().joints[0].name == "robot2/joint1"
