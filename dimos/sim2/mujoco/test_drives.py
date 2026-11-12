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

import pytest

pytest.importorskip("pxr")

from dimos.sim2.mujoco.drives import usd_drives, usd_with_mjc_servos

_DRIVEN_USDA = """#usda 1.0
(
    defaultPrim = "arm"
)

def Xform "arm"
{
    def Xform "base" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Xform "hand" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        )
        {
            def PhysicsRevoluteJoint "elbow" (
                prepend apiSchemas = ["PhysicsDriveAPI:angular"]
            )
            {
                float drive:angular:physics:damping = 2
                float drive:angular:physics:stiffness = 40
                float drive:angular:physics:targetPosition = 0.25
                uniform token physics:axis = "Z"
                rel physics:body0 = </arm/base>
                rel physics:body1 = </arm/base/hand>
            }

            def PhysicsPrismaticJoint "rail" (
                prepend apiSchemas = ["PhysicsDriveAPI:linear"]
            )
            {
                float drive:linear:physics:damping = 0.5
                float drive:linear:physics:stiffness = 12
                float drive:linear:physics:targetPosition = 0.03
                uniform token physics:axis = "X"
                rel physics:body0 = </arm/base>
                rel physics:body1 = </arm/base/hand>
            }

            def PhysicsRevoluteJoint "idler"
            {
                uniform token physics:axis = "Z"
                rel physics:body0 = </arm/base>
                rel physics:body1 = </arm/base/hand>
            }
        }
    }
}
"""


def test_a_driven_joint_reads_back_the_gains_its_layer_authored() -> None:
    assert usd_drives(_DRIVEN_USDA)["elbow"] == pytest.approx((40.0, 2.0, 0.25))


def test_a_prismatic_joint_is_read_as_a_linear_drive() -> None:
    """A linear drive lives under a different schema instance, so the kind has to be guessed right."""
    assert usd_drives(_DRIVEN_USDA)["rail"] == pytest.approx((12.0, 0.5, 0.03))


def test_a_joint_with_no_drive_is_not_reported() -> None:
    assert "idler" not in usd_drives(_DRIVEN_USDA)


_ACTUATED_USDA = (
    _DRIVEN_USDA.rstrip()[: -len("}")]
    + """
    def Scope "Physics"
    {
        def MjcActuator "elbow"
        {
            rel mjc:target = </arm/base/hand/elbow>
        }
    }
}
"""
)


def _servo_of(usd: str) -> dict[str, Any]:
    """The MuJoCo-side gains of the layer's one actuator, read out while its stage is still open."""
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    stage = Usd.Stage.Open(layer)
    actuator = next(prim for prim in stage.Traverse() if prim.GetTypeName() == "MjcActuator")
    return {
        name: actuator.GetAttribute(f"mjc:{name}").Get()
        for name in ("biasType", "gainPrm", "biasPrm")
    }


def test_a_usd_drive_becomes_the_affine_bias_mujoco_servos_on() -> None:
    """MuJoCo ignores DriveAPI, so the gains have to be restated in its own attributes."""
    servo = _servo_of(usd_with_mjc_servos(_ACTUATED_USDA))

    assert str(servo["biasType"]) == "affine"
    assert list(servo["gainPrm"])[:1] == pytest.approx([40.0])
    assert list(servo["biasPrm"])[1:3] == pytest.approx([-40.0, -2.0])


def test_an_actuator_whose_joint_has_no_drive_keeps_being_a_torque_motor() -> None:
    """A joint with no drive is a motor the backend pushes by effort; converting must not steal that."""
    plain = "\n".join(line for line in _ACTUATED_USDA.splitlines() if "drive:" not in line)
    plain = plain.replace('            prepend apiSchemas = ["PhysicsDriveAPI:angular"]\n', "")

    assert usd_with_mjc_servos(plain) is plain
