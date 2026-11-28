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

import pytest

pytest.importorskip("pxr")

from pxr import Sdf, Usd

from dimos.sim2.mujoco.drives import usd_drives, usd_with_mjc_servos

_DRIVEN_USDA = """#usda 1.0
(
    defaultPrim = "arm"
)
def Xform "arm" {
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
        def Xform "hand" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
            def PhysicsRevoluteJoint "elbow" (prepend apiSchemas = ["PhysicsDriveAPI:angular"]) {
                float drive:angular:physics:damping = 2
                float drive:angular:physics:stiffness = 40
                float drive:angular:physics:targetPosition = 0.25
                uniform token physics:axis = "Z"
                rel physics:body0 = </arm/base>
                rel physics:body1 = </arm/base/hand>
            }
            def PhysicsPrismaticJoint "rail" (prepend apiSchemas = ["PhysicsDriveAPI:linear"]) {
                float drive:linear:physics:damping = 0.5
                float drive:linear:physics:stiffness = 12
                float drive:linear:physics:targetPosition = 0.03
                uniform token physics:axis = "X"
                rel physics:body0 = </arm/base>
                rel physics:body1 = </arm/base/hand>
            }
            def PhysicsRevoluteJoint "idler" {
                uniform token physics:axis = "Z"
                rel physics:body0 = </arm/base>
                rel physics:body1 = </arm/base/hand>
            }
        }
    }
}
"""


def test_each_driven_joint_reads_back_its_gains_by_the_kind_of_drive_it_has() -> None:
    # A linear drive lives under a different schema instance, so the kind has to be guessed right.
    assert usd_drives(_DRIVEN_USDA) == {
        "elbow": pytest.approx((40.0, 2.0, 0.25)),
        "rail": pytest.approx((12.0, 0.5, 0.03)),
    }


def test_a_usd_drive_becomes_a_mujoco_servo_and_an_undriven_joint_stays_a_motor() -> None:
    actuated = (
        _DRIVEN_USDA.rstrip()[: -len("}")]
        + 'def Scope "Physics" {\n    def MjcActuator "elbow" {\n'
        + "        rel mjc:target = </arm/base/hand/elbow>\n    }\n}\n}\n"
    )
    # MuJoCo ignores DriveAPI, so the gains have to be restated in its own attributes.
    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd_with_mjc_servos(actuated))
    stage = Usd.Stage.Open(layer)
    actuator = next(prim for prim in stage.Traverse() if prim.GetTypeName() == "MjcActuator")
    assert str(actuator.GetAttribute("mjc:biasType").Get()) == "affine"
    assert list(actuator.GetAttribute("mjc:gainPrm").Get())[:1] == pytest.approx([40.0])
    assert list(actuator.GetAttribute("mjc:biasPrm").Get())[1:3] == pytest.approx([-40.0, -2.0])

    plain = "\n".join(line for line in actuated.splitlines() if "drive:" not in line)
    plain = plain.replace(' (prepend apiSchemas = ["PhysicsDriveAPI:angular"])', "")
    assert usd_with_mjc_servos(plain) is plain
