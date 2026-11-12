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

import pytest

pytest.importorskip("pxr")

from dimos.sim2.mujoco.drives import usd_drives

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
