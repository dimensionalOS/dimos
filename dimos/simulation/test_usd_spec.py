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

"""The USD reader that both simulators load their robot through."""

from __future__ import annotations

import mujoco
import pytest

from dimos.simulation.usd_spec import shorten_names

pytest.importorskip("mujoco._usd", reason="mujoco-usd is a 3.11+ sim extra")


def _spec() -> mujoco.MjSpec:
    """A two-body robot named the way MuJoCo names one it read out of a USD layer."""
    spec = mujoco.MjSpec()
    mesh = spec.add_mesh()
    mesh.name = "/robot/Geometry/base/base"
    mesh.uservert = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]

    base = spec.worldbody.add_body()
    base.name = "/robot/Geometry/base"
    skin = base.add_geom()
    skin.name = "/robot/Geometry/base/skin"
    skin.type = mujoco.mjtGeom.mjGEOM_MESH
    skin.meshname = "/robot/Geometry/base/base"

    arm = base.add_body()
    arm.name = "/robot/Geometry/base/arm"
    arm.add_geom().size = [0.1, 0.1, 0.1]
    hinge = arm.add_joint()
    hinge.name = "/robot/Physics/shoulder"
    hinge.type = mujoco.mjtJoint.mjJNT_HINGE

    motor = spec.add_actuator()
    motor.name = "/robot/Physics/shoulder_motor"
    motor.trntype = mujoco.mjtTrn.mjTRN_JOINT
    motor.target = "/robot/Physics/shoulder"

    exclude = spec.add_exclude()
    exclude.bodyname1 = "/robot/Geometry/base"
    exclude.bodyname2 = "/robot/Geometry/base/arm"
    return spec


def test_elements_are_found_by_the_name_an_mjcf_would_have_given_them() -> None:
    """A USD layer names every element after its full prim path.

    mj_name2id then returns -1 for the name the simulators actually use, which is a body index
    of nothing rather than an error -- it shows up much later as a robot that will not stand.
    """
    spec = _spec()
    shorten_names(spec)
    model = spec.compile()

    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_motor") >= 0


def test_the_references_between_elements_are_renamed_too() -> None:
    """MjSpec holds them as strings, so a rename alone fails at compile, not at the rename."""
    spec = _spec()
    shorten_names(spec)

    # Each of these is a separate compile-time failure if its reference was left behind:
    # "mesh ... not found in geom 0", an unattached actuator, "body ... not found in bodypair 0".
    model = spec.compile()

    assert model.nu == 1
    assert model.ngeom == 2
    assert model.nexclude == 1


def test_a_name_that_would_collide_leaves_its_whole_kind_alone() -> None:
    """Half a renamed kind is worse than none: lookups would work for some names and not others."""
    spec = _spec()
    twin = spec.body("/robot/Geometry/base/arm").add_body()
    twin.name = "/robot/Geometry/elsewhere/arm"
    shorten_names(spec)

    assert spec.body("/robot/Geometry/base") is not None
    assert mujoco.mj_name2id(spec.compile(), mujoco.mjtObj.mjOBJ_BODY, "base") == -1
    # The kinds that did not collide are still shortened.
    assert mujoco.mj_name2id(spec.compile(), mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_motor") >= 0
