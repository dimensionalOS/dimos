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


"""An MJCF's bodies and hinges come out as the URDF links and joints a RobotModel loads, with their inertia and limits."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from dimos.robot.assets.mjcf import mjcf_to_urdf

TETRAHEDRON_OBJ = """v 0 0 0
v 0.1 0 0
v 0 0.1 0
v 0 0 0.1
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
"""

LEG = """
<mujoco model="dog">
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <asset><mesh name="shell" file="shell.obj"/></asset>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="base_link" pos="0 0 0.5">
      <freejoint/>
      <inertial pos="0.01 0 -0.02" mass="15" diaginertia="0.1 0.5 0.6"/>
      <geom type="mesh" mesh="shell" contype="0" conaffinity="0" group="2"/>
      <geom type="box" size="0.3 0.1 0.05"/>
      <body name="hip" pos="0.3 0.07 0">
        <inertial pos="0 0.05 -0.1" mass="2" diaginertia="0.04 0.04 0.003"/>
        <joint name="hip_joint" axis="0 -1 0" range="-2.2 2.5" actuatorfrcrange="-76.4 76.4"/>
        <geom type="cylinder" size="0.05 0.04" pos="0 0.1 -0.25" quat="0.707107 0.707107 0 0"/>
        <body name="wheel" pos="0 0.06 -0.25">
          <inertial pos="0 0 0" mass="0.6" diaginertia="0.003 0.0015 0.0015"/>
          <joint name="wheel_joint" axis="0 -1 0" actuatorfrcrange="-21.6 21.6"/>
          <geom type="cylinder" size="0.09 0.027" quat="0.707107 0.707107 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def mjcf(tmp_path: Path) -> Path:
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "shell.obj").write_text(TETRAHEDRON_OBJ)
    path = tmp_path / "dog.xml"
    path.write_text(LEG)
    return path


def test_bodies_and_hinges_become_links_and_joints(mjcf: Path) -> None:
    loaded = mjcf_to_urdf(mujoco.MjSpec.from_file(str(mjcf))).load()
    assert loaded.root_link == "base_link"
    hip, wheel = loaded.joints
    assert (hip.name, hip.type, hip.parent_link, hip.child_link) == (
        "hip_joint",
        "revolute",
        "base_link",
        "hip",
    )
    assert (hip.lower, hip.upper, hip.effort, hip.axis) == (-2.2, 2.5, 76.4, (0.0, -1.0, 0.0))
    assert hip.origin_xyz == pytest.approx((0.3, 0.07, 0.0))
    assert (wheel.name, wheel.type, wheel.lower, wheel.effort) == (
        "wheel_joint",
        "continuous",
        None,
        21.6,
    )
    assert wheel.origin_xyz == pytest.approx((0.0, 0.06, -0.25))


def test_inertia_meshes_and_collisions_come_across(mjcf: Path) -> None:
    from xml.etree import ElementTree as ET

    robot = ET.fromstring(mjcf_to_urdf(mujoco.MjSpec.from_file(str(mjcf))).load().xml)
    base = robot.find("link[@name='base_link']")
    assert base is not None
    assert float(base.find("inertial/mass").get("value")) == 15.0  # type: ignore[union-attr]
    inertia = base.find("inertial/inertia")
    assert inertia is not None
    assert [float(inertia.get(k)) for k in ("ixx", "iyy", "izz")] == [0.1, 0.5, 0.6]  # type: ignore[arg-type]
    mesh = base.find("visual/geometry/mesh")
    assert mesh is not None and Path(mesh.get("filename")).exists()  # type: ignore[arg-type]
    assert base.find("collision/geometry/box").get("size") == "0.6 0.2 0.1"  # type: ignore[union-attr]
    # the world's floor is not part of the robot
    assert {link.get("name") for link in robot.iter("link")} == {"base_link", "hip", "wheel"}


def test_the_urdf_poses_the_robot_as_mujoco_does(mjcf: Path) -> None:
    from scipy.spatial.transform import Rotation

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    joints = {j.name: j for j in mjcf_to_urdf(mujoco.MjSpec.from_file(str(mjcf))).load().joints}
    hip = joints["hip_joint"]
    wheel = joints["wheel_joint"]
    base = np.array(data.xpos[model.body("base_link").id])
    wheel_in_base = np.array(hip.origin_xyz) + Rotation.from_euler("xyz", hip.origin_rpy).apply(
        wheel.origin_xyz
    )
    assert np.allclose(base + wheel_in_base, data.xpos[model.body("wheel").id])


def test_model_edits_still_apply(mjcf: Path) -> None:
    trimmed = (
        mjcf_to_urdf(mujoco.MjSpec.from_file(str(mjcf)))
        .without_joint_subtrees("wheel_joint")
        .load()
    )
    assert [j.name for j in trimmed.joints] == ["hip_joint"]
