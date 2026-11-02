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

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import RobotModel
from dimos.sim2.utils.mjcf_to_urdf import mjcf_to_urdf

_ARM = """
<mujoco model="arm">
  <asset><mesh name="tri" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset>
  <worldbody>
    <body name="base" pos="5 5 5">
      <freejoint/>
      <geom type="box" size="0.1 0.2 0.3"/>
      <body name="upper" pos="0.1 0 0.2" euler="0.3 0 0.5">
        <joint name="shoulder" axis="0 1 0" pos="0.05 0.02 0" range="-1 1"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.4" size="0.05"/>
        <body name="lower" pos="0 0 0.4">
          <joint name="elbow" axis="1 0 0"/>
          <joint name="slide" type="slide" axis="0 0 1" range="0 0.1"/>
          <geom type="mesh" mesh="tri" pos="0.1 0 0"/>
          <body name="tool" pos="0 0 0.3" quat="0.9 0.1 0.2 0.3"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def assert_urdf_matches_mujoco(mjcf: Path, tmp_path: Path, configurations: int = 5) -> None:
    """Every body's pose relative to the root, from the URDF and from MuJoCo, at random joint positions."""
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    description = RobotModel.from_file(mjcf_to_urdf(mjcf, out_dir=tmp_path)).load()
    joints = {joint.child_link: joint for joint in description.joints}
    root = next(b for b in range(1, model.nbody) if model.body_parentid[b] == 0)
    random = np.random.default_rng(0)
    for _ in range(configurations):
        mujoco.mj_resetData(model, data)
        positions: dict[str, float] = {}
        for joint in range(model.njnt):
            if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            low, high = model.jnt_range[joint] if model.jnt_limited[joint] else (-np.pi, np.pi)
            value = float(random.uniform(low, high))
            data.qpos[model.jnt_qposadr[joint]] = value
            positions[model.joint(joint).name] = value
        mujoco.mj_kinematics(model, data)
        world_from_root = _pose(data.xpos[root], data.xquat[root])
        for body in range(1, model.nbody):
            expected = np.linalg.inv(world_from_root) @ _pose(data.xpos[body], data.xquat[body])
            actual = _root_from(description.root_link, model.body(body).name, joints, positions)
            assert np.linalg.norm(actual[:3, 3] - expected[:3, 3]) < 1e-3, model.body(body).name
            angle = Rotation.from_matrix(actual[:3, :3].T @ expected[:3, :3]).magnitude()
            assert np.degrees(angle) < 0.1, model.body(body).name


def _root_from(
    root: str, link: str, joints: dict, positions: dict[str, float]
) -> NDArray[np.float64]:
    transform = np.eye(4)
    while link != root:
        joint = joints[link]
        transform = joint.parent_from_child(positions.get(joint.name, 0.0)) @ transform
        link = joint.parent_link
    return transform


def _pose(xyz: NDArray[np.float64], wxyz: NDArray[np.float64]) -> NDArray[np.float64]:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(wxyz, scalar_first=True).as_matrix()
    transform[:3, 3] = xyz
    return transform


def test_bodies_joints_and_offset_pivots_match(tmp_path: Path) -> None:
    mjcf = tmp_path / "arm.xml"
    mjcf.write_text(_ARM)
    assert_urdf_matches_mujoco(mjcf, tmp_path)


def test_root_rename_and_meshes(tmp_path: Path) -> None:
    mjcf = tmp_path / "arm.xml"
    mjcf.write_text(_ARM)
    description = RobotModel.from_file(
        mjcf_to_urdf(mjcf, root_link="base_link", out_dir=tmp_path)
    ).load()
    assert description.root_link == "base_link"
    assert 'robot name="arm"' in description.xml
    assert {j.name for j in description.joints} >= {"shoulder", "elbow", "slide"}
    meshes = list(tmp_path.glob("arm-*/tri.obj"))
    assert len(meshes) == 1 and "tri.obj" in description.xml


def test_cached_until_the_mjcf_changes(tmp_path: Path) -> None:
    mjcf = tmp_path / "arm.xml"
    mjcf.write_text(_ARM)
    first = mjcf_to_urdf(mjcf, out_dir=tmp_path)
    assert mjcf_to_urdf(mjcf, out_dir=tmp_path) == first
    mjcf.write_text(_ARM.replace('range="-1 1"', 'range="-2 2"'))
    assert mjcf_to_urdf(mjcf, out_dir=tmp_path) != first


def test_ball_joints_are_refused(tmp_path: Path) -> None:
    mjcf = tmp_path / "ball.xml"
    mjcf.write_text(
        '<mujoco><worldbody><body name="a"><body name="b"><joint type="ball"/>'
        '<geom size="0.1"/></body></body></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="hinge and slide"):
        mjcf_to_urdf(mjcf, out_dir=tmp_path)
