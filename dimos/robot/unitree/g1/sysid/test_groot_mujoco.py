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

"""Guards on the composed model: silent name/index drift poisons the policy obs."""

import mujoco
import numpy as np
import pytest

from dimos.robot.unitree.g1.sysid.groot_mujoco import (
    DEFAULT_29,
    NUM_MOTORS,
    build_model,
    name2id,
    touchdown_z,
    world_T_pelvis,
)


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return build_model(ghost=True)


def test_joint_and_actuator_layout(model: mujoco.MjModel) -> None:
    """qpos[7:36]/qvel[6:35] must be the 29 DDS motors behind one freejoint."""
    assert model.nu == NUM_MOTORS
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert model.jnt_qposadr[1] == 7
    assert model.nq == 7 + NUM_MOTORS


def test_names_survive_scene_attach(model: mujoco.MjModel) -> None:
    """attach() prefixes names unless prefix=''; a -1 lookup would feed the
    policy the accelerometer instead of the gyro."""
    assert name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu-angular-velocity") >= 0
    assert name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ghost") >= 0
    with pytest.raises(KeyError):
        name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "nope")


def test_ghost_is_visual_only(model: mujoco.MjModel) -> None:
    gid = name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ghost_box")
    assert model.geom_contype[gid] == 0
    assert model.geom_conaffinity[gid] == 0
    assert model.nmocap == 1


def test_stands_at_default_pose(model: mujoco.MjModel) -> None:
    """The policy's zero-offset pose must be a standing pose, not a heap."""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[7 : 7 + NUM_MOTORS] = DEFAULT_29
    mujoco.mj_forward(model, data)
    assert data.qpos[2] > 0.7


def test_world_T_pelvis_is_rigid() -> None:
    class Odom:
        x, y, z = 1.0, 2.0, 3.0
        orientation = type("Q", (), {"to_rotation_matrix": staticmethod(lambda: np.eye(3))})()

    T = world_T_pelvis(Odom())
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)
    assert T[3, 3] == 1.0


def test_touchdown_z_brackets_first_contact(model: mujoco.MjModel) -> None:
    """Spawn height must be measured, not inherited from the MJCF: a plant with
    different foot geometry would otherwise get a different landing transient."""
    data = mujoco.MjData(model)
    z = touchdown_z(model, data)
    assert 0.6 < z < float(model.body_pos[1][2])

    def ncon_at(height: float) -> int:
        mujoco.mj_resetData(model, data)
        data.qpos[7 : 7 + NUM_MOTORS] = DEFAULT_29
        data.qpos[2] = height
        mujoco.mj_forward(model, data)
        return int(data.ncon)

    assert ncon_at(z - 1e-3) > 0, "just below touchdown must touch the floor"
    assert ncon_at(z + 1e-3) == 0, "just above touchdown must be clear"
    assert ncon_at(z + 0.01) == 0, "the 1 cm drop must start in the air"
