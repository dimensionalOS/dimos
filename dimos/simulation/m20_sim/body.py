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

"""The vendor M20 model in MuJoCo, walking and rolling under its locomotion policy."""

from __future__ import annotations

from collections.abc import Callable

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.msgs.sim_msgs.Contacts import Part
from dimos.simulation.go2_legged.policy import Proprioception
from dimos.simulation.go2_legged.robot import (
    BASE_ANGULAR_VELOCITY,
    BASE_LINEAR_VELOCITY,
    BASE_QUAT,
    projected_gravity,
    yaw_of,
)
from dimos.simulation.m20_sim.policy import LEG_JOINTS, LEGS, M20Policy
from dimos.utils.data import get_data

CONTROL_DT = 0.02
MODEL = DIMOS_PROJECT_ROOT / "dimos" / "robot" / "deeprobotics" / "m20" / "assets" / "m20.xml"


def m20_spec() -> mujoco.MjSpec:
    """An editable spec of the vendor M20 with no ground, for adding a scene to."""
    spec = mujoco.MjSpec.from_file(str(MODEL))
    spec.meshdir = str(get_data("m20_sdk") / "meshes")
    return spec


class LeggedM20:
    """An M20 in a compiled model, stepped one policy tick at a time from a velocity command."""

    dt = CONTROL_DT

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, policy: M20Policy) -> None:
        self.model = model
        self.data = data
        self.policy = policy
        self.trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        wheel_bodies = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_wheel") for leg in LEGS
        ]
        self.wheels = tuple(
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] in wheel_bodies and model.geom_contype[g]
        )
        self.substeps = max(1, round(CONTROL_DT / model.opt.timestep))
        joints = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in policy.joint_names
        ]
        self._qpos_adr = np.array([model.jnt_qposadr[j] for j in joints])
        self._dof_adr = np.array([model.jnt_dofadr[j] for j in joints])
        actuator_of = {int(model.actuator_trnid[a, 0]): a for a in range(model.nu)}
        self._actuator_ids = np.array([actuator_of[j] for j in joints])
        self._torque_limit = model.actuator_ctrlrange[self._actuator_ids, 1]
        self._target = policy.default_pose.copy()

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None:
        """Stand in the policy's default pose with the wheels on z_feet, at rest."""
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)  # type: ignore[attr-defined]
        d.qpos[0:2] = [x, y]
        d.qpos[BASE_QUAT] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        d.qpos[self._qpos_adr] = self.policy.default_pose
        mujoco.mj_forward(m, d)
        lowest = min(float(d.geom_xpos[g][2] - m.geom_size[g][0]) for g in self.wheels)
        d.qpos[2] += z_feet - lowest
        mujoco.mj_forward(m, d)
        self._target = self.policy.default_pose.copy()
        self.policy.reset()

    def tick(
        self, command: NDArray[np.float64], on_substep: Callable[[], None] | None = None
    ) -> None:
        """Advance one policy tick toward a velocity command."""
        self._target = self.policy.act(self._observe(), command)
        position = np.where(np.arange(len(self._target)) < LEG_JOINTS, self._target, 0.0)
        velocity = self._target - position
        kp, kd = self.policy.kp, self.policy.kd
        d = self.data
        for _ in range(self.substeps):
            q, dq = d.qpos[self._qpos_adr], d.qvel[self._dof_adr]
            tau = kp * (position - q) + kd * (velocity - dq)
            d.ctrl[self._actuator_ids] = np.clip(tau, -self._torque_limit, self._torque_limit)
            mujoco.mj_step(self.model, d)
            if on_substep is not None:
                on_substep()

    def _observe(self) -> Proprioception:
        d = self.data
        return Proprioception(
            angular_velocity=d.qvel[BASE_ANGULAR_VELOCITY].copy(),
            gravity=projected_gravity(d.qpos[BASE_QUAT]),
            joint_position=d.qpos[self._qpos_adr].copy(),
            joint_velocity=d.qvel[self._dof_adr].copy(),
        )

    def base_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.data.xpos[self.trunk].copy(), self.data.xmat[self.trunk].reshape(3, 3).copy()

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.policy.joint_names

    def joint_state(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Position and velocity of every leg and wheel joint, in the policy's joint order."""
        return self.data.qpos[self._qpos_adr].copy(), self.data.qvel[self._dof_adr].copy()

    def base_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the world frame and angular velocity in the base frame."""
        return self.data.qvel[BASE_LINEAR_VELOCITY].copy(), self.data.qvel[
            BASE_ANGULAR_VELOCITY
        ].copy()

    def part(self, geom: int) -> Part:
        if geom in self.wheels:
            return "wheel"
        if self.model.geom_bodyid[geom] == self.trunk:
            return "trunk"
        return "leg"

    def yaw(self) -> float:
        return yaw_of(self.data.qpos[BASE_QUAT])

    def upright(self) -> float:
        """z of the trunk's up axis, 1 when level."""
        return float(self.data.xmat[self.trunk][8])
