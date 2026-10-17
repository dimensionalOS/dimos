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

"""The Go2 in MuJoCo walking under a locomotion policy, with physics fitted to real runs."""

from __future__ import annotations

from collections.abc import Callable

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.simulation.go2_legged.policy import LEGS, Go2Policy, Proprioception
from dimos.utils.data import get_data

CONTROL_DT = 0.02
COMMAND_SLEW = np.array([0.05, 0.04, 0.10])
LEG_DOFS = slice(6, 18)
BASE_QUAT = slice(3, 7)
BASE_LINEAR_VELOCITY = slice(0, 3)
BASE_ANGULAR_VELOCITY = slice(3, 6)

FITTED_PHYSICS = {
    "armature": 0.00712,
    "damping": 0.2850,
    "frictionloss": 0.3650,
    "trunk_mass_scale": 0.9412,
    "trunk_inertia_scale": 0.8601,
    "foot_friction": 0.7860,
    "foot_friction_torsional": 0.006134,
    "trunk_com_x": -0.006845,
    "leg_mass_scale": 1.616,
}
FITTED_ACTUATOR_TAU = 0.01510


def go2_spec() -> mujoco.MjSpec:
    """An editable spec of the menagerie Go2 with no ground, for adding a scene to."""
    return mujoco.MjSpec.from_file(str(get_data("go2_sim") / "unitree_go2" / "go2.xml"))


def apply_fitted_physics(model: mujoco.MjModel) -> None:
    # the fit was made on the compiled model without mj_setConst, so this must not call it
    p = FITTED_PHYSICS
    model.dof_armature[LEG_DOFS] = p["armature"]
    model.dof_damping[LEG_DOFS] = p["damping"]
    model.dof_frictionloss[LEG_DOFS] = p["frictionloss"]
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    model.body_mass[trunk] *= p["trunk_mass_scale"]
    model.body_inertia[trunk] *= p["trunk_inertia_scale"]
    model.body_ipos[trunk][0] += p["trunk_com_x"]
    for leg in LEGS:
        for part in ("thigh", "calf"):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_{part}")
            model.body_mass[body] *= p["leg_mass_scale"]
            model.body_inertia[body] *= p["leg_mass_scale"]
        foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)
        model.geom_friction[foot, 0] = p["foot_friction"]
        model.geom_friction[foot, 1] = p["foot_friction_torsional"]


def projected_gravity(quat_wxyz: NDArray[np.float64]) -> NDArray[np.float64]:
    w, x, y, z = quat_wxyz
    return np.array([-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))])


def yaw_of(quat_wxyz: NDArray[np.float64]) -> float:
    w, x, y, z = quat_wxyz
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class LeggedGo2:
    """A Go2 in a compiled model, stepped one policy tick at a time from a velocity command."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, policy: Go2Policy) -> None:
        self.model = model
        self.data = data
        self.policy = policy
        self.trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.feet = tuple(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg) for leg in LEGS)
        self.substeps = max(1, round(CONTROL_DT / model.opt.timestep))
        joints = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in policy.joint_names
        ]
        self._qpos_adr = np.array([model.jnt_qposadr[j] for j in joints])
        self._dof_adr = np.array([model.jnt_dofadr[j] for j in joints])
        actuator_of = {int(model.actuator_trnid[a, 0]): a for a in range(model.nu)}
        self._actuator_ids = np.array([actuator_of[j] for j in joints])
        self._torque_limit = model.actuator_ctrlrange[self._actuator_ids, 1]
        self._home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self._command = np.zeros(3)
        self._applied = np.zeros(len(joints))
        self._target = policy.default_pose.copy()

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None:
        """Stand in the policy's default pose with the feet on z_feet, at rest."""
        m, d = self.model, self.data
        mujoco.mj_resetDataKeyframe(m, d, self._home)
        d.qpos[0:2] = [x, y]
        d.qpos[BASE_QUAT] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        d.qpos[self._qpos_adr] = self.policy.default_pose
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        lowest = min(float(d.geom_xpos[g][2] - m.geom_size[g][0]) for g in self.feet)
        d.qpos[2] += z_feet - lowest
        mujoco.mj_forward(m, d)
        self._command = np.zeros(3)
        self._applied = np.zeros(len(self._applied))
        self._target = self.policy.default_pose.copy()
        self.policy.reset()

    def tick(
        self, command: NDArray[np.float64], on_substep: Callable[[], None] | None = None
    ) -> None:
        """Advance one policy tick toward a velocity command."""
        self._command += np.clip(command - self._command, -COMMAND_SLEW, COMMAND_SLEW)
        self._target = self.policy.act(self._observe(), self._command)
        kp, kd = self.policy.kp, self.policy.kd
        d = self.data
        dt = self.model.opt.timestep
        alpha = dt / (FITTED_ACTUATOR_TAU + dt)
        for _ in range(self.substeps):
            tau = kp * (self._target - d.qpos[self._qpos_adr]) - kd * d.qvel[self._dof_adr]
            tau = np.clip(tau, -self._torque_limit, self._torque_limit)
            self._applied += alpha * (tau - self._applied)
            d.ctrl[self._actuator_ids] = self._applied
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

    def base_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the world frame and angular velocity in the base frame."""
        return self.data.qvel[BASE_LINEAR_VELOCITY].copy(), self.data.qvel[
            BASE_ANGULAR_VELOCITY
        ].copy()

    def yaw(self) -> float:
        return yaw_of(self.data.qpos[BASE_QUAT])

    def upright(self) -> float:
        """z of the trunk's up axis, 1 when level."""
        return float(self.data.xmat[self.trunk][8])
