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

"""robosuite controller for the motor command contract used by DimOS connections."""

from collections.abc import Sequence
from typing import Protocol

import mujoco
import numpy as np
from numpy.typing import NDArray
from robosuite.controllers.parts.controller import Controller
from robosuite.utils.binding_utils import MjSim


class MotorModel(Protocol):
    """Actuator names needed by the shared scalar motor controller."""

    @property
    def actuators(self) -> Sequence[str]: ...

    @property
    def naming_prefix(self) -> str: ...


class MotorController(Controller):
    """Apply q/dq/kp/kd/tau in the model's actuator order, without gravity compensation.

    Anatomy and actuator behavior come from the compiled model. ``scale`` and
    ``offset`` only translate public position units to model joint coordinates.
    Native position servos accept q alone and retain their authored gains.
    """

    def __init__(
        self,
        sim: MjSim,
        robot_model: MotorModel,
        *,
        scale: Sequence[float] | None = None,
        offset: Sequence[float] | None = None,
    ) -> None:
        model = sim.model
        self.actuators = np.asarray(
            [model.actuator_name2id(name) for name in robot_model.actuators], dtype=np.int64
        )
        if not self.actuators.size:
            raise ValueError("a motor controller requires at least one actuator")
        if np.any(model.actuator_trntype[self.actuators] != mujoco.mjtTrn.mjTRN_JOINT):
            raise ValueError("motor commands require joint-transmission actuators")
        joints = model.actuator_trnid[self.actuators, 0]
        if (
            len(set(joints)) != len(joints)
            or not np.isin(
                model.jnt_type[joints], (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
            ).all()
        ):
            raise ValueError("motor commands require one actuator per scalar joint")
        self.qpos = np.asarray(model.jnt_qposadr[joints], dtype=np.int64)
        self.dofs = np.asarray(model.jnt_dofadr[joints], dtype=np.int64)
        count = len(joints)
        gain = model.actuator_gainprm[self.actuators, 0]
        bias = model.actuator_biasprm[self.actuators]
        bias_type = model.actuator_biastype[self.actuators]
        self.position_servo = (
            (bias_type == mujoco.mjtBias.mjBIAS_AFFINE)
            & (bias[:, 0] == 0)
            & np.isclose(bias[:, 1], -gain)
            & (bias[:, 2] <= 0)
        )
        effort_motor = bias_type == mujoco.mjtBias.mjBIAS_NONE
        self.gear = np.asarray(model.actuator_gear[self.actuators, 0], dtype=np.float64)
        if (
            np.any(model.actuator_gaintype[self.actuators] != mujoco.mjtGain.mjGAIN_FIXED)
            or not np.all(self.position_servo | effort_motor)
            or np.any(gain <= 0)
            or np.any(self.gear == 0)
        ):
            raise ValueError("motor commands support effort motors and native position servos")
        self._effort_scale = gain * self.gear
        limited = model.actuator_ctrllimited[self.actuators].astype(bool)
        ranges = model.actuator_ctrlrange[self.actuators]
        minimum = np.where(limited, ranges[:, 0], -np.inf)
        maximum = np.where(limited, ranges[:, 1], np.inf)
        super().__init__(
            sim,
            joint_indexes={"joints": joints, "qpos": self.qpos, "qvel": self.dofs},
            actuator_range=(minimum, maximum),
            naming_prefix=robot_model.naming_prefix,
        )
        self.control_dim = 5 * count
        self._goal = np.zeros((5, count))
        self.set_motor_units(scale=scale, offset=offset)

    def set_motor_units(
        self, *, scale: Sequence[float] | None = None, offset: Sequence[float] | None = None
    ) -> None:
        """Bind public motor units and hold the current pose in those units."""
        count = len(self.actuators)
        scales = np.ones(count) if scale is None else np.array(scale, dtype=np.float64, copy=True)
        offsets = (
            np.zeros(count) if offset is None else np.array(offset, dtype=np.float64, copy=True)
        )
        if (
            scales.shape != (count,)
            or offsets.shape != (count,)
            or not np.isfinite(scales).all()
            or not np.isfinite(offsets).all()
            or np.any(scales == 0)
        ):
            raise ValueError("motor unit conversions require finite offsets and nonzero scales")
        self.scale, self.offset = scales, offsets
        self.reset_goal()

    @property
    def positions(self) -> NDArray[np.float64]:
        return np.asarray((self.sim.data.qpos[self.qpos] - self.offset) / self.scale)

    @property
    def velocities(self) -> NDArray[np.float64]:
        return np.asarray(self.sim.data.qvel[self.dofs] / self.scale)

    def update(self, force: bool = False) -> None:
        """Raw motor PD needs joint feedback, without Jacobians or a mass matrix."""
        if force:
            self.sim.forward()
        self.joint_pos = np.asarray(self.sim.data.qpos[self.qpos]).copy()
        self.joint_vel = np.asarray(self.sim.data.qvel[self.dofs]).copy()
        self.new_update = False

    def set_goal(self, action: NDArray[np.float64]) -> None:
        """Accept rows q, dq, kp, kd, tau; invalid actions preserve the previous goal."""
        values = np.asarray(action, dtype=np.float64)
        if values.shape not in ((self.control_dim,), self._goal.shape):
            raise ValueError("motor action must contain five rows in actuator order")
        values = values.reshape(self._goal.shape)
        if not np.isfinite(values).all() or np.any(values[2:4] < 0):
            raise ValueError("motor commands require finite values and nonnegative gains")
        if np.any(values[1:, self.position_servo] != 0):
            raise ValueError("native position servos accept position targets only")
        self._goal = values.copy()

    def reset_goal(self) -> None:
        self._goal.fill(0)
        self._goal[0] = self.positions

    def run_controller(self) -> NDArray[np.float64]:
        self.update()
        q, dq, kp, kd, tau = self._goal
        target = q * self.scale + self.offset
        effort = kp * (target - self.joint_pos) + kd * (dq * self.scale - self.joint_vel) + tau
        controls = np.where(self.position_servo, target * self.gear, effort / self._effort_scale)
        self.torques = np.clip(controls, self.actuator_min, self.actuator_max)
        super().run_controller()
        return np.asarray(self.torques, dtype=np.float64)

    @property
    def name(self) -> str:
        return "DIMOS_MOTOR"
