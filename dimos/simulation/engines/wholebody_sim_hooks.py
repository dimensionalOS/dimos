# Copyright 2025-2026 Dimensional Inc.
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

"""Pre-step + post-step hooks bridging ``MujocoEngine`` to the whole-body SHM.

Extracted from ``MujocoSimModule`` so the same hook bodies run regardless of
*where* the engine lives:

* ``MujocoSimModule(engine_mode="thread")`` — engine runs on a dimos worker
  thread; hooks fire there.
* ``MujocoSimModule(engine_mode="subprocess")`` — engine runs in a child
  process via ``mujoco_engine.engine_main``; hooks fire in that process's
  main thread.

The same SHM layout serves both modes, so the ``WholeBodyAdapter`` reading
SHM on the dimos side is unaffected by which mode the engine is using.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.engines.mujoco_shm import CMD_MODE_PD_TAU
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.simulation.engines.mujoco_engine import MujocoEngine
    from dimos.simulation.engines.mujoco_shm import ManipShmWriter

logger = setup_logger()


class WholeBodySimHooks:
    """Owns the per-step state bridging engine ↔ whole-body SHM.

    Pre-step:  read motor commands (pos/vel/kp/kd/tau and an optional gripper
               command) from SHM, latch them for PD-with-feedforward, and
               drive the engine actuators accordingly.
    Post-step: write motor states (positions, velocities, efforts) and the
               gripper position back to SHM for the ``WholeBodyAdapter`` to
               consume on the dimos side.
    """

    def __init__(
        self,
        shm: ManipShmWriter,
        dof: int,
        *,
        gripper_idx: int | None = None,
        gripper_ctrl_range: tuple[float, float] = (0.0, 1.0),
        gripper_joint_range: tuple[float, float] = (0.0, 1.0),
    ) -> None:
        self._shm = shm
        self._dof = dof
        self._gripper_idx = gripper_idx
        self._gripper_ctrl_range = gripper_ctrl_range
        self._gripper_joint_range = gripper_joint_range
        # PD-with-feedforward latching: kp/kd come from the controller's
        # WholeBodyConfig (constant across ticks once set) so we keep the
        # last seen values and apply them every step against the live q/dq.
        self._latest_pd_pos_target: np.ndarray | None = None
        self._latest_pd_kp: np.ndarray | None = None
        self._latest_pd_kd: np.ndarray | None = None
        self._latest_pd_tau: np.ndarray | None = None

    def pre_step(self, engine: MujocoEngine) -> None:
        """Pull command targets from SHM into the engine."""
        shm = self._shm
        dof = self._dof

        pos_cmd = shm.read_position_command(dof)
        if pos_cmd is not None:
            if shm.read_command_mode() == CMD_MODE_PD_TAU:
                # Latch position target for the per-step PD computation
                # below; do NOT route through engine.write_joint_command
                # (that would set position-mode and override our tau).
                self._latest_pd_pos_target = pos_cmd
            else:
                engine.write_joint_command(JointState(position=pos_cmd.tolist()))

        vel_cmd = shm.read_velocity_command(dof)
        if vel_cmd is not None:
            engine.write_joint_command(JointState(velocity=vel_cmd.tolist()))

        kp_cmd = shm.read_kp_command(dof)
        if kp_cmd is not None:
            self._latest_pd_kp = kp_cmd
        kd_cmd = shm.read_kd_command(dof)
        if kd_cmd is not None:
            self._latest_pd_kd = kd_cmd
        tau_cmd = shm.read_tau_command(dof)
        if tau_cmd is not None:
            self._latest_pd_tau = tau_cmd

        # Apply latched PD-tau if all four pieces have arrived at least once.
        # Manipulator path (no kp/kd writes) skips this entirely.
        if (
            self._latest_pd_pos_target is not None
            and self._latest_pd_kp is not None
            and self._latest_pd_kd is not None
        ):
            q = np.asarray(engine.joint_positions[:dof], dtype=np.float64)
            dq = np.asarray(engine.joint_velocities[:dof], dtype=np.float64)
            tau_ff = self._latest_pd_tau if self._latest_pd_tau is not None else np.zeros(dof)
            tau = (
                self._latest_pd_kp * (self._latest_pd_pos_target - q)
                + self._latest_pd_kd * (-dq)
                + tau_ff
            )
            engine.write_joint_command(JointState(effort=tau.tolist()))

        if self._gripper_idx is not None:
            gripper_cmd = shm.read_gripper_command()
            if gripper_cmd is not None:
                ctrl_value = self._gripper_joint_to_ctrl(gripper_cmd)
                engine.set_position_target(self._gripper_idx, ctrl_value)

    def post_step(self, engine: MujocoEngine) -> None:
        """Publish joint state (and optionally gripper) back into SHM."""
        shm = self._shm
        shm.write_joint_state(
            positions=engine.joint_positions,
            velocities=engine.joint_velocities,
            efforts=engine.joint_efforts,
        )
        if self._gripper_idx is not None:
            positions = engine.joint_positions
            if self._gripper_idx < len(positions):
                shm.write_gripper_state(positions[self._gripper_idx])

    def _gripper_joint_to_ctrl(self, joint_position: float) -> float:
        """Map joint-space gripper position to actuator control value."""
        jlo, jhi = self._gripper_joint_range
        clo, chi = self._gripper_ctrl_range
        clamped = max(jlo, min(jhi, joint_position))
        if jhi == jlo:
            return clo
        t = (clamped - jlo) / (jhi - jlo)
        return chi - t * (chi - clo)


__all__ = ["WholeBodySimHooks"]
