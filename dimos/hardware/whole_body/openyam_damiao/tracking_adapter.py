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

"""Opt-in holding correction shared by single and dual OpenYAM adapters."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
from importlib.metadata import version
import time
from typing import Any

import numpy as np

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.openyam_damiao.tracking import (
    J3_MOTION_COEFFICIENTS,
    BoundedHoldingCorrection,
    J3MotionCorrection,
)
from dimos.hardware.whole_body.openyam_damiao.trajectory import CosineJointTrajectory
from dimos.hardware.whole_body.spec import POS_STOP, MotorCommand, MotorState


class OpenYamTrackingMixin(DamiaoWholeBodyAdapter):
    """Add independently stateful holding correction for each OpenYAM arm."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._holding_corrections = {name: BoundedHoldingCorrection() for name in self.arm_joints}
        self._motion_corrections: dict[str, J3MotionCorrection] = {}

    def connect(self) -> bool:
        if self._runtime_config.tracking_comp and not self._runtime_config.openyam_anvil_parity:
            raise RuntimeError("OpenYAM tracking requires the anvil parity motor codec")
        if self._runtime_config.openyam_anvil_parity and tuple(
            int(part) for part in version("can-motor-control").split(".")[:2]
        ) < (0, 1):
            raise RuntimeError("Install the tested can-motor-control 0.1.0 parity build")
        return super().connect()

    def activate(self) -> bool:
        activated = super().activate()
        if activated:
            # The first trial refresh otherwise follows mode/enable frames too closely
            # for the tested SocketCAN queue of ten frames.
            time.sleep(0.01)
        return activated

    def _command_velocity_scale(self, index: int) -> float:
        return 0.8 if self._runtime_config.openyam_anvil_parity and index < 3 else 1.0

    def read_motor_states(self) -> list[MotorState]:
        states = super().read_motor_states()
        if self._runtime_config.openyam_anvil_parity:
            count = sum(len(joints) for joints in self.arm_joints.values())
            states = [
                replace(state, dq=state.dq * 1.25) if i < count and i % 6 < 3 else state
                for i, state in enumerate(states)
            ]
            self._last_arm_states = states[:count]
        return states

    def begin_j3_motion(self, arm: str, side: str, direction: int) -> None:
        """Start a local J3 segment, preserving the learned holding torque."""
        if not self._active or not self._runtime_config.tracking_comp:
            raise RuntimeError("Activate tracking compensation before starting J3 motion")
        if arm not in self.arm_joints or side not in J3_MOTION_COEFFICIENTS:
            raise ValueError("Unknown arm or model side")
        if len(self.arm_joints) == 2 and arm != side + "_arm":
            raise ValueError("Model side does not match arm")
        policy = self._motion_corrections.setdefault(
            arm, J3MotionCorrection(J3_MOTION_COEFFICIENTS[side])
        )
        if policy.active:
            raise RuntimeError("J3 motion is already active")
        policy.begin(float(self._holding_corrections[arm].bias[2]), direction)

    def end_j3_motion(self, arm: str) -> None:
        """Transfer the delivered moving torque back into the holding integrator."""
        policy = self._motion_corrections.get(arm)
        if policy is not None and policy.active:
            holding = self._holding_corrections[arm]
            holding.bias[2] = policy.end()
            holding.holding[2] = False

    def rebase_j3_reference(
        self, arm: str, kp: float, old_reference: float, measured: float
    ) -> float:
        """Align a temporary reference while preserving its P plus I torque."""
        if not np.isfinite([kp, old_reference, measured]).all() or kp <= 0:
            raise ValueError("Invalid reference alignment")
        holding = self._holding_corrections[arm]
        bias = float(np.clip(holding.bias[2] + kp * (old_reference - measured), -5.0, 5.0))
        reference = old_reference - (bias - holding.bias[2]) / kp
        if not 0.16 <= reference <= 0.40:
            raise ValueError("Rebased reference is outside the measured envelope")
        holding.bias[2] = bias
        holding.holding[2] = False
        return float(reference)

    def execute_j3_trajectory(
        self,
        arm: str,
        side: str,
        commands: list[MotorCommand],
        target: float,
        speed: float,
        check_path: Callable[[list[MotorCommand], list[MotorCommand]], None],
        observe: Callable[[list[MotorState]], None],
    ) -> None:
        """Execute C's cosine profile on an active adapter at 100 Hz.

        check_path must reject collisions/table clearance before movement. observe
        runs after every feedback read and may abort for workspace, temperature,
        cancellation or other deployment conditions. The caller owns recovery.
        """
        if self.gripper_joints:
            raise RuntimeError("J3 motion requires the arm-only OpenYAM adapter")
        index = list(self.arm_joints).index(arm) * 6 + 2
        arm_count = sum(len(joints) for joints in self.arm_joints.values())
        commands = [
            command if i < arm_count else replace(command, q=POS_STOP)
            for i, command in enumerate(commands)
        ]
        trajectory = CosineJointTrajectory(commands[index].q, target, speed)
        final = commands.copy()
        final[index] = replace(commands[index], q=target, dq=0.0, ddq=0.0)
        check_path(commands, final)
        states = self.read_motor_states()
        observe(states)
        if abs(states[index].q - commands[index].q) > np.radians(0.3):
            raise ValueError("Prepare the starting pose to within 0.3 degrees")
        self.begin_j3_motion(arm, side, 1 if target > commands[index].q else -1)
        begun = time.monotonic()
        next_tick = begun
        try:
            while True:
                elapsed = time.monotonic() - begun
                q, dq, ddq = trajectory.sample(elapsed)
                current = commands.copy()
                current[index] = replace(commands[index], q=q, dq=dq, ddq=ddq)
                states = self.read_motor_states()
                observe(states)
                arm_count = sum(len(joints) for joints in self.arm_joints.values())
                if (
                    max(
                        abs(state.q - command.q)
                        for state, command in zip(
                            states[:arm_count], current[:arm_count], strict=True
                        )
                    )
                    > 0.15
                ):
                    raise RuntimeError("Position tracking error exceeded 0.15 rad")
                for name, joints in self.arm_joints.items():
                    for j in range(len(joints)):
                        motor = self._arms[name].motor_at(j)
                        if not motor.is_enabled or motor.fault is not None:
                            raise RuntimeError("Motor is disabled or faulted")
                        if max(motor.temperature_mos, motor.temperature_rotor) >= 53:
                            raise RuntimeError("Motor temperature margin reached")
                if not self.write_motor_commands(current):
                    raise RuntimeError("J3 trajectory command was rejected")
                if elapsed >= trajectory.duration:
                    break
                next_tick += 0.01
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            self.end_j3_motion(arm)

    def _tracking_torques(self, commands: list[MotorCommand], gravity: np.ndarray) -> np.ndarray:
        if not self._runtime_config.tracking_comp:
            return np.zeros(len(commands), dtype=np.float64)
        now = time.monotonic()
        recently_read = (
            self._last_arm_state_time is not None
            and 0 <= now - self._last_arm_state_time <= 0.05
            and len(self._last_arm_states) == len(commands)
        )
        if not recently_read:
            active_motion = any(policy.active for policy in self._motion_corrections.values())
            self._reset_tracking()
            if active_motion:
                raise RuntimeError("Feedback is stale during J3 motion")
            self._last_tracking_command_time = now
            return np.zeros(len(commands), dtype=np.float64)
        dt = (
            now - self._last_tracking_command_time
            if self._last_tracking_command_time is not None
            else 0.01
        )
        self._last_tracking_command_time = now
        result = np.zeros(len(commands), dtype=np.float64)
        offset = 0
        for name, joints in self.arm_joints.items():
            count = len(joints)
            if count != 6:
                raise ValueError("OpenYAM holding policy requires six arm joints")
            group_commands = commands[offset : offset + count]
            states = self._last_arm_states[offset : offset + count]
            target = np.array([command.q for command in group_commands])
            measured = np.array([state.q for state in states])
            velocity = np.array([state.dq for state in states])
            target_velocity = np.array([command.dq for command in group_commands])
            motion = self._motion_corrections.get(name)
            moving = motion is not None and motion.active
            holding_error = target - measured
            if moving:
                holding_error[2] = 0.0
            base_torque = np.array(
                [
                    command.kp * (command.q - state.q)
                    + command.kd * (command.dq - state.dq)
                    + command.tau
                    + gravity[offset + index]
                    for index, (command, state) in enumerate(
                        zip(group_commands, states, strict=True)
                    )
                ]
            )
            result[offset : offset + count] = self._holding_corrections[name].update(
                holding_error, velocity, target_velocity, base_torque, dt, eligible=True
            )
            if moving and motion is not None:
                acceleration = group_commands[2].ddq
                if acceleration is None:
                    raise ValueError("Active J3 motion requires desired acceleration")
                result[offset + 2] = motion.update(
                    float(measured[2]),
                    float(target_velocity[2]),
                    acceleration,
                    float(target[2] - measured[2]),
                    float(base_torque[2]),
                    dt,
                )
            offset += count
        return result

    def _reset_tracking(self) -> None:
        super()._reset_tracking()
        for policy in self._holding_corrections.values():
            policy.reset()
        self._motion_corrections.clear()
