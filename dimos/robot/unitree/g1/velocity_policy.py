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

"""G1 velocity-policy surrogate driven only by engine-independent robot state.

This is the existing learned locomotion policy, not Unitree firmware or GR00T.
The physics backend owns all integration, sensing and actuator writes.
"""

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.physics import RobotState
from dimos.simulation.policy import POLICY_ACTION_SCALE, POLICY_CONTROL_DT, OnnxPolicy

G1_PHYSICS_DT = 0.002
G1_DRIFT_COMPENSATION = (-0.18, 0.0, -0.09)


class G1VelocityController:
    def __init__(
        self,
        dt: float,
        *,
        policy_path: str,
        default_angles: Sequence[float] | None = None,
        control_dt: float = POLICY_CONTROL_DT,
        action_scale: float = POLICY_ACTION_SCALE,
        drift_compensation: Sequence[float] = G1_DRIFT_COMPENSATION,
    ) -> None:
        n_substeps = round(control_dt / dt)
        if n_substeps < 1 or not np.isclose(n_substeps * dt, control_dt):
            raise ValueError(
                "G1 velocity policy requires a physics timestep dividing its control period"
            )
        self._inference = OnnxPolicy(policy_path)
        self._n_substeps = n_substeps
        self._action_scale = action_scale
        self._drift_compensation = np.asarray(drift_compensation, dtype=np.float32)
        self._home = (
            None if default_angles is None else np.asarray(default_angles, dtype=np.float64).copy()
        )
        self._phase_dt = 2 * np.pi * 1.5 * control_dt
        self._count = 0
        self._last_action = np.zeros(29, dtype=np.float32)
        self._phase = np.array([0.0, np.pi])
        self._targets = None if self._home is None else self._home.copy()

    def reset(self) -> None:
        """Clear policy history while retaining its original reference posture."""
        self._count = 0
        self._last_action.fill(0)
        self._phase = np.array([0.0, np.pi])
        self._targets = None if self._home is None else self._home.copy()

    def observation(self, state: RobotState, command: Twist) -> NDArray[np.float32]:
        if len(state.position) != 29 or len(state.velocity) != 29:
            raise ValueError("G1 policy requires the 29 motors in its declared order")
        if state.imu is None:
            raise ValueError("G1 velocity policy requires its pelvis IMU state")
        if self._home is None:
            # The first sample is read before integration from the model's
            # authored reset posture. A pause never changes this reference.
            self._home = np.asarray(state.position, dtype=np.float64).copy()
            self._targets = self._home.copy()
        gravity = Rotation.from_quat(state.imu.orientation).inv().apply([0.0, 0.0, -1.0])
        velocity = (
            np.asarray(
                [2 * command.linear.x, 2 * command.linear.y, command.angular.z], dtype=np.float32
            )
            + self._drift_compensation
        )
        return np.concatenate(
            (
                state.imu.linear_velocity,
                state.imu.angular_velocity,
                gravity,
                velocity,
                np.asarray(state.position) - self._home,
                state.velocity,
                self._last_action,
                np.cos(self._phase),
                np.sin(self._phase),
            )
        ).astype(np.float32)

    def step(self, state: RobotState, command: Twist) -> MotorCommandArray:
        observation = self.observation(state, command)
        self._count += 1
        if self._count % self._n_substeps == 0:
            action = self._inference.predict(observation)
            if action.shape != (29,) or not np.isfinite(action).all():
                raise ValueError("G1 policy returned an invalid 29-motor action")
            self._last_action = action.copy()
            assert self._home is not None
            self._targets = action * self._action_scale + self._home
            self._phase = np.fmod(self._phase + self._phase_dt + np.pi, 2 * np.pi) - np.pi
        assert self._targets is not None
        return MotorCommandArray(q=self._targets.tolist())
