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

"""Booster's K1 walking actor, including its observation and action history."""

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import torch

from dimos.hardware.whole_body.spec import IMUState
from dimos.robot.booster.k1.control_config import HOME, POLICY_INDICES


class K1Policy:
    def __init__(self, model_path: Path) -> None:
        self.model_path = model_path
        self._actor: torch.jit.ScriptModule | None = None
        self._home = torch.tensor(HOME, dtype=torch.float32)
        self._indices = torch.tensor(POLICY_INDICES)
        self._history: torch.Tensor | None = None
        self._last_action = torch.zeros(20)
        self._targets = self._home.clone()

    def start(self) -> None:
        model = torch.jit.load(str(self.model_path), map_location="cpu").eval()  # type: ignore[no-untyped-call]
        self._actor = model.actor
        # TorchScript optimizes its first executions. Do that before physics starts.
        for _ in range(3):
            self.step(
                np.asarray(HOME, dtype=np.float32),
                np.zeros(22, dtype=np.float32),
                IMUState((1, 0, 0, 0), (0, 0, 0), (0, 0, 0)),
                np.zeros(3, dtype=np.float32),
            )
        self.reset()

    def reset(self) -> None:
        self._history = None
        self._last_action.zero_()
        self._targets.copy_(self._home)

    def step(
        self,
        positions: NDArray[np.float32],
        velocities: NDArray[np.float32],
        imu: IMUState,
        command: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        if self._actor is None:
            raise RuntimeError("K1 policy must be started before inference")
        if positions.shape != (22,) or velocities.shape != (22,) or command.shape != (3,):
            raise ValueError("K1 requires 22 serial joint states and a three-axis velocity")
        quat = np.asarray(imu.quaternion)
        if not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat), 1, atol=0.01):
            raise ValueError("K1 requires a finite unit orientation quaternion")
        w, x, y, z = quat
        gravity = (2 * (w * y - x * z), -2 * (y * z + w * x), -(w * w - x * x - y * y + z * z))
        if gravity[2] > -0.5:
            raise ValueError("K1 tilt exceeds the released policy's 60-degree cutoff")
        observation = torch.cat(
            (
                torch.tensor(imu.gyroscope, dtype=torch.float32),
                torch.tensor(gravity, dtype=torch.float32),
                torch.as_tensor(command, dtype=torch.float32),
                (torch.as_tensor(positions) - self._home)[self._indices],
                torch.as_tensor(velocities)[self._indices] * 0.1,
                self._last_action,
            )
        )
        if not torch.isfinite(observation).all():
            raise ValueError("K1 observations must be finite")
        observation.clamp_(-100, 100)
        if self._history is None:
            self._history = observation.repeat(10, 1)
        else:
            self._history = self._history.roll(-1, dims=0)
            self._history[-1] = observation
        with torch.inference_mode():
            action = self._actor(self._history.flatten()).reshape(-1)
        if action.shape != (20,) or not torch.isfinite(action).all():
            raise ValueError("K1 actor must return 20 finite actions")
        self._last_action.copy_(action.clamp(-100, 100))
        offset = torch.zeros(22)
        offset[self._indices] = self._last_action
        # This correction is part of Booster's released checkpoint contract.
        offset[8] -= 0.2
        target = self._home + offset * 0.25
        self._targets.lerp_(target, 0.8)
        return self._targets.numpy().copy()
