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

"""The M20 locomotion policy: proprioception and a velocity command in, leg and wheel targets out."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.simulation.go2_legged.policy import Proprioception
from dimos.utils.data import get_data

LEGS = ("fl", "fr", "hl", "hr")
LEG_JOINTS = 12
OBSERVATIONS = 57


class M20Policy:
    """The open sdk_deploy actor: 12 leg position targets, then 4 wheel velocity targets."""

    joint_names = tuple(
        f"{leg}_{part}_joint" for leg in LEGS for part in ("hipx", "hipy", "knee")
    ) + tuple(f"{leg}_wheel_joint" for leg in LEGS)
    default_pose = np.array([0.0, -0.3, 0.6] * 2 + [0.0, 0.3, -0.6] * 2 + [0.0] * 4)
    kp = np.array([80.0] * LEG_JOINTS + [0.0] * 4)
    kd = np.array([2.0] * LEG_JOINTS + [0.6] * 4)
    action_scale = np.array([0.125, 0.25, 0.25] * 4 + [5.0] * 4)
    angular_velocity_scale = 0.25
    joint_velocity_scale = 0.05

    def __init__(self, path: Path) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(
            str(path.expanduser()), options, providers=["CPUExecutionProvider"]
        )
        (inputs,), (outputs,) = self._session.get_inputs(), self._session.get_outputs()
        actions = len(self.joint_names)
        if inputs.shape != [1, OBSERVATIONS] or outputs.shape != [1, actions]:
            raise ValueError(
                f"{path} is not an M20 actor with obs[1,{OBSERVATIONS}] and actions[1,{actions}]"
            )
        self._input = inputs.name
        self._last_action = np.zeros(actions)

    @classmethod
    def load(cls) -> M20Policy:
        return cls(get_data("m20_sdk") / "policy.onnx")

    def reset(self) -> None:
        self._last_action = np.zeros(len(self.joint_names))

    def act(self, obs: Proprioception, command: NDArray[np.float64]) -> NDArray[np.float64]:
        """Leg position and wheel velocity targets for a (vx, vy, vyaw) command."""
        offsets = obs.joint_position - self.default_pose
        offsets[LEG_JOINTS:] = 0.0  # the wheels' angles are not observed
        raw = np.concatenate(
            [
                obs.angular_velocity * self.angular_velocity_scale,
                obs.gravity,
                command,
                offsets,
                obs.joint_velocity * self.joint_velocity_scale,
                self._last_action,
            ]
        )
        (out,) = self._session.run(None, {self._input: raw.astype(np.float32)[None, :]})
        action = out[0].astype(np.float64)
        self._last_action = action
        targets: NDArray[np.float64] = self.default_pose + action * self.action_scale
        return targets
