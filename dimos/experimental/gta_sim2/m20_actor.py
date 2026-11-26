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

"""Explicit M20 actor profiles, independent of a simulator or transport.

Source: DeepRoboticsLab/sdk_deploy at 2367375f922f2a739f7b578849ae2affd4d8660d.
DreamWaQ source: Lain-Ego0/ArenaX at 438c7c5cb704ebdf01fdc60a2a50ec9a8bb4239d.
Neither checkpoint passed the 10 cm stair fixture in this experiment. Other checkpoints must be
matched to their own observation/action contract rather than selected by shape.
"""

from pathlib import Path
from typing import Literal

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

LEGS = ("fl", "fr", "hl", "hr")
JOINTS = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in ("hipx", "hipy", "knee")) + tuple(
    f"{leg}_wheel_joint" for leg in LEGS
)
HOME = np.array([0.0, -0.3, 0.6] * 2 + [0.0, 0.3, -0.6] * 2 + [0.0] * 4)
KP = np.array([80.0] * 12 + [0.0] * 4)
KD = np.array([2.0] * 12 + [0.6] * 4)
SCALE = np.array([0.125, 0.25, 0.25] * 4 + [5.0] * 4)
Profile = Literal["sdk57", "dreamwaq342"]
INTERLEAVED = np.array(
    [
        JOINTS.index(f"{leg}_{part}_joint")
        for leg in LEGS
        for part in ("hipx", "hipy", "knee", "wheel")
    ]
)


def home_pose(profile: Profile) -> NDArray[np.float64]:
    if profile == "sdk57":
        return HOME.copy()
    if profile != "dreamwaq342":
        raise ValueError(f"Unknown M20 profile: {profile}")
    return np.array([0.0, -0.6, 1.0] * 2 + [0.0, 0.6, -1.0] * 2 + [0.0] * 4)


class M20Actor:
    def __init__(self, path: Path, profile: Profile = "sdk57") -> None:
        self.profile = profile
        self.home = home_pose(profile)
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        inputs, outputs = self._session.get_inputs(), self._session.get_outputs()
        if (
            len(inputs) != 1
            or inputs[0].shape[-1] != (57 if profile == "sdk57" else 342)
            or len(outputs) != 1
            or outputs[0].shape != [1, 16]
        ):
            raise ValueError(f"Checkpoint shape does not match {profile}")
        self._input = inputs[0].name
        self._history = np.zeros(16)
        self._observations = np.zeros((6, 57), dtype=np.float32)

    def reset(self) -> None:
        self._history.fill(0)
        self._observations.fill(0)

    def act(
        self,
        q: NDArray[np.float64],
        dq: NDArray[np.float64],
        gyro: NDArray[np.float64],
        gravity: NDArray[np.float64],
        command: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        offsets = q - self.home
        offsets[12:] = 0
        if self.profile == "sdk57":
            obs = np.concatenate((gyro * 0.25, gravity, command, offsets, dq * 0.05, self._history))
        else:
            obs = np.concatenate(
                (
                    command * [2, 2, 0.25],
                    gyro * 0.25,
                    gravity,
                    offsets[INTERLEAVED],
                    dq[INTERLEAVED] * 0.05,
                    self._history,
                )
            )
        if obs.shape != (57,) or not np.isfinite(obs).all():
            raise ValueError("Invalid M20 policy observation")
        if self.profile == "dreamwaq342":
            self._observations[:-1] = self._observations[1:]
            self._observations[-1] = obs
            policy_input = self._observations.ravel()
        else:
            policy_input = obs.astype(np.float32)
        action = np.asarray(
            self._session.run(None, {self._input: policy_input[None]})[0][0],
            dtype=np.float64,
        )
        if action.shape != (16,) or not np.isfinite(action).all():
            raise ValueError("Invalid M20 policy action")
        self._history = action
        if self.profile == "dreamwaq342":
            action = action[np.argsort(INTERLEAVED)]
        scale = SCALE if self.profile == "sdk57" else np.r_[np.full(12, 0.25), np.full(4, 5)]
        target = self.home + action * scale
        return np.r_[target[:12], np.zeros(4)], np.r_[np.zeros(12), target[12:]]
