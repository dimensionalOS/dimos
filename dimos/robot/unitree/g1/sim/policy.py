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

"""The GR00T balance/walk ONNX pair behind one step() call.

The observation contract mirrors :class:`G1GrootWBCTask` verbatim; changing
it drifts the policy away from what it was trained for.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import _DEFAULT_POSITIONS_29

OBS_DIM = 86
OBS_HISTORY = 6
NUM_ACTIONS = 15
NUM_MOTORS = 29
ACTION_SCALE = 0.25
ANG_VEL_SCALE = 0.5
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
CMD_SCALE = np.array([2.0, 2.0, 0.5], dtype=np.float32)
CMD_NORM_THRESHOLD = 0.05
HEIGHT_CMD = 0.74
POLICY_HZ = 50.0
CONTROL_DT = 1.0 / POLICY_HZ

DEFAULT_29 = np.asarray(_DEFAULT_POSITIONS_29, dtype=np.float32)


def projected_gravity(quat_wxyz: NDArray[Any]) -> NDArray[np.float32]:
    w, x, y, z = quat_wxyz
    return np.array(
        [2.0 * (-x * z + w * y), 2.0 * (-y * z - w * x), -(w * w - x * x - y * y + z * z)],
        dtype=np.float32,
    )


def observation(
    cmd: NDArray[Any],
    gyro: NDArray[Any],
    quat_wxyz: NDArray[Any],
    q29: NDArray[Any],
    dq29: NDArray[Any],
    last_action: NDArray[Any],
) -> NDArray[np.float32]:
    obs: NDArray[np.float32] = np.zeros(OBS_DIM, dtype=np.float32)
    obs[0:3] = np.asarray(cmd, dtype=np.float32) * CMD_SCALE
    obs[3] = HEIGHT_CMD
    obs[7:10] = np.asarray(gyro, dtype=np.float32) * ANG_VEL_SCALE
    obs[10:13] = projected_gravity(quat_wxyz)
    obs[13:42] = (np.asarray(q29, dtype=np.float32) - DEFAULT_29) * DOF_POS_SCALE
    obs[42:71] = np.asarray(dq29, dtype=np.float32) * DOF_VEL_SCALE
    obs[71:86] = last_action
    return obs


def action_of_target(target_15: NDArray[Any]) -> NDArray[np.float32]:
    """Invert the action law: the action that produced a recorded target."""
    out: NDArray[np.float32] = (
        np.asarray(target_15, dtype=np.float32) - DEFAULT_29[:NUM_ACTIONS]
    ) / ACTION_SCALE
    return out


class GrootPolicy:
    """Balance + walk ONNX pair with a 6-frame observation history."""

    def __init__(self, model_dir: Path) -> None:
        providers = ["CPUExecutionProvider"]
        if "CUDAExecutionProvider" in ort.get_available_providers():
            providers.insert(0, "CUDAExecutionProvider")
        opts = ort.SessionOptions()  # type: ignore[attr-defined]  # absent from the bundled stubs
        opts.intra_op_num_threads = 1  # one rollout per process; threads would fight
        self._balance = ort.InferenceSession(
            str(model_dir / "balance.onnx"), opts, providers=providers
        )
        self._walk = ort.InferenceSession(str(model_dir / "walk.onnx"), opts, providers=providers)
        self._balance_in = self._balance.get_inputs()[0].name
        self._walk_in = self._walk.get_inputs()[0].name
        self.last_action = np.zeros(NUM_ACTIONS, dtype=np.float32)
        self._buf = np.zeros((1, OBS_DIM * OBS_HISTORY), dtype=np.float32)
        self._first = True

    def push(self, obs: NDArray[np.float32]) -> None:
        """Append one frame to the history (the first frame fills all slots)."""
        if self._first:
            self._buf[0, :] = np.tile(obs, OBS_HISTORY)
            self._first = False
        else:
            self._buf[0, : OBS_DIM * (OBS_HISTORY - 1)] = self._buf[0, OBS_DIM:]
            self._buf[0, OBS_DIM * (OBS_HISTORY - 1) :] = obs

    def warm_start(self, frames: list[NDArray[np.float32]], last_action: NDArray[Any]) -> None:
        """Teacher-force the history from recorded frames (oldest first)."""
        for f in frames:
            self.push(f)
        self.last_action[:] = last_action

    def step(
        self,
        cmd: NDArray[Any],
        gyro: NDArray[Any],
        quat_wxyz: NDArray[Any],
        q29: NDArray[Any],
        dq29: NDArray[Any],
    ) -> NDArray[np.float32]:
        """One policy tick: joint targets for the 15 legs+waist joints."""
        self.push(observation(cmd, gyro, quat_wxyz, q29, dq29, self.last_action))
        if float(np.linalg.norm(cmd)) <= CMD_NORM_THRESHOLD:
            raw = self._balance.run(None, {self._balance_in: self._buf})[0]
        else:
            raw = self._walk.run(None, {self._walk_in: self._buf})[0]
        self.last_action[:] = raw[0, :NUM_ACTIONS].astype(np.float32)
        out: NDArray[np.float32] = self.last_action * ACTION_SCALE + DEFAULT_29[:NUM_ACTIONS]
        return out
