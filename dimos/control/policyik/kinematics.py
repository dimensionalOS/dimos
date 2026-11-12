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

"""Encoder-only tool poses in the fixed chassis frame."""

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin
from scipy.spatial.transform import Rotation


class ToolKinematics:
    """Own a Pinocchio data cache, independent of the simulator's body truth."""

    def __init__(self, urdf: Path, joints: tuple[str, ...], tools: tuple[str, str]) -> None:
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        if self.model.nq != len(joints) or self.model.nv != len(joints):
            raise ValueError("Policy model must have exactly the exported scalar joints")
        self.indices = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in joints])
        if set(self.model.names[1:]) != set(joints):
            raise ValueError("Joint names differ from the policy manifest")
        self.frames = [self.model.getFrameId(n) for n in tools]
        if any(f >= self.model.nframes for f in self.frames):
            raise ValueError("Missing policy tool frame")
        self.limits = np.stack(
            (
                self.model.lowerPositionLimit[self.indices],
                self.model.upperPositionLimit[self.indices],
            ),
            axis=-1,
        ).astype(np.float32)

    def poses(self, q: NDArray[np.floating]) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        ordered = np.zeros(self.model.nq)
        ordered[self.indices] = q
        pin.forwardKinematics(self.model, self.data, ordered)
        pin.updateFramePlacements(self.model, self.data)
        positions = np.array([self.data.oMf[f].translation for f in self.frames], dtype=np.float32)
        rotations = np.array([self.data.oMf[f].rotation for f in self.frames])
        quaternions = Rotation.from_matrix(rotations).as_quat(scalar_first=True).astype(np.float32)
        return positions, quaternions
