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


"""Encode standard YAM tool poses in Dream-1's training convention using DimOS FK."""

from typing import Any

import numpy as np
from numpy.typing import NDArray
import pinocchio

from dimos.imitation.policy.dreamlabs.conversion import observation
from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL

TCP_OFFSET_M = 0.1347


class DreamlabsKinematics:
    """Canonical dual-YAM geometry, expressed in Dream-1's arm-local TCP convention.

    Canonical gripper -z is the tool approach; SDK tool +z is approach, with
    x=-canonical y and y=-canonical x. This fixed link-frame conversion is not
    a joint-angle offset, an IK solve, or measured physical tool calibration.
    """

    def __init__(self) -> None:
        description = DUAL_OPENYAM_MODEL.load()
        model = pinocchio.buildModelFromXML(description.xml)
        if tuple(model.names)[1:] != tuple(DUAL_OPENYAM_ARM_JOINTS) or model.nq != 12:
            raise ValueError("Dreamlabs requires the canonical dual six-joint YAM model")
        self._arms: dict[str, tuple[PinocchioIK, pinocchio.SE3, list[int]]] = {}
        for side in ("left", "right"):
            base = model.frames[model.getFrameId(f"{side}_base")]
            if base.parentJoint != 0:
                raise ValueError("Dreamlabs arm base must be fixed to the model root")
            indices = [
                model.joints[model.getJointId(f"{side}_joint{i}")].idx_q for i in range(1, 7)
            ]
            fk = PinocchioIK(model, model.createData(), model.getJointId(f"{side}_joint6"))
            self._arms[side] = (fk, base.placement.inverse(), indices)
        # Preserve the SDK TCP distance in the canonical flange/link coordinates.
        # The canonical grasp_frame is only 0.1 m along -z, not this TCP.
        self._tool = pinocchio.SE3(
            np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]),
            np.array([0.0, 0.0, -TCP_OFFSET_M]),
        )

    def pose(self, joints_rad: NDArray[np.float64], side: str = "left") -> pinocchio.SE3:
        if joints_rad.shape != (6,) or not np.isfinite(joints_rad).all():
            raise ValueError("expected six finite joint angles in radians")
        fk, base_inverse, indices = self._arms[side]
        q = pinocchio.neutral(fk.model)
        q[indices] = joints_rad
        return base_inverse * fk.forward_kinematics(q) * self._tool

    def observation(self, vector: NDArray[np.float64], prompt: str) -> dict[str, Any]:
        result = observation(vector, prompt)
        for side, arm, grip in (
            ("left", vector[:6], vector[12:13]),
            ("right", vector[6:12], vector[13:14]),
        ):
            tcp = self.pose(arm, side)
            result[f"state.{side}_eef_xyz"] = tcp.translation.astype(np.float32)
            # The model reads the first two rotation columns, concatenated by column.
            result[f"state.{side}_eef_rot6"] = np.concatenate(
                (tcp.rotation[:, 0], tcp.rotation[:, 1])
            ).astype(np.float32)
            result[f"state.{side}_grip"] = grip.astype(np.float32)
        return result
