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

from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pinocchio

from dimos.imitation.policy.dreamlabs.conversion import observation
from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.dual_openyam import joints

REFERENCE_URDF = Path(joints.__file__).with_name("dreamlabs_yam.urdf")
TCP_OFFSET_M = 0.1347


class DreamlabsKinematics:
    """One arm-local FK model, used serially under the rollout's observation lock.

    The observation reference reproduces the SDK training model. The physical
    coordinator keeps its own model, bounds, and calibration.
    """

    def __init__(self) -> None:
        description = RobotModel.from_file(REFERENCE_URDF).load()
        model = pinocchio.buildModelFromXML(description.xml)
        expected = tuple(f"joint{i}" for i in range(1, 7))
        if tuple(model.names)[1:] != expected or model.nq != 6:
            raise ValueError("Dreamlabs observation model must be joint1 through joint6")
        self._fk = PinocchioIK(model, model.createData(), model.getJointId("joint6"))
        self._tool = pinocchio.SE3(np.eye(3), np.array([0.0, 0.0, TCP_OFFSET_M]))

    def pose(self, joints_rad: NDArray[np.float64]) -> pinocchio.SE3:
        if joints_rad.shape != (6,) or not np.isfinite(joints_rad).all():
            raise ValueError("expected six finite joint angles in radians")
        # SDK TCP is along flange z. It is not DimOS's grasp_frame or world frame.
        return self._fk.forward_kinematics(joints_rad) * self._tool

    def observation(self, vector: NDArray[np.float64], prompt: str) -> dict[str, Any]:
        result = observation(vector, prompt)
        for side, arm, grip in (
            ("left", vector[:6], vector[12:13]),
            ("right", vector[6:12], vector[13:14]),
        ):
            tcp = self.pose(arm)
            result[f"state.{side}_eef_xyz"] = tcp.translation.astype(np.float32)
            # The model reads the first two rotation columns, concatenated by column.
            result[f"state.{side}_eef_rot6"] = np.concatenate(
                (tcp.rotation[:, 0], tcp.rotation[:, 1])
            ).astype(np.float32)
            result[f"state.{side}_grip"] = grip.astype(np.float32)
        return result
