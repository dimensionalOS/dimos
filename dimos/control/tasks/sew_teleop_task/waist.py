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

"""R1 torso-only Pink step: projected pitch/yaw orientation and chest height."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np
import pink
import pinocchio as pin
from scipy.spatial.transform import Rotation

from dimos.robot.galaxea.r1pro.sew_model import R1SewModel


class WaistSolver:
    def __init__(self, model: R1SewModel) -> None:
        full = pin.buildModelFromXML(ET.tostring(model.root, encoding="unicode"))
        keep = [full.getJointId(f"torso_joint{i}") for i in range(1, 5)]
        self.model = pin.buildReducedModel(
            full, [i for i in range(1, full.njoints) if i not in keep], pin.neutral(full)
        )
        self.data = self.model.createData()
        self.frame = self.model.getFrameId("torso_link4")
        self.task = pink.tasks.FrameTask(
            "torso_link4",
            position_cost=[0.0, 0.0, 1.0],
            orientation_cost=1.0,
            lm_damping=0.01,
            gain=0.3,
        )
        self.posture = pink.tasks.PostureTask(cost=0.01)

    def target(self, pitch: float, yaw: float, height: float) -> pin.SE3:
        # Actual reachable orientation family is Ry(pitch) Rz(yaw); no roll axis.
        r = Rotation.from_euler("Y", pitch).as_matrix() @ Rotation.from_euler("Z", yaw).as_matrix()
        return pin.SE3(r, np.array([0.0, 0.0, height]))

    def step(self, q: np.ndarray, target: pin.SE3, dt: float) -> np.ndarray:
        configuration = pink.Configuration(self.model, self.data, q.copy())
        self.task.set_target(target)
        self.posture.set_target(q)
        velocity = pink.solve_ik(
            configuration, [self.task, self.posture], dt, solver="proxqp", safety_break=True
        )
        configuration.integrate_inplace(velocity, dt)
        return np.asarray(configuration.q, dtype=float).copy()
