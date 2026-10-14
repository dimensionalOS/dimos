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

"""Model-derived head FK and the upstream two-axis gaze objective."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
from numpy.typing import NDArray
import pinocchio as pin

from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.msgs.geometry_msgs.Quaternion import Quaternion

HeadJoints = tuple[float, float, float, float]
HEAD_JOINT_NAMES = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")

# Command limits belong to the trained policy, not the mechanical MJCF limits.
HEAD_COMMAND_LOWER = np.asarray((-1.10, -1.10, -1.40, -0.31), dtype=np.float64)
HEAD_COMMAND_UPPER = np.asarray((1.10, 1.10, 1.40, 0.31), dtype=np.float64)
_SITE_TO_CV2 = pin.SE3(Quaternion(-0.5, 0.5, -0.5, 0.5).to_rotation_matrix(), np.zeros(3))


@dataclass(frozen=True)
class Gaze:
    joints: HeadJoints
    clamped: bool


class HeadKinematics:
    """Use dimOS Pinocchio FK for the MJCF camera site. Caller owns synchronization."""

    def __init__(self, mjcf_path: str | Path) -> None:
        # Pinocchio's MJCF parser does not handle this model's repeated default
        # blocks. MuJoCo normalizes them (and includes) without changing geometry.
        spec = mujoco.MjSpec.from_file(str(mjcf_path))
        spec.compile()
        with TemporaryDirectory(prefix="microduck-fk-") as directory:
            normalized = Path(directory) / "robot.xml"
            normalized.write_text(spec.to_xml(), encoding="utf-8")
            model = PinocchioIK.from_model_path(normalized, ee_joint_id=0).model

        for name in ("head_camera", "trunk_base"):
            if not model.existFrame(name):
                raise ValueError(f"MicroDuck MJCF is missing frame {name!r}")
        indices: list[int] = []
        for name in HEAD_JOINT_NAMES:
            if not model.existJointName(name):
                raise ValueError(f"MicroDuck MJCF is missing joint {name!r}")
            joint = model.joints[model.getJointId(name)]
            if joint.nq != 1:
                raise ValueError(f"MicroDuck head joint {name!r} must have one coordinate")
            indices.append(int(joint.idx_q))
        self._indices = np.asarray(indices)
        self._neutral = pin.neutral(model)
        data = model.createData()
        pin.forwardKinematics(model, data, self._neutral)
        pin.updateFramePlacements(model, data)
        self._trunk_from_world = data.oMf[model.getFrameId("trunk_base")].inverse()
        camera = model.frames[model.getFrameId("head_camera")]
        self._camera_from_joint = camera.placement * _SITE_TO_CV2
        self._fk = PinocchioIK(model, data, int(camera.parentJoint))

    def camera_in_trunk_cv2(self, joints: HeadJoints) -> pin.SE3:
        """Return the camera optical pose in the trunk frame."""

        q = self._neutral.copy()
        q[self._indices] = joints
        return self._trunk_from_world * self._fk.forward_kinematics(q) * self._camera_from_joint

    def look_at(self, target_in_trunk: tuple[float, float, float], neck_pitch: float = 0.0) -> Gaze:
        """Point the camera toward a target, retaining upstream gaze semantics."""

        target = np.asarray(target_in_trunk, dtype=np.float64)
        joints = np.asarray((neck_pitch, 0.0, 0.0, 0.0), dtype=np.float64)
        joints = np.clip(joints, HEAD_COMMAND_LOWER, HEAD_COMMAND_UPPER)
        tolerance, step_h, damping, max_step = 1e-4, 1e-5, 1e-3, 0.7

        def pointing_error(values: NDArray[np.float64]) -> NDArray[np.float64]:
            angles = (float(values[0]), float(values[1]), float(values[2]), float(values[3]))
            camera_delta = self.camera_in_trunk_cv2(angles).inverse().act(target)
            flat = math.hypot(float(camera_delta[0]), float(camera_delta[2]))
            return np.asarray(
                (
                    math.atan2(float(camera_delta[0]), float(camera_delta[2])),
                    math.atan2(float(camera_delta[1]), flat),
                ),
                dtype=np.float64,
            )

        for _ in range(30):
            error = pointing_error(joints)
            if float(np.max(np.abs(error))) < tolerance:
                break
            jacobian = np.empty((2, 2), dtype=np.float64)
            for column, joint_index in enumerate((1, 2)):
                probe = joints.copy()
                probe[joint_index] += step_h
                jacobian[:, column] = (pointing_error(probe) - error) / step_h
            lhs = jacobian.T @ jacobian + damping * np.eye(2)
            rhs = -(jacobian.T @ error)
            try:
                step = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                break
            norm = float(np.linalg.norm(step))
            if norm > max_step:
                step *= max_step / norm
            joints[1:3] += step
            joints = np.clip(joints, HEAD_COMMAND_LOWER, HEAD_COMMAND_UPPER)

        residual = float(np.max(np.abs(pointing_error(joints))))
        return Gaze(
            joints=(float(joints[0]), float(joints[1]), float(joints[2]), float(joints[3])),
            clamped=residual >= tolerance,
        )
