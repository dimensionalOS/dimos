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


"""Model FK and invertible SDK TCP conversion. This module never solves IK."""

from collections.abc import Mapping
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.imitation.policy.dreamlabs.config import ArmSide, DreamlabsBackendConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class DreamlabsKinematics:
    def __init__(self, config: DreamlabsBackendConfig) -> None:
        self.config = config
        self.model = config.model.load()
        self._parents = {joint.child_link: joint for joint in self.model.joints}
        self._links = {self.model.root_link, *self._parents}
        self._tools: dict[str, NDArray[np.float64]] = {}
        self.joint_bounds: dict[str, tuple[float, float]] = {}
        for side, arm in config.arms.items():
            if arm.base_frame not in self._links or arm.target_frame not in self._links:
                raise ValueError(f"Unknown Dreamlabs base or target frame for {side}")
            current = arm.base_frame
            while current != self.model.root_link:
                joint = self._parents[current]
                if joint.type != "fixed":
                    raise ValueError("Dreamlabs requires each arm base fixed to the model root")
                current = joint.parent_link
            tool = np.eye(4)
            xyz, quat = np.asarray(arm.target_to_sdk_xyz), np.asarray(arm.target_to_sdk_xyzw)
            if (
                not np.isfinite(xyz).all()
                or not np.isfinite(quat).all()
                or not np.isclose(np.linalg.norm(quat), 1.0)
            ):
                raise ValueError("Tool transform must have finite metres and a unit quaternion")
            tool[:3, :3] = Rotation.from_quat(quat).as_matrix()
            tool[:3, 3] = xyz
            self._tools[side] = tool
            for name in arm.joint_names:
                description = self.model.get_joint(name)
                if description is None or description.type not in {
                    "revolute",
                    "continuous",
                    "prismatic",
                }:
                    raise ValueError(f"Unknown movable arm joint {name}")
                if description.lower is not None and description.upper is not None:
                    self.joint_bounds[name] = (description.lower, description.upper)

    def root_pose(self, frame: str, positions: Mapping[str, float]) -> NDArray[np.float64]:
        if frame == self.model.root_link:
            return np.eye(4)
        joint = self._parents[frame]
        if joint.type == "fixed":
            value = 0.0
        elif joint.mimic is not None:
            leader, multiplier, offset = joint.mimic
            value = positions[leader] * multiplier + offset
        else:
            value = positions[joint.name]
        return self.root_pose(joint.parent_link, positions) @ joint.parent_from_child(value)

    def sdk_pose(self, side: ArmSide, positions: Mapping[str, float]) -> NDArray[np.float64]:
        arm = self.config.arms[side]
        return (
            np.linalg.inv(self.root_pose(arm.base_frame, positions))
            @ self.root_pose(arm.target_frame, positions)
            @ self._tools[side]
        )

    def canonical_pose(self, side: ArmSide, sdk_pose: NDArray[np.float64]) -> PoseStamped:
        arm = self.config.arms[side]
        root_from_target = (
            self.root_pose(arm.base_frame, {}) @ sdk_pose @ np.linalg.inv(self._tools[side])
        )
        return self.to_pose(root_from_target)

    def to_pose(self, matrix: NDArray[np.float64]) -> PoseStamped:
        return PoseStamped(
            frame_id=self.model.root_link,
            position=matrix[:3, 3].tolist(),
            orientation=Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
        )

    def observation(self, positions: Mapping[str, float], prompt: str) -> dict[str, Any]:
        result: dict[str, Any] = {"prompt": prompt}
        for side in ("left", "right"):
            arm = self.config.arms.get(side)
            tcp = self.sdk_pose(side, positions) if arm is not None else np.eye(4)
            grip = positions[arm.gripper_joint] if arm is not None else 0.0
            # The EEF transfer model takes YAM-shaped zero joint slots, not
            # another robot's joint angles. robot=other_embodiment masks state.
            joints = (
                [positions[name] for name in arm.joint_names]
                if arm is not None and self.config.mode == "joint"
                else [0.0] * 6
            )
            result[f"state.{side}_arm"] = np.asarray(joints, dtype=np.float32)
            result[f"state.{side}_gripper"] = np.asarray([grip], dtype=np.float32)
            result[f"state.{side}_grip"] = np.asarray([grip], dtype=np.float32)
            result[f"state.{side}_eef_xyz"] = tcp[:3, 3].astype(np.float32)
            result[f"state.{side}_eef_rot6"] = np.concatenate((tcp[:3, 0], tcp[:3, 1])).astype(
                np.float32
            )
        return result
