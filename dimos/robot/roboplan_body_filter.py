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

"""UrdfManager's self-filter backend on RoboPlan's narrowphase robot body filter."""

from __future__ import annotations

from collections.abc import Mapping
import importlib
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import LoadedRobotModel
from dimos.utils.transform_utils import matrix_to_pose


class RoboPlanBodyFilter:
    """Which points (in the URDF root frame) lie on the robot's collision geometry, padded by padding_m."""

    def __init__(self, description: LoadedRobotModel, padding_m: float) -> None:
        # RoboPlan is optional for stacks that never self-filter.
        native = importlib.import_module("roboplan.core")
        root = ET.fromstring(description.xml)
        root.set("version", "1.0")
        self._scene = native.Scene(
            "dimos_self_filter",
            native.loadUrdfSceneDescriptionFromXml(
                ET.tostring(root, encoding="unicode"),
                [str(path) for path in description.package_paths.values()],
            ),
        )
        self._neutral_q = np.asarray(self._scene.getCurrentJointPositions()).copy()
        self._context = native.SceneContext(self._scene)
        self._body_filter = native.RobotBodyFilter(
            self._scene,
            native.RobotBodyFilterOptions(
                padding=padding_m,
                method=native.RobotBodyFilterMethod.Narrowphase,
                num_threads=1,
            ),
        )

    def keep_mask(
        self,
        root_points: NDArray[np.float64],
        joint_positions: Mapping[str, float],
        joint_motions: Mapping[str, NDArray[np.float64]],
    ) -> NDArray[np.bool_] | None:
        joint_configuration = self.configuration(joint_positions, joint_motions)
        if joint_configuration is None:
            return None
        # This Scene is rooted at the URDF root, so its world is the root frame.
        return ~np.asarray(
            self._body_filter.computeMask(joint_configuration, root_points), dtype=bool
        )

    def configuration(
        self,
        joint_positions: Mapping[str, float],
        joint_motions: Mapping[str, NDArray[np.float64]],
    ) -> NDArray[np.float64] | None:
        """RoboPlan's q for the state, or None when a joint it moves is missing or a planar motion leaves its plane."""
        # Start from the model's neutral configuration, never its latest state.
        joint_configuration = self._neutral_q.copy()
        for name in self._scene.getJointNames():
            info = self._scene.getJointInfo(name)
            if info.num_velocity_dofs == 1:
                if name not in joint_positions:
                    return None
                value = joint_positions[name]
                values = [np.cos(value), np.sin(value)] if info.num_position_dofs == 2 else [value]
            else:
                if name not in joint_motions:
                    return None
                motion = joint_motions[name]
                if info.num_position_dofs == 4:
                    if not np.allclose(motion[2], [0, 0, 1, 0], atol=1e-6):
                        return None
                    angle = np.arctan2(motion[1, 0], motion[0, 0])
                    values = [motion[0, 3], motion[1, 3], np.cos(angle), np.sin(angle)]
                elif info.num_position_dofs == 7:
                    orientation = matrix_to_pose(motion).orientation
                    values = [
                        *motion[:3, 3],
                        orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w,
                    ]
                else:
                    raise ValueError(f"Unsupported robot joint layout: {name}")
            joint_configuration[self._scene.getJointPositionIndices([name])] = values
        return joint_configuration
