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

"""Robot geometry for operator views, independent of robot brand and simulator."""

from collections.abc import Mapping
from io import BytesIO
from typing import Any

import numpy as np
from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation
from yourdfpy import URDF  # type: ignore[import-untyped]

from dimos.manipulation.planning.spec.config import RobotModelConfig


class SpatialGroup(BaseModel):
    label: str
    joint_names: tuple[str, ...]
    tip_frame: str
    base_frame: str
    gripper_joint: str | None = None
    gripper_open_position: float = Field(default=1.0, gt=0)


class SpatialRobot:
    def __init__(self, config: RobotModelConfig, groups: Mapping[str, SpatialGroup]) -> None:
        self.description = config.model.load()
        self.root_link = self.description.root_link
        self.groups = groups
        self.robot = URDF.load(BytesIO(self.description.xml.encode()), load_collision_meshes=False)
        self.nodes: dict[str, list[str]] = {name: [] for name in groups}
        self.nodes["body"] = []
        links: dict[str, set[str]] = {}
        for name, group in groups.items():
            links[name] = {
                j.child_link for j in self.description.joints if j.name in group.joint_names
            }
            while True:
                expanded = links[name] | {
                    j.child_link for j in self.description.joints if j.parent_link in links[name]
                }
                if expanded == links[name]:
                    break
                links[name] = expanded
        meshes: dict[str, Any] = {}
        parts: dict[str, list[dict[str, Any]]] = {name: [] for name in self.nodes}
        for parent, node, edge in self.robot.scene.graph.to_edgelist():
            mesh_name = edge.get("geometry")
            if mesh_name is None:
                continue
            group_name = next(
                (name for name, children in links.items() if parent in children), "body"
            )
            mesh = self.robot.scene.geometry[mesh_name]
            meshes[mesh_name] = {
                "vertices": mesh.vertices.ravel().tolist(),
                "faces": mesh.faces.ravel().tolist(),
            }
            parts[group_name].append(
                {
                    "mesh": mesh_name,
                    "rgba": (np.asarray(mesh.visual.main_color, dtype=float) / 255).tolist(),
                }
            )
            self.nodes[group_name].append(node)
        self.geometry = {
            "meshes": meshes,
            "arms": {name: parts[name] for name in groups},
            "body": parts["body"],
            "sides": list(groups),
            "groups": {name: g.model_dump() for name, g in groups.items()},
            "finger_geometry": True,
        }

    def feedback(self, positions: dict[str, float], world_from_root: np.ndarray) -> dict[str, Any]:
        self.robot.update_cfg(
            {name: positions[name] for name in self.robot.actuated_joint_names if name in positions}
        )
        result = {}
        for name, group in self.groups.items():
            tip = world_from_root @ self.robot.get_transform(group.tip_frame)
            result[name] = {
                "transforms": [
                    (world_from_root @ self.robot.get_transform(node)).T.ravel().tolist()
                    for node in self.nodes[name]
                ],
                "grasp": tip[:3, 3].tolist(),
                "orientation": Rotation.from_matrix(tip[:3, :3]).as_quat().tolist(),
                "opening": float(
                    np.clip(
                        positions.get(group.gripper_joint, 0.0) / group.gripper_open_position, 0, 1
                    )
                )
                if group.gripper_joint
                else 0.0,
            }
        return result

    def body_transforms(self, world_from_root: np.ndarray) -> list[list[float]]:
        return [
            (world_from_root @ self.robot.get_transform(node)).T.ravel().tolist()
            for node in self.nodes["body"]
        ]
