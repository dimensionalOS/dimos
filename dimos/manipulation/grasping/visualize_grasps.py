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
"""Grasp visualization debug tool: python -m dimos.manipulation.grasping.visualize_grasps"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

GRIPPER_WIDTH = 0.086
FINGER_LENGTH = 0.052
PALM_DEPTH = 0.04
MAX_GRASPS = 100
VISUALIZATION_FILE = "/tmp/grasp_visualization.json"


def create_gripper_geometry(transform: np.ndarray[Any, Any], color: list[float]) -> list[Any]:
    w = GRIPPER_WIDTH / 2.0
    fl = FINGER_LENGTH
    pd = PALM_DEPTH
    wrist = np.array([0.0, 0.0, -(pd + fl)])
    palm = np.array([0.0, 0.0, -fl])
    l_base = np.array([-w, 0.0, -fl])
    r_base = np.array([w, 0.0, -fl])
    l_tip = np.array([-w, 0.0, 0.25 * fl])
    r_tip = np.array([w, 0.0, 0.25 * fl])
    points = np.vstack([wrist, palm, l_base, r_base, l_tip, r_tip])
    lines = [[0, 1], [1, 2], [1, 3], [2, 4], [3, 5]]
    points_h = np.hstack([points, np.ones((len(points), 1))])
    points_world = (transform @ points_h.T).T[:, :3]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points_world)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))

    return [line_set]


def visualize_grasps(point_cloud: np.ndarray[Any, Any], grasps: list[np.ndarray[Any, Any]]) -> None:
    geometries = []

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd.paint_uniform_color([0.0, 0.8, 0.8])
    geometries.append(pcd)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geometries.append(coord_frame)

    num_to_show = min(len(grasps), MAX_GRASPS)
    for i in range(num_to_show):
        t = i / max(num_to_show - 1, 1) if i > 0 else 0.0
        color = [min(1.0, 2 * t), max(0.0, 1.0 - t), 0.0]
        geometries.extend(create_gripper_geometry(grasps[i], color))

    o3d.visualization.draw_geometries(geometries, window_name="GraspGen", width=1280, height=720)


def main() -> int:
    filepath = Path(VISUALIZATION_FILE)
    if not filepath.exists():
        print(f"File not found: {filepath}")
        return 1

    with open(filepath) as f:
        data = json.load(f)

    point_cloud = np.array(data["point_cloud"])
    grasps = [np.array(g).reshape(4, 4) for g in data["grasps"]]

    visualize_grasps(point_cloud, grasps)
    return 0


if __name__ == "__main__":
    exit(main())
