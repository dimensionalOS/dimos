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


"""What a robot's Mid-360s return walking a route through a scene, as a prior map for the stack."""

from __future__ import annotations

from dataclasses import replace
import math

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.simulation.scenes.mjcf import SCENE_GROUPS, add_boxes
from dimos.simulation.scenes.procedural import Scene
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.simulation.sim_world.body import Rig

PREMAP_STEP_M = 0.1
PREMAP_SPEED = 0.5
PREMAP_SEED = 0
VOXEL_M = 0.05


def without_objects(scene: Scene) -> Scene:
    """The scene with only its floor and walls."""
    return replace(scene, boxes=[box for box in scene.boxes if box.kind != "clutter"])


def base_poses(
    route: NDArray[np.float64], base_height: float, step: float
) -> list[tuple[NDArray[np.float64], NDArray[np.float64]]]:
    """Base poses of the robot walking the route, facing along it, one every step of arc length."""
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))])
    stops = np.minimum(np.arange(0.0, arc[-1] + step / 2, step), arc[-1])
    bases = np.column_stack([np.interp(stops, arc, route[:, i]) for i in range(3)])
    poses = []
    for stop, base in zip(stops, bases, strict=True):
        k = int(np.clip(np.searchsorted(arc, stop, side="right"), 1, len(route) - 1))
        heading = route[k, :2] - route[k - 1, :2] if len(route) > 1 else np.array([1.0, 0.0])
        yaw = math.atan2(heading[1], heading[0])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]])
        poses.append((base + np.array([0.0, 0.0, base_height]), rotation))
    return poses


def premap_cloud(scene: Scene, rig: Rig, route: NDArray[np.float64]) -> NDArray[np.float32]:
    """The rig's Mid-360 returns walking the route through the scene, one world-frame point per voxel."""
    spec = mujoco.MjSpec()
    add_boxes(spec, scene)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lidars = [
        sensor.lidar(MujocoRaycaster(model, data, SCENE_GROUPS), PREMAP_SEED + i)
        for i, sensor in enumerate(rig.sensors)
    ]
    per_pose = int(POINT_RATE * PREMAP_STEP_M / PREMAP_SPEED)
    lo, hi = scene.bounds()
    dims = np.ceil((hi - lo) / VOXEL_M).astype(np.int64) + 3
    seen = np.zeros(dims, dtype=bool)
    kept = []
    for base, turn in base_poses(route, rig.shape.base_height, PREMAP_STEP_M):
        for sensor, lidar in zip(rig.sensors, lidars, strict=True):
            position, rotation = base + turn @ sensor.mount_xyz, turn @ sensor.mount_r
            returns = lidar.cast(position, rotation, per_pose).astype(np.float64)
            returns = returns[np.linalg.norm(returns, axis=1) >= sensor.lio_blind_range]
            points = position + returns @ rotation.T
            keys = np.clip(np.floor((points - lo) / VOXEL_M).astype(np.int64) + 1, 0, dims - 1)
            flat = np.ravel_multi_index((keys[:, 0], keys[:, 1], keys[:, 2]), tuple(dims))
            _, first = np.unique(flat, return_index=True)
            first = first[~seen.flat[flat[first]]]
            seen.flat[flat[first]] = True
            kept.append(points[np.sort(first)].astype(np.float32))
    return np.concatenate(kept)
