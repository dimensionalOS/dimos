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

"""Collision-checked planar-stage transport after ACT loads the onboard tray.

This is a scripted navigation controller for a simulated planar base. ACT controls
manipulation; no claim is made about learned navigation or real wheel dynamics.
"""

from __future__ import annotations

from collections.abc import Iterator
import copy
import heapq
from itertools import pairwise
import math
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS


class PlanarTransport:
    """Plan against actual scene contacts without changing the physical state."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        cargo_bodies: tuple[str, ...] = ("task_bottle",),
    ) -> None:
        self.model = copy.copy(model)
        self.qids = np.array([model.joint(name).qposadr[0] for name in VIRTUAL_BASE_JOINTS])
        self.aids = np.array([model.actuator(name).id for name in VIRTUAL_BASE_JOINTS])
        self.probe = mujoco.MjData(self.model)
        self.probe.qpos[:] = data.qpos
        self.probe.ctrl[:] = data.ctrl
        root = model.body("base_link").id
        self.robot_bodies = {root}
        for body in range(root + 1, model.nbody):
            if int(model.body_parentid[body]) in self.robot_bodies:
                self.robot_bodies.add(body)
        self.cargo_bodies = cargo_bodies
        self.cargo_ids = {model.body(name).id for name in cargo_bodies}
        self.tray_id = model.body("task_bin").id
        self.carried_qpos: list[tuple[int, NDArray[np.float64]]] = []
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "task_tray_free") >= 0:
            for body_name in ("task_bin", *cargo_bodies):
                body = model.body(body_name).id
                joint = int(model.body_jntadr[body])
                if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
                    raise ValueError(f"Carried object {body_name!r} must have a free joint")
                address = int(model.jnt_qposadr[joint])
                self.carried_qpos.append((address, data.qpos[address : address + 7].copy()))
                self.robot_bodies.add(body)
        # Inflate only the planning copy. The tray intentionally parks just
        # above the worktop; its actual geometry is checked without inflation.
        tray_id = model.body("task_bin").id
        for gid in range(model.ngeom):
            if int(model.geom_bodyid[gid]) in self.robot_bodies - {tray_id}:
                self.model.geom_margin[gid] = max(float(model.geom_margin[gid]), 0.02)
        self.start = data.qpos[self.qids].copy()

    def collisions(self, data: mujoco.MjData, *, ignore_cargo: bool = False) -> list[str]:
        """Report robot/environment penetration, excluding floor support and cargo."""
        obstacles = set()
        for contact in data.contact:
            if contact.dist > (0.02 if data is self.probe else 0.0) or contact.pos[2] < 0.06:
                continue
            bodies = [int(self.model.geom_bodyid[geom]) for geom in contact.geom]
            if self.cargo_ids.intersection(bodies) and not self.carried_qpos:
                continue
            if ignore_cargo and (self.cargo_ids.intersection(bodies) or self.tray_id in bodies):
                continue
            if (bodies[0] in self.robot_bodies) != (bodies[1] in self.robot_bodies):
                other = bodies[1] if bodies[0] in self.robot_bodies else bodies[0]
                obstacles.add(self.model.body(other).name or f"body:{other}")
        return sorted(obstacles)

    def clear_pose_segment(self, first: NDArray[Any], second: NDArray[Any]) -> bool:
        """Check translations and turns, including the full held-cargo envelope."""
        count = max(
            1,
            math.ceil(float(np.linalg.norm(second[:2] - first[:2])) / 0.05),
            math.ceil(abs(float(second[2] - first[2])) / 0.08),
        )
        for fraction in np.linspace(0, 1, count + 1):
            pose = first + fraction * (second - first)
            self.probe.qpos[self.qids] = pose
            angle = float(pose[2] - self.start[2])
            rotation = np.array(
                [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
            )
            quaternion = np.array([math.cos(angle / 2), 0, 0, math.sin(angle / 2)])
            for address, initial in self.carried_qpos:
                self.probe.qpos[address : address + 3] = initial[:3]
                self.probe.qpos[address : address + 2] = pose[:2] + rotation @ (
                    initial[:2] - self.start[:2]
                )
                mujoco.mju_mulQuat(
                    self.probe.qpos[address + 3 : address + 7], quaternion, initial[3:7]
                )
            mujoco.mj_forward(self.model, self.probe)
            if self.collisions(self.probe):
                return False
        return True

    def shorten_path(self, path: list[list[float]]) -> list[list[float]]:
        """Shorten a proposed path only across collision-free full-pose sweeps."""
        poses = np.asarray(path, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2 or not np.isfinite(poses).all():
            raise ValueError("Expected at least two finite planar poses")
        poses = np.vstack([self.start, poses])
        poses[:, 2] = np.unwrap(poses[:, 2])
        shortened = [poses[0]]
        index = 0
        while index < len(poses) - 1:
            for candidate in range(len(poses) - 1, index, -1):
                if self.clear_pose_segment(poses[index], poses[candidate]):
                    shortened.append(poses[candidate])
                    index = candidate
                    break
            else:
                raise RuntimeError(f"Native path cannot clear the loaded robot at pose {index}")
        return [[float(value) for value in pose] for pose in shortened]

    def clear_segment(self, first: NDArray[Any], second: NDArray[Any]) -> bool:
        return self.clear_pose_segment(np.r_[first, self.start[2]], np.r_[second, self.start[2]])

    def plan_delivery(self, goal: list[float]) -> list[list[float]]:
        """Translate through the room, turn clear of the desk, then approach it."""
        target = np.array(goal, dtype=np.float64)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Delivery destination must be a finite planar pose")
        if not self.clear_pose_segment(target, target):
            raise RuntimeError("Tray delivery parking pose is obstructed")
        deadline = time.monotonic() + 30.0
        # The kitchen exit is narrow: turn near the workbench, then plan
        # translation with the carrying posture aligned down the corridor.
        for dx, dy in ((-0.2, 0.0), (-0.1, 0.0), (-0.3, 0.0), (-0.2, -0.1), (0.0, 0.0)):
            departure = self.start + np.array([dx, dy, 0.0])
            turned = np.r_[departure[:2], target[2]]
            if not self.clear_pose_segment(self.start, departure) or not self.clear_pose_segment(
                departure, turned
            ):
                continue
            aligned = PlanarTransport(self.model, self.probe, cargo_bodies=self.cargo_bodies)
            try:
                path = aligned.plan(
                    tuple(target[:2]),
                    resolution=0.025,
                    max_distance=6.0,
                    timeout=max(0.0, deadline - time.monotonic()),
                )
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Local docking planning exceeded 30 seconds") from None
                continue
            return [self.start.tolist(), departure.tolist(), *path]
        raise RuntimeError("No collision-free departure turn and route to the destination")

    def plan(
        self,
        goal: tuple[float, float],
        *,
        resolution: float = 0.05,
        max_distance: float = 3.0,
        timeout: float = 30.0,
    ) -> list[list[float]]:
        """Use bounded A* over translation with fixed yaw and a carried tray."""
        if (
            not np.isfinite(goal).all()
            or np.linalg.norm(np.asarray(goal) - self.start[:2]) > max_distance
        ):
            raise ValueError(f"Transport requires finite goals within {max_distance:g} metres")
        exact_goal = np.asarray(goal)
        if not self.clear_segment(exact_goal, exact_goal):
            raise RuntimeError("Transport goal is obstructed")
        origin = self.start[:2]
        # Docking often needs just a translation or an elbow around furniture.
        # Check these full-body sweeps before exploring a fine grid in the house.
        for points in (
            [origin, exact_goal],
            [origin, np.array([origin[0], exact_goal[1]]), exact_goal],
            [origin, np.array([exact_goal[0], origin[1]]), exact_goal],
        ):
            if all(self.clear_segment(a, b) for a, b in pairwise(points)):
                return [[float(x), float(y), float(self.start[2])] for x, y in points]
        deadline = time.monotonic() + timeout
        target = tuple(round(value / resolution) for value in (np.asarray(goal) - origin))
        start = (0, 0)
        frontier = [(0.0, start)]
        parents: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start: 0.0}
        blocked: dict[tuple[tuple[int, int], tuple[int, int]], bool] = {}
        found = False
        while frontier and len(cost) < 12000:
            if time.monotonic() >= deadline:
                raise RuntimeError("Local transport planning timed out")
            _, current = heapq.heappop(frontier)
            if current == target:
                found = True
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (current[0] + dx, current[1] + dy)
                if any(
                    abs(index) > math.ceil((max_distance + 0.5) / resolution) for index in neighbor
                ):
                    continue
                new_cost = cost[current] + 1
                if new_cost >= cost.get(neighbor, math.inf):
                    continue
                edge = (current, neighbor)
                if edge not in blocked:
                    blocked[edge] = not self.clear_segment(
                        origin + np.asarray(current) * resolution,
                        origin + np.asarray(neighbor) * resolution,
                    )
                if blocked[edge]:
                    continue
                cost[neighbor] = new_cost
                parents[neighbor] = current
                heuristic = abs(neighbor[0] - target[0]) + abs(neighbor[1] - target[1])
                heapq.heappush(frontier, (new_cost + heuristic, neighbor))
        if not found:
            raise RuntimeError("No collision-free local transport path to this destination")
        cells = [target]
        while cells[-1] != start:
            cells.append(parents[cells[-1]])
        cells.reverse()
        points = [origin + np.asarray(cell) * resolution for cell in cells]
        exact_goal = np.asarray(goal)
        if not self.clear_segment(points[-1], exact_goal):
            raise RuntimeError("Transport goal is obstructed")
        points.append(exact_goal)
        # Keep corners; merge collinear cells to avoid stopping every 20 cm.
        merged = [points[0]]
        for index in range(1, len(points) - 1):
            before, after = points[index] - points[index - 1], points[index + 1] - points[index]
            if abs(before[0] * after[1] - before[1] * after[0]) > 1e-8:
                merged.append(points[index])
        merged.append(exact_goal)
        return [[float(x), float(y), float(self.start[2])] for x, y in merged]

    @staticmethod
    def targets(
        path: list[list[float]], fps: int, speed: float = 0.1
    ) -> Iterator[NDArray[np.float64]]:
        """Smooth position targets with a bounded peak translational speed."""
        for first, second in pairwise(path):
            start, goal = np.asarray(first), np.asarray(second)
            seconds = max(
                0.5,
                1.5 * float(np.linalg.norm(goal[:2] - start[:2])) / speed,
                1.5 * abs(float(goal[2] - start[2])) / 0.15,
            )
            frames = math.ceil(seconds * fps)
            for frame in range(frames):
                t = (frame + 1) / frames
                yield start + (goal - start) * (3 * t**2 - 2 * t**3)
