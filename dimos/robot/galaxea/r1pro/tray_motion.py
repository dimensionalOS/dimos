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

"""Two-arm kinematic targets for grasping a physically free tray."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.tray_sim import TRAY_HANDLE_Y, TRAY_TCP_HEIGHT


@dataclass(frozen=True)
class TrayWaypoint:
    phase: str
    positions: list[float]
    seconds: float


class TrayMotion:
    """Plan in a copied kinematic state; never set the real tray pose."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        cargo_bodies: tuple[str, ...] = ("task_bottle",),
    ) -> None:
        self.model = model
        self.cargo_bodies = cargo_bodies
        self.probe = mujoco.MjData(model)
        self.probe.qpos[:] = data.qpos
        self.qids = np.array([model.joint(n).qposadr[0] for n in R1PRO_PICK_PLACE_JOINTS])
        self.limits = np.array([model.joint(n).range for n in R1PRO_PICK_PLACE_JOINTS])
        self.initial = data.qpos[self.qids].copy()
        mujoco.mj_kinematics(model, self.probe)  # type: ignore[attr-defined]

    def arm_pose(self, side: str, target: NDArray[Any]) -> NDArray[np.float64]:
        section = slice(4, 11) if side == "left" else slice(11, 18)
        names = R1PRO_PICK_PLACE_JOINTS[section]
        qids = self.qids[section]
        dofs = np.array([self.model.joint(n).dofadr[0] for n in names])
        limits = self.limits[section]
        site = self.model.site(f"{side}_tcp").id
        base_rotation = self.probe.body("base_link").xmat.reshape(3, 3).copy()
        rng = np.random.default_rng(24 if side == "left" else 4)
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        seed = self.probe.qpos[qids].copy()
        best: NDArray[np.float64] | None = None
        best_cost = float("inf")
        for attempt in range(16):
            self.probe.qpos[qids] = (
                seed if attempt == 0 else rng.uniform(limits[:, 0] + 0.1, limits[:, 1] - 0.1)
            )
            for _ in range(250):
                mujoco.mj_kinematics(self.model, self.probe)  # type: ignore[attr-defined]
                mujoco.mj_comPos(self.model, self.probe)  # type: ignore[attr-defined]
                ep = target - self.probe.site_xpos[site]
                current = self.probe.site_xmat[site].reshape(3, 3)
                er = Rotation.from_matrix(base_rotation @ current.T).as_rotvec()
                if np.linalg.norm(ep) < 0.0005 and np.linalg.norm(er) < 0.005:
                    candidate = np.asarray(self.probe.qpos[qids], dtype=np.float64).copy()
                    cost = float(np.linalg.norm(candidate - seed))
                    if cost < 0.8:
                        return candidate
                    if cost < best_cost:
                        best, best_cost = candidate, cost
                    break
                mujoco.mj_jacSite(self.model, self.probe, jac_pos, jac_rot, site)  # type: ignore[attr-defined]
                jac = np.vstack((jac_pos[:, dofs], 0.25 * jac_rot[:, dofs]))
                delta = jac.T @ np.linalg.solve(
                    jac @ jac.T + 1e-4 * np.eye(6), np.r_[ep, 0.25 * er]
                )
                delta *= min(1.0, 0.15 / max(float(np.linalg.norm(delta)), 1e-10))
                self.probe.qpos[qids] = np.clip(
                    self.probe.qpos[qids] + delta, limits[:, 0] + 0.001, limits[:, 1] - 0.001
                )
        if best is not None:
            self.probe.qpos[qids] = best
            return best
        raise RuntimeError(f"Unreachable {side} gripper target {target.tolist()}")

    def waypoint(
        self, phase: str, centre: NDArray[Any], opening: float, seconds: float
    ) -> TrayWaypoint:
        mujoco.mj_kinematics(self.model, self.probe)  # type: ignore[attr-defined]
        rotation = self.probe.body("base_link").xmat.reshape(3, 3).copy()
        for side, sign in (("left", 1), ("right", -1)):
            self.arm_pose(side, centre + rotation @ np.array([0, sign * TRAY_HANDLE_Y, 0]))
        self.probe.qpos[self.qids[-2:]] = opening
        return TrayWaypoint(phase, self.probe.qpos[self.qids].tolist(), seconds)

    def _checked(self, points: list[TrayWaypoint], data: mujoco.MjData) -> list[TrayWaypoint]:
        planner = PlanarTransport(self.model, data, cargo_bodies=self.cargo_bodies)
        start = self.initial.copy()
        for point in points:
            goal = np.array(point.positions)
            count = max(1, int(np.ceil(np.max(np.abs(goal - start)) / 0.05)))
            for t in np.linspace(0, 1, count + 1):
                planner.probe.qpos[self.qids] = start + (goal - start) * t
                mujoco.mj_forward(planner.model, planner.probe)
                obstacles = planner.collisions(planner.probe, ignore_cargo=True)
                if obstacles:
                    raise RuntimeError(f"{point.phase} arm path is obstructed: {obstacles}")
            start = goal
        return points

    def pickup(self, data: mujoco.MjData) -> list[TrayWaypoint]:
        position = data.body("task_bin").xpos.copy()
        grasp = position + np.array([0.0, 0.0, TRAY_TCP_HEIGHT])
        above = grasp + np.array([0.0, 0.0, 0.15])
        # Raise the parked left arm sideways before reaching across the table.
        # The complete joint interpolation is collision checked below.
        shoulder = self.qids[5]
        self.probe.qpos[shoulder] = max(float(self.probe.qpos[shoulder]), 0.8)
        self.probe.qpos[self.qids[-2:]] = 0.05
        points = [
            TrayWaypoint("raise_hands", self.probe.qpos[self.qids].tolist(), 2.0),
            self.waypoint("approach_tray", above, 0.05, 1.5),
            self.waypoint("lower_to_handles", grasp, 0.05, 1.5),
            self.waypoint("grasp_handles", grasp, 0.009, 2.0),
            *[
                self.waypoint(f"lift_tray_{i}", grasp + np.array([0, 0, i * 0.015]), 0.009, 0.5)
                for i in range(1, 11)
            ],
        ]
        return self._checked(points, data)

    def placement(self, data: mujoco.MjData, target: list[float]) -> list[TrayWaypoint]:
        """Extend over the real tabletop, lower onto it, then release and retreat."""
        position = data.body("task_bin").xpos.copy()
        centre = (data.site("left_tcp").xpos + data.site("right_tcp").xpos) / 2
        offset = centre - position
        above = np.array(target) + offset + [0, 0, 0.07]
        resting = np.array(target) + offset - [0, 0, 0.005]
        points = []
        for phase, first, last, opening in (
            ("extend_over_table", centre, above, 0.009),
            ("lower_onto_table", above, resting, 0.009),
        ):
            count = max(1, int(np.ceil(np.linalg.norm(last - first) / 0.02)))
            for i in range(1, count + 1):
                points.append(
                    self.waypoint(phase, first + (last - first) * i / count, opening, 0.5)
                )
        points.append(self.waypoint("release_tray", resting, 0.05, 1.5))
        points.append(self.waypoint("retreat_from_tray", above, 0.05, 2.0))
        return self._checked(points, data)

    def actions(
        self, waypoints: list[TrayWaypoint], fps: int = 20
    ) -> Iterator[tuple[str, NDArray[np.float64]]]:
        start = self.initial.copy()
        for waypoint in waypoints:
            target = np.array(waypoint.positions)
            frames = round(waypoint.seconds * fps)
            for frame in range(frames):
                t = (frame + 1) / frames
                yield waypoint.phase, start + (target - start) * (3 * t * t - 2 * t * t * t)
            start = target
            for _ in range(round(1.0 * fps)):
                yield waypoint.phase, target.copy()
