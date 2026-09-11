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

"""Contact-based R1Pro bottle task and offline demonstration teacher.

Only the teacher and evaluator read object ground truth. A policy receives the
same RGB cameras and measured joint positions used by the live DimOS profile.
There are no object attachments or object-position writes after reset.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Self

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.robot.galaxea.r1pro.grasping_sim import (
    BIN_FLOOR_Z,
    BIN_INNER_HALF_SIZE,
    BIN_XY,
    BOTTLE_HALF_HEIGHT,
    BOTTLE_RADIUS,
    BOTTLE_XY,
    MANIPULATION_JOINTS,
    TABLE_Z,
    VIRTUAL_BASE_JOINTS,
)
from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PICK_PLACE_FPS as FPS,
    R1PRO_PICK_PLACE_IMAGE_SIZE as IMAGE_SIZE,
)

HOME_TCP = (0.33, -0.32, 0.92)


@dataclass(frozen=True)
class TaskResult:
    """Physical success requires lift, release, containment, and settled motion."""

    success: bool
    peak_lift_m: float
    bilateral_grasp: bool
    inside_bin: bool
    released: bool
    settled: bool
    bottle_position: list[float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def score_task(
    data: mujoco.MjData,
    *,
    peak_lift: float,
    bilateral_grasp: bool,
    touching_pads: set[int],
    bottle_name: str = "task_bottle",
    joint_name: str = "task_bottle_free",
    radius: float = BOTTLE_RADIUS,
    half_height: float = BOTTLE_HALF_HEIGHT,
    require_open_gripper: bool = True,
) -> TaskResult:
    """Apply the same physical success criteria in offline and distributed runs."""
    bottle = data.body(bottle_name)
    world_pos = bottle.xpos.copy()
    container = data.body("task_bin")
    bin_rotation = container.xmat.reshape(3, 3)
    pos = bin_rotation.T @ (world_pos - container.xpos)
    # Project the cylinder onto each world axis, including tilt. Checking
    # the centre alone would count bottles balanced across a bin wall.
    axis = bin_rotation.T @ bottle.xmat.reshape(3, 3)[:, 2]
    extent = half_height * np.abs(axis) + radius * np.sqrt(np.maximum(0, 1 - axis**2))
    inside = bool(
        np.all(np.abs(pos[:2]) + extent[:2] < BIN_INNER_HALF_SIZE)
        and abs(pos[2] - extent[2] - (BIN_FLOOR_Z - TABLE_Z)) < 0.008
    )
    released = not touching_pads and (
        not require_open_gripper or data.joint("r1pro/right_gripper").qpos[0] > 0.04
    )
    settled = bool(np.linalg.norm(data.joint(joint_name).qvel) < 0.03)
    success = peak_lift > 0.06 and bilateral_grasp and inside and released and settled
    return TaskResult(
        bool(success),
        peak_lift,
        bilateral_grasp,
        inside,
        bool(released),
        settled,
        world_pos.tolist(),
    )


class GraspingTask:
    """Deterministic MuJoCo task, sharing its MJCF with native deployment."""

    def __init__(self, scene: Path, *, images: bool = True) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.qids = np.array([self.model.joint(n).qposadr[0] for n in MANIPULATION_JOINTS])
        self.aids = np.array([self.model.actuator(n).id for n in MANIPULATION_JOINTS])
        self.base_aids = np.array(
            [
                self.model.actuator(n).id
                for n in VIRTUAL_BASE_JOINTS
                if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) >= 0
            ],
            dtype=np.int64,
        )
        self.limits = np.array([self.model.joint(n).range for n in MANIPULATION_JOINTS])
        self.arm_qids = self.qids[11:18]
        self.arm_dofs = np.array(
            [self.model.joint(n).dofadr[0] for n in MANIPULATION_JOINTS[11:18]]
        )
        self.tcp_id = self.model.site("right_tcp").id
        self.bottle_id = self.model.body("task_bottle").id
        self.pad_ids = {self.model.geom(f"right_finger_pad{i}").id for i in (1, 2)}
        self.bottle_geoms = {self.model.geom(n).id for n in ("bottle_body", "bottle_cap")}
        self.frame_steps = round(1 / FPS / self.model.opt.timestep)
        if not np.isclose(self.frame_steps * self.model.opt.timestep, 1 / FPS):
            raise ValueError("Physics timestep must divide the action period")
        self.home = np.zeros(len(MANIPULATION_JOINTS))
        self.home[:2] = (0.4, -0.4)
        self.home[-2:] = 0.05
        self.data.qpos[self.qids] = self.home
        mujoco.mj_forward(self.model, self.data)
        self.home[11:18] = self.solve_ik(HOME_TCP)
        self.renderer = mujoco.Renderer(self.model, IMAGE_SIZE, IMAGE_SIZE) if images else None
        self.peak_lift = 0.0
        self.bilateral_grasp = False
        self.reset(0)

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def reset(self, seed: int, jitter: float = 0.012) -> None:
        """Reset the complete episode; randomization is confined to this call."""
        mujoco.mj_resetData(self.model, self.data)  # type: ignore[attr-defined]
        self.data.qpos[self.qids] = self.home
        self.data.ctrl[self.aids] = self.home
        for side in ("left", "right"):
            self.data.joint(f"{side}_gripper_follower").qpos[:] = 0.05
        rng = np.random.default_rng(seed)
        self.data.joint("task_bottle_free").qpos[:2] = np.array(BOTTLE_XY) + rng.uniform(
            -jitter, jitter, 2
        )
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(0.4 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.initial_height = float(self.data.body("task_bottle").xpos[2])
        self.peak_lift = 0.0
        self.bilateral_grasp = False

    def observation(self) -> dict[str, NDArray[Any]]:
        observation = {"observation.state": self.data.qpos[self.qids].astype(np.float32)}
        if self.renderer is not None:
            for camera in ("head", "right_wrist"):
                self.renderer.update_scene(self.data, camera=camera)
                observation[f"observation.images.{camera}"] = self.renderer.render().copy()
        return observation

    def step(self, action: NDArray[Any], *, base_target: NDArray[Any] | None = None) -> None:
        target = np.asarray(action, dtype=np.float64)
        if target.shape != self.home.shape or not np.isfinite(target).all():
            raise ValueError("Action must be a finite vector in the declared joint order")
        self.data.ctrl[self.aids] = np.clip(target, self.limits[:, 0], self.limits[:, 1])
        base_start = self.data.ctrl[self.base_aids].copy()
        if base_target is not None and (
            base_target.shape != (3,)
            or len(self.base_aids) != 3
            or not np.isfinite(base_target).all()
        ):
            raise ValueError("Base targets require three finite planar coordinates")
        for substep in range(self.frame_steps):
            if base_target is not None:
                # The native coordinator interpolates base trajectories between
                # frames. A 20 Hz staircase would create artificial accelerations.
                self.data.ctrl[self.base_aids] = (
                    base_start + (base_target - base_start) * (substep + 1) / self.frame_steps
                )
            mujoco.mj_step(self.model, self.data)
            lift = float(self.data.xpos[self.bottle_id, 2]) - self.initial_height
            self.peak_lift = max(self.peak_lift, lift)
            if lift > 0.04 and self.touching_pads() == self.pad_ids:
                self.bilateral_grasp = True

    def touching_pads(self) -> set[int]:
        touching = set()
        for contact in self.data.contact:
            geoms = set(map(int, contact.geom))
            if contact.dist <= 0 and geoms & self.bottle_geoms:
                touching.update(geoms & self.pad_ids)
        return touching

    def result(self) -> TaskResult:
        return score_task(
            self.data,
            peak_lift=self.peak_lift,
            bilateral_grasp=self.bilateral_grasp,
            touching_pads=self.touching_pads(),
        )

    def solve_ik(
        self, target: tuple[float, float, float] | NDArray[Any], seed: NDArray[Any] | None = None
    ) -> NDArray[np.float64]:
        """Solve a vertical right-gripper pose using a separate kinematic state."""
        probe = mujoco.MjData(self.model)
        probe.qpos[:] = self.data.qpos
        limits = self.limits[11:18]
        rng = np.random.default_rng(4)
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        for attempt in range(16):
            probe.qpos[self.arm_qids] = (
                (self.data.qpos[self.arm_qids] if seed is None else seed)
                if attempt == 0
                else rng.uniform(limits[:, 0] + 0.1, limits[:, 1] - 0.1)
            )
            for _ in range(250):
                mujoco.mj_kinematics(self.model, probe)  # type: ignore[attr-defined]
                mujoco.mj_comPos(self.model, probe)  # type: ignore[attr-defined]
                ep = np.asarray(target) - probe.site_xpos[self.tcp_id]
                er = Rotation.from_matrix(probe.site_xmat[self.tcp_id].reshape(3, 3).T).as_rotvec()
                if np.linalg.norm(ep) < 0.0005 and np.linalg.norm(er) < 0.005:
                    return np.asarray(probe.qpos[self.arm_qids], dtype=np.float64).copy()
                mujoco.mj_jacSite(self.model, probe, jac_pos, jac_rot, self.tcp_id)  # type: ignore[attr-defined]
                jac = np.vstack((jac_pos[:, self.arm_dofs], 0.25 * jac_rot[:, self.arm_dofs]))
                delta = jac.T @ np.linalg.solve(
                    jac @ jac.T + 1e-4 * np.eye(6), np.r_[ep, 0.25 * er]
                )
                delta *= min(1.0, 0.15 / max(float(np.linalg.norm(delta)), 1e-10))
                probe.qpos[self.arm_qids] = np.clip(
                    probe.qpos[self.arm_qids] + delta, limits[:, 0] + 0.001, limits[:, 1] - 0.001
                )
        raise RuntimeError(f"Unreachable right TCP target: {target}")

    def teacher_actions(self) -> Iterator[tuple[str, NDArray[np.float32]]]:
        """Yield position targets; the caller records observation before step()."""
        x, y = self.data.body("task_bottle").xpos[:2]
        bx, by = BIN_XY
        waypoints = [
            ("above", (x, y, 0.92), 0.05, 1.0),
            ("down", (x, y, 0.80), 0.05, 1.2),
            ("grasp", (x, y, 0.80), 0.0, 0.8),
            ("lift", (x, y, 0.92), 0.0, 1.2),
            ("transfer", (bx, by, 0.92), 0.0, 1.5),
            ("place", (bx, by, 0.83), 0.0, 1.0),
            ("release", (bx, by, 0.83), 0.05, 0.8),
            ("retreat", (bx, by, 0.92), 0.05, 1.0),
        ]
        seed = self.home[11:18].copy()
        for phase, target, opening, seconds in waypoints:
            goal = self.home.copy()
            seed = self.solve_ik(target, seed)
            goal[11:18] = seed
            goal[-1] = opening
            start = self.data.ctrl[self.aids].copy()
            frames = round(seconds * FPS)
            for frame in range(frames):
                t = (frame + 1) / frames
                yield phase, (start + (goal - start) * (3 * t**2 - 2 * t**3)).astype(np.float32)
            for _ in range(round(0.2 * FPS)):
                yield phase, goal.astype(np.float32)
        for _ in range(FPS):
            yield "settle", goal.astype(np.float32)
