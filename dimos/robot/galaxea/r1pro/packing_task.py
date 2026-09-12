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

"""Selected-bottle ACT task; geometry chooses goals, physics verifies outcomes."""

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP, GraspingTask
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_FPS as FPS
from dimos.robot.galaxea.r1pro.packing import clear_pick_order
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES, PACKING_JOINTS, PACKING_SOURCES
from dimos.robot.galaxea.r1pro.packing_state import (
    PackingResult,
    open_gripper_at_home,
    plan_bottle_goal,
    score_packing,
)


class PackingTask(GraspingTask):
    """One policy skill repeatedly selects, picks, places, and returns home.

    Simulator geometry supplies the selected source and target once per pick.
    Policy observations contain no phase clock, teacher actions, or IK solutions.
    """

    def __init__(self, scene: Path, *, images: bool = True) -> None:
        super().__init__(scene, images=images)
        self.selected = 0
        self.goal = np.zeros(8, dtype=np.float32)
        self.evidence: dict[int, tuple[float, bool]] = {}
        self.reset_packing(0)

    def reset_packing(self, seed: int, jitter: float = 0.006) -> None:
        """Reset all five bottles upright; no subsequent object-pose writes."""
        if not 0 <= jitter <= 0.01:
            raise ValueError("Packing position jitter must be between zero and one centimetre")
        # Reset directly: the single-bottle reset would move bottle 1 into this tray.
        mujoco.mj_resetData(self.model, self.data)  # type: ignore[attr-defined]
        self.data.qpos[self.qids] = self.home
        self.data.ctrl[self.aids] = self.home
        for side in ("left", "right"):
            self.data.joint(f"{side}_gripper_follower").qpos[:] = 0.05
        rng = np.random.default_rng(seed)
        for name, xy in zip(PACKING_JOINTS, PACKING_SOURCES, strict=True):
            pos = np.array(xy) + rng.uniform(-jitter, jitter, 2)
            self.data.joint(name).qpos[:] = (*pos, 0.771, 1, 0, 0, 0)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(0.6 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.evidence = {}
        self.goal[:] = 0
        self.select_bottle(0)

    def pick_order(self, seed: int | None = None) -> list[int]:
        """Clear front bottles first; an optional seed randomizes accessible choices."""
        positions = tuple(
            (float(self.data.body(name).xpos[0]), float(self.data.body(name).xpos[1]))
            for name in PACKING_BODIES
        )
        priority = (
            tuple(map(int, np.random.default_rng(seed).permutation(5)))
            if seed is not None
            else None
        )
        return clear_pick_order(positions, priority)

    def select_bottle(self, index: int) -> bool:
        """Choose the next empty slot; return False without movement when full."""
        if not 0 <= index < len(PACKING_BODIES):
            raise ValueError("Unknown bottle index")
        goal = plan_bottle_goal(self.data, index)
        if goal is None:
            return False
        self.selected = index
        self.bottle_id = self.model.body(PACKING_BODIES[index]).id
        self.bottle_geoms = set(map(int, np.flatnonzero(self.model.geom_bodyid == self.bottle_id)))
        self.goal[:] = goal
        source = goal[:3]
        self.initial_height = float(source[2])
        self.peak_lift = 0.0
        self.bilateral_grasp = False
        return True

    def observation(self, *, render_images: bool = True) -> dict[str, NDArray[Any]]:
        return {
            **super().observation(render_images=render_images),
            "observation.environment_state": self.goal.copy(),
        }

    def result(self) -> PackingResult:
        return score_packing(
            self.data,
            self.selected,
            peak_lift=self.peak_lift,
            bilateral_grasp=self.bilateral_grasp,
            touching_pads=self.touching_pads(),
        )

    def pick_complete(self) -> bool:
        """Containment plus a clear, open gripper at home before the next pick."""
        return self.result().success and open_gripper_at_home(self.data)

    def remember_result(self) -> None:
        self.evidence[self.selected] = (self.peak_lift, self.bilateral_grasp)

    def _bottle_contacts(self, index: int) -> set[int]:
        body_id = self.model.body(PACKING_BODIES[index]).id
        geoms = set(map(int, np.flatnonzero(self.model.geom_bodyid == body_id)))
        touching = set()
        for contact in self.data.contact:
            pair = set(map(int, contact.geom))
            if contact.dist <= 0 and pair & geoms:
                touching.update(pair & self.pad_ids)
        return touching

    def report(self) -> dict[str, Any]:
        rows = []
        for index in range(len(PACKING_BODIES)):
            peak, grasped = self.evidence.get(index, (0.0, False))
            result = score_packing(
                self.data,
                index,
                peak_lift=peak,
                bilateral_grasp=grasped,
                touching_pads=self._bottle_contacts(index),
            )
            rows.append({"bottle": index + 1, **result.to_dict()})
        return {
            "success": all(row["success"] for row in rows),
            "bottles": rows,
            "packed": sum(row["success"] for row in rows),
        }

    def teacher_actions(self) -> Iterator[tuple[str, NDArray[np.float32]]]:
        """Demonstration-only Cartesian waypoints; never called by ACT evaluation."""
        x, y = self.goal[:2]
        bx, by = self.goal[3:5]
        waypoints = [
            ("above", (x, y, 0.92), 0.05, 1.0),
            ("down", (x, y, 0.80), 0.05, 1.2),
            ("grasp", (x, y, 0.80), 0.0, 0.8),
            ("lift", (x, y, 0.92), 0.0, 1.2),
            ("transfer", (bx, by, 0.92), 0.0, 1.5),
            ("place", (bx, by, 0.855), 0.0, 1.0),
            ("release", (bx, by, 0.855), 0.05, 0.8),
            ("retreat", (bx, by, 0.92), 0.05, 1.0),
            ("home", HOME_TCP, 0.05, 1.0),
        ]
        seed = self.data.qpos[self.arm_qids].copy()
        for phase, target, opening, seconds in waypoints:
            start_tcp = self.data.site_xpos[self.tcp_id].copy()
            start_opening = float(self.data.ctrl[self.aids[-1]])
            frames = round(seconds * FPS)
            for frame in range(frames):
                t = (frame + 1) / frames
                smooth = 3 * t * t - 2 * t * t * t
                tcp = start_tcp + (np.asarray(target) - start_tcp) * smooth
                seed = self.solve_ik(tcp, seed)
                goal = self.home.copy()
                goal[11:18] = seed
                goal[-1] = start_opening + (opening - start_opening) * smooth
                yield phase, goal.astype(np.float32)
            for _ in range(round(0.3 * FPS)):
                yield phase, goal.astype(np.float32)
        for _ in range(FPS):
            yield "settle", goal.astype(np.float32)
