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

"""Higher-clearance demonstrations for individually requested ACT bottle picks."""

from collections.abc import Iterator
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics
from dimos.robot.galaxea.r1pro.packing_task import PackingTask
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion


class FlexiblePackingTask(PackingTask):
    """SDK torso/arm IK teacher; never used as a learned rollout fallback."""

    def __init__(self, scene: Path, *, images: bool = True) -> None:
        super().__init__(scene, images=images)
        # Park the unused left arm clear of the worktop before torso motion.
        self.home[5] = 0.8
        self.reset_packing(0)
        self.kinematics = HomeKinematics(self.model, self.data, lock_lower_torso=True)

    def pick_order(self, seed: int | None = None) -> list[int]:
        return list(map(int, np.random.default_rng(seed).permutation(5)))

    def teacher_actions(self) -> Iterator[tuple[str, NDArray[np.float32]]]:
        x, y = self.goal[:2]
        bx, by = self.goal[3:5]
        clearance = 1.04 if self.selected in (2, 4) else 0.98
        waypoints = [
            ("above", (x, y, 0.92), 0.05, 1.2),
            ("down", (x, y, 0.80), 0.05, 1.2),
            ("grasp", (x, y, 0.80), 0.0, 0.8),
            ("lift", (x, y, clearance), 0.0, 2.0),
            ("clear_sources", (x, -0.20, clearance), 0.0, 2.0),
            ("lower_clear", (x, -0.20, 0.92), 0.0, 1.5),
            ("restore_torso", (x, -0.20, 0.92), 0.0, 2.0),
            ("transfer", (bx, by, 0.92), 0.0, 1.5),
            ("place", (bx, by, 0.855), 0.0, 1.5),
            ("release", (bx, by, 0.855), 0.05, 0.8),
            ("retreat", (bx, by, 0.92), 0.05, 1.2),
            ("home", HOME_TCP, 0.05, 1.5),
        ]
        probe = mujoco.MjData(self.model)
        probe.qpos[:] = self.data.qpos
        probe.qpos[self.qids] = self.home
        mujoco.mj_forward(self.model, probe)
        for phase, target, opening, seconds in waypoints:
            if phase == "restore_torso":
                motion = TrayMotion(self.model, probe)
                start_torso = probe.qpos[self.qids[:4]].copy()
                start_xyz = probe.site("right_tcp").xpos.copy()
                for frame in range(60):
                    t = (frame + 1) / 60
                    u = 3 * t * t - 2 * t * t * t
                    motion.probe.qpos[motion.qids[:4]] = (
                        start_torso + (self.home[:4] - start_torso) * u
                    )
                    xyz = start_xyz + (np.asarray(target) - start_xyz) * u
                    motion.arm_pose("right", xyz)
                    goal = self.home.copy()
                    goal[:4] = motion.probe.qpos[motion.qids[:4]]
                    goal[11:18] = motion.probe.qpos[motion.qids[11:18]]
                    goal[-1] = 0.0
                    yield phase, goal.astype(np.float32)
                for _ in range(20):
                    yield phase, goal.astype(np.float32)
                probe.qpos[:] = self.data.qpos
                probe.qpos[self.qids] = goal
                mujoco.mj_forward(self.model, probe)
                continue
            start = self.data.site("right_tcp").xpos.copy()
            start_opening = float(self.data.ctrl[self.aids[-1]])
            frames = round(seconds * 20)
            for frame in range(frames):
                t = (frame + 1) / frames
                blend = 3 * t * t - 2 * t * t * t
                xyz = start + (np.asarray(target) - start) * blend
                if phase in ("lift", "clear_sources", "lower_clear"):
                    goal = self.kinematics.solve(probe, {"right": xyz})
                else:
                    motion = TrayMotion(self.model, probe)
                    motion.probe.qpos[motion.qids[:4]] = self.home[:4]
                    motion.arm_pose("right", xyz)
                    goal = self.home.copy()
                    goal[11:18] = motion.probe.qpos[motion.qids[11:18]]
                goal[-1] = start_opening + (opening - start_opening) * blend
                probe.qpos[self.qids] = goal
                mujoco.mj_forward(self.model, probe)
                yield phase, goal.astype(np.float32)
            for _ in range(20):
                yield phase, goal.astype(np.float32)
        initial = self.data.ctrl[self.aids].copy()
        for frame in range(40):
            t = (frame + 1) / 40
            blend = 3 * t * t - 2 * t * t * t
            yield "home_posture", (initial + (self.home - initial) * blend).astype(np.float32)
        for _ in range(20):
            yield "settle", self.home.astype(np.float32)
