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

"""Physical selected-object task with a demonstration-only SDK teacher."""

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP, GraspingTask, TaskResult
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_FPS as FPS
from dimos.robot.galaxea.r1pro.object_packing_scene import ObjectLayout
from dimos.robot.galaxea.r1pro.object_packing_state import ObjectPackingState


class ObjectPackingTask(GraspingTask):
    def __init__(self, scene: Path, layout: ObjectLayout, *, images: bool = True) -> None:
        self.layout = layout
        self._state: ObjectPackingState | None = None
        super().__init__(scene, images=images, object_body=layout.objects[0].name)
        self.home[4:11] = (0.6, 0.1, 0.0, -1.9, 0.0, 0.0, 0.0)
        self.reset(layout.seed)
        self._state = ObjectPackingState(self.model, self.data, self.layout, self.home)
        self._kinematics: HomeKinematics | None = None

    @property
    def state(self) -> ObjectPackingState:
        assert self._state is not None
        return self._state

    @property
    def selected(self) -> int:
        return self.state.selected

    @property
    def target(self) -> NDArray[np.float64]:
        return self.state.target

    def reset(self, seed: int, jitter: float = 0.0, *, start_joint_jitter: float = 0.0) -> None:
        """Restore this compiled layout; geometry randomization happens at generation."""
        if jitter != 0:
            raise ValueError("Generate a new layout instead of perturbing compiled geometry")
        mujoco.mj_resetData(self.model, self.data)  # type: ignore[attr-defined]
        if not 0 <= start_joint_jitter <= 0.05:
            raise ValueError("Start variation must be between zero and 0.05 radians")
        initial = self.home.copy()
        if start_joint_jitter:
            initial[11:18] += np.random.default_rng(seed + 911).uniform(
                -start_joint_jitter, start_joint_jitter, 7
            )
        # Start-state augmentation is confined to reset, never applied during a rollout.
        self.data.qpos[self.qids] = initial
        self.data.ctrl[self.aids] = initial
        for side in ("left", "right"):
            self.data.joint(f"{side}_gripper_follower").qpos[:] = 0.05
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(0.6 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.peak_lift, self.bilateral_grasp = 0.0, False
        self.initial_height = float(self.data.body(self.bottle_id).xpos[2])
        if self._state is not None:
            self._state = ObjectPackingState(self.model, self.data, self.layout, self.home)

    def geometry(self, index: int) -> dict[str, Any]:
        return self.state.geometry(index)

    def inventory(self) -> list[dict[str, Any]]:
        return self.state.inventory()

    def select_object(self, index: int) -> bool:
        if not self.state.select_object(index):
            return False
        self.bottle_id = self.state.bottle_id
        self.bottle_geoms = self.state.bottle_geoms
        self.initial_height = self.state.initial_height
        self.peak_lift, self.bilateral_grasp = 0.0, False
        return True

    def observation(self, *, render_images: bool = True) -> dict[str, NDArray[Any]]:
        obs = super().observation(render_images=render_images)
        obs["observation.environment_state"] = self.state.goal()
        return obs

    def result(self) -> TaskResult:
        self.state.peak_lift = self.peak_lift
        self.state.bilateral_grasp = self.bilateral_grasp
        return self.state.result()

    def pick_complete(self) -> bool:
        return bool(
            self.result().success and np.max(np.abs(self.data.qpos[self.qids] - self.home)) < 0.015
        )

    def validate(self, initial: list[dict[str, Any]]) -> None:
        self.state.validate(initial)

    def teacher_actions(self) -> Iterator[tuple[str, NDArray[np.float32]]]:
        """SDK-generated training actions; never called by learned evaluation."""
        if self._kinematics is None:
            self._kinematics = HomeKinematics(self.model, self.data, lock_lower_torso=True)
        kinematics = self._kinematics
        source = self.data.body(self.bottle_id).xpos.copy()
        obj = self.layout.objects[self.selected]
        grasp = source + np.array([0, 0, min(0.03, obj.half_size[2] * 0.45)])
        clearance = max(o.position[2] + o.half_size[2] for o in self.layout.objects) + 0.16
        probe = mujoco.MjData(self.model)
        probe.qpos[:] = self.data.qpos
        mujoco.mj_forward(self.model, probe)

        def move(
            phase: str, target: NDArray[Any], opening: float, seconds: float
        ) -> Iterator[tuple[str, NDArray[np.float32]]]:
            start = self.data.site("right_tcp").xpos.copy()
            start_opening = float(self.data.ctrl[self.aids[-1]])
            for frame in range(round(seconds * FPS)):
                t = (frame + 1) / (seconds * FPS)
                u = t * t * (3 - 2 * t)
                xyz = start + (target - start) * u
                goal = kinematics.solve(
                    probe,
                    {"right": xyz},
                    torso_yaw_only=phase not in ("lift", "clear_sources", "lower_clear"),
                    position_tolerance=0.001
                    if phase in ("approach", "grasp", "place", "seek_support", "release")
                    else 0.003,
                    orientation_tolerance=0.005,
                )
                if np.max(np.abs(goal[:18] - self.data.ctrl[self.aids[:18]])) > 0.14:
                    raise RuntimeError("Teacher IK produced a discontinuous joint command")
                goal[-1] = start_opening + (opening - start_opening) * u
                probe.qpos[self.qids] = goal
                mujoco.mj_forward(self.model, probe)
                yield phase, goal.astype(np.float32)
            for _ in range(FPS // 2):
                yield phase, goal.astype(np.float32)

        above = np.r_[source[:2], clearance]
        yield from move("above", np.r_[source[:2], 0.92], 0.05, 1.5)
        yield from move("approach", grasp, 0.05, 1.8)
        yield from move("grasp", grasp, 0.0, 0.8)
        yield from move("lift", above, 0.0, 1.5)
        if not self.bilateral_grasp:
            raise RuntimeError("Teacher did not establish a bilateral physical grasp")
        # Measure the actual carried offset after lift, including any slip. This
        # informs only the demonstrator; ACT receives current scene observations.
        offset = self.data.site("right_tcp").xpos - self.data.body(self.bottle_id).xpos
        destination = self.target + offset
        staging = np.array([0.33, -0.24, clearance])
        yield from move("clear_sources", staging, 0.0, 1.8)
        staging[2] = 0.94
        yield from move("lower_clear", staging, 0.0, 1.5)
        start_torso = self.data.qpos[self.qids[:4]].copy()
        for frame in range(3 * FPS):
            t = (frame + 1) / (3 * FPS)
            probe.qpos[self.qids[:4]] = start_torso + (self.home[:4] - start_torso) * t * t * (
                3 - 2 * t
            )
            mujoco.mj_forward(self.model, probe)
            goal = kinematics.solve(
                probe,
                {"right": staging},
                torso_yaw_only=True,
                position_tolerance=0.003,
                orientation_tolerance=0.005,
            )
            if np.max(np.abs(goal[:18] - self.data.ctrl[self.aids[:18]])) > 0.14:
                raise RuntimeError("Teacher torso return produced a discontinuous joint command")
            goal[-1] = 0.0
            probe.qpos[self.qids] = goal
            yield "restore_torso", goal.astype(np.float32)
        for _ in range(FPS):
            yield "restore_torso", goal.astype(np.float32)
        # Refresh the held offset after the torso settles.
        offset = self.data.site("right_tcp").xpos - self.data.body(self.bottle_id).xpos
        destination = self.target + offset
        yield from move("transfer", np.r_[destination[:2], 0.94], 0.0, 1.8)
        yield from move("place", destination, 0.0, 1.8)
        for _ in range(8):
            if self.geometry(self.selected)["supported"]:
                break
            destination = destination - np.array([0, 0, 0.003])
            yield from move("seek_support", destination, 0.0, 0.25)
        if not self.geometry(self.selected)["supported"]:
            raise RuntimeError("Teacher found no tray support before release")
        yield from move("release", destination, 0.05, 0.8)
        yield from move("retreat", np.r_[destination[:2], 0.94], 0.05, 1.5)
        yield from move("home", np.asarray(HOME_TCP), 0.05, 1.5)
        initial = self.data.ctrl[self.aids].copy()
        for frame in range(2 * FPS):
            t = (frame + 1) / (2 * FPS)
            yield (
                "home_posture",
                (initial + (self.home - initial) * t * t * (3 - 2 * t)).astype(np.float32),
            )
        for _ in range(2 * FPS):
            yield "settle", self.home.astype(np.float32)
