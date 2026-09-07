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

"""Scene inspection and repeatable reset for the simulated bottle task."""

import time

import numpy as np

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.imitation.policy.skills import PolicyRolloutSpec
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    dual_openyam_arm_joints,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


class SimDemoSkills(Module):
    _sim: MujocoSimModule
    _pick: PickAndPlaceModule
    _manipulation: ManipulationSpec
    _policy: PolicyRolloutSpec | None = None

    @skill
    def inspect_sim_scene(self) -> SkillResult:
        """Read bottle positions and bin bounds in world meters from simulation."""
        names = ["bin_container", *(f"bottle_{i}" for i in range(1, 7))]
        points = np.asarray(self._sim.sample_body_surface("bin_container", 4096))
        if points.size == 0:
            return SkillResult.fail("NOT_CONFIGURED", "Bin geometry is unavailable")
        poses = self._sim.get_body_poses(names)
        contained = {
            name: inside_bin(self._sim.sample_body_surface(name, 4096), points.tolist())
            for name in poses
            if name.startswith("bottle_")
        }
        return SkillResult.ok(
            "Simulation ground truth",
            poses=poses,
            inside_bin=contained,
            bin_lower=points.min(axis=0).tolist(),
            bin_upper=points.max(axis=0).tolist(),
        )

    @skill
    def reset_scene(self) -> SkillResult:
        """Stop motion, return both arms home, and restore the simulated bottles."""
        if self._policy is not None:
            status = self._policy.stop_rollout()
            if status["active"]:
                return SkillResult.fail("EXECUTION_FAILED", str(status["last_error"]))
        self._manipulation.cancel()
        targets: dict[str, JointState] = {}
        for side in ("left", "right"):
            group = f"{side}_manipulator"
            gripper = self._manipulation.set_gripper_position(0.0, group)
            if not gripper.succeeded:
                return SkillResult.fail("EXECUTION_FAILED", gripper.message)
            targets[group] = JointState(
                name=dual_openyam_arm_joints(side), position=list(DUAL_OPENYAM_HOME_PER_ARM)
            )
        plan = self._manipulation.plan_to_joints(targets)
        if not plan.succeeded:
            return SkillResult.fail("PLANNING_FAILED", plan.message)
        result = self._manipulation.execute(blocking=True)
        if not result.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", result.message)
        if not self._sim.reset():
            return SkillResult.fail("EXECUTION_FAILED", "Simulation reset failed")
        self._pick.reset_selection()
        if not self._await_scene_settled():
            return SkillResult.fail("EXECUTION_FAILED", "Scene did not settle after reset")
        return SkillResult.ok("Both arms home; scene and held-object state reset")

    def _await_scene_settled(self) -> bool:
        # Scene registration caches its scan. Returning during the initial drop
        # produces a grasp for a bottle pose that no longer exists at execution.
        names = ["bin_container", "bottle_1", "bottle_4", "bottle_6"]
        deadline = time.monotonic() + 3.0
        quiet_since = time.monotonic()
        reference: np.ndarray | None = None
        while time.monotonic() < deadline:
            poses = self._sim.get_body_poses(names)
            if set(poses) != set(names):
                return False
            current = np.asarray([poses[name] for name in names])
            if not np.all(np.isfinite(current)):
                return False
            moved = reference is None or bool(
                np.any(np.linalg.norm(current[:, :3] - reference[:, :3], axis=1) > 0.0005)
                or np.any(np.abs(np.sum(current[:, 3:] * reference[:, 3:], axis=1)) < np.cos(0.005))
            )
            if moved:
                reference = current
                quiet_since = time.monotonic()
            elif time.monotonic() - quiet_since >= 0.25:
                return True
            time.sleep(0.05)
        return False


def planning_group_for_y(y: float) -> str:
    return "left_manipulator" if y > 0.0 else "right_manipulator"


def bounds(points: list[list[float]]) -> tuple[np.ndarray, np.ndarray]:
    cloud = np.asarray(points, dtype=float)
    if cloud.ndim != 2 or cloud.shape[0] == 0 or cloud.shape[1] != 3:
        raise ValueError("No object surface points")
    return cloud.min(axis=0), cloud.max(axis=0)


def inside_bin(
    object_points: list[list[float]], bin_points: list[list[float]], margin: float = 0.003
) -> bool:
    """Require the complete sampled object inside the round bin's rim and depth."""
    lower, upper = bounds(object_points)
    bin_lower, bin_upper = bounds(bin_points)
    center = (bin_lower[:2] + bin_upper[:2]) / 2
    radius = float(np.min(bin_upper[:2] - bin_lower[:2]) / 2) - margin
    return bool(
        np.all(lower[:2] >= bin_lower[:2] + margin)
        and np.all(upper[:2] <= bin_upper[:2] - margin)
        and np.all(np.linalg.norm(np.asarray(object_points)[:, :2] - center, axis=1) <= radius)
        and lower[2] >= bin_lower[2] - margin
        and upper[2] <= bin_upper[2] + margin
    )
