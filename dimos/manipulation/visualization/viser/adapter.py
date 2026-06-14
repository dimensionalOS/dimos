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

from __future__ import annotations

from collections.abc import Mapping, Sequence
import time
from typing import TYPE_CHECKING, Protocol, cast, runtime_checkable

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import matrix_to_pose

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import JointPath, RobotName, WorldRobotID
    from dimos.msgs.geometry_msgs.Pose import Pose


@runtime_checkable
class _RobotInfoProvider(Protocol):
    def get_robot_info(self, robot_name: RobotName) -> Mapping[str, object] | None: ...


@runtime_checkable
class _InitJointsProvider(Protocol):
    def get_init_joints(self, robot_name: RobotName) -> JointState | None: ...


@runtime_checkable
class _JointTargetEvaluator(Protocol):
    def evaluate_joint_target(
        self, joints: JointState | None, robot_name: RobotName
    ) -> Mapping[str, object]: ...


@runtime_checkable
class _PoseTargetEvaluator(Protocol):
    def evaluate_pose_target(self, pose: Pose, robot_name: RobotName) -> Mapping[str, object]: ...


@runtime_checkable
class _StateProvider(Protocol):
    def get_state(self) -> object: ...


@runtime_checkable
class _ErrorProvider(Protocol):
    def get_error(self) -> str: ...


def copy_joint_state(joint_state: JointState | None) -> JointState | None:
    """Make a local copy of a JointState-like message for rendering."""
    if joint_state is None:
        return None
    return JointState(
        {
            "name": list(joint_state.name),
            "position": list(joint_state.position),
            "velocity": list(joint_state.velocity),
            "effort": list(joint_state.effort),
        }
    )


class InProcessViserAdapter:
    """Small in-process boundary between Viser callbacks and manipulation internals."""

    def __init__(
        self,
        *,
        world_monitor: WorldMonitor,
        manipulation_module: ManipulationModule | None,
    ) -> None:
        self._world_monitor = world_monitor
        self._module = manipulation_module
        # Last successful live-eval IK solution per robot. Used as the IK seed for
        # the next gizmo drag so each update is a tiny differential step (fast),
        # instead of re-solving from the robot's home pose every time.
        self._last_eval_solution: dict[RobotName, JointState] = {}
        # Dedicated high-gain Pink solver for the LIVE gizmo preview only (Plan keeps
        # the module's accurate solver). gain=1.0/dt=0.1 converges a big drag in ~3
        # iters vs ~7 at the planning default (gain=2.0 diverges, so 1.0 is the cap).
        self._preview_kinematics = self._build_preview_kinematics()

    @staticmethod
    def _build_preview_kinematics() -> object | None:
        try:
            from dimos.manipulation.planning.factory import create_kinematics
            from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig

            return create_kinematics(
                config=PinkKinematicsConfig(gain=1.0, dt=0.1, max_iterations=100)
            )
        except Exception:
            return None

    def list_robots(self) -> list[RobotName]:
        if self._module is None:
            return []
        return list(self._module._robots.keys())

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        if self._module is None:
            return []
        return [
            (name, robot_id, config)
            for name, (robot_id, config, _) in list(self._module._robots.items())
        ]

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        if self._module is None:
            return None
        entry = self._module._robots.get(robot_name)
        return entry[0] if entry is not None else None

    def robot_name_for_id(self, robot_id: WorldRobotID) -> RobotName | None:
        if self._module is None:
            return None
        for robot_name, (candidate_id, _, _) in list(self._module._robots.items()):
            if candidate_id == robot_id:
                return robot_name
        return None

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        if self._module is None:
            return None
        entry = self._module._robots.get(robot_name)
        return entry[1] if entry is not None else None

    def get_robot_info(self, robot_name: RobotName) -> dict[str, object] | None:
        if self._module is None:
            return None
        if isinstance(self._module, _RobotInfoProvider):
            info = self._module.get_robot_info(robot_name)
            return None if info is None else dict(info)
        config = self.get_robot_config(robot_name)
        if config is None:
            return None
        init = self.get_init_joints(robot_name)
        try:
            config_name = config.name
        except AttributeError:
            config_name = robot_name
        try:
            end_effector_link = config.end_effector_link
        except AttributeError:
            end_effector_link = None
        try:
            base_link = config.base_link
        except AttributeError:
            base_link = None
        try:
            home_joints = config.home_joints
        except AttributeError:
            home_joints = None
        return {
            "name": config_name,
            "joint_names": list(config.joint_names),
            "end_effector_link": end_effector_link,
            "base_link": base_link,
            "home_joints": list(home_joints) if home_joints is not None else None,
            "init_joints": list(init.position) if init is not None else None,
        }

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        if self._module is None:
            return None
        if isinstance(self._module, _InitJointsProvider):
            return copy_joint_state(self._module.get_init_joints(robot_name))
        try:
            init_joints = self._module._init_joints
        except AttributeError:
            init_joints = {}
        copied = copy_joint_state(init_joints.get(robot_name))
        if copied is not None:
            return copied
        config = self.get_robot_config(robot_name)
        if config is None:
            return None
        try:
            positions = config.init_joints
        except AttributeError:
            return None
        return self.joints_from_values(config.joint_names, positions)

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return copy_joint_state(self._world_monitor.get_current_joint_state(robot_id))

    def is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        robot_id = self.robot_id_for_name(robot_name)
        return True if robot_id is None else self._world_monitor.is_state_stale(robot_id, max_age)

    def get_ee_pose(
        self, robot_name: RobotName, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return self._world_monitor.get_ee_pose(robot_id, copy_joint_state(joint_state))

    def evaluate_joint_target(self, joints: JointState, robot_name: RobotName) -> dict[str, object]:
        """Evaluate a joint target through WorldMonitor helpers, not raw WorldSpec access."""
        if isinstance(self._module, _JointTargetEvaluator):
            result = dict(self._module.evaluate_joint_target(copy_joint_state(joints), robot_name))
            result["joint_state"] = copy_joint_state(
                cast("JointState | None", result.get("joint_state"))
            )
            return result
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return {
                "success": False,
                "status": "NO_ROBOT",
                "message": f"Unknown robot: {robot_name}",
                "collision_free": False,
                "ee_pose": None,
            }
        target = copy_joint_state(joints)
        if target is None:
            return {
                "success": False,
                "status": "NO_TARGET",
                "message": "No joint target provided",
                "collision_free": False,
                "ee_pose": None,
            }
        collision_free = self._world_monitor.is_state_valid(robot_id, target)
        return {
            "success": True,
            "status": "FEASIBLE" if collision_free else "COLLISION",
            "message": "Target is collision-free" if collision_free else "Target is in collision",
            "collision_free": collision_free,
            "pose": self._world_monitor.get_ee_pose(robot_id, target),
            "ee_pose": self._world_monitor.get_ee_pose(robot_id, target),
            "joint_state": target,
        }

    def evaluate_pose_target(self, pose: Pose, robot_name: RobotName) -> dict[str, object]:
        """Evaluate a Cartesian target through module/WorldMonitor helper boundaries."""
        if isinstance(self._module, _PoseTargetEvaluator):
            result = dict(self._module.evaluate_pose_target(pose, robot_name))
            result["joint_state"] = copy_joint_state(
                cast("JointState | None", result.get("joint_state"))
            )
            return result
        _t_enter = time.perf_counter()
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNKNOWN_ROBOT",
                "message": f"Unknown robot: {robot_name}",
                "collision_free": False,
            }
        current = self._world_monitor.get_current_joint_state(robot_id)
        # Prefer the dedicated high-gain preview solver; fall back to the module's.
        kinematics = self._preview_kinematics
        if kinematics is None and self._module is not None:
            kinematics = getattr(self._module, "_kinematics", None)
        if current is None or kinematics is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Planning is not initialized or current state is unavailable",
                "collision_free": False,
            }
        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )
        # Seed from the LAST live solution (a tiny delta while dragging) instead of the
        # robot's home pose, so Pink's differential solver converges in a few iterations
        # rather than the ~100-200 it needs to crawl from home (dt=0.05/gain=0.5).
        seed = self._last_eval_solution.get(robot_name) or current
        _t0 = time.perf_counter()
        # Live gizmo feedback: ONE differential Pink solve, NO Drake collision check, and a
        # loose preview tolerance (1cm / ~6deg) so it converges fast. Each Drake collision
        # check clones the full-body context (~0.2-0.8s); Plan re-validates collision and
        # solves to full tolerance before executing.
        ik = kinematics.solve(
            world=self._world_monitor.world,
            robot_id=robot_id,
            target_pose=target_pose,
            seed=seed,
            position_tolerance=0.01,
            orientation_tolerance=0.1,
            check_collision=False,
            max_attempts=1,
        )
        _t1 = time.perf_counter()
        joint_state = copy_joint_state(ik.joint_state if ik.is_success() else None)
        collision_free = joint_state is not None
        if joint_state is not None:
            self._last_eval_solution[robot_name] = joint_state
        print(
            f"[viz-eval] pre(seed)={_t0 - _t_enter:.3f}s solve={_t1 - _t0:.3f}s "
            f"status={ik.status.name} iters={getattr(ik, 'iterations', '?')}",
            flush=True,
        )
        return {
            "success": joint_state is not None and collision_free,
            "joint_state": joint_state,
            "status": ik.status.name,
            "message": ik.message,
            "position_error": ik.position_error,
            "orientation_error": ik.orientation_error,
            "collision_free": collision_free,
        }

    def evaluate_pose_targets(
        self, pose_targets: Mapping[str, Pose], robot_name: RobotName
    ) -> dict[str, object]:
        """Multi-target (e.g. bimanual) live preview eval: solve all tip poses at once."""
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNKNOWN_ROBOT",
                "message": f"Unknown robot: {robot_name}",
                "collision_free": False,
            }
        current = self._world_monitor.get_current_joint_state(robot_id)
        kinematics = self._preview_kinematics
        if kinematics is None and self._module is not None:
            kinematics = getattr(self._module, "_kinematics", None)
        solve_pose_targets = getattr(kinematics, "solve_pose_targets", None)
        if current is None or solve_pose_targets is None or not pose_targets:
            return {
                "success": False,
                "joint_state": None,
                "status": "UNAVAILABLE",
                "message": "Multi-target IK unavailable",
                "collision_free": False,
            }
        targets = {
            tip: PoseStamped(frame_id="world", position=p.position, orientation=p.orientation)
            for tip, p in pose_targets.items()
        }
        seed = self._last_eval_solution.get(robot_name) or current
        ik = solve_pose_targets(
            world=self._world_monitor.world,
            robot_id=robot_id,
            pose_targets=targets,
            seed=seed,
            position_tolerance=0.01,
            orientation_tolerance=0.1,
            check_collision=False,
            max_attempts=1,
        )
        joint_state = copy_joint_state(ik.joint_state if ik.is_success() else None)
        if joint_state is not None:
            self._last_eval_solution[robot_name] = joint_state
        return {
            "success": joint_state is not None,
            "joint_state": joint_state,
            "status": ik.status.name,
            "message": ik.message,
            "collision_free": joint_state is not None,
        }

    def get_link_pose(self, robot_name: RobotName, link_name: str) -> Pose | None:
        """World-frame pose of a robot link at the current config (for gizmo init)."""
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        world = self._world_monitor.world
        if not getattr(world, "is_finalized", False):
            return None
        try:
            with world.scratch_context() as ctx:
                matrix = world.get_link_pose(ctx, robot_id, link_name)
        except (KeyError, RuntimeError, AttributeError):
            return None
        return matrix_to_pose(np.asarray(matrix, dtype=float))

    def get_planned_path(self, robot_name: RobotName) -> JointPath | None:
        if self._module is None:
            return None
        path = self._module._planned_paths.get(robot_name)
        if path is None:
            return None
        copied = [copy_joint_state(point) for point in path]
        return [point for point in copied if point is not None]

    def get_planned_trajectory_duration(self, robot_name: RobotName) -> float | None:
        if self._module is None:
            return None
        trajectory = self._module._planned_trajectories.get(robot_name)
        if trajectory is None:
            return None
        try:
            return float(trajectory.duration)
        except AttributeError:
            return None

    def get_module_state(self) -> str:
        if self._module is None:
            return "DISCONNECTED"
        if isinstance(self._module, _StateProvider):
            return str(self._module.get_state())
        try:
            state = self._module._state
        except AttributeError:
            return "UNKNOWN"
        try:
            return str(state.name)
        except AttributeError:
            return str(state or "UNKNOWN")

    def get_error(self) -> str:
        if self._module is None:
            return ""
        if isinstance(self._module, _ErrorProvider):
            return self._module.get_error()
        try:
            return str(self._module._error_message)
        except AttributeError:
            return ""

    def plan_to_pose(self, pose: Pose, robot_name: RobotName | None = None) -> bool:
        return False if self._module is None else self._module.plan_to_pose(pose, robot_name)

    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        return False if self._module is None else self._module.plan_to_joints(joints, robot_name)

    def preview_path(self, robot_name: RobotName | None = None) -> bool:
        return False if self._module is None else self._module.preview_path(robot_name=robot_name)

    def execute(self, robot_name: RobotName | None = None) -> bool:
        return False if self._module is None else self._module.execute(robot_name)

    def cancel(self) -> bool:
        return False if self._module is None else self._module.cancel()

    def clear_planned_path(self) -> bool:
        return False if self._module is None else self._module.clear_planned_path()

    def reset(self) -> bool:
        """Reset the manipulation module to IDLE, clearing any fault."""
        if self._module is None:
            return False
        result = self._module.reset()
        return bool(getattr(result, "success", result))

    def go_home(self, robot_name: RobotName | None = None) -> bool:
        """Send the robot to its home configuration."""
        if self._module is None:
            return False
        result = self._module.go_home(robot_name)
        return bool(getattr(result, "success", result))

    @staticmethod
    def joints_from_values(joint_names: Sequence[str], values: Sequence[float]) -> JointState:
        return JointState(
            {
                "name": list(joint_names),
                "position": [float(value) for value in values],
            }
        )
