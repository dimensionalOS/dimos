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

"""Manipulation Module - Motion planning with ControlCoordinator execution.

Base module providing core manipulation infrastructure:
- @rpc: Low-level building blocks (plan_to_pose, plan_to_joints, preview_plan, execute)
- @skill (short-horizon): Single-step actions (move_to_pose, open_gripper, go_home, go_init)

Subclass PickAndPlaceModule (pick_and_place_module.py) adds perception integration
(scan_objects, get_scene_info) and long-horizon skills (pick, place, pick_and_place).
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from enum import Enum
import threading
import time
import traceback
from typing import TYPE_CHECKING, Any, TypeAlias, cast

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.planning.factory import (
    KinematicsName,
    PlannerName,
    WorldBackend,
    create_planning_specs,
    create_world,
)
from dimos.manipulation.planning.groups.identifiers import (
    assert_global_joint_names,
    assert_local_joint_names,
    local_joint_name_from_global,
    make_global_joint_names,
)
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.utils import (
    filter_joint_state_to_selected_joints,
    joint_target_to_global_names,
    planning_group_id_from_selector,
)
from dimos.manipulation.planning.kinematics.config import (
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
)
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import (
    RobotModelConfig,
    TrajectoryParametrizationConfig,
)
from dimos.manipulation.planning.spec.enums import (
    IKStatus,
    ObstacleType,
    ParametrizationStatus,
    TrajectoryDispatchStatus,
)
from dimos.manipulation.planning.spec.models import (
    CartesianDelta,
    CartesianPathMode,
    CollisionCheckResult,
    ForwardKinematicsResult,
    GeneratedPlan,
    GeneratedTrajectory,
    IKResult,
    Obstacle,
    PlanningGroupID,
    PlanningResult,
    RobotName,
    TrajectoryDispatch,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import (
    KinematicsSpec,
    PlannerSpec,
    TrajectoryParametrizerSpec,
)
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.planning.trajectory_generator.parametrizers import (
    create_trajectory_parametrizer,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.factory import create_manipulation_visualization
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import RPCClient

logger = setup_logger()

# Composite type aliases for readability (using semantic IDs from planning.spec)
RobotEntry: TypeAlias = tuple[WorldRobotID, RobotModelConfig, JointTrajectoryGenerator]
"""(world_robot_id, config, trajectory_generator)"""

RobotRegistry: TypeAlias = dict[RobotName, RobotEntry]
"""Maps robot_name -> RobotEntry"""

RobotInfoValue: TypeAlias = (
    RobotName | WorldRobotID | list[str] | list[float] | float | None | list[PlanningGroup]
)
RobotInfoPayload: TypeAlias = dict[str, RobotInfoValue]
"""Legacy RPC payload derived from RobotModelConfig and planning-group registry."""


class ManipulationState(Enum):
    """State machine for manipulation module."""

    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    COMPLETED = 3
    FAULT = 4


class ManipulationModuleConfig(ModuleConfig):
    """Configuration for ManipulationModule."""

    robots: list[RobotModelConfig] = Field(default_factory=list)
    planning_timeout: float = 10.0
    world_backend: WorldBackend = "drake"
    visualization: ManipulationVisualizationConfig = Field(
        default_factory=NoManipulationVisualizationConfig
    )
    planner_name: PlannerName = "rrt_connect"
    kinematics: ManipulationKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    # Deprecated: use kinematics.backend instead.
    kinematics_name: KinematicsName | None = None
    # Floor plane Z height (meters). When set, a box obstacle is added at startup
    # to prevent the planner from routing trajectories below this height.
    # Set to None to disable.
    floor_z: float | None = None
    coordinator_rpc_timeout: float = 3.0
    trajectory_parametrization: TrajectoryParametrizationConfig = Field(
        default_factory=TrajectoryParametrizationConfig
    )


class ManipulationModule(Module):
    """Base motion planning module with ControlCoordinator execution.

    - @rpc: Low-level building blocks (plan, execute, gripper)
    - @skill (short-horizon): Single-step actions (move_to_pose, open_gripper, go_home)

    Subclass PickAndPlaceModule adds perception integration and long-horizon skills.
    """

    config: ManipulationModuleConfig

    # Input: Joint state from coordinator (for world sync)
    coordinator_joint_state: In[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # State machine
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""
        self._planning_epoch = 0

        # Planning components (initialized in start())
        self._world_monitor: WorldMonitor | None = None
        self._planner: PlannerSpec | None = None
        self._kinematics: KinematicsSpec | None = None
        self._trajectory_parametrizer: TrajectoryParametrizerSpec | None = None

        # Robot registry: maps robot_name -> (world_robot_id, config, trajectory_gen)
        self._robots: RobotRegistry = {}

        # Stored generated plan for preview/execute workflow.
        self._last_plan: GeneratedPlan | None = None
        self._last_trajectory: GeneratedTrajectory | None = None
        self._motion_speed_scale = 1.0

        # Coordinator integration (lazy initialized)
        self._coordinator_client: RPCClient | None = None

        # Init joints: captured from first joint state per robot, used by go_init
        self._init_joints: dict[RobotName, JointState] = {}

        # TF publishing thread
        self._tf_stop_event = threading.Event()
        self._tf_thread: threading.Thread | None = None

        logger.info("ManipulationModule initialized")

    @rpc
    def start(self) -> None:
        """Start the manipulation module."""
        super().start()

        # Initialize planning stack
        self._initialize_planning()

        # Subscribe to joint state via port
        if self.coordinator_joint_state is not None:
            self.coordinator_joint_state.subscribe(self._on_joint_state)
            logger.info("Subscribed to coordinator_joint_state port")

        logger.info("ManipulationModule started")

    def _initialize_planning(self) -> None:
        """Initialize world, planner, and trajectory generator."""
        if not self.config.robots:
            logger.warning("No robots configured, planning disabled")
            return

        world = create_world(
            backend=self.config.world_backend,
            visualization=self.config.visualization,
        )
        planning_specs = create_planning_specs(
            world=world,
            world_backend=self.config.world_backend,
            planner_name=self.config.planner_name,
            kinematics_name=self.config.kinematics_name,
            kinematics=self.config.kinematics,
        )
        self._world_monitor = planning_specs.world_monitor
        self._planner = planning_specs.planner
        self._kinematics = planning_specs.kinematics
        self._trajectory_parametrizer = None
        visualization = create_manipulation_visualization(
            self.config.visualization,
            world=world,
            world_monitor=self._world_monitor,
            manipulation_module=self,
        )

        for robot_config in self.config.robots:
            robot_id = self._world_monitor.add_robot(robot_config)
            traj_gen = JointTrajectoryGenerator(
                num_joints=len(robot_config.joint_names),
                max_velocity=robot_config.max_velocity,
                max_acceleration=robot_config.max_acceleration,
            )
            self._robots[robot_config.name] = (robot_id, robot_config, traj_gen)

        self._world_monitor.finalize()
        self._trajectory_parametrizer = self._trajectory_parametrizer_for_config(
            self.config.trajectory_parametrization
        )

        # Add floor obstacle to prevent trajectories below the table surface
        if self.config.floor_z is not None:
            fz = self.config.floor_z
            thickness = 0.2
            floor_pose = Pose(
                Vector3(0.7, 0.0, fz - thickness / 2),
                Quaternion(0.0, 0.0, 0.0, 1.0),
            )
            floor_obs = Obstacle(
                name="floor",
                pose=floor_pose,
                obstacle_type=ObstacleType.BOX,
                dimensions=(0.6, 1.2, thickness),
            )
            self._world_monitor.add_obstacle(floor_obs)
            logger.info(f"Floor obstacle added at z={fz:.3f}")

        for _, (robot_id, _, _) in self._robots.items():
            self._world_monitor.start_state_monitor(robot_id)

        self._world_monitor.set_visualization(visualization)
        self._world_monitor.sync_visualization_scene()

        if self._world_monitor.visualization is not None:
            self._world_monitor.start_visualization_thread(rate_hz=10.0)
            if url := self._world_monitor.get_visualization_url():
                logger.info(f"Visualization: {url}")

        # Start TF publishing thread if any robot has tf_extra_links
        if any(c.tf_extra_links for _, c, _ in self._robots.values()):
            _ = self.tf  # Eager init
            self._tf_stop_event.clear()
            self._tf_thread = threading.Thread(
                target=self._tf_publish_loop, name="ManipTFThread", daemon=True
            )
            self._tf_thread.start()
            logger.info("TF publishing thread started")

    def _get_default_robot_name(self) -> RobotName | None:
        """Get default robot name (first robot if only one, else None)."""
        if len(self._robots) == 1:
            return next(iter(self._robots.keys()))
        return None

    def _get_robot(
        self, robot_name: RobotName | None = None
    ) -> tuple[RobotName, WorldRobotID, RobotModelConfig, JointTrajectoryGenerator] | None:
        """Get robot by name or default.

        Args:
            robot_name: Robot name or None for default (if single robot)

        Returns:
            (robot_name, robot_id, config, traj_gen) or None if not found
        """
        if not robot_name:  # None or empty string (LLMs often pass "")
            robot_name = self._get_default_robot_name()
            if robot_name is None:
                logger.error("Multiple robots configured, must specify robot_name")
                return None

        if robot_name not in self._robots:
            logger.error(f"Unknown robot: {robot_name}")
            return None

        robot_id, config, traj_gen = self._robots[robot_name]
        return (robot_name, robot_id, config, traj_gen)

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback when joint state received from driver.

        Splits the aggregated global JointState by robot, then routes local
        robot-scoped states to the correct monitor.
        """
        try:
            if self._world_monitor is None:
                return

            assert_global_joint_names(msg.name)

            # Build name → index map once for the whole message
            name_to_idx = {name: i for i, name in enumerate(msg.name)}

            for robot_name, (robot_id, config, _) in self._robots.items():
                global_names = make_global_joint_names(robot_name, config.joint_names)
                indices = [name_to_idx.get(global_name) for global_name in global_names]
                if any(idx is None for idx in indices):
                    missing = [
                        name
                        for name, idx in zip(global_names, indices, strict=False)
                        if idx is None
                    ]
                    logger.warning(f"Skipping '{robot_name}': missing joints {missing}")
                    continue

                # Build per-robot sub-message (local model namespace)
                sub_positions = [msg.position[idx] for idx in indices]  # type: ignore[index]
                sub_velocities = (
                    [msg.velocity[idx] for idx in indices]  # type: ignore[index]
                    if msg.velocity and len(msg.velocity) == len(msg.name)
                    else []
                )
                sub_msg = JointState(
                    name=list(config.joint_names),
                    position=sub_positions,
                    velocity=sub_velocities,
                )

                # Route to specific monitor
                self._world_monitor.on_joint_state(sub_msg, robot_id=robot_id)

                # Capture per-robot init joints on first receipt
                if robot_name not in self._init_joints:
                    self._init_joints[robot_name] = sub_msg
                    logger.info(
                        f"Init joints captured for '{robot_name}': "
                        f"[{', '.join(f'{j:.3f}' for j in sub_positions)}]"
                    )

        except Exception as e:
            logger.error(f"Exception in _on_joint_state: {e}")
            logger.error(traceback.format_exc())

    def _tf_publish_loop(self) -> None:
        """Publish TF transforms at 10Hz for EE and extra links."""
        period = 0.1  # 10Hz
        while not self._tf_stop_event.is_set():
            try:
                if self._world_monitor is None:
                    break
                transforms: list[Transform] = []
                for robot_id, config, _ in self._robots.values():
                    # Publish world → primary planning-group target frame.
                    # Fall back to robot-scoped EE only for compatibility configs.
                    # TODO: Publish one TF per pose-targetable group, or expose the
                    # backend's full robot TF tree, once consumers stop assuming a
                    # single robot-scoped end-effector frame.
                    target_frame = config.end_effector_link
                    ee_pose: PoseStamped | None
                    pose_group_id = self._primary_pose_group_id_for_robot(config.name)
                    if pose_group_id is not None:
                        pose_group = self._world_monitor.planning_groups.get(pose_group_id)
                        target_frame = pose_group.tip_link
                        ee_pose = self._world_monitor.get_group_ee_pose(pose_group_id)
                    else:
                        ee_pose = (
                            self._world_monitor.get_link_pose(robot_id, target_frame)
                            if target_frame is not None
                            else None
                        )
                    if ee_pose is not None and target_frame is not None:
                        ee_tf = Transform.from_pose(target_frame, ee_pose)
                        ee_tf.frame_id = "world"
                        transforms.append(ee_tf)

                    # Publish world → each extra link
                    for link_name in config.tf_extra_links:
                        link_pose = self._world_monitor.get_link_pose(robot_id, link_name)
                        if link_pose is not None:
                            link_tf = Transform.from_pose(link_name, link_pose)
                            link_tf.frame_id = "world"
                            transforms.append(link_tf)

                if transforms:
                    self.tf.publish(*transforms)
            except Exception as e:
                logger.debug(f"TF publish error: {e}")

            self._tf_stop_event.wait(period)

    @rpc
    def get_state(self) -> str:
        """Get current manipulation state name."""
        return self._state.name

    @rpc
    def get_error(self) -> str:
        """Get last error message.

        Returns:
            Error message or empty string
        """
        return self._error_message

    @rpc
    def set_motion_speed(self, speed_scale: float) -> bool:
        """Set runtime manipulation motion speed for future generated plans.

        This scale multiplies the configured trajectory parametrization velocity and
        acceleration scales. It is captured by the next generated trajectory and does
        not mutate an existing plan, an existing generated trajectory, or already-
        dispatched controller tasks. Re-plan to apply a changed speed setting.

        Args:
            speed_scale: Runtime speed multiplier in the range `(0, 1]`.
                Use values below 1.0 for slower, gentler motion.
        """
        if speed_scale <= 0.0 or speed_scale > 1.0:
            self._error_message = "motion speed scale must be > 0 and <= 1"
            return False
        self._motion_speed_scale = float(speed_scale)
        return True

    @rpc
    def get_motion_speed(self) -> float:
        """Get the runtime manipulation motion speed scale."""
        return self._motion_speed_scale

    @rpc
    def cancel(self) -> bool:
        """Cancel current motion or invalidate an in-progress plan."""
        if self._state == ManipulationState.PLANNING:
            self._planning_epoch += 1
            self._state = ManipulationState.IDLE
            logger.info("Planning cancelled")
            return True
        if self._state != ManipulationState.EXECUTING:
            return False
        self._state = ManipulationState.IDLE
        logger.info("Motion cancelled")
        return True

    @rpc
    @skill
    def reset(self) -> SkillResult[ManipulationSkillError]:
        """Reset the robot module to IDLE state, clearing any fault.

        Use this after an error or fault to allow new commands.
        Cannot reset while a motion is executing — cancel first.
        """
        if self._state == ManipulationState.EXECUTING:
            return SkillResult.fail(
                "INVALID_STATE",
                "Cannot reset while executing — cancel the motion first",
            )
        if self._state == ManipulationState.PLANNING:
            self._planning_epoch += 1
        self._state = ManipulationState.IDLE
        self._error_message = ""
        return SkillResult.ok("Reset to IDLE — ready for new commands")

    @rpc
    def get_current_joints(self, robot_name: RobotName | None = None) -> list[float] | None:
        """Get current joint positions.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            state = self._world_monitor.get_current_joint_state(robot[1])
            if state is not None:
                return list(state.position)
        return None

    @rpc
    def get_ee_pose(self, robot_name: RobotName | None = None) -> Pose | None:
        """Get current end-effector pose.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            _, robot_id, config, _ = robot
            pose_group_id = self._primary_pose_group_id_for_robot(config.name)
            if pose_group_id is not None:
                return self._world_monitor.get_group_ee_pose(pose_group_id, joint_state=None)
            if config.end_effector_link is None:
                return None
            return self._world_monitor.get_link_pose(robot_id, config.end_effector_link)
        return None

    @rpc
    def is_collision_free(self, joints: list[float], robot_name: RobotName | None = None) -> bool:
        """Check if joint configuration is collision-free.

        Args:
            joints: Joint configuration to check
            robot_name: Robot to check (required if multiple robots configured)
        """
        if (robot := self._get_robot(robot_name)) and self._world_monitor:
            _, robot_id, config, _ = robot
            joint_state = JointState(name=config.joint_names, position=joints)
            return self._world_monitor.is_state_valid(robot_id, joint_state)
        return False

    def _begin_planning(self) -> bool:
        """Check state and begin planning for the selected planning groups."""
        if self._world_monitor is None:
            logger.error("Planning not initialized")
            return False
        with self._lock:
            if self._state not in (ManipulationState.IDLE, ManipulationState.COMPLETED):
                logger.warning(f"Cannot plan: state is {self._state.name}")
                return False
            self._planning_epoch += 1
            self._state = ManipulationState.PLANNING
        return True

    def _fail(self, msg: str) -> bool:
        """Set FAULT state with error message."""
        logger.warning(msg)
        self._state = ManipulationState.FAULT
        self._error_message = msg
        return False

    def _default_group_id_for_robot(self, robot_name: RobotName) -> PlanningGroupID | None:
        """Return the generated fallback group used by robot-scoped wrappers."""
        assert self._world_monitor is not None
        group_id = self._world_monitor.planning_groups.default_group_id_for_robot(robot_name)
        if group_id is not None:
            return group_id
        logger.error(
            "Robot '%s' has no generated default planning group; use explicit group APIs",
            robot_name,
        )
        return None

    def _primary_pose_group_id_for_robot(self, robot_name: RobotName) -> PlanningGroupID | None:
        """Return the first pose-targetable group for robot-scoped compatibility paths."""
        assert self._world_monitor is not None
        return self._world_monitor.planning_groups.primary_pose_group_id_for_robot(robot_name)

    def _selected_joint_state(self, group_ids: tuple[PlanningGroupID, ...]) -> JointState | None:
        """Collect current state for exactly the selected global joints."""
        assert self._world_monitor is not None
        selection = self._world_monitor.planning_groups.select(group_ids)
        current = self._world_monitor.current_global_joint_state()
        if isinstance(current, JointState):
            try:
                return filter_joint_state_to_selected_joints(current, selection.joint_names)
            except ValueError as exc:
                logger.error("Current state missing selected joints: %s", exc)
                return None
        if current is None:
            logger.error("No fresh planning-world joint state")
            return None
        logger.error("Invalid planning-world joint state")
        return None

    def _joint_target_to_global_names(
        self, group_id: PlanningGroupID, target: JointState
    ) -> JointState | None:
        """Convert a group joint target to global joint names in group order."""
        assert self._world_monitor is not None
        group = self._world_monitor.planning_groups.get(group_id)
        try:
            return joint_target_to_global_names(group, target)
        except ValueError as exc:
            logger.error(str(exc))
            return None

    def _affected_robot_names(self, plan: GeneratedPlan) -> list[RobotName]:
        """Get stable robot names affected by a generated plan."""
        return self._affected_robot_names_for_group_ids(plan.group_ids)

    def _affected_robot_names_for_group_ids(
        self, group_ids: Sequence[PlanningGroupID]
    ) -> list[RobotName]:
        """Get stable robot names affected by planning group IDs."""
        assert self._world_monitor is not None
        return list(self._world_monitor.planning_groups.select(tuple(group_ids)).robot_names)

    def _store_generated_plan(
        self, group_ids: tuple[PlanningGroupID, ...], result: PlanningResult
    ) -> None:
        """Store the canonical generated plan."""
        self._last_plan = GeneratedPlan(
            group_ids=group_ids,
            path=result.path,
            status=result.status,
            planning_time=result.planning_time,
            path_length=result.path_length,
            iterations=result.iterations,
            message=result.message,
            path_constraints=result.path_constraints,
        )
        self._last_trajectory = None

    def _trajectory_config_for_plan(self, plan: GeneratedPlan) -> TrajectoryParametrizationConfig:
        """Resolve parametrization config and robot defaults for the selected plan joints."""
        values = self.config.trajectory_parametrization.model_dump()
        if plan.path and values.get("velocity_limits") is None:
            values["velocity_limits"] = self._velocity_limits_for_plan(plan)
        if plan.path and values.get("acceleration_limits") is None:
            values["acceleration_limits"] = self._acceleration_limits_for_plan(plan)
        return TrajectoryParametrizationConfig(**values)

    def _trajectory_parametrizer_for_config(
        self, config: TrajectoryParametrizationConfig
    ) -> TrajectoryParametrizerSpec:
        """Resolve the configured trajectory parametrizer backend."""
        if config.backend != "roboplan":
            return create_trajectory_parametrizer(config)
        if self._world_monitor is None:
            raise ValueError(
                'trajectory_parametrization.backend="roboplan" requires an initialized world'
            )
        world = self._world_monitor.world
        if not isinstance(world, TrajectoryParametrizerSpec):
            raise ValueError(
                'trajectory_parametrization.backend="roboplan" requires world_backend="roboplan"'
            )
        configure = getattr(world, "configure_trajectory_parametrization", None)
        if configure is not None:
            typed_configure = cast("Callable[[TrajectoryParametrizationConfig], None]", configure)
            typed_configure(config)
        return world

    def _velocity_limits_for_plan(self, plan: GeneratedPlan) -> list[float]:
        """Return per-selected-joint velocity limits in generated-plan joint order."""
        return [self._robot_limit_for_global_joint(name, "velocity") for name in plan.path[0].name]

    def _acceleration_limits_for_plan(self, plan: GeneratedPlan) -> list[float]:
        """Return per-selected-joint acceleration limits in generated-plan joint order."""
        return [
            self._robot_limit_for_global_joint(name, "acceleration") for name in plan.path[0].name
        ]

    def _robot_limit_for_global_joint(self, global_joint_name: str, limit_kind: str) -> float:
        """Resolve a robot model limit for a selected global joint."""
        assert_global_joint_names([global_joint_name])
        robot_name = global_joint_name.split("/", maxsplit=1)[0]
        robot = self._robots.get(robot_name)
        if robot is None:
            raise ValueError(f"No robot registered for joint '{global_joint_name}'")
        _, config, _ = robot
        local_name = local_joint_name_from_global(robot_name, global_joint_name)
        try:
            joint_index = config.joint_names.index(local_name)
        except ValueError as exc:
            raise ValueError(f"Joint '{global_joint_name}' is not in robot config") from exc
        if limit_kind == "velocity" and config.velocity_limits is not None:
            return float(config.velocity_limits[joint_index])
        if limit_kind == "velocity":
            return float(config.max_velocity)
        return float(config.max_acceleration)

    def _parametrize_plan(self, plan: GeneratedPlan) -> GeneratedTrajectory:
        """Parametrize and store the canonical global generated trajectory."""
        try:
            parametrizer = self._trajectory_parametrizer_for_config(
                self._trajectory_config_for_plan(plan)
            )
            trajectory = parametrizer.parametrize(plan, speed_scale=self._motion_speed_scale)
        except ValueError as exc:
            trajectory = GeneratedTrajectory(
                status=ParametrizationStatus.INVALID_PLAN,
                message=str(exc),
                speed_scale=self._motion_speed_scale,
                source_group_ids=plan.group_ids,
                source_plan_status=plan.status,
                source_plan_message=plan.message,
            )
        if plan is self._last_plan:
            self._last_trajectory = trajectory
        return trajectory

    def _trajectory_for_plan(self, plan: GeneratedPlan) -> GeneratedTrajectory:
        """Return the stored trajectory for `_last_plan` or parametrize an explicit plan."""
        if plan is self._last_plan and self._last_trajectory is not None:
            return self._last_trajectory
        return self._parametrize_plan(plan)

    def _prepare_trajectory_dispatch(self, trajectory: GeneratedTrajectory) -> TrajectoryDispatch:
        """Project a global generated trajectory into per-task dispatch messages."""
        if not trajectory.is_success():
            return TrajectoryDispatch(
                status=TrajectoryDispatchStatus.INVALID_TRAJECTORY,
                message=f"GeneratedTrajectory is not successful: {trajectory.message}",
            )
        if not trajectory.points or not trajectory.joint_names:
            return TrajectoryDispatch(
                status=TrajectoryDispatchStatus.INVALID_TRAJECTORY,
                message="GeneratedTrajectory has no timed points",
            )
        try:
            assert_global_joint_names(trajectory.joint_names)
            affected = self._affected_robot_names_for_group_ids(trajectory.source_group_ids)
        except Exception as exc:
            return TrajectoryDispatch(
                status=TrajectoryDispatchStatus.INVALID_TRAJECTORY,
                message=f"Failed to resolve generated trajectory: {exc}",
            )
        assert self._world_monitor is not None

        trajectories_by_task: dict[str, JointTrajectory] = {}
        robot_names_by_task: dict[str, RobotName] = {}
        for name in affected:
            robot = self._get_robot(name)
            if robot is None:
                return TrajectoryDispatch(
                    status=TrajectoryDispatchStatus.FAILED,
                    message=f"Robot '{name}' is not registered",
                )
            resolved_name, robot_id, config, _ = robot
            task_name = config.coordinator_task_name
            if not task_name:
                return TrajectoryDispatch(
                    status=TrajectoryDispatchStatus.MISSING_TASK,
                    message=f"No coordinator_task_name for '{resolved_name}'",
                )

            current = self._world_monitor.get_current_joint_state(robot_id)
            current_by_name = (
                dict(zip(current.name, current.position, strict=False))
                if current is not None
                else {}
            )
            global_joint_names = make_global_joint_names(resolved_name, config.joint_names)
            selected_indices = {
                joint_name: index for index, joint_name in enumerate(trajectory.joint_names)
            }
            points = []
            for point in trajectory.points:
                if len(point.positions) != len(trajectory.joint_names):
                    return TrajectoryDispatch(
                        status=TrajectoryDispatchStatus.INVALID_TRAJECTORY,
                        message="GeneratedTrajectory point positions do not match joint_names",
                    )
                positions: list[float] = []
                velocities: list[float] = []
                for local_name, global_joint_name in zip(
                    config.joint_names, global_joint_names, strict=True
                ):
                    selected_index = selected_indices.get(global_joint_name)
                    if selected_index is not None:
                        positions.append(float(point.positions[selected_index]))
                        velocity = (
                            float(point.velocities[selected_index])
                            if len(point.velocities) == len(trajectory.joint_names)
                            else 0.0
                        )
                        velocities.append(velocity)
                    elif local_name in current_by_name:
                        positions.append(float(current_by_name[local_name]))
                        velocities.append(0.0)
                    else:
                        return TrajectoryDispatch(
                            status=TrajectoryDispatchStatus.MISSING_JOINT,
                            message=(
                                f"Cannot dispatch trajectory for '{resolved_name}': "
                                f"missing joint '{global_joint_name}'"
                            ),
                        )
                points.append(
                    TrajectoryPoint(
                        time_from_start=point.time_from_start,
                        positions=positions,
                        velocities=velocities,
                    )
                )
            trajectories_by_task[task_name] = JointTrajectory(
                joint_names=list(global_joint_names),
                points=points,
            )
            robot_names_by_task[task_name] = resolved_name

        return TrajectoryDispatch(
            trajectories_by_task=trajectories_by_task,
            robot_names_by_task=robot_names_by_task,
            status=TrajectoryDispatchStatus.SUCCESS,
            message="Trajectory dispatch prepared",
        )

    def _plan_selected_path(
        self, group_ids: tuple[PlanningGroupID, ...], start: JointState, goal: JointState
    ) -> bool:
        """Plan over an explicit planning group selection and store the result."""
        assert self._world_monitor and self._planner
        result = self._planner.plan_selected_joint_path(
            world=self._world_monitor.world,
            selection=self._world_monitor.planning_groups.select(group_ids),
            start=start,
            goal=goal,
            timeout=self.config.planning_timeout,
        )
        if not result.is_success():
            return self._fail(f"Planning failed: {result.status.name}")

        path_joints = list(result.path[-1].name) if result.path else []
        logger.info(
            "Path: %d waypoints, groups=%s, joints=%s",
            len(result.path),
            group_ids,
            path_joints,
        )
        self._store_generated_plan(group_ids, result)
        assert self._last_plan is not None
        trajectory = self._parametrize_plan(self._last_plan)
        if not trajectory.is_success():
            return self._fail(f"Trajectory parametrization failed: {trajectory.message}")
        self._state = ManipulationState.COMPLETED
        return True

    def _finish_cartesian_plan(
        self,
        group_ids: tuple[PlanningGroupID, ...],
        result: PlanningResult,
    ) -> bool:
        """Store a successful Cartesian planner result or report its failure."""
        if not result.is_success():
            detail = f": {result.message}" if result.message else ""
            return self._fail(f"Planning failed: {result.status.name}{detail}")
        self._store_generated_plan(group_ids, result)
        self._state = ManipulationState.COMPLETED
        return True

    def _resolve_group_ids_for_pose_targets(
        self,
        target_groups: Sequence[PlanningGroupID | PlanningGroup],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup],
    ) -> tuple[PlanningGroupID, ...]:
        """Return selected planning group IDs with target groups before auxiliaries."""
        target_ids = tuple(planning_group_id_from_selector(group) for group in target_groups)
        auxiliary_ids = tuple(planning_group_id_from_selector(group) for group in auxiliary_groups)
        return tuple(dict.fromkeys((*target_ids, *auxiliary_ids)))

    def _selected_start_for_groups(
        self, group_ids: tuple[PlanningGroupID, ...]
    ) -> JointState | None:
        """Resolve the selected global-joint start state for a group selection."""
        try:
            start = self._selected_joint_state(group_ids)
        except Exception as exc:
            self._fail(f"Failed to resolve planning groups: {exc}")
            return None
        if start is None:
            self._fail("No joint state")
            return None
        return start

    def _dismiss_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        """Hide the preview ghost if the world supports it."""
        if self._world_monitor is None:
            return
        self._world_monitor.hide_preview(group_ids)
        self._world_monitor.publish_visualization()

    @rpc
    def check_collision(
        self,
        target_joints: JointState,
        max_age: float = 1.0,
    ) -> CollisionCheckResult:
        """Check a partial global joint target against the planning world."""
        if self._world_monitor is None:
            return CollisionCheckResult(
                status="UNAVAILABLE",
                collision_free=None,
                message="Planning is not initialized",
            )
        return self._world_monitor.check_collision(target_joints, max_age=max_age)

    def _planning_group_models(self) -> list[PlanningGroup]:
        """Return all planning group models in stable registry order."""
        if self._world_monitor is None:
            return []
        return list(self._world_monitor.planning_groups.list())

    @rpc
    def list_planning_groups(self) -> list[PlanningGroup]:
        """Return all planning groups."""
        return self._planning_group_models()

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        """Return the named robot's current local joint state with names."""
        if self._world_monitor is None:
            return None
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return self._world_monitor.get_current_joint_state(robot_id)

    @rpc
    def forward_kinematics(
        self,
        group_id: PlanningGroupID,
        target_joints: JointState | None = None,
        max_age: float = 1.0,
    ) -> ForwardKinematicsResult:
        """Compute the selected planning group's end-effector pose."""
        if self._world_monitor is None:
            return ForwardKinematicsResult(
                status="UNAVAILABLE",
                pose=None,
                message="Planning is not initialized",
            )
        try:
            group = self._world_monitor.planning_groups.get(group_id)
        except KeyError as exc:
            return ForwardKinematicsResult(status="INVALID", pose=None, message=str(exc))
        if not group.has_pose_target:
            return ForwardKinematicsResult(
                status="INVALID",
                pose=None,
                message=f"Planning group '{group_id}' has no pose target frame",
            )
        robot = self._robots.get(group.robot_name)
        if robot is None:
            return ForwardKinematicsResult(
                status="INVALID",
                pose=None,
                message=f"Robot '{group.robot_name}' is not registered",
            )
        robot_id, config, _ = robot

        if target_joints is None:
            monitor = self._world_monitor.get_state_monitor(robot_id)
            if monitor is None or monitor.is_state_stale(max_age):
                return ForwardKinematicsResult(
                    status="STALE_STATE",
                    pose=None,
                    message="Fresh monitored robot joint state is unavailable",
                )
            joint_state = self._world_monitor.get_current_joint_state(robot_id)
            if joint_state is None:
                return ForwardKinematicsResult(
                    status="STALE_STATE",
                    pose=None,
                    message="Fresh monitored robot joint state is unavailable",
                )
        else:
            if len(target_joints.name) != len(target_joints.position):
                return ForwardKinematicsResult(
                    status="INVALID",
                    pose=None,
                    message="FK target name and position lengths must match",
                )
            if len(set(target_joints.name)) != len(target_joints.name):
                return ForwardKinematicsResult(
                    status="INVALID",
                    pose=None,
                    message="FK target contains duplicate joint names",
                )
            try:
                assert_global_joint_names(target_joints.name)
            except ValueError as exc:
                return ForwardKinematicsResult(status="INVALID", pose=None, message=str(exc))
            positions_by_global_name = dict(
                zip(target_joints.name, target_joints.position, strict=True)
            )
            missing = [name for name in group.joint_names if name not in positions_by_global_name]
            if missing:
                return ForwardKinematicsResult(
                    status="INVALID",
                    pose=None,
                    message=f"FK target missing group joints: {missing}",
                )
            current = self._world_monitor.get_current_joint_state(robot_id)
            current_by_local_name = (
                dict(zip(current.name, current.position, strict=False))
                if current is not None
                else {}
            )
            positions: list[float] = []
            for local_name, global_name in zip(
                config.joint_names,
                make_global_joint_names(group.robot_name, config.joint_names),
                strict=True,
            ):
                if global_name in positions_by_global_name:
                    positions.append(float(positions_by_global_name[global_name]))
                else:
                    positions.append(float((current_by_local_name or {}).get(local_name, 0.0)))
            joint_state = JointState(name=list(config.joint_names), position=positions)

        try:
            pose = self._world_monitor.get_group_ee_pose(group_id, joint_state)
        except Exception as exc:
            return ForwardKinematicsResult(
                status="UNAVAILABLE",
                pose=None,
                message=f"Forward kinematics failed: {exc}",
            )
        return ForwardKinematicsResult(
            status="VALID", pose=pose, message="Forward kinematics solved"
        )

    @rpc
    def inverse_kinematics(
        self,
        pose_targets: Mapping[PlanningGroupID, PoseStamped],
        auxiliary_group_ids: Sequence[PlanningGroupID] = (),
        seed: JointState | None = None,
    ) -> IKResult:
        """Solve planning-group pose targets without collision filtering."""
        if self._kinematics is None or self._world_monitor is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
        if not pose_targets:
            return IKResult(
                status=IKStatus.NO_SOLUTION, message="At least one pose target is required"
            )
        group_ids = tuple(dict.fromkeys((*pose_targets.keys(), *auxiliary_group_ids)))
        try:
            target_groups = {
                self._world_monitor.planning_groups.get(group_id): pose
                for group_id, pose in pose_targets.items()
            }
            auxiliary_groups = tuple(
                self._world_monitor.planning_groups.get(group_id)
                for group_id in auxiliary_group_ids
            )
            seed_state = seed or self._selected_joint_state(group_ids)
        except (KeyError, ValueError) as exc:
            return IKResult(status=IKStatus.NO_SOLUTION, message=str(exc))
        if seed_state is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="No joint state")
        return self._kinematics.solve_pose_targets(
            world=self._world_monitor.world,
            pose_targets=target_groups,
            auxiliary_groups=auxiliary_groups,
            seed=seed_state,
        )

    @rpc
    def inverse_kinematics_single(
        self,
        pose: Pose,
        robot_name: RobotName | None = None,
        seed: JointState | None = None,
    ) -> IKResult:
        """Solve IK for one robot's primary pose-targetable planning group.

        Args:
            pose: Target end-effector pose
            robot_name: Robot to solve for (required if multiple robots configured).
            seed: Optional joint state to initialize local IK. Uses current state when omitted.
        """
        if self._world_monitor is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Planning not initialized")
        robot = self._get_robot(robot_name)
        if robot is None:
            return IKResult(status=IKStatus.NO_SOLUTION, message="Robot not found")
        selected_robot_name, _, _, _ = robot
        group_id = self._primary_pose_group_id_for_robot(selected_robot_name)
        if group_id is None:
            return IKResult(
                status=IKStatus.NO_SOLUTION, message="No pose-targetable planning group"
            )
        target_pose = PoseStamped(
            frame_id="world",
            position=pose.position,
            orientation=pose.orientation,
        )
        return self.inverse_kinematics({group_id: target_pose}, seed=seed)

    @rpc
    def solve_ik(
        self,
        pose: Pose,
        robot_name: RobotName | None = None,
        seed: JointState | None = None,
    ) -> IKResult:
        """Compatibility wrapper for inverse_kinematics_single()."""
        return self.inverse_kinematics_single(pose, robot_name=robot_name, seed=seed)

    @rpc
    def plan_to_pose(self, pose: Pose, robot_name: RobotName | None = None) -> bool:
        """Plan motion to pose. Use preview_plan() then execute().

        Args:
            pose: Target end-effector pose
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        if self._kinematics is None or self._world_monitor is None:
            return False
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        selected_robot_name, _, _, _ = robot
        group_id = self._default_group_id_for_robot(selected_robot_name)
        if group_id is None:
            return False
        return self.plan_to_pose_targets({group_id: pose})

    @rpc
    def plan_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
    ) -> bool:
        """Plan to one or more group pose targets with optional auxiliary groups."""
        if self._world_monitor is None or self._kinematics is None:
            return False
        if not pose_targets:
            return self._fail("At least one pose target is required")

        stamped_targets = {
            planning_group_id_from_selector(group): PoseStamped(
                frame_id="world",
                position=pose.position,
                orientation=pose.orientation,
            )
            for group, pose in pose_targets.items()
        }
        auxiliary_ids = tuple(planning_group_id_from_selector(group) for group in auxiliary_groups)
        group_ids = tuple(dict.fromkeys((*stamped_targets.keys(), *auxiliary_ids)))
        if not self._begin_planning():
            return False

        try:
            start = self._selected_joint_state(group_ids)
        except Exception as exc:
            return self._fail(f"Failed to resolve planning groups: {exc}")
        if start is None:
            return self._fail("No joint state")

        ik = self.inverse_kinematics(
            pose_targets=stamped_targets,
            auxiliary_group_ids=auxiliary_ids,
            seed=start,
        )
        if not ik.is_success() or ik.joint_state is None:
            return self._fail(f"IK failed: {ik.status.name}")
        return self._plan_selected_path(group_ids, start, ik.joint_state)

    @rpc
    def plan_linear_to_pose_targets(
        self,
        pose_targets: Mapping[PlanningGroupID | PlanningGroup, Pose],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
        timeout: float = 10.0,
    ) -> bool:
        """Plan a linear TCP path to one or more absolute group pose targets."""
        if self._world_monitor is None or self._planner is None:
            return False
        if not pose_targets:
            return self._fail("At least one pose target is required")

        stamped_targets = {
            planning_group_id_from_selector(group): PoseStamped(
                frame_id="world",
                position=pose.position,
                orientation=pose.orientation,
            )
            for group, pose in pose_targets.items()
        }
        auxiliary_ids = tuple(planning_group_id_from_selector(group) for group in auxiliary_groups)
        group_ids = self._resolve_group_ids_for_pose_targets(
            tuple(pose_targets.keys()), auxiliary_groups
        )
        if not self._begin_planning():
            return False
        start = self._selected_start_for_groups(group_ids)
        if start is None:
            return False

        result = self._planner.plan_cartesian_path(
            world=self._world_monitor.world,
            selection=self._world_monitor.planning_groups.select(group_ids),
            start=start,
            pose_targets=stamped_targets,
            auxiliary_groups=auxiliary_ids,
            path_mode="linear",
            timeout=timeout,
        )
        return self._finish_cartesian_plan(group_ids, result)

    @rpc
    def plan_relative_to_pose_targets(
        self,
        delta_targets: Mapping[PlanningGroupID | PlanningGroup, CartesianDelta],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
        timeout: float = 10.0,
    ) -> bool:
        """Plan freely to one or more relative group pose deltas."""
        return self._plan_relative_pose_targets(
            delta_targets=delta_targets,
            auxiliary_groups=auxiliary_groups,
            path_mode="free",
            timeout=timeout,
        )

    @rpc
    def plan_linear_relative_to_pose_targets(
        self,
        delta_targets: Mapping[PlanningGroupID | PlanningGroup, CartesianDelta],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup] = (),
        timeout: float = 10.0,
    ) -> bool:
        """Plan a linear TCP path to one or more relative group pose deltas."""
        return self._plan_relative_pose_targets(
            delta_targets=delta_targets,
            auxiliary_groups=auxiliary_groups,
            path_mode="linear",
            timeout=timeout,
        )

    def _plan_relative_pose_targets(
        self,
        delta_targets: Mapping[PlanningGroupID | PlanningGroup, CartesianDelta],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroup],
        path_mode: CartesianPathMode,
        timeout: float,
    ) -> bool:
        """Shared relative pose-target planner entrypoint."""
        if self._world_monitor is None or self._planner is None:
            return False
        if not delta_targets:
            return self._fail("At least one relative pose target is required")

        target_ids = tuple(planning_group_id_from_selector(group) for group in delta_targets)
        resolved_targets = {
            planning_group_id_from_selector(group): delta for group, delta in delta_targets.items()
        }
        auxiliary_ids = tuple(planning_group_id_from_selector(group) for group in auxiliary_groups)
        group_ids = tuple(dict.fromkeys((*target_ids, *auxiliary_ids)))
        if not self._begin_planning():
            return False
        start = self._selected_start_for_groups(group_ids)
        if start is None:
            return False

        result = self._planner.plan_relative_cartesian_path(
            world=self._world_monitor.world,
            selection=self._world_monitor.planning_groups.select(group_ids),
            start=start,
            delta_targets=resolved_targets,
            auxiliary_groups=auxiliary_ids,
            path_mode=path_mode,
            timeout=timeout,
        )
        return self._finish_cartesian_plan(group_ids, result)

    @rpc
    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        """Plan motion to joint config. Use preview_plan() then execute().

        Args:
            joints: Target joint state (names + positions)
            robot_name: Robot to plan for (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        robot_name, _, _, _ = robot
        logger.info(f"Planning to joints for {robot_name}: {[f'{j:.3f}' for j in joints.position]}")
        group_id = self._default_group_id_for_robot(robot_name)
        if group_id is None:
            return False
        return self.plan_to_joint_targets({group_id: joints})

    @rpc
    def plan_to_joint_targets(
        self, joint_targets: Mapping[PlanningGroupID | PlanningGroup, JointState]
    ) -> bool:
        """Plan to joint targets keyed by planning group."""
        if self._world_monitor is None or self._planner is None:
            return False
        if not joint_targets:
            return self._fail("At least one joint target is required")

        group_ids = tuple(
            dict.fromkeys(planning_group_id_from_selector(group) for group in joint_targets)
        )
        if not self._begin_planning():
            return False
        try:
            start = self._selected_joint_state(group_ids)
        except Exception as exc:
            return self._fail(f"Failed to resolve planning groups: {exc}")
        if start is None:
            return self._fail("No joint state")

        goal_names: list[str] = []
        goal_positions: list[float] = []
        for group, target in joint_targets.items():
            group_id = planning_group_id_from_selector(group)
            target_global = self._joint_target_to_global_names(group_id, target)
            if target_global is None:
                return self._fail(f"Invalid joint target for '{group_id}'")
            goal_names.extend(target_global.name)
            goal_positions.extend(target_global.position)

        goal = JointState(name=goal_names, position=goal_positions)
        return self._plan_selected_path(group_ids, start, goal)

    @rpc
    def preview_plan(
        self,
        plan: GeneratedPlan | None = None,
        duration: float | None = None,
        robot_name: RobotName | None = None,
    ) -> bool:
        """Preview a generated plan, defaulting to `_last_plan` when omitted."""
        if self._world_monitor is None:
            return False
        plan = plan or self._last_plan
        if plan is None or not plan.path:
            logger.warning("No generated plan to preview")
            return False
        if robot_name is not None and robot_name not in self._affected_robot_names(plan):
            logger.error("Generated plan does not affect robot '%s'", robot_name)
            return False
        trajectory = self._trajectory_for_plan(plan)
        if not trajectory.is_success():
            logger.error(
                "Cannot preview plan: trajectory parametrization failed: %s", trajectory.message
            )
            return False
        animation_duration = duration if duration is not None else trajectory.duration
        preview_plan = GeneratedPlan(
            group_ids=trajectory.source_group_ids or plan.group_ids,
            path=[
                JointState(name=list(trajectory.joint_names), position=list(point.positions))
                for point in trajectory.points
            ],
            status=trajectory.source_plan_status,
            planning_time=plan.planning_time,
            path_length=plan.path_length,
            iterations=plan.iterations,
            message=trajectory.source_plan_message or plan.message,
        )
        self._world_monitor.animate_plan(preview_plan, animation_duration)
        return True

    @rpc
    def has_planned_path(self) -> bool:
        """Check if there's a planned path ready.

        Returns:
            True if a path is planned and ready
        """
        return self._last_plan is not None and bool(self._last_plan.path)

    @rpc
    def get_visualization_url(self) -> str | None:
        """Get the visualization URL.

        Returns:
            URL string or None if visualization not enabled
        """
        if self._world_monitor is None:
            return None
        return self._world_monitor.get_visualization_url()

    @rpc
    def clear_planned_path(self) -> bool:
        """Clear the stored planned path.

        Returns:
            True if cleared
        """
        self._last_plan = None
        self._last_trajectory = None
        return True

    @rpc
    def list_robots(self) -> list[str]:
        """List all configured robot names.

        Returns:
            List of robot names
        """
        return list(self._robots.keys())

    @rpc
    def get_robot_info(self, robot_name: RobotName | None = None) -> RobotInfoPayload | None:
        """Get information about a robot.

        Args:
            robot_name: Robot name (uses default if None)

        Returns:
            Dict with robot info or None if not found
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return None

        robot_name, robot_id, config, _ = robot
        planning_groups = (
            [group for group in self._world_monitor.planning_groups.groups_for_robot(robot_name)]
            if self._world_monitor is not None
            else []
        )

        info: RobotInfoPayload = {
            "name": config.name,
            "world_robot_id": robot_id,
            "joint_names": config.joint_names,
            "planning_groups": planning_groups,
            "end_effector_link": config.end_effector_link,
            "base_link": config.base_link,
            "max_velocity": config.max_velocity,
            "max_acceleration": config.max_acceleration,
            "coordinator_task_name": config.coordinator_task_name,
            "home_joints": config.home_joints,
            "pre_grasp_offset": config.pre_grasp_offset,
            "init_joints": list(init.position)
            if (init := self._init_joints.get(robot_name))
            else None,
        }
        return info

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        """Return configured robots for in-process visualization adapters."""
        return [(name, robot_id, config) for name, (robot_id, config, _) in self._robots.items()]

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        """Return the planning-world robot id for a configured robot name."""
        entry = self._robots.get(robot_name)
        return entry[0] if entry is not None else None

    def robot_name_for_id(self, robot_id: WorldRobotID) -> RobotName | None:
        """Return the configured robot name for a planning-world robot id."""
        for robot_name, (candidate_id, _, _) in self._robots.items():
            if candidate_id == robot_id:
                return robot_name
        return None

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        """Return the robot model config for an in-process visualization adapter."""
        entry = self._robots.get(robot_name)
        return entry[1] if entry is not None else None

    @rpc
    def get_init_joints(self, robot_name: RobotName | None = None) -> JointState | None:
        """Get the init joint state (captured at startup or set manually).

        Args:
            robot_name: Robot name (uses default if None and only one robot)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return None
        return self._init_joints.get(robot[0])

    @rpc
    def set_init_joints(self, joint_state: JointState, robot_name: RobotName | None = None) -> bool:
        """Set the init joint state.

        Args:
            joint_state: New init joint state (names + positions)
            robot_name: Robot name (uses default if None and only one robot)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        robot_name_resolved, _, config, _ = robot
        try:
            normalized = self._local_robot_joint_state(config, joint_state)
        except ValueError as exc:
            logger.error(str(exc))
            return False
        self._init_joints[robot_name_resolved] = normalized
        logger.info(
            f"Init joints set for '{robot_name_resolved}': "
            f"[{', '.join(f'{j:.3f}' for j in normalized.position)}]"
        )
        return True

    def _local_robot_joint_state(
        self, config: RobotModelConfig, joint_state: JointState
    ) -> JointState:
        """Normalize a robot-scoped joint state to local model joint order."""
        if not joint_state.name:
            if len(joint_state.position) != len(config.joint_names):
                raise ValueError(
                    f"JointState has {len(joint_state.position)} positions, "
                    f"expected {len(config.joint_names)} for robot '{config.name}'"
                )
            return JointState(name=list(config.joint_names), position=list(joint_state.position))

        assert_local_joint_names(joint_state.name)
        positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=False))
        missing = [name for name in config.joint_names if name not in positions_by_name]
        if missing:
            raise ValueError(f"JointState for robot '{config.name}' is missing joints: {missing}")
        extra = set(joint_state.name) - set(config.joint_names)
        if extra:
            raise ValueError(
                f"JointState for robot '{config.name}' has extra joints: {sorted(extra)}"
            )
        return JointState(
            name=list(config.joint_names),
            position=[positions_by_name[name] for name in config.joint_names],
        )

    @rpc
    def set_init_joints_to_current(self, robot_name: RobotName | None = None) -> bool:
        """Set init joints to the current joint positions.

        Args:
            robot_name: Robot to capture from (required if multiple robots configured)
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return False
        robot_name_resolved, robot_id, _, _ = robot
        if self._world_monitor is None:
            return False
        current = self._world_monitor.get_current_joint_state(robot_id)
        if current is None:
            logger.error("Cannot capture init joints — no current joint state")
            return False
        self._init_joints[robot_name_resolved] = current
        logger.info(
            f"Init joints set to current for '{robot_name_resolved}': "
            f"[{', '.join(f'{j:.3f}' for j in current.position)}]"
        )
        return True

    def _get_coordinator_client(self) -> RPCClient | None:
        """Get or create coordinator RPC client (lazy init)."""
        if not any(
            c.coordinator_task_name or c.gripper_hardware_id for _, c, _ in self._robots.values()
        ):
            return None
        if self._coordinator_client is None:
            from dimos.control.coordinator import ControlCoordinator
            from dimos.core.rpc_client import RPCClient

            self._coordinator_client = RPCClient(None, ControlCoordinator)
        return self._coordinator_client

    def _invoke_coordinator_task(
        self,
        client: RPCClient,
        task_name: str,
        method: str,
        kwargs: dict[str, Any],
    ) -> Any:
        """Invoke a ControlCoordinator task with an execution-specific timeout."""
        remote_name = getattr(client, "remote_name", None)
        rpc_client = getattr(client, "rpc", None)
        call_sync = getattr(rpc_client, "call_sync", None)
        if isinstance(remote_name, str) and callable(call_sync):
            result, unsub_fn = call_sync(
                f"{remote_name}/task_invoke",
                ([task_name, method, kwargs], {}),
                rpc_timeout=self.config.coordinator_rpc_timeout,
            )
            unsub_fns = getattr(client, "_unsub_fns", None)
            if isinstance(unsub_fns, list):
                unsub_fns.append(unsub_fn)
            return result
        return client.task_invoke(task_name, method, kwargs)

    @rpc
    def execute(self) -> bool:
        """Execute planned trajectory via ControlCoordinator."""
        return self.execute_plan(self._last_plan)

    @rpc
    def execute_plan(self, plan: GeneratedPlan | None = None) -> bool:
        """Execute a generated plan through its canonical generated trajectory."""
        plan = plan or self._last_plan
        if plan is None or not plan.path:
            logger.warning("No generated plan")
            return False
        if (client := self._get_coordinator_client()) is None:
            logger.error("No coordinator client")
            return False

        trajectory = self._trajectory_for_plan(plan)
        if not trajectory.is_success():
            return self._fail(f"Trajectory parametrization failed: {trajectory.message}")
        dispatch = self._prepare_trajectory_dispatch(trajectory)
        if not dispatch.is_success():
            return self._fail(f"Trajectory dispatch failed: {dispatch.message}")
        logger.info(
            "Execute plan: groups=%s, affected=%s",
            plan.group_ids,
            list(dispatch.robot_names_by_task.values()),
        )

        self._state = ManipulationState.EXECUTING
        for task_name, task_trajectory in dispatch.trajectories_by_task.items():
            logger.info(
                "Executing: task='%s', %d pts, %.2fs",
                task_name,
                len(task_trajectory.points),
                task_trajectory.duration,
            )
            try:
                result = self._invoke_coordinator_task(
                    client,
                    task_name,
                    "execute",
                    {"trajectory": task_trajectory},
                )
            except TimeoutError as exc:
                return self._fail(f"Coordinator RPC timed out for task '{task_name}': {exc}")
            except Exception as exc:
                return self._fail(f"Coordinator RPC failed for task '{task_name}': {exc}")
            logger.info(
                "Coordinator execute result: task='%s', result=%r",
                task_name,
                result,
            )
            if not result:
                return self._fail("Coordinator rejected trajectory")

        logger.info("Trajectory accepted")
        self._state = ManipulationState.COMPLETED
        return True

    @rpc
    def get_trajectory_status(self, robot_name: RobotName | None = None) -> dict[str, Any] | None:
        """Get trajectory execution status via coordinator task_invoke."""
        last_plan = self._last_plan
        if robot_name is None and last_plan is not None and last_plan.path:
            statuses = {
                name: self.get_trajectory_status(name)
                for name in self._affected_robot_names(last_plan)
            }
            return {"robots": statuses}

        if (robot := self._get_robot(robot_name)) is None:
            return None
        _, _, config, _ = robot
        if not config.coordinator_task_name or (client := self._get_coordinator_client()) is None:
            return None
        try:
            state = client.task_invoke(config.coordinator_task_name, "get_state", {})
            if state is not None:
                return {"state": int(state), "task": config.coordinator_task_name}
            return None
        except Exception:
            return None

    @property
    def world_monitor(self) -> WorldMonitor | None:
        """Access the world monitor for advanced obstacle/world operations."""
        return self._world_monitor

    @rpc
    def add_obstacle(
        self,
        name: str,
        pose: Pose,
        shape: str,
        dimensions: list[float] | None = None,
        mesh_path: str | None = None,
    ) -> str:
        """Add obstacle: shape='box'|'sphere'|'cylinder'|'mesh'. Returns obstacle_id."""
        if not self._world_monitor:
            return ""

        # Map shape string to ObstacleType
        shape_map = {
            "box": ObstacleType.BOX,
            "sphere": ObstacleType.SPHERE,
            "cylinder": ObstacleType.CYLINDER,
            "mesh": ObstacleType.MESH,
        }
        obstacle_type = shape_map.get(shape)
        if obstacle_type is None:
            logger.warning(f"Unknown obstacle shape: {shape}")
            return ""

        # Validate mesh_path for mesh type
        if obstacle_type == ObstacleType.MESH and not mesh_path:
            logger.warning("mesh_path required for mesh obstacles")
            return ""

        obstacle = Obstacle(
            name=name,
            obstacle_type=obstacle_type,
            pose=PoseStamped(position=pose.position, orientation=pose.orientation),
            dimensions=tuple(dimensions) if dimensions else (),
            mesh_path=mesh_path,
        )
        return self._world_monitor.add_obstacle(obstacle)

    @rpc
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the planning world."""
        if self._world_monitor is None:
            return False
        return self._world_monitor.remove_obstacle(obstacle_id)

    def _get_gripper_hardware_id(self, robot_name: RobotName | None = None) -> str | None:
        """Get gripper hardware ID for a robot."""
        robot = self._get_robot(robot_name)
        if robot is None:
            return None
        _, _, config, _ = robot
        if not config.gripper_hardware_id:
            logger.warning(f"No gripper_hardware_id configured for '{config.name}'")
            return None
        return str(config.gripper_hardware_id)

    def _set_gripper_position(self, position: float, robot_name: RobotName | None = None) -> bool:
        """Internal: set gripper position in meters."""
        hw_id = self._get_gripper_hardware_id(robot_name)
        if hw_id is None:
            return False
        client = self._get_coordinator_client()
        if client is None:
            logger.error("No coordinator client for gripper control")
            return False
        return bool(client.set_gripper_position(hw_id, position))

    @rpc
    def get_gripper(self, robot_name: RobotName | None = None) -> float | None:
        """Get gripper position in meters.

        Args:
            robot_name: Robot to query (required if multiple robots configured)
        """
        hw_id = self._get_gripper_hardware_id(robot_name)
        if hw_id is None:
            return None
        client = self._get_coordinator_client()
        if client is None:
            return None
        result = client.get_gripper_position(hw_id)
        return float(result) if result is not None else None

    @skill
    def set_gripper(
        self, position: float, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Set gripper to a specific opening in meters.

        Args:
            position: Gripper opening in meters (0.0 = closed, 0.85 = fully open).
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        if self._set_gripper_position(position, robot_name):
            return SkillResult.ok(f"Gripper set to {position:.3f}m")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to set gripper position")

    @skill
    def open_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Open the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        if self._set_gripper_position(0.85, robot_name):
            return SkillResult.ok("Gripper opened")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to open gripper")

    @skill
    def close_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Close the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        if self._set_gripper_position(0.0, robot_name):
            return SkillResult.ok("Gripper closed")
        return SkillResult.fail("GRIPPER_FAILED", "Failed to close gripper")

    def _wait_for_trajectory_completion(
        self, robot_name: RobotName | None = None, timeout: float = 60.0, poll_interval: float = 0.2
    ) -> bool:
        """Wait for trajectory execution to complete.

        Polls the coordinator task state via task_invoke. Falls back to waiting
        for the trajectory duration if the coordinator is unavailable.

        Args:
            robot_name: Robot to monitor
            timeout: Maximum wait time in seconds
            poll_interval: Time between status checks

        Returns:
            True if trajectory completed successfully
        """
        last_plan = self._last_plan
        if robot_name is None and last_plan is not None and last_plan.path:
            try:
                robot_names = self._affected_robot_names(last_plan)
            except Exception as exc:
                logger.warning("Failed to resolve generated plan while waiting: %s", exc)
                return False
            return all(
                self._wait_for_trajectory_completion(name, timeout, poll_interval)
                for name in robot_names
            )

        robot = self._get_robot(robot_name)
        if robot is None:
            return True
        rname, _, config, _ = robot
        client = self._get_coordinator_client()

        if client is None or not config.coordinator_task_name:
            logger.info("No coordinator status available for '%s'", rname)
            return True

        # Poll task state via task_invoke
        start = time.time()
        while (time.time() - start) < timeout:
            try:
                state = client.task_invoke(config.coordinator_task_name, "get_state", {})
                # TrajectoryState is an IntEnum: IDLE=0, EXECUTING=1, COMPLETED=2, ABORTED=3, FAULT=4
                if state is not None:
                    state_val = int(state)
                    if state_val in (0, 2):  # IDLE or COMPLETED
                        return True
                    if state_val in (3, 4):  # ABORTED or FAULT
                        logger.warning(f"Trajectory failed: state={state}")
                        return False
                    # state_val == 1 means EXECUTING, keep polling
                else:
                    # task_invoke returned None — task not found, assume done
                    return True
            except Exception:
                logger.info("Status poll failed for '%s'", rname)
                return True
            time.sleep(poll_interval)

        logger.warning(f"Trajectory execution timed out after {timeout}s")
        return False

    def _lift_if_low(
        self, robot_name: RobotName | None = None, min_z: float = 0.05
    ) -> SkillResult[ManipulationSkillError]:
        """If the end-effector is below *min_z*, plan and execute a short lift."""
        ee = self.get_ee_pose(robot_name)
        if ee is None or ee.position.z >= min_z:
            return SkillResult.ok()

        lift_z = min_z + 0.05
        logger.info(f"EE z={ee.position.z:.3f} < {min_z}, lifting to z={lift_z:.3f}")
        lift_pose = Pose(Vector3(ee.position.x, ee.position.y, lift_z), ee.orientation)
        if not self.plan_to_pose(lift_pose, robot_name):
            return SkillResult.fail(
                "PLANNING_FAILED",
                f"Failed to plan lift from z={ee.position.z:.3f}",
            )
        return self._preview_execute_wait(robot_name)

    def _preview_execute_wait(
        self, robot_name: RobotName | None = None, preview_duration: float | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Preview planned path, execute, and wait for completion.

        Args:
            robot_name: Robot to operate on
            preview_duration: Optional override for preview animation duration. When unset,
                preview uses the generated trajectory duration.
        """
        logger.info("Previewing trajectory...")
        self.preview_plan(duration=preview_duration, robot_name=robot_name)

        logger.info("Executing trajectory...")
        if not self.execute():
            return SkillResult.fail("EXECUTION_FAILED", "Trajectory execution failed")

        if not self._wait_for_trajectory_completion(robot_name):
            return SkillResult.fail("EXECUTION_TIMEOUT", "Trajectory execution timed out")

        return SkillResult.ok()

    @skill
    def get_robot_state(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Get current robot state: joint positions, end-effector pose, and gripper.

        Args:
            robot_name: Robot to query (only needed for multi-arm setups).
        """
        lines: list[str] = []

        joints = self.get_current_joints(robot_name)
        if joints is not None:
            lines.append(f"Joints: [{', '.join(f'{j:.3f}' for j in joints)}]")
        else:
            lines.append("Joints: unavailable (no state received)")

        ee_pose = self.get_ee_pose(robot_name)
        if ee_pose is not None:
            p = ee_pose.position
            lines.append(f"EE pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
        else:
            lines.append("EE pose: unavailable")

        gripper_pos = self.get_gripper(robot_name)
        if gripper_pos is not None:
            lines.append(f"Gripper: {gripper_pos:.3f}m")
        else:
            lines.append("Gripper: not configured")

        lines.append(f"State: {self.get_state()}")

        return SkillResult.ok("\n".join(lines))

    @skill
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot end-effector to a target pose.

        Plans a collision-free trajectory and executes it.
        If roll/pitch/yaw are omitted, the current EE orientation is preserved.

        Args:
            x: Target X position in meters.
            y: Target Y position in meters.
            z: Target Z position in meters.
            roll: Target roll in radians (omit to keep current orientation).
            pitch: Target pitch in radians (omit to keep current orientation).
            yaw: Target yaw in radians (omit to keep current orientation).
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        logger.info(f"Planning motion to ({x:.3f}, {y:.3f}, {z:.3f})...")

        # If no orientation specified, preserve the current EE orientation.
        # If partially specified, fill unspecified angles from current orientation.
        if roll is None and pitch is None and yaw is None:
            current_pose = self.get_ee_pose(robot_name)
            if current_pose is not None:
                orientation = current_pose.orientation
            else:
                orientation = Quaternion(0, 0, 0, 1)  # identity fallback
        else:
            current_pose = self.get_ee_pose(robot_name)
            if current_pose is not None:
                current_euler = current_pose.orientation.to_euler()
                orientation = Quaternion.from_euler(
                    Vector3(
                        roll if roll is not None else current_euler.x,
                        pitch if pitch is not None else current_euler.y,
                        yaw if yaw is not None else current_euler.z,
                    )
                )
            else:
                orientation = Quaternion.from_euler(Vector3(roll or 0.0, pitch or 0.0, yaw or 0.0))

        pose = Pose(Vector3(x, y, z), orientation)

        # If EE is low, lift up first to clear obstacles
        lift = self._lift_if_low(robot_name)
        if not lift.is_success():
            return lift

        if not self.plan_to_pose(pose, robot_name):
            return SkillResult.fail(
                "PLANNING_FAILED",
                f"Pose ({x:.3f}, {y:.3f}, {z:.3f}) may be unreachable or in collision",
            )

        exec_result = self._preview_execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok(f"Reached target pose ({x:.3f}, {y:.3f}, {z:.3f})")

    @skill
    def move_to_joints(
        self,
        joints: str,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to a target joint configuration.

        Plans a collision-free trajectory and executes it.

        Args:
            joints: Comma-separated joint positions in radians, e.g. "0.1, -0.5, 1.2, 0.0, 0.3, -0.1".
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        try:
            joint_values = [float(j.strip()) for j in joints.split(",")]
        except ValueError:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"Invalid joints format '{joints}'. Expected comma-separated floats.",
            )

        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot
        goal = JointState(name=config.joint_names, position=joint_values)

        logger.info(f"Planning motion to joints [{', '.join(f'{j:.3f}' for j in joint_values)}]...")
        if not self.plan_to_joints(goal, rname):
            return SkillResult.fail(
                "PLANNING_FAILED",
                "Joint configuration may be unreachable or in collision",
            )

        exec_result = self._preview_execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached target joint configuration")

    @skill
    def go_home(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its home/observe joint configuration.

        Opens the gripper and moves to the predefined home position.

        Args:
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot

        if config.home_joints is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "No home_joints configured for this robot",
            )

        logger.info("Opening gripper...")
        self._set_gripper_position(0.85, rname)
        time.sleep(0.5)

        goal = JointState(name=config.joint_names, position=config.home_joints)
        logger.info("Planning motion to home position...")
        if not self.plan_to_joints(goal, rname):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to home position")

        exec_result = self._preview_execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached home position")

    @skill
    def go_init(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its init position (captured at startup or set manually).

        The init position is the joint configuration the robot was in when the
        module first received joint state. It can be changed with set_init_joints().

        Args:
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, robot_id, _, _ = robot

        init = self._init_joints.get(rname)
        if init is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "No init joints captured — robot may not have reported joint state yet",
            )

        # Lift if EE is low before moving to init
        lift = self._lift_if_low(robot_name)
        if not lift.is_success():
            return lift

        # Move through a safe waypoint: 10cm above and 5cm in front of init pose.
        # This avoids direct paths through the workspace that could collide with objects.
        if self._world_monitor is not None:
            init_ee = self._world_monitor.get_ee_pose(robot_id, joint_state=init)
            if init_ee is not None:
                wp = Pose(
                    Vector3(
                        init_ee.position.x + 0.05,
                        init_ee.position.y,
                        init_ee.position.z + 0.10,
                    ),
                    init_ee.orientation,
                )
                if self.plan_to_pose(wp, robot_name):
                    wp_result = self._preview_execute_wait(robot_name)
                    if not wp_result.is_success():
                        return wp_result
                else:
                    logger.warning("Safe waypoint unreachable, going directly to init")

        logger.info(
            f"Planning motion to init position [{', '.join(f'{j:.3f}' for j in init.position)}]..."
        )
        if not self.plan_to_joints(init, robot_name):
            return SkillResult.fail("PLANNING_FAILED", "Failed to plan path to init position")

        exec_result = self._preview_execute_wait(robot_name)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok("Reached init position")

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        logger.info("Stopping ManipulationModule")

        # Stop TF thread
        if self._tf_thread is not None:
            self._tf_stop_event.set()
            self._tf_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._tf_thread = None

        # Stop world monitor (includes visualization thread)
        if self._world_monitor is not None:
            self._world_monitor.stop_all_monitors()

        super().stop()
