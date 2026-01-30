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

"""Drake World Implementation - WorldSpec using Drake's MultibodyPlant and SceneGraph."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.spec import (
    JointPath,
    Obstacle,
    ObstacleType,
    RobotModelConfig,
    WorldRobotID,
    WorldSpec,
)
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

from dimos.msgs.geometry_msgs import PoseStamped, Transform
from dimos.msgs.sensor_msgs import JointState

try:
    from pydrake.geometry import (  # type: ignore[import-not-found]
        AddContactMaterial,
        Box,
        CollisionFilterDeclaration,
        Cylinder,
        GeometryInstance,
        GeometrySet,
        MakePhongIllustrationProperties,
        Meshcat,
        MeshcatVisualizer,
        MeshcatVisualizerParams,
        ProximityProperties,
        Rgba,
        Role,
        SceneGraph,
        Sphere,
    )
    from pydrake.math import RigidTransform  # type: ignore[import-not-found]
    from pydrake.multibody.parsing import Parser  # type: ignore[import-not-found]
    from pydrake.multibody.plant import (  # type: ignore[import-not-found]
        AddMultibodyPlantSceneGraph,
        CoulombFriction,
        MultibodyPlant,
    )
    from pydrake.multibody.tree import JacobianWrtVariable  # type: ignore[import-not-found]
    from pydrake.systems.framework import Context, DiagramBuilder  # type: ignore[import-not-found]

    DRAKE_AVAILABLE = True
except ImportError:
    DRAKE_AVAILABLE = False

logger = setup_logger()


@dataclass
class _RobotData:
    """Internal data for tracking a robot in the world."""

    robot_id: WorldRobotID
    config: RobotModelConfig
    model_instance: Any  # ModelInstanceIndex
    joint_indices: list[int]  # Indices into plant's position vector
    ee_frame: Any  # BodyFrame for end-effector
    base_frame: Any  # BodyFrame for base


@dataclass
class _ObstacleData:
    """Internal data for tracking an obstacle in the world."""

    obstacle_id: str
    obstacle: Obstacle
    geometry_id: Any  # GeometryId
    source_id: Any  # SourceId


class DrakeWorld(WorldSpec):
    """Drake implementation of WorldSpec with MultibodyPlant, SceneGraph, optional Meshcat."""

    def __init__(self, time_step: float = 0.0, enable_viz: bool = False):
        if not DRAKE_AVAILABLE:
            raise ImportError("Drake is not installed. Install with: pip install drake")

        self._time_step = time_step
        self._enable_viz = enable_viz
        self._lock = RLock()

        # Build Drake diagram
        self._builder = DiagramBuilder()
        self._plant: MultibodyPlant
        self._scene_graph: SceneGraph
        self._plant, self._scene_graph = AddMultibodyPlantSceneGraph(
            self._builder, time_step=time_step
        )
        self._parser = Parser(self._plant)
        # Enable auto-renaming to avoid conflicts when adding multiple robots
        # with the same URDF (e.g., 4 XArm6 arms all have model name "UF_ROBOT")
        self._parser.SetAutoRenaming(True)

        # Visualization
        self._meshcat: Meshcat | None = None
        self._meshcat_visualizer: MeshcatVisualizer | None = None
        if enable_viz:
            self._meshcat = Meshcat()

        # Create model instance for obstacles
        self._obstacles_model_instance = self._plant.AddModelInstance("obstacles")

        # Tracking data
        self._robots: dict[WorldRobotID, _RobotData] = {}
        self._obstacles: dict[str, _ObstacleData] = {}
        self._robot_counter = 0
        self._obstacle_counter = 0

        # Built diagram and contexts (created after finalize)
        self._diagram: Any = None
        self._live_context: Context | None = None
        self._plant_context: Context | None = None
        self._scene_graph_context: Context | None = None
        self._finalized = False

        # Obstacle source for dynamic obstacles
        self._obstacle_source_id: Any = None

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a robot to the world. Returns robot_id."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")

        with self._lock:
            self._robot_counter += 1
            robot_id = f"robot_{self._robot_counter}"

            model_instance = self._load_urdf(config)
            self._weld_base_if_needed(config, model_instance)
            self._validate_joints(config, model_instance)

            ee_frame = self._plant.GetBodyByName(
                config.end_effector_link, model_instance
            ).body_frame()
            base_frame = self._plant.GetBodyByName(config.base_link, model_instance).body_frame()

            self._robots[robot_id] = _RobotData(
                robot_id=robot_id,
                config=config,
                model_instance=model_instance,
                joint_indices=[],
                ee_frame=ee_frame,
                base_frame=base_frame,
            )

            logger.info(f"Added robot '{robot_id}' ({config.name})")
            return robot_id

    def _load_urdf(self, config: RobotModelConfig) -> Any:
        """Load URDF/xacro and return model instance."""
        original_path = config.urdf_path.resolve()
        if not original_path.exists():
            raise FileNotFoundError(f"URDF/xacro not found: {original_path}")

        urdf_path = prepare_urdf_for_drake(
            urdf_path=original_path,
            package_paths=config.package_paths,
            xacro_args=config.xacro_args,
            convert_meshes=config.auto_convert_meshes,
        )
        urdf_path_obj = Path(urdf_path)
        logger.info(f"Using prepared URDF: {urdf_path_obj}")

        # Register package paths
        if config.package_paths:
            for pkg_name, pkg_path in config.package_paths.items():
                self._parser.package_map().Add(pkg_name, Path(pkg_path))
        else:
            self._parser.package_map().Add(f"{config.name}_description", urdf_path_obj.parent)

        model_instances = self._parser.AddModels(urdf_path_obj)
        if not model_instances:
            raise ValueError(f"Failed to parse URDF: {urdf_path}")
        return model_instances[0]

    def _weld_base_if_needed(self, config: RobotModelConfig, model_instance: Any) -> None:
        """Weld robot base to world if not already welded in URDF."""
        base_body = self._plant.GetBodyByName(config.base_link, model_instance)

        # Check if URDF already has world_joint
        try:
            world_joint = self._plant.GetJointByName("world_joint", model_instance)
            if (
                world_joint.parent_body().name() == "world"
                and world_joint.child_body().name() == config.base_link
            ):
                logger.info("URDF has 'world_joint', skipping weld")
                return
        except RuntimeError:
            pass

        # Weld base to world
        base_transform = self._pose_to_rigid_transform(config.base_pose)
        self._plant.WeldFrames(
            self._plant.world_frame(),
            base_body.body_frame(),
            base_transform,
        )

    def _validate_joints(self, config: RobotModelConfig, model_instance: Any) -> None:
        """Validate that all configured joints exist in URDF."""
        for joint_name in config.joint_names:
            try:
                self._plant.GetJointByName(joint_name, model_instance)
            except RuntimeError:
                raise ValueError(f"Joint '{joint_name}' not found in URDF")

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs in the world."""
        return list(self._robots.keys())

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get robot configuration by ID."""
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")
        return self._robots[robot_id].config

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits (lower, upper) in radians."""
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        config = self._robots[robot_id].config

        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            return (
                np.array(config.joint_limits_lower),
                np.array(config.joint_limits_upper),
            )

        # Default to ±π
        n_joints = len(config.joint_names)
        return (
            np.full(n_joints, -np.pi),
            np.full(n_joints, np.pi),
        )

    # ============= Obstacle Management =============

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle to the world."""
        with self._lock:
            # Use obstacle's name as ID (allows external ID management)
            obstacle_id = obstacle.name

            # Check for duplicate in our tracking
            if obstacle_id in self._obstacles:
                logger.debug(f"Obstacle '{obstacle_id}' already exists, skipping")
                return obstacle_id

            try:
                if not self._finalized:
                    geometry_id = self._add_obstacle_to_plant(obstacle, obstacle_id)
                    self._obstacles[obstacle_id] = _ObstacleData(
                        obstacle_id=obstacle_id,
                        obstacle=obstacle,
                        geometry_id=geometry_id,
                        source_id=self._plant.get_source_id(),
                    )
                else:
                    geometry_id = self._add_obstacle_to_scene_graph(obstacle, obstacle_id)
                    self._obstacles[obstacle_id] = _ObstacleData(
                        obstacle_id=obstacle_id,
                        obstacle=obstacle,
                        geometry_id=geometry_id,
                        source_id=self._obstacle_source_id,
                    )

                logger.debug(f"Added obstacle '{obstacle_id}': {obstacle.obstacle_type.value}")
            except RuntimeError as e:
                # Handle case where geometry name already exists in SceneGraph
                # (can happen with concurrent access)
                if "already been used" in str(e):
                    logger.debug(f"Obstacle '{obstacle_id}' already in SceneGraph, skipping")
                else:
                    raise

            return obstacle_id

    def _add_obstacle_to_plant(self, obstacle: Obstacle, obstacle_id: str) -> Any:
        """Add obstacle to plant (before finalization)."""
        shape = self._create_shape(obstacle)

        body = self._plant.AddRigidBody(
            obstacle_id,
            self._obstacles_model_instance,
        )

        transform = self._pose_to_rigid_transform(obstacle.pose)
        geometry_id = self._plant.RegisterCollisionGeometry(
            body,
            RigidTransform(),
            shape,
            obstacle_id + "_collision",
            ProximityProperties(),
        )

        diffuse_color = np.array(obstacle.color)
        self._plant.RegisterVisualGeometry(
            body,
            RigidTransform(),
            shape,
            obstacle_id + "_visual",
            diffuse_color,
        )

        self._plant.WeldFrames(
            self._plant.world_frame(),
            body.body_frame(),
            transform,
        )

        return geometry_id

    def _add_obstacle_to_scene_graph(self, obstacle: Obstacle, obstacle_id: str) -> Any:
        """Add obstacle to scene graph (after finalization)."""
        if self._obstacle_source_id is None:
            raise RuntimeError("Obstacle source not initialized")

        shape = self._create_shape(obstacle)
        transform = self._pose_to_rigid_transform(obstacle.pose)
        # MakePhongIllustrationProperties expects numpy array, not Rgba
        rgba_array = np.array(obstacle.color, dtype=np.float64)

        # Create proximity properties with contact material for collision detection
        # Without these properties, the geometry is invisible to collision queries
        proximity_props = ProximityProperties()
        AddContactMaterial(
            dissipation=0.0,
            point_stiffness=1e6,
            friction=CoulombFriction(static_friction=1.0, dynamic_friction=1.0),
            properties=proximity_props,
        )

        geometry_instance = GeometryInstance(
            X_PG=transform,
            shape=shape,
            name=obstacle_id,
        )
        geometry_instance.set_illustration_properties(MakePhongIllustrationProperties(rgba_array))
        geometry_instance.set_proximity_properties(proximity_props)

        frame_id = self._scene_graph.world_frame_id()
        geometry_id = self._scene_graph.RegisterGeometry(
            self._obstacle_source_id,
            frame_id,
            geometry_instance,
        )

        # Also add to Meshcat directly (MeshcatVisualizer doesn't show dynamic geometries)
        if self._meshcat is not None:
            self._add_obstacle_to_meshcat(obstacle, obstacle_id)

        return geometry_id

    def _add_obstacle_to_meshcat(self, obstacle: Obstacle, obstacle_id: str) -> None:
        """Add obstacle visualization directly to Meshcat."""
        if self._meshcat is None:
            return

        # Use Drake's geometry types for Meshcat
        path = f"obstacles/{obstacle_id}"
        transform = self._pose_to_rigid_transform(obstacle.pose)
        rgba = Rgba(*obstacle.color)

        # Create Drake shape and add to Meshcat
        shape: Box | Sphere | Cylinder
        if obstacle.obstacle_type == ObstacleType.BOX:
            shape = Box(*obstacle.dimensions)
        elif obstacle.obstacle_type == ObstacleType.SPHERE:
            shape = Sphere(obstacle.dimensions[0])
        elif obstacle.obstacle_type == ObstacleType.CYLINDER:
            shape = Cylinder(obstacle.dimensions[0], obstacle.dimensions[1])
        else:
            logger.warning(f"Cannot visualize obstacle type: {obstacle.obstacle_type}")
            return

        # Use Drake's Meshcat.SetObject with shape and color
        self._meshcat.SetObject(path, shape, rgba)
        self._meshcat.SetTransform(path, transform)

    def _pose_to_rigid_transform(self, pose: PoseStamped) -> Any:
        """Convert PoseStamped to Drake RigidTransform."""
        pose_matrix = Transform(
            translation=pose.position,
            rotation=pose.orientation,
        ).to_matrix()
        return RigidTransform(pose_matrix)

    def _create_shape(self, obstacle: Obstacle) -> Any:
        """Create Drake shape from obstacle specification."""
        if obstacle.obstacle_type == ObstacleType.BOX:
            return Box(*obstacle.dimensions)
        elif obstacle.obstacle_type == ObstacleType.SPHERE:
            return Sphere(obstacle.dimensions[0])
        elif obstacle.obstacle_type == ObstacleType.CYLINDER:
            return Cylinder(obstacle.dimensions[0], obstacle.dimensions[1])
        else:
            raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle by ID."""
        with self._lock:
            if obstacle_id not in self._obstacles:
                return False

            obstacle_data = self._obstacles[obstacle_id]

            if self._finalized and self._scene_graph_context is not None:
                self._scene_graph.RemoveGeometry(
                    obstacle_data.source_id,
                    obstacle_data.geometry_id,
                )

            # Also remove from Meshcat
            if self._meshcat is not None:
                path = f"obstacles/{obstacle_id}"
                self._meshcat.Delete(path)

            del self._obstacles[obstacle_id]
            logger.debug(f"Removed obstacle '{obstacle_id}'")
            return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update obstacle pose."""
        with self._lock:
            if obstacle_id not in self._obstacles:
                return False

            # Store PoseStamped directly
            self._obstacles[obstacle_id].obstacle.pose = pose

            # Update Meshcat visualization
            if self._meshcat is not None:
                path = f"obstacles/{obstacle_id}"
                transform = self._pose_to_rigid_transform(pose)
                self._meshcat.SetTransform(path, transform)

            # Note: SceneGraph geometry pose is fixed after registration
            # Meshcat is updated for visualization, but collision checking
            # uses the original pose. For dynamic obstacles, remove and re-add.

            return True

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        with self._lock:
            obstacle_ids = list(self._obstacles.keys())
            for obs_id in obstacle_ids:
                self.remove_obstacle(obs_id)

    # ============= Lifecycle =============

    def finalize(self) -> None:
        """Finalize world - locks robot topology, enables collision checking."""
        if self._finalized:
            logger.warning("World already finalized")
            return

        with self._lock:
            # Finalize plant
            self._plant.Finalize()

            # Compute joint indices for each robot
            for robot_id, robot_data in self._robots.items():
                joint_indices: list[int] = []
                for joint_name in robot_data.config.joint_names:
                    joint = self._plant.GetJointByName(joint_name, robot_data.model_instance)
                    start_idx = joint.position_start()
                    num_positions = joint.num_positions()
                    joint_indices.extend(range(start_idx, start_idx + num_positions))
                robot_data.joint_indices = joint_indices
                logger.debug(f"Robot '{robot_id}' joint indices: {joint_indices}")

            # Setup collision filters
            self._setup_collision_filters()

            # Register obstacle source for dynamic obstacles
            self._obstacle_source_id = self._scene_graph.RegisterSource("dynamic_obstacles")

            # Add visualization if enabled
            if self._meshcat is not None:
                params = MeshcatVisualizerParams()
                params.role = Role.kIllustration
                self._meshcat_visualizer = MeshcatVisualizer.AddToBuilder(
                    self._builder,
                    self._scene_graph,
                    self._meshcat,
                    params,
                )

            # Build diagram
            self._diagram = self._builder.Build()
            self._live_context = self._diagram.CreateDefaultContext()

            # Get subsystem contexts
            self._plant_context = self._diagram.GetMutableSubsystemContext(
                self._plant, self._live_context
            )
            self._scene_graph_context = self._diagram.GetMutableSubsystemContext(
                self._scene_graph, self._live_context
            )

            self._finalized = True
            logger.info(f"World finalized with {len(self._robots)} robots")

            # Initial visualization publish
            if self._meshcat_visualizer is not None:
                self._meshcat_visualizer.ForcedPublish(
                    self._diagram.GetSubsystemContext(self._meshcat_visualizer, self._live_context)
                )

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        return self._finalized

    def _setup_collision_filters(self) -> None:
        """Filter collisions between adjacent links and user-specified pairs."""
        for robot_data in self._robots.values():
            # Filter parent-child pairs (adjacent links always "collide")
            for joint_idx in self._plant.GetJointIndices(robot_data.model_instance):
                joint = self._plant.get_joint(joint_idx)
                parent, child = joint.parent_body(), joint.child_body()
                if parent.index() != self._plant.world_body().index():
                    self._exclude_body_pair(parent, child)

            # Filter user-specified pairs (e.g., parallel linkage grippers)
            for name1, name2 in robot_data.config.collision_exclusion_pairs:
                try:
                    body1 = self._plant.GetBodyByName(name1, robot_data.model_instance)
                    body2 = self._plant.GetBodyByName(name2, robot_data.model_instance)
                    self._exclude_body_pair(body1, body2)
                except RuntimeError:
                    logger.warning(f"Collision exclusion: link not found: {name1} or {name2}")

        logger.info("Collision filters applied")

    def _exclude_body_pair(self, body1: Any, body2: Any) -> None:
        """Exclude collision between two bodies."""
        geoms1 = self._plant.GetCollisionGeometriesForBody(body1)
        geoms2 = self._plant.GetCollisionGeometriesForBody(body2)
        if geoms1 and geoms2:
            self._scene_graph.collision_filter_manager().Apply(
                CollisionFilterDeclaration().ExcludeBetween(
                    GeometrySet(geoms1), GeometrySet(geoms2)
                )
            )

    # ============= Context Management =============

    def get_live_context(self) -> Context:
        """Get the live context (mirrors current robot state).

        WARNING: Not thread-safe for reads during writes.
        Use scratch_context() for planning operations.
        """
        if not self._finalized or self._live_context is None:
            raise RuntimeError("World must be finalized first")
        return self._live_context

    @contextmanager
    def scratch_context(self) -> Generator[Context, None, None]:
        """Thread-safe context for planning. Copies current robot states for inter-robot collision checking."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        ctx = self._diagram.CreateDefaultContext()

        # Copy live robot states so inter-robot collision checking works
        with self._lock:
            if self._plant_context is not None:
                plant_ctx = self._diagram.GetMutableSubsystemContext(self._plant, ctx)
                for robot_data in self._robots.values():
                    try:
                        positions = self._plant.GetPositions(
                            self._plant_context, robot_data.model_instance
                        )
                        self._plant.SetPositions(plant_ctx, robot_data.model_instance, positions)
                    except RuntimeError:
                        pass  # Robot not yet synced

        yield ctx

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live context from driver's joint state message.

        Called by StateMonitor when new JointState arrives.
        """
        if not self._finalized or self._plant_context is None:
            return  # Silently ignore before finalization

        # Extract positions as numpy array for internal use
        positions = np.array(joint_state.position, dtype=np.float64)

        with self._lock:
            self._set_positions_internal(self._plant_context, robot_id, positions)

            # NOTE: ForcedPublish is intentionally NOT called here.
            # Calling ForcedPublish from the LCM callback thread blocks message processing.
            # Visualization can be updated via publish_to_meshcat() from non-callback contexts.

    # ============= State Operations (context-based) =============

    def set_joint_state(
        self, ctx: Context, robot_id: WorldRobotID, joint_state: JointState
    ) -> None:
        """Set robot joint state in given context."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        # Extract positions as numpy array for internal use
        positions = np.array(joint_state.position, dtype=np.float64)

        # Get plant context from diagram context
        plant_ctx = self._diagram.GetMutableSubsystemContext(self._plant, ctx)
        self._set_positions_internal(plant_ctx, robot_id, positions)

    def _set_positions_internal(
        self, plant_ctx: Context, robot_id: WorldRobotID, positions: NDArray[np.float64]
    ) -> None:
        """Internal: Set positions in a plant context."""
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        full_positions = self._plant.GetPositions(plant_ctx).copy()

        for i, joint_idx in enumerate(robot_data.joint_indices):
            full_positions[joint_idx] = positions[i]

        self._plant.SetPositions(plant_ctx, full_positions)

    def get_joint_state(self, ctx: Context, robot_id: WorldRobotID) -> JointState:
        """Get robot joint state from given context."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)
        full_positions = self._plant.GetPositions(plant_ctx)

        positions = [float(full_positions[idx]) for idx in robot_data.joint_indices]
        return JointState(name=robot_data.config.joint_names, position=positions)

    # ============= Collision Checking (context-based) =============

    def is_collision_free(self, ctx: Context, robot_id: WorldRobotID) -> bool:
        """Check if current configuration in context is collision-free."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        scene_graph_ctx = self._diagram.GetSubsystemContext(self._scene_graph, ctx)
        query_object = self._scene_graph.get_query_output_port().Eval(scene_graph_ctx)

        return not query_object.HasCollisions()  # type: ignore[attr-defined]

    def get_min_distance(self, ctx: Context, robot_id: WorldRobotID) -> float:
        """Get minimum signed distance (positive = clearance, negative = penetration)."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        scene_graph_ctx = self._diagram.GetSubsystemContext(self._scene_graph, ctx)
        query_object = self._scene_graph.get_query_output_port().Eval(scene_graph_ctx)

        signed_distance_pairs = query_object.ComputeSignedDistancePairwiseClosestPoints()  # type: ignore[attr-defined]

        if not signed_distance_pairs:
            return float("inf")

        return float(min(pair.distance for pair in signed_distance_pairs))

    # ============= Collision Checking (context-free, for planning) =============

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check if a joint state is collision-free (manages context internally).

        This is a convenience method for planners that don't need to manage contexts.
        """
        with self.scratch_context() as ctx:
            self.set_joint_state(ctx, robot_id, joint_state)
            return self.is_collision_free(ctx, robot_id)

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check if the entire edge between two joint states is collision-free.

        Interpolates between start and end at the given step_size and checks
        each configuration for collisions. This is more efficient than checking
        each configuration separately as it uses a single scratch context.
        """
        # Extract positions as numpy arrays for interpolation
        q_start = np.array(start.position, dtype=np.float64)
        q_end = np.array(end.position, dtype=np.float64)

        # Compute number of steps needed
        dist = float(np.linalg.norm(q_end - q_start))
        if dist < 1e-8:
            return self.check_config_collision_free(robot_id, start)

        n_steps = max(2, int(np.ceil(dist / step_size)) + 1)

        with self.scratch_context() as ctx:
            for i in range(n_steps):
                t = i / (n_steps - 1)
                q = q_start + t * (q_end - q_start)
                # Create interpolated JointState
                interp_state = JointState(name=start.name, position=q.tolist())
                self.set_joint_state(ctx, robot_id, interp_state)
                if not self.is_collision_free(ctx, robot_id):
                    return False

        return True

    # ============= Forward Kinematics (context-based) =============

    def get_ee_pose(self, ctx: Context, robot_id: WorldRobotID) -> PoseStamped:
        """Get end-effector pose."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)

        ee_body = robot_data.ee_frame.body()
        X_WE = self._plant.EvalBodyPoseInWorld(plant_ctx, ee_body)

        # Extract position and quaternion from Drake transform
        pos = X_WE.translation()
        quat = X_WE.rotation().ToQuaternion()  # Drake returns [w, x, y, z]

        return PoseStamped(
            frame_id="world",
            position=[float(pos[0]), float(pos[1]), float(pos[2])],
            orientation=[float(quat.x()), float(quat.y()), float(quat.z()), float(quat.w())],
        )

    def get_link_pose(
        self, ctx: Context, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        """Get link pose as 4x4 transform."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)

        try:
            body = self._plant.GetBodyByName(link_name, robot_data.model_instance)
        except RuntimeError:
            raise KeyError(f"Link '{link_name}' not found in robot '{robot_id}'")

        X_WL = self._plant.EvalBodyPoseInWorld(plant_ctx, body)

        result: NDArray[np.float64] = X_WL.GetAsMatrix4()
        return result

    def get_jacobian(self, ctx: Context, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """Get geometric Jacobian (6 x n_joints).

        Rows: [vx, vy, vz, wx, wy, wz] (linear, then angular)
        """
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)

        # Compute full Jacobian
        J_full = self._plant.CalcJacobianSpatialVelocity(
            plant_ctx,
            JacobianWrtVariable.kQDot,
            robot_data.ee_frame,
            np.array([0.0, 0.0, 0.0]),  # Point on end-effector
            self._plant.world_frame(),
            self._plant.world_frame(),
        )

        # Extract columns for this robot's joints
        n_joints = len(robot_data.joint_indices)
        J_robot = np.zeros((6, n_joints))

        for i, joint_idx in enumerate(robot_data.joint_indices):
            J_robot[:, i] = J_full[:, joint_idx]

        # Reorder rows: Drake uses [angular, linear], we want [linear, angular]
        J_reordered = np.vstack([J_robot[3:6, :], J_robot[0:3, :]])

        return J_reordered

    # ============= Visualization =============

    def get_visualization_url(self) -> str | None:
        """Get visualization URL if enabled."""
        if self._meshcat is not None:
            url: str = self._meshcat.web_url()
            return url
        return None

    def publish_visualization(self, ctx: Context | None = None) -> None:
        """Publish current state to visualization.

        Args:
            ctx: Context to publish. Uses live context if None.
        """
        if self._meshcat_visualizer is None:
            return

        if ctx is None:
            ctx = self._live_context

        if ctx is not None:
            self._meshcat_visualizer.ForcedPublish(
                self._diagram.GetSubsystemContext(self._meshcat_visualizer, ctx)
            )

    def animate_path(
        self,
        robot_id: WorldRobotID,
        path: JointPath,
        duration: float = 3.0,
    ) -> None:
        """Animate a path in Meshcat visualization.

        Args:
            robot_id: Robot to animate
            path: List of joint states forming the path
            duration: Total animation duration in seconds
        """
        import time

        if self._meshcat is None or len(path) < 2:
            return

        # Capture current states of all OTHER robots so they don't snap to zero
        other_robot_states: dict[WorldRobotID, JointState] = {}
        for rid, _robot_data in self._robots.items():
            if rid != robot_id:
                other_robot_states[rid] = self.get_joint_state(self.get_live_context(), rid)

        dt = duration / (len(path) - 1)
        for joint_state in path:
            with self.scratch_context() as ctx:
                # Restore other robots to their current states
                for rid, state in other_robot_states.items():
                    self.set_joint_state(ctx, rid, state)
                # Set animated robot's state
                self.set_joint_state(ctx, robot_id, joint_state)
                self.publish_visualization(ctx)
            time.sleep(dt)

    # ============= Direct Access (use with caution) =============

    @property
    def plant(self) -> MultibodyPlant:
        """Get underlying MultibodyPlant."""
        return self._plant

    @property
    def scene_graph(self) -> SceneGraph:
        """Get underlying SceneGraph."""
        return self._scene_graph

    @property
    def diagram(self) -> Any:
        """Get underlying Diagram."""
        return self._diagram
