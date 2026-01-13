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

"""
Drake World Implementation

Implements WorldSpec using Drake's MultibodyPlant and SceneGraph.

## Architecture

Uses Drake's DiagramBuilder pattern:
```
DiagramBuilder
├── MultibodyPlant (kinematics, dynamics)
├── SceneGraph (collision geometry)
└── MeshcatVisualizer (optional)
```

## Context Management

- Live context: Mirrors current robot state (synced from driver)
- Scratch contexts: Thread-safe clones for planning/IK operations

Example:
    world = DrakeWorld(enable_viz=True)
    robot_id = world.add_robot(config)
    world.add_obstacle(table)
    world.finalize()

    # Sync live state from driver
    world.sync_from_joint_state(robot_id, joint_positions)

    # Planning uses scratch contexts
    with world.scratch_context() as ctx:
        world.set_positions(ctx, robot_id, q_test)
        if world.is_collision_free(ctx, robot_id):
            ee_pose = world.get_ee_pose(ctx, robot_id)
"""

from __future__ import annotations

from contextlib import contextmanager
import copy
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.planning.spec import (
    Obstacle,
    ObstacleType,
    RobotModelConfig,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

try:
    from pydrake.geometry import (
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
        QueryObject,
        Rgba,
        Role,
        SceneGraph,
        Sphere,
    )
    from pydrake.math import RigidTransform
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import (
        AddMultibodyPlantSceneGraph,
        CoulombFriction,
        MultibodyPlant,
    )
    from pydrake.multibody.tree import JacobianWrtVariable
    from pydrake.systems.framework import Context, DiagramBuilder

    DRAKE_AVAILABLE = True
except ImportError:
    DRAKE_AVAILABLE = False

logger = setup_logger()


@dataclass
class _RobotData:
    """Internal data for tracking a robot in the world."""

    robot_id: str
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


class DrakeWorld:
    """Drake implementation of WorldSpec.

    Owns a single Drake Diagram containing:
    - MultibodyPlant (kinematics, dynamics)
    - SceneGraph (collision geometry)
    - Optional MeshcatVisualizer

    ## Context Management

    - _live_context: Mirrors current robot state (synced from driver)
    - scratch_context(): Returns cloned context for planning/IK

    ## Thread Safety

    - Live context writes are protected by RLock
    - scratch_context() returns independent clones
    - All public methods that modify state are thread-safe
    """

    def __init__(
        self,
        time_step: float = 0.0,
        enable_viz: bool = False,
    ):
        """Create a Drake world.

        Args:
            time_step: Simulation time step (0 for kinematics-only)
            enable_viz: If True, enable Meshcat visualization
        """
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

        # Visualization
        self._meshcat: Meshcat | None = None
        self._meshcat_visualizer: MeshcatVisualizer | None = None
        if enable_viz:
            self._meshcat = Meshcat()

        # Create model instance for obstacles
        self._obstacles_model_instance = self._plant.AddModelInstance("obstacles")

        # Tracking data
        self._robots: dict[str, _RobotData] = {}
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

    # ============= Robot Management =============

    def add_robot(self, config: RobotModelConfig) -> str:
        """Add a robot to the world.

        Args:
            config: Robot configuration including URDF path and joint names

        Returns:
            robot_id: Unique identifier for the robot

        Raises:
            RuntimeError: If world is already finalized
        """
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")

        with self._lock:
            # Generate unique robot ID
            self._robot_counter += 1
            robot_id = f"robot_{self._robot_counter}"

            # Prepare URDF: process xacro and convert STL meshes if needed
            original_path = Path(config.urdf_path).resolve()
            if not original_path.exists():
                raise FileNotFoundError(f"URDF/xacro not found: {original_path}")

            urdf_path = prepare_urdf_for_drake(
                urdf_path=original_path,
                package_paths=config.package_paths,
                xacro_args=config.xacro_args,
                # Always convert meshes when requested - Drake needs OBJ for collision
                convert_meshes=config.auto_convert_meshes,
            )

            urdf_path_obj = Path(urdf_path)
            logger.info(f"Using prepared URDF: {urdf_path_obj}")

            # Register package paths for mesh resolution
            if config.package_paths:
                for pkg_name, pkg_path in config.package_paths.items():
                    self._parser.package_map().Add(pkg_name, Path(pkg_path))
            else:
                # Fallback: register URDF directory as package
                package_dir = urdf_path_obj.parent
                self._parser.package_map().Add(
                    package_name=f"{config.name}_description",
                    package_path=package_dir,
                )

            # Parse the URDF
            model_instances = self._parser.AddModels(urdf_path_obj)
            if not model_instances:
                raise ValueError(f"Failed to parse URDF: {urdf_path}")

            model_instance = model_instances[0]

            # Get base body
            base_body = self._plant.GetBodyByName(config.base_link, model_instance)

            # Check if URDF already welded base to world
            base_already_welded = False
            try:
                world_joint = self._plant.GetJointByName("world_joint", model_instance)
                if (
                    world_joint.parent_body().name() == "world"
                    and world_joint.child_body().name() == config.base_link
                ):
                    base_already_welded = True
                    logger.info("URDF has 'world_joint', skipping weld")
            except RuntimeError:
                pass

            # Weld base to world if needed
            if not base_already_welded:
                world_frame = self._plant.world_frame()
                base_transform = RigidTransform(config.base_pose)
                self._plant.WeldFrames(
                    world_frame,
                    base_body.body_frame(),
                    base_transform,
                )

            # Verify joints exist
            for joint_name in config.joint_names:
                try:
                    self._plant.GetJointByName(joint_name, model_instance)
                except RuntimeError:
                    raise ValueError(f"Joint '{joint_name}' not found in URDF")

            # Get end-effector frame
            try:
                ee_body = self._plant.GetBodyByName(config.end_effector_link, model_instance)
                ee_frame = ee_body.body_frame()
            except RuntimeError:
                raise ValueError(
                    f"End-effector link '{config.end_effector_link}' not found in URDF"
                )

            # Store robot data (joint_indices computed after finalize)
            self._robots[robot_id] = _RobotData(
                robot_id=robot_id,
                config=config,
                model_instance=model_instance,
                joint_indices=[],
                ee_frame=ee_frame,
                base_frame=base_body.body_frame(),
            )

            logger.info(f"Added robot '{robot_id}' ({config.name})")
            return robot_id

    def get_robot_ids(self) -> list[str]:
        """Get all robot IDs in the world."""
        return list(self._robots.keys())

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        """Get robot configuration by ID."""
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")
        return self._robots[robot_id].config

    def get_joint_limits(self, robot_id: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
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

        transform = RigidTransform(obstacle.pose)
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
        transform = RigidTransform(obstacle.pose)
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
        transform = RigidTransform(obstacle.pose)
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

    def update_obstacle_pose(self, obstacle_id: str, pose: NDArray[np.float64]) -> bool:
        """Update obstacle pose (4x4 transform)."""
        with self._lock:
            if obstacle_id not in self._obstacles:
                return False

            if pose.shape != (4, 4):
                raise ValueError(f"Pose must be 4x4, got {pose.shape}")

            self._obstacles[obstacle_id].obstacle.pose = pose.copy()

            # Update Meshcat visualization
            if self._meshcat is not None:
                path = f"obstacles/{obstacle_id}"
                transform = RigidTransform(pose)
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
        """Filter out collisions between adjacent links in kinematic chain."""
        scene_graph = self._scene_graph

        for _robot_id, robot_data in self._robots.items():
            model_instance = robot_data.model_instance
            body_indices = self._plant.GetBodyIndices(model_instance)
            bodies = [self._plant.get_body(bi) for bi in body_indices]

            # Build parent-child relationships
            parent_map: dict[str, str] = {}
            for joint_idx in self._plant.GetJointIndices(model_instance):
                joint = self._plant.get_joint(joint_idx)
                parent_body = joint.parent_body()
                child_body = joint.child_body()
                if parent_body.index() != self._plant.world_body().index():
                    parent_map[child_body.name()] = parent_body.name()

            # Compute kinematic distance
            def get_chain_distance(name1: str, name2: str) -> int:
                def path_to_root(name: str) -> list[str]:
                    path = [name]
                    while name in parent_map:
                        name = parent_map[name]
                        path.append(name)
                    return path

                path1 = path_to_root(name1)
                path2 = path_to_root(name2)

                set1 = set(path1)
                for i, node in enumerate(path2):
                    if node in set1:
                        idx1 = path1.index(node)
                        return idx1 + i

                return len(path1) + len(path2)

            # Filter collisions within 2 joints
            filter_distance = 2
            filtered_pairs: set[tuple[str, str]] = set()

            for i, body1 in enumerate(bodies):
                for body2 in bodies[i + 1 :]:
                    name1, name2 = body1.name(), body2.name()
                    dist = get_chain_distance(name1, name2)

                    if dist <= filter_distance:
                        pair = (min(name1, name2), max(name1, name2))
                        if pair not in filtered_pairs:
                            filtered_pairs.add(pair)

                            try:
                                geoms1 = self._plant.GetCollisionGeometriesForBody(body1)
                                geoms2 = self._plant.GetCollisionGeometriesForBody(body2)

                                if geoms1 and geoms2:
                                    scene_graph.collision_filter_manager().Apply(
                                        CollisionFilterDeclaration().ExcludeBetween(
                                            GeometrySet(geoms1), GeometrySet(geoms2)
                                        )
                                    )
                            except Exception as e:
                                logger.warning(f"Could not filter {name1}<->{name2}: {e}")

            # Add user-specified collision exclusions from config
            # Useful for parallel linkage mechanisms (grippers) where non-adjacent
            # links may legitimately overlap due to mimic joints
            exclusion_pairs = robot_data.config.collision_exclusion_pairs
            if exclusion_pairs:
                body_name_map = {body.name(): body for body in bodies}
                for name1, name2 in exclusion_pairs:
                    if name1 in body_name_map and name2 in body_name_map:
                        body1 = body_name_map[name1]
                        body2 = body_name_map[name2]
                        try:
                            geoms1 = self._plant.GetCollisionGeometriesForBody(body1)
                            geoms2 = self._plant.GetCollisionGeometriesForBody(body2)
                            if geoms1 and geoms2:
                                scene_graph.collision_filter_manager().Apply(
                                    CollisionFilterDeclaration().ExcludeBetween(
                                        GeometrySet(geoms1), GeometrySet(geoms2)
                                    )
                                )
                                logger.debug(f"Excluded collision pair: {name1}<->{name2}")
                        except Exception as e:
                            logger.warning(f"Could not filter pair {name1}<->{name2}: {e}")
                    else:
                        missing = [n for n in (name1, name2) if n not in body_name_map]
                        logger.warning(f"Collision exclusion: links not found: {missing}")

        logger.info("Collision filters applied")

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
        """Get a scratch context for planning (thread-safe).

        Uses CreateDefaultContext() instead of Clone() so that dynamically
        added obstacles are visible to collision queries.

        Usage:
            with world.scratch_context() as ctx:
                world.set_positions(ctx, robot_id, q)
                if world.is_collision_free(ctx, robot_id):
                    ...
        """
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        # Create fresh context - this sees all geometries including dynamic obstacles
        # (Clone() doesn't see geometries added after the original context was created)
        ctx = self._diagram.CreateDefaultContext()

        yield ctx
        # Context automatically cleaned up when exiting

    def sync_from_joint_state(self, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Sync live context from driver's joint state.

        Called by StateMonitor when new JointState arrives.
        """
        if not self._finalized or self._plant_context is None:
            return  # Silently ignore before finalization

        with self._lock:
            self._set_positions_internal(self._plant_context, robot_id, positions)

            # NOTE: ForcedPublish is intentionally NOT called here.
            # Calling ForcedPublish from the LCM callback thread blocks message processing.
            # Visualization can be updated via publish_to_meshcat() from non-callback contexts.

    # ============= State Operations (context-based) =============

    def set_positions(self, ctx: Context, robot_id: str, positions: NDArray[np.float64]) -> None:
        """Set robot positions in given context."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        # Get plant context from diagram context
        plant_ctx = self._diagram.GetMutableSubsystemContext(self._plant, ctx)
        self._set_positions_internal(plant_ctx, robot_id, positions)

    def _set_positions_internal(
        self, plant_ctx: Context, robot_id: str, positions: NDArray[np.float64]
    ) -> None:
        """Internal: Set positions in a plant context."""
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        full_positions = self._plant.GetPositions(plant_ctx).copy()

        for i, joint_idx in enumerate(robot_data.joint_indices):
            full_positions[joint_idx] = positions[i]

        self._plant.SetPositions(plant_ctx, full_positions)

    def get_positions(self, ctx: Context, robot_id: str) -> NDArray[np.float64]:
        """Get robot positions from given context."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)
        full_positions = self._plant.GetPositions(plant_ctx)

        return np.array([full_positions[idx] for idx in robot_data.joint_indices])

    # ============= Collision Checking (context-based) =============

    def is_collision_free(self, ctx: Context, robot_id: str) -> bool:
        """Check if current configuration in context is collision-free."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        scene_graph_ctx = self._diagram.GetSubsystemContext(self._scene_graph, ctx)
        query_object = self._scene_graph.get_query_output_port().Eval(scene_graph_ctx)

        return not query_object.HasCollisions()  # type: ignore[attr-defined]

    def get_min_distance(self, ctx: Context, robot_id: str) -> float:
        """Get minimum signed distance (positive = clearance, negative = penetration)."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        scene_graph_ctx = self._diagram.GetSubsystemContext(self._scene_graph, ctx)
        query_object = self._scene_graph.get_query_output_port().Eval(scene_graph_ctx)

        signed_distance_pairs = query_object.ComputeSignedDistancePairwiseClosestPoints()  # type: ignore[attr-defined]

        if not signed_distance_pairs:
            return float("inf")

        return float(min(pair.distance for pair in signed_distance_pairs))

    # ============= Forward Kinematics (context-based) =============

    def get_ee_pose(self, ctx: Context, robot_id: str) -> NDArray[np.float64]:
        """Get end-effector pose as 4x4 transform."""
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")

        robot_data = self._robots[robot_id]
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)

        ee_body = robot_data.ee_frame.body()
        X_WE = self._plant.EvalBodyPoseInWorld(plant_ctx, ee_body)

        return X_WE.GetAsMatrix4()

    def get_link_pose(self, ctx: Context, robot_id: str, link_name: str) -> NDArray[np.float64]:
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

        return X_WL.GetAsMatrix4()

    def get_jacobian(self, ctx: Context, robot_id: str) -> NDArray[np.float64]:
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

    def get_meshcat_url(self) -> str | None:
        """Get Meshcat visualization URL."""
        if self._meshcat is not None:
            return self._meshcat.web_url()
        return None

    def publish_to_meshcat(self, ctx: Context | None = None) -> None:
        """Force publish current state to Meshcat.

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
