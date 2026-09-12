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

from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, field, replace
from pathlib import Path
from threading import RLock, current_thread
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

import numpy as np

from dimos.manipulation.planning.groups.identifiers import assert_valid_group_id
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.utils import joint_state_to_ordered_positions
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle, PlanningGroupID
from dimos.manipulation.planning.spec.protocols import VisualizationSpec, WorldSpec
from dimos.manipulation.planning.spec.validation import (
    PreparedRobotModel,
    validate_obstacle,
)
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.robot.assets.model import LoadedRobotModel
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.models import (
        VisualizationSession,
        VisualizationStateFrame,
    )

try:
    from pydrake.geometry import (
        AddContactMaterial,
        Box,
        CollisionFilterDeclaration,
        Convex,
        Cylinder,
        GeometryInstance,
        GeometrySet,
        IllustrationProperties,
        MakePhongIllustrationProperties,
        Meshcat,
        MeshcatVisualizer,
        MeshcatVisualizerParams,
        ProximityProperties,
        Rgba,
        Role,
        RoleAssign,
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

    prepared: PreparedRobotModel
    model_instance: Any  # ModelInstanceIndex
    joint_indices: list[int]  # Indices into plant's position vector
    ee_frame: Any  # BodyFrame for end-effector
    base_frame: Any  # BodyFrame for base
    preview_model_instance: Any = None  # ModelInstanceIndex for preview (yellow) robot
    preview_joint_indices: list[int] = field(default_factory=list)

    @property
    def config(self) -> RobotModelConfig:
        return self.prepared.config


@dataclass
class _ObstacleData:
    """Internal data for tracking an obstacle in the world."""

    obstacle_id: str
    obstacle: Obstacle
    geometry_id: Any  # GeometryId
    source_id: Any  # SourceId


class _ThreadSafeMeshcat:
    """Wraps Drake Meshcat so all calls run on the creator thread.

    Drake throws SystemExit from non-creator threads for every Meshcat operation.
    This class creates a single-thread executor, constructs Meshcat on it,
    and proxies all calls through it.
    """

    def __init__(self) -> None:
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="meshcat")
        self._thread = self._executor.submit(current_thread).result()
        self._inner: Meshcat = self._executor.submit(Meshcat).result()

    def _call(self, fn: Any, *args: Any, **kwargs: Any) -> Any:
        if current_thread() is self._thread:
            return fn(*args, **kwargs)
        return self._executor.submit(fn, *args, **kwargs).result()

    def SetObject(self, *args: Any, **kwargs: Any) -> Any:
        return self._call(self._inner.SetObject, *args, **kwargs)

    def SetTransform(self, *args: Any, **kwargs: Any) -> Any:
        return self._call(self._inner.SetTransform, *args, **kwargs)

    def SetProperty(self, *args: Any, **kwargs: Any) -> Any:
        return self._call(self._inner.SetProperty, *args, **kwargs)

    def Delete(self, *args: Any, **kwargs: Any) -> Any:
        return self._call(self._inner.Delete, *args, **kwargs)

    def web_url(self) -> str:
        result: str = self._call(self._inner.web_url)
        return result

    def forced_publish(self, visualizer: Any, viz_ctx: Any) -> None:
        """Run MeshcatVisualizer.ForcedPublish on the creator thread."""
        self._call(visualizer.ForcedPublish, viz_ctx)

    def close(self) -> None:
        self._executor.shutdown(wait=False)


class DrakeWorld(WorldSpec, VisualizationSpec):
    """Drake implementation of WorldSpec and VisualizationSpec."""

    def __init__(self, time_step: float = 0.0, enable_viz: bool = False) -> None:
        if not DRAKE_AVAILABLE:
            raise ImportError("Drake is not installed. Install with: pip install drake")

        self._time_step = time_step
        self._enable_viz = enable_viz
        self._lock = RLock()
        self._usable = True

        # Build Drake diagram
        self._builder = DiagramBuilder()
        self._plant: MultibodyPlant
        self._scene_graph: SceneGraph
        self._plant, self._scene_graph = AddMultibodyPlantSceneGraph(
            self._builder, time_step=time_step
        )
        self._parser = Parser(self._plant)
        # The visualization preview loads a second copy of the configured model.
        # Auto-renaming prevents its internal model name from colliding with the live copy.
        self._parser.SetAutoRenaming(True)

        # Visualization — wrapped to enforce Drake's thread affinity
        self._meshcat: _ThreadSafeMeshcat | None = None
        self._meshcat_visualizer: MeshcatVisualizer | None = None
        if enable_viz:
            self._meshcat = _ThreadSafeMeshcat()

        # Create model instance for obstacles
        self._obstacles_model_instance = self._plant.AddModelInstance("obstacles")

        # Tracking data
        self._model: _RobotData | None = None
        self._obstacles: dict[str, _ObstacleData] = {}
        self._obstacle_counter = 0

        # Built diagram and contexts (created after finalize)
        self._diagram: Any = None
        self._live_context: Context | None = None
        self._plant_context: Context | None = None
        self._scene_graph_context: Context | None = None
        self._finalized = False
        self._preview_animation_generation = 0

        # Obstacle source for dynamic obstacles
        self._obstacle_source_id: Any = None

    def load_model(self, prepared: PreparedRobotModel) -> None:
        """Load the one logical robot model."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")

        with self._lock:
            if self._model is not None:
                raise ValueError("A model is already loaded")
            config = prepared.config
            self._validate_planning_group_config(config)

            model_instance = self._load_model(prepared)
            self._weld_base_if_needed(config, model_instance)

            self._validate_joints(config, model_instance)

            ee_link = config.base_link
            try:
                primary_group_id = self._primary_pose_group_id_for_config(config)
            except ValueError:
                primary_group_id = None
            if primary_group_id is not None:
                primary_group = self._planning_group_from_config(config, primary_group_id)
                if primary_group.tip_link is not None:
                    ee_link = primary_group.tip_link
            ee_frame = self._plant.GetBodyByName(ee_link, model_instance).body_frame()
            base_frame = self._plant.GetBodyByName(config.base_link, model_instance).body_frame()

            # Preview (yellow ghost) — always a separate instance per robot
            preview_model_instance = None
            if self._enable_viz:
                preview_model_instance = self._load_model(prepared)
                self._weld_base_if_needed(config, preview_model_instance)

            self._model = _RobotData(
                prepared=prepared,
                model_instance=model_instance,
                joint_indices=[],
                ee_frame=ee_frame,
                base_frame=base_frame,
                preview_model_instance=preview_model_instance,
            )

    def _load_model(self, prepared: PreparedRobotModel) -> Any:
        """Load the configured in-memory robot model."""
        config = prepared.config
        description = prepare_urdf_for_drake(
            prepared.description,
            convert_meshes=config.auto_convert_meshes,
        )
        description = self._strip_world_base_joint(description, config)
        for package_name, package_path in description.package_paths.items():
            self._parser.package_map().Add(package_name, package_path)
        if not description.package_paths:
            self._parser.package_map().Add("robot_description", description.source_path.parent)
        logger.info("Using in-memory model", model_path=str(description.source_path))
        model_instances = self._parser.AddModelsFromString(description.xml, "urdf")
        if not model_instances:
            raise ValueError(f"Failed to parse model: {description.source_path}")
        return model_instances[0]

    @staticmethod
    def _strip_world_base_joint(
        description: LoadedRobotModel, config: RobotModelConfig
    ) -> LoadedRobotModel:
        root = ET.fromstring(description.xml)
        joints = root.findall("joint")
        joints_to_remove = [
            joint
            for joint in joints
            if joint.get("type") == "fixed"
            and (parent := joint.find("parent")) is not None
            and (child := joint.find("child")) is not None
            and parent.get("link") == "world"
            and child.get("link") == config.base_link
        ]

        if not joints_to_remove:
            return description

        for joint in joints_to_remove:
            root.remove(joint)

        if not any(
            element is not None and element.get("link") == "world"
            for joint in root.findall("joint")
            for element in (joint.find("parent"), joint.find("child"))
        ):
            for link in list(root.findall("link")):
                if link.get("name") == "world":
                    root.remove(link)

        return LoadedRobotModel(
            xml=ET.tostring(root, encoding="unicode"),
            source_path=description.source_path,
            package_paths=description.package_paths,
        )

    def _weld_base_if_needed(self, config: RobotModelConfig, model_instance: Any) -> None:
        """Weld robot base to world if not already welded in URDF."""
        base_body = self._plant.GetBodyByName(config.base_link, model_instance)

        # Check if any joint already connects world to base_link
        for joint_index in self._plant.GetJointIndices(model_instance):
            joint = self._plant.get_joint(joint_index)
            if (
                joint.parent_body().name() == "world"
                and joint.child_body().name() == config.base_link
            ):
                logger.info(
                    f"URDF already has joint '{joint.name()}' welding "
                    f"world→{config.base_link}, skipping weld"
                )
                return

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

    def get_prepared_model(self) -> PreparedRobotModel:
        """Get the immutable prepared robot model."""
        return self._require_model().prepared

    def get_body_frame(self, link_name: str) -> Any:
        """Return a configured model link frame for Drake-native planning backends."""
        robot_data = self._require_model()
        return self._plant.GetBodyByName(link_name, robot_data.model_instance).body_frame()

    def get_model_joint_indices(self) -> list[int]:
        """Return Drake position indices in canonical model-joint order."""
        return list(self._require_model().joint_indices)

    def _require_model(self) -> _RobotData:
        if self._model is None:
            raise RuntimeError("Model is not loaded")
        return self._model

    @staticmethod
    def _validate_planning_group_config(config: RobotModelConfig) -> None:
        seen_group_names: set[str] = set()
        for definition in config.planning_groups:
            assert_valid_group_id(definition.name)
            if definition.name in seen_group_names:
                raise ValueError(f"Planning group '{definition.name}' is already registered")
            seen_group_names.add(definition.name)

    @staticmethod
    def _planning_group_from_config(
        config: RobotModelConfig, group_id: PlanningGroupID
    ) -> PlanningGroup:
        for definition in config.planning_groups:
            if definition.name == group_id:
                return PlanningGroup(
                    group_id,
                    definition.joint_names,
                    definition.base_link,
                    definition.tip_link,
                    definition.source,
                )
        raise KeyError(f"Unknown planning group ID: {group_id}")

    def _planning_group_from_id(self, group_id: PlanningGroupID) -> PlanningGroup:
        return self._planning_group_from_config(self._require_model().config, group_id)

    @staticmethod
    def _primary_pose_group_id_for_config(config: RobotModelConfig) -> PlanningGroupID | None:
        pose_groups = [group for group in config.planning_groups if group.has_pose_target]
        if not pose_groups:
            return None
        if len(pose_groups) > 1:
            raise ValueError("Model has multiple pose groups")
        return pose_groups[0].name

    # Obstacle Management

    def add_obstacle(self, obstacle: Obstacle) -> str | None:
        """Add an obstacle to the world."""
        with self._lock:
            self._require_finalized()
            self._validate_obstacle(obstacle, allow_empty_name=True)
            # Use obstacle's name as ID (allows external ID management)
            obstacle_id = obstacle.name
            if not obstacle_id:
                return None

            # Check for duplicate in our tracking
            if obstacle_id in self._obstacles:
                logger.debug("Obstacle already exists", obstacle_id=obstacle_id)
                return None

            snapshot = deepcopy(obstacle)
            try:
                geometry_id = self._add_obstacle_to_scene_graph(snapshot, obstacle_id)
                self._obstacles[obstacle_id] = _ObstacleData(
                    obstacle_id=obstacle_id,
                    obstacle=snapshot,
                    geometry_id=geometry_id,
                    source_id=self._obstacle_source_id,
                )
                logger.debug(
                    "Added obstacle",
                    obstacle_id=obstacle_id,
                    obstacle_type=obstacle.obstacle_type.value,
                )
            except RuntimeError as e:
                # Handle case where geometry name already exists in SceneGraph
                # (can happen with concurrent access)
                if "already been used" in str(e):
                    logger.debug("Obstacle already in SceneGraph", obstacle_id=obstacle_id)
                    return None
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
            diffuse_color,  # type: ignore[arg-type]
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
        geometry_instance.set_illustration_properties(
            MakePhongIllustrationProperties(rgba_array)  # type: ignore[arg-type]
        )
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
        drake_shape = self._create_shape(obstacle)
        self._meshcat.SetObject(path, drake_shape, rgba)
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
        elif obstacle.obstacle_type == ObstacleType.MESH:
            if not obstacle.mesh_path:
                raise ValueError("MESH obstacle requires mesh_path")
            return Convex(Path(obstacle.mesh_path))
        else:
            raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")

    def _validate_obstacle(self, obstacle: Obstacle, *, allow_empty_name: bool = False) -> None:
        pose_matrix = Transform(
            translation=obstacle.pose.position,
            rotation=obstacle.pose.orientation,
        ).to_matrix()
        validate_obstacle(obstacle, pose_matrix, allow_empty_name=allow_empty_name)

    def _remove_obstacle_geometry(self, obstacle_data: _ObstacleData) -> None:
        self._scene_graph.RemoveGeometry(
            obstacle_data.source_id,
            obstacle_data.geometry_id,
        )
        if self._meshcat is not None:
            self._meshcat.Delete(f"obstacles/{obstacle_data.obstacle_id}")

    def _replace_obstacle_locked(self, obstacle: Obstacle) -> None:
        obstacle_id = obstacle.name
        previous = self._obstacles[obstacle_id]
        try:
            self._remove_obstacle_geometry(previous)
            geometry_id = self._add_obstacle_to_scene_graph(obstacle, obstacle_id)
        except Exception:
            self._usable = False
            raise
        self._obstacles[obstacle_id] = _ObstacleData(
            obstacle_id=obstacle_id,
            obstacle=obstacle,
            geometry_id=geometry_id,
            source_id=self._obstacle_source_id,
        )

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle by ID."""
        with self._lock:
            self._require_finalized()
            if obstacle_id not in self._obstacles:
                return False

            obstacle_data = self._obstacles[obstacle_id]
            self._remove_obstacle_geometry(obstacle_data)
            del self._obstacles[obstacle_id]
            logger.debug("Removed obstacle", obstacle_id=obstacle_id)
            return True

    def update_obstacle(self, obstacle: Obstacle) -> bool:
        """Atomically replace a complete obstacle."""
        with self._lock:
            self._require_finalized()
            self._validate_obstacle(obstacle)
            snapshot = deepcopy(obstacle)
            if snapshot.name not in self._obstacles:
                return False
            self._replace_obstacle_locked(snapshot)
            return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Atomically update only an obstacle pose."""
        replacement_pose = deepcopy(pose)
        with self._lock:
            self._require_finalized()
            if obstacle_id not in self._obstacles:
                return False
            replacement = replace(
                self._obstacles[obstacle_id].obstacle,
                pose=replacement_pose,
            )
            self._validate_obstacle(replacement)
            self._replace_obstacle_locked(replacement)
            return True

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        with self._lock:
            self._require_finalized()
            obstacle_ids = list(self._obstacles.keys())
            for obs_id in obstacle_ids:
                self.remove_obstacle(obs_id)

    def get_obstacles(self) -> list[Obstacle]:
        """Get all obstacles currently in the world."""
        with self._lock:
            self._require_finalized()
            return deepcopy([data.obstacle for data in self._obstacles.values()])

    # Preview Robot Setup

    def _set_preview_colors(self) -> None:
        """Set all preview robot visual geometries to yellow/semi-transparent."""
        source_id: Any = self._plant.get_source_id()
        preview_color = Rgba(1.0, 0.8, 0.0, 0.4)

        robot_data = self._model
        if robot_data is not None:
            if robot_data.preview_model_instance is None:
                return
            for body_idx in self._plant.GetBodyIndices(robot_data.preview_model_instance):
                body = self._plant.get_body(body_idx)
                for geom_id in self._plant.GetVisualGeometriesForBody(body):
                    props = IllustrationProperties()
                    props.AddProperty("phong", "diffuse", preview_color)
                    self._scene_graph.AssignRole(source_id, geom_id, props, RoleAssign.kReplace)  # type: ignore[call-overload]

    def _remove_preview_collision_roles(self) -> None:
        """Remove proximity (collision) role from all preview robot geometries."""
        source_id: Any = self._plant.get_source_id()  # SourceId

        robot_data = self._model
        if robot_data is not None:
            if robot_data.preview_model_instance is None:
                return
            for body_idx in self._plant.GetBodyIndices(robot_data.preview_model_instance):
                body = self._plant.get_body(body_idx)
                for geom_id in self._plant.GetCollisionGeometriesForBody(body):
                    self._scene_graph.RemoveRole(source_id, geom_id, Role.kProximity)

    # Lifecycle

    def finalize(self) -> None:
        """Finalize world - locks robot topology, enables collision checking."""
        if self._finalized:
            logger.warning("World already finalized")
            return

        with self._lock:
            self._require_usable()
            # Finalize plant
            self._plant.Finalize()

            robot_data = self._require_model()
            # Compute joint indices for the model (live + preview)
            if robot_data is not None:
                joint_indices: list[int] = []
                for joint_name in robot_data.config.joint_names:
                    joint = self._plant.GetJointByName(joint_name, robot_data.model_instance)
                    start_idx = joint.position_start()
                    num_positions = joint.num_positions()
                    joint_indices.extend(range(start_idx, start_idx + num_positions))
                robot_data.joint_indices = joint_indices
                logger.debug("Computed model joint indices", joint_indices=joint_indices)

                # Compute preview joint indices
                if robot_data.preview_model_instance is not None:
                    preview_indices: list[int] = []
                    for joint_name in robot_data.config.joint_names:
                        joint = self._plant.GetJointByName(
                            joint_name, robot_data.preview_model_instance
                        )
                        start_idx = joint.position_start()
                        num_positions = joint.num_positions()
                        preview_indices.extend(range(start_idx, start_idx + num_positions))
                    robot_data.preview_joint_indices = preview_indices
                    logger.debug("Computed preview joint indices", joint_indices=preview_indices)

            # Setup collision filters
            self._setup_collision_filters()

            # Remove collision roles from preview robots (visual-only)
            self._remove_preview_collision_roles()

            # Set preview robots to yellow/semi-transparent
            self._set_preview_colors()

            # Register obstacle source for dynamic obstacles
            self._obstacle_source_id = self._scene_graph.RegisterSource("dynamic_obstacles")

            # Add visualization if enabled
            if self._meshcat is not None:
                params = MeshcatVisualizerParams()
                params.role = Role.kIllustration
                self._meshcat_visualizer = MeshcatVisualizer.AddToBuilder(
                    self._builder,
                    self._scene_graph,
                    self._meshcat._inner,
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

            # Set home pose for robots that have one configured
            if robot_data.config.home_joints is not None:
                home = np.array(robot_data.config.home_joints, dtype=np.float64)
                self._set_positions_internal(self._plant_context, home)

            self._finalized = True

            # Initial visualization publish (routed to Meshcat thread)
            if self._meshcat_visualizer is not None:
                self._publish_visualization()
                # Hide all preview robots initially
                self._set_preview_visibility(False)

    @property
    def is_finalized(self) -> bool:
        """Check if world is finalized."""
        return self._finalized

    def _require_usable(self) -> None:
        if not self._usable:
            raise RuntimeError("Planning world is invalid and must be reconstructed")

    def _require_finalized(self) -> None:
        self._require_usable()
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

    def _setup_collision_filters(self) -> None:
        """Filter collisions between adjacent links and user-specified pairs."""
        robot_data = self._require_model()
        if robot_data is not None:
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
                    logger.warning(
                        "Collision exclusion link not found", first_link=name1, second_link=name2
                    )

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

    # Context Management

    def get_live_context(self) -> Context:
        """Get the live context (mirrors current robot state).

        WARNING: Not thread-safe for reads during writes.
        Use scratch_context() for planning operations.
        """
        with self._lock:
            self._require_finalized()
            assert self._live_context is not None
            return self._live_context

    @contextmanager
    def scratch_context(self) -> Generator[Context, None, None]:
        """Thread-safe context for planning. Copies current robot states for inter-robot collision checking."""
        with self._lock:
            self._require_finalized()
            ctx = self._diagram.CreateDefaultContext()

            # Copy live robot states so inter-robot collision checking works
            if self._plant_context is not None:
                plant_ctx = self._diagram.GetMutableSubsystemContext(self._plant, ctx)
                robot_data = self._require_model()
                try:
                    positions = self._plant.GetPositions(
                        self._plant_context, robot_data.model_instance
                    )
                    self._plant.SetPositions(plant_ctx, robot_data.model_instance, positions)
                except RuntimeError:
                    pass  # Model not yet synced

        yield ctx

    def sync_from_joint_state(self, joint_state: JointState) -> None:
        """Sync live context from driver's joint state message.

        Called by StateMonitor when new JointState arrives.
        """
        if not self._finalized or self._plant_context is None:
            return  # Silently ignore before finalization

        positions = self._joint_state_to_q(joint_state)

        with self._lock:
            self._require_usable()
            self._set_positions_internal(self._plant_context, positions)

            # NOTE: ForcedPublish is intentionally NOT called here.
            # Calling ForcedPublish from the LCM callback thread blocks message processing.
            # Visualization can be updated via publish_to_meshcat() from non-callback contexts.

    # State Operations (context-based)

    def set_joint_state(self, ctx: Context, joint_state: JointState) -> None:
        """Set robot joint state in given context."""
        with self._lock:
            self._require_finalized()
            positions = self._joint_state_to_q(joint_state)
            plant_ctx = self._diagram.GetMutableSubsystemContext(self._plant, ctx)
            self._set_positions_internal(plant_ctx, positions)

    def _set_positions_internal(self, plant_ctx: Context, positions: NDArray[np.float64]) -> None:
        """Internal: Set positions in a plant context."""
        robot_data = self._require_model()
        full_positions = self._plant.GetPositions(plant_ctx).copy()

        for i, joint_idx in enumerate(robot_data.joint_indices):
            full_positions[joint_idx] = positions[i]

        self._plant.SetPositions(plant_ctx, full_positions)

    def _joint_state_to_q(self, joint_state: JointState) -> NDArray[np.float64]:
        """Normalize a canonical JointState to model joint order."""
        robot_data = self._require_model()
        return joint_state_to_ordered_positions(
            joint_state,
            joint_names=robot_data.config.joint_names,
        )

    def get_joint_state(self, ctx: Context) -> JointState:
        """Get robot joint state from given context."""
        with self._lock:
            self._require_finalized()
            robot_data = self._require_model()
            plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)
            full_positions = self._plant.GetPositions(plant_ctx)
            positions = [float(full_positions[idx]) for idx in robot_data.joint_indices]
            return JointState(name=robot_data.config.joint_names, position=positions)

    # Collision Checking (context-based)

    def is_collision_free(self, ctx: Context) -> bool:
        """Check if current configuration in context is collision-free."""
        with self._lock:
            self._require_finalized()
            self._require_model()
            scene_graph_ctx = self._diagram.GetSubsystemContext(self._scene_graph, ctx)
            query_object = self._scene_graph.get_query_output_port().Eval(scene_graph_ctx)
            return not query_object.HasCollisions()

    def get_min_distance(self, ctx: Context) -> float:
        """Get minimum signed distance (positive = clearance, negative = penetration)."""
        with self._lock:
            self._require_finalized()
            scene_graph_ctx = self._diagram.GetSubsystemContext(self._scene_graph, ctx)
            query_object = self._scene_graph.get_query_output_port().Eval(scene_graph_ctx)
            signed_distance_pairs = query_object.ComputeSignedDistancePairwiseClosestPoints()
            if not signed_distance_pairs:
                return float("inf")
            return float(min(pair.distance for pair in signed_distance_pairs))

    # Collision Checking (context-free, for planning)

    def check_config_collision_free(self, joint_state: JointState) -> bool:
        """Check if a joint state is collision-free (manages context internally).

        This is a convenience method for planners that don't need to manage contexts.
        """
        with self._lock:
            self._require_finalized()
            with self.scratch_context() as ctx:
                self.set_joint_state(ctx, joint_state)
                return self.is_collision_free(ctx)

    def check_edge_collision_free(
        self,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check if the entire edge between two joint states is collision-free.

        Interpolates between start and end at the given step_size and checks
        each configuration for collisions. This is more efficient than checking
        each configuration separately as it uses a single scratch context.
        """
        with self._lock:
            self._require_finalized()
            q_start = np.array(start.position, dtype=np.float64)
            q_end = np.array(end.position, dtype=np.float64)
            dist = float(np.linalg.norm(q_end - q_start))
            if dist < 1e-8:
                return self.check_config_collision_free(start)
            n_steps = max(2, int(np.ceil(dist / step_size)) + 1)
            with self.scratch_context() as ctx:
                for i in range(n_steps):
                    t = i / (n_steps - 1)
                    q = q_start + t * (q_end - q_start)
                    interp_state = JointState(name=start.name, position=q.tolist())
                    self.set_joint_state(ctx, interp_state)
                    if not self.is_collision_free(ctx):
                        return False
            return True

    # Forward Kinematics (context-based)

    def get_ee_pose(self, ctx: Context) -> PoseStamped:
        """Get end-effector pose."""
        robot_data = self._require_model()
        group_id = self._primary_pose_group_id_for_config(robot_data.config)
        if group_id is None:
            raise ValueError("Model has no pose-targetable planning group")
        return self.get_group_ee_pose(ctx, group_id)

    def get_group_ee_pose(self, ctx: Context, group_id: PlanningGroupID) -> PoseStamped:
        """Get planning-group tip pose."""
        with self._lock:
            self._require_finalized()
            return self._get_group_ee_pose(ctx, group_id)

    def _get_group_ee_pose(self, ctx: Context, group_id: PlanningGroupID) -> PoseStamped:
        group = self._planning_group_from_id(group_id)
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group_id}' has no tip link")
        robot_data = self._require_model()
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)

        ee_body = self._plant.GetBodyByName(group.tip_link, robot_data.model_instance)
        X_WE = self._plant.EvalBodyPoseInWorld(plant_ctx, ee_body)

        # Extract position and quaternion from Drake transform
        pos = X_WE.translation()
        quat = X_WE.rotation().ToQuaternion()  # Drake returns [w, x, y, z]

        return PoseStamped(
            frame_id="world",
            position=[float(pos[0]), float(pos[1]), float(pos[2])],
            orientation=[float(quat.x()), float(quat.y()), float(quat.z()), float(quat.w())],
        )

    def get_link_pose(self, ctx: Context, link_name: str) -> NDArray[np.float64]:
        """Get link pose as 4x4 transform."""
        with self._lock:
            self._require_finalized()
            return self._get_link_pose(ctx, link_name)

    def _get_link_pose(self, ctx: Context, link_name: str) -> NDArray[np.float64]:
        robot_data = self._require_model()
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)

        try:
            body = self._plant.GetBodyByName(link_name, robot_data.model_instance)
        except RuntimeError:
            raise KeyError(f"Link '{link_name}' not found in model")

        X_WL = self._plant.EvalBodyPoseInWorld(plant_ctx, body)

        result = X_WL.GetAsMatrix4()
        return result  # type: ignore[no-any-return]

    def get_jacobian(self, ctx: Context) -> NDArray[np.float64]:
        """Get geometric Jacobian (6 x n_joints).

        Rows: [vx, vy, vz, wx, wy, wz] (linear, then angular)
        """
        robot_data = self._require_model()
        group_id = self._primary_pose_group_id_for_config(robot_data.config)
        if group_id is None:
            raise ValueError("Model has no pose-targetable planning group")
        return self.get_group_jacobian(ctx, group_id)

    def get_group_jacobian(self, ctx: Context, group_id: PlanningGroupID) -> NDArray[np.float64]:
        """Get geometric Jacobian (6 x group joints) in planning-group order."""
        with self._lock:
            self._require_finalized()
            return self._get_group_jacobian(ctx, group_id)

    def _get_group_jacobian(self, ctx: Context, group_id: PlanningGroupID) -> NDArray[np.float64]:
        group = self._planning_group_from_id(group_id)
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group_id}' has no tip link")
        robot_data = self._require_model()
        plant_ctx = self._diagram.GetSubsystemContext(self._plant, ctx)
        tip_frame = self._plant.GetBodyByName(
            group.tip_link, robot_data.model_instance
        ).body_frame()

        # Compute full Jacobian
        J_full = self._plant.CalcJacobianSpatialVelocity(
            plant_ctx,
            JacobianWrtVariable.kQDot,
            tip_frame,
            np.array([0.0, 0.0, 0.0]),  # type: ignore[arg-type]  # Point on end-effector
            self._plant.world_frame(),
            self._plant.world_frame(),
        )

        # Extract columns for configured controllable joints only.
        joint_indices_by_name = dict(
            zip(robot_data.config.joint_names, robot_data.joint_indices, strict=True)
        )
        missing = [
            joint_name
            for joint_name in group.joint_names
            if joint_name not in joint_indices_by_name
        ]
        if missing:
            raise ValueError(
                f"Planning group '{group_id}' references non-controllable joints: {missing}"
            )
        group_joint_indices = [joint_indices_by_name[name] for name in group.joint_names]
        n_joints = len(group_joint_indices)
        J_robot = np.zeros((6, n_joints))

        for i, joint_idx in enumerate(group_joint_indices):
            J_robot[:, i] = J_full[:, joint_idx]

        # Reorder rows: Drake uses [angular, linear], we want [linear, angular]
        J_reordered = np.vstack([J_robot[3:6, :], J_robot[0:3, :]])

        return J_reordered

    # Visualization

    def initialize(self, session: VisualizationSession) -> None:
        """Embedded Meshcat observes the Drake world directly; no extra sync needed."""
        return None

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        """Embedded Meshcat observes native WorldSpec obstacle mutations."""
        return None

    def update_vis_obstacle(self, obstacle: Obstacle) -> None:
        """Embedded Meshcat observes native WorldSpec obstacle replacement."""
        return None

    def update_vis_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> None:
        """Embedded Meshcat observes native WorldSpec obstacle pose updates."""
        return None

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        """Embedded Meshcat observes native WorldSpec obstacle mutations."""
        return None

    def clear_vis_obstacles(self) -> None:
        """Embedded Meshcat observes native WorldSpec obstacle mutations."""
        return None

    def get_visualization_url(self) -> str | None:
        """Get visualization URL if enabled."""
        if self._meshcat is not None:
            return self._meshcat.web_url()
        return None

    def _publish_visualization(self, ctx: Context | None = None) -> None:
        """Publish current state to visualization."""
        with self._lock:
            if self._meshcat_visualizer is None or self._meshcat is None:
                return
            self._require_finalized()
            if ctx is None:
                ctx = self._live_context
            if ctx is not None:
                viz_ctx = self._diagram.GetSubsystemContext(self._meshcat_visualizer, ctx)
                self._meshcat.forced_publish(self._meshcat_visualizer, viz_ctx)

    def update_state(self, frame: VisualizationStateFrame) -> None:
        """Receive pushed state frame; embedded Meshcat uses Drake live context."""
        self._publish_visualization()

    def _set_preview_positions(self, plant_ctx: Context, positions: NDArray[np.float64]) -> None:
        """Set preview model positions in a plant context."""
        robot_data = self._model
        if robot_data is None or robot_data.preview_model_instance is None:
            return

        full_positions = self._plant.GetPositions(plant_ctx).copy()
        for i, idx in enumerate(robot_data.preview_joint_indices):
            full_positions[idx] = positions[i]
        self._plant.SetPositions(plant_ctx, full_positions)

    def _set_preview_visibility(self, visible: bool) -> None:
        """Set preview model Meshcat visibility."""
        if self._meshcat is None:
            return
        robot_data = self._model
        if robot_data is None or robot_data.preview_model_instance is None:
            return
        model_name = self._plant.GetModelInstanceName(robot_data.preview_model_instance)
        self._meshcat.SetProperty(f"visualizer/{model_name}", "visible", visible)

    def cancel_preview_animation(self) -> None:
        """Invalidate active preview frames and hide preview ghosts immediately."""
        with self._lock:
            self._preview_animation_generation += 1
            self._set_preview_visibility(False)

    def _trajectory_indices(self, trajectory: JointTrajectory) -> list[tuple[int, str]]:
        known = set(self._require_model().config.joint_names)
        unknown = [name for name in trajectory.joint_names if name not in known]
        if unknown:
            raise ValueError(f"trajectory references unknown joints: {unknown}")
        return list(enumerate(trajectory.joint_names))

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        """Render a canonical trajectory on its stored shared clock."""
        if self._meshcat is None or len(trajectory.points) < 2:
            return

        import time

        trajectory_indices = self._trajectory_indices(trajectory)
        playback_scale = 1.0
        if duration is not None:
            if duration <= 0.0 or trajectory.duration <= 0.0:
                raise ValueError("preview duration must be positive")
            playback_scale = duration / trajectory.duration
        with self._lock:
            assert self._plant_context is not None
            assert self._live_context is not None
            self._preview_animation_generation += 1
            generation = self._preview_animation_generation
            robot_data = self._require_model()
            self._set_preview_visibility(True)
            baseline = np.array(self.get_joint_state(self._live_context).position, dtype=np.float64)
            joint_positions_by_name = dict(
                zip(
                    robot_data.config.joint_names,
                    range(len(robot_data.config.joint_names)),
                    strict=True,
                )
            )

        try:
            previous_time = trajectory.points[0].time_from_start
            for frame_index, point in enumerate(trajectory.points):
                with self._lock:
                    if self._preview_animation_generation != generation:
                        return
                    assert self._plant_context is not None
                    positions = baseline.copy()
                    for trajectory_index, joint_name in trajectory_indices:
                        positions[joint_positions_by_name[joint_name]] = point.positions[
                            trajectory_index
                        ]
                    self._set_preview_positions(self._plant_context, positions)
                    self._publish_visualization()
                if frame_index < len(trajectory.points) - 1:
                    next_time = trajectory.points[frame_index + 1].time_from_start
                    time.sleep((next_time - previous_time) * playback_scale)
                    previous_time = next_time
        finally:
            with self._lock:
                if self._preview_animation_generation == generation:
                    self._set_preview_visibility(False)

    def close(self) -> None:
        """Shut down the viz thread."""
        self.cancel_preview_animation()
        if self._meshcat is not None:
            self._meshcat.close()

    # Direct Access (use with caution)

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
