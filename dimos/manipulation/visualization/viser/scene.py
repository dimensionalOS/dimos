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

from collections.abc import Callable, Sequence
from copy import deepcopy
from dataclasses import replace
from enum import StrEnum
from io import BytesIO
import math
from threading import RLock
import time
from typing import Any, Protocol, TypeAlias, cast
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
import trimesh
from yourdfpy import URDF  # type: ignore[import-untyped]

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import DEFAULT_OBSTACLE_RGBA, Obstacle
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.visualization.viser.animation import (
    PreviewAnimation,
    scaled_frame_delays,
)
from dimos.manipulation.visualization.viser.runtime import (
    VISER_INSTALL_HINT,
    VISER_URDF_INSTALL_HINT,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import LoadedRobotModel
from dimos.utils.logging_config import setup_logger

try:
    from viser import (
        FrameHandle,
        GridHandle,
        MeshHandle,
        TransformControlsEvent,
        TransformControlsHandle,
        ViserServer,
    )
except ModuleNotFoundError as e:
    if e.name != "viser":
        raise
    raise ModuleNotFoundError(VISER_INSTALL_HINT) from e

try:
    from viser.extras import ViserUrdf
except ModuleNotFoundError as e:
    if e.name not in {"viser", "viser.extras", "yourdfpy"}:
        raise
    raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
except ImportError as e:
    if "ViserUrdf" not in str(e):
        raise
    raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e

logger = setup_logger()


GOAL_ROBOT_FEASIBLE_COLOR = (255, 122, 0)
GOAL_ROBOT_INFEASIBLE_COLOR = (255, 30, 30)
GOAL_ROBOT_FEASIBLE_OPACITY = 0.7
GOAL_ROBOT_INFEASIBLE_OPACITY = 0.75
GOAL_ROBOT_MESH_COLOR = (*GOAL_ROBOT_FEASIBLE_COLOR, GOAL_ROBOT_FEASIBLE_OPACITY)
PREVIEW_ROBOT_COLOR = (80, 180, 255)
PREVIEW_ROBOT_OPACITY = 0.55
PREVIEW_ROBOT_MESH_COLOR = (*PREVIEW_ROBOT_COLOR, PREVIEW_ROBOT_OPACITY)
TARGET_CONTROL_FEASIBLE_COLOR = (0, 180, 255)
TARGET_CONTROL_INFEASIBLE_COLOR = (255, 40, 40)
REFERENCE_GRID_NAME = "/reference_grid"
REFERENCE_GRID_CELL_COLOR = (44, 54, 58)
REFERENCE_GRID_SECTION_COLOR = (90, 145, 165)
COLLISION_MESH_COLOR = (210, 40, 220)
COLLISION_MESH_OPACITY = 0.35
OBSTACLE_NAMESPACE = "/manipulation/obstacles"
OBSTACLE_DEFAULT_RGBA = DEFAULT_OBSTACLE_RGBA
OBSTACLE_FALLBACK_COLOR = (55, 190, 210)
OBSTACLE_FALLBACK_OPACITY = 0.55
OBSTACLE_PROXY_COLOR = (255, 45, 25)
GRASP_PROPOSAL_NAMESPACE = "/manipulation/grasp-proposals"
# Best rank green through worst orange, so the ordering reads at a glance.
GRASP_PROPOSAL_BEST_COLOR = (60, 220, 60)
GRASP_PROPOSAL_WORST_COLOR = (255, 160, 60)
# Drawing every proposal of a hundred buries the ranking; the leaders are what
# matters when you are deciding whether the generator is pointing anywhere sane.
GRASP_PROPOSAL_DRAW_LIMIT = 20
# The leaders are drawn thicker and labelled with their score.
GRASP_PROPOSAL_EMPHASIS_COUNT = 3
# Glyph geometry, in metres. Deliberately not the gripper: this marks where a
# grasp points, and a jaw-shaped drawing sized by anything but the configured
# sweep volume would be a claim about the hardware that nothing checks.
GRASP_PROPOSAL_APPROACH_LENGTH = 0.05
GRASP_PROPOSAL_CLOSING_HALF_WIDTH = 0.02
GRASP_PROPOSAL_WIDTH = 1.5
GRASP_PROPOSAL_EMPHASIS_WIDTH = 3.5


class RobotDisplayMode(StrEnum):
    VISUAL = "visual"
    COLLISION = "collision"
    BOTH = "both"


SceneHandle: TypeAlias = ViserUrdf | TransformControlsHandle | GridHandle | MeshHandle | FrameHandle


class _ColorHandle(Protocol):
    color: tuple[int, int, int]


class ViserManipulationScene:
    """Viser scene graph helpers for current robot, ghost robot, and path rendering."""

    def __init__(
        self, server: ViserServer, viser_urdf: type[ViserUrdf], preview_fps: float | None = None
    ) -> None:
        self.server = server
        self.viser_urdf = viser_urdf
        self._configs_by_id: dict[str, RobotModelConfig] = {}
        self._models_by_id: dict[str, URDF] = {}
        self._urdfs: dict[str, ViserUrdf] = {}
        self._joint_names_by_urdf: dict[int, tuple[str, ...]] = {}
        self._handles: dict[str, TransformControlsHandle] = {}
        self._root_frames: dict[str, FrameHandle] = {}
        self._grid_handle: GridHandle | None = None
        self._grid_visible = True
        self._preview_visible: dict[str, bool] = {}
        self._target_active: dict[str, bool] = {}
        self._target_tracks_current: dict[str, bool] = {}
        self._scene_lock = RLock()
        self._animation_generation = 0
        self._animation_generations: dict[str, int] = {}
        self._collision_fallback_urdfs: dict[str, ViserUrdf] = {}
        self._robot_display_mode = RobotDisplayMode.VISUAL
        self._obstacle_handles: dict[str, list[Any]] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._obstacle_render_failures: dict[str, Exception] = {}
        self._obstacles_visible = True
        self._obstacle_gui_handles: list[object] = []
        self._obstacle_warning_handle: Any | None = None
        self._grasp_proposal_handles: list[Any] = []
        self._grasp_proposals_visible = True
        self._closed = False
        self._ensure_obstacle_control()
        self._ensure_reference_grid()

    def set_obstacles_visible(self, visible: bool) -> None:
        """Toggle obstacle entities without discarding their scene handles."""
        with self._scene_lock:
            if self._closed:
                return
            self._obstacles_visible = bool(visible)
            for handles in self._obstacle_handles.values():
                for handle in handles:
                    self._set_handle_visibility(handle, self._obstacles_visible)

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        """Render one accepted planner obstacle under the local obstacle namespace."""
        with self._scene_lock:
            if self._closed:
                return
            self.remove_vis_obstacle(obstacle_id)
            self._obstacle_render_failures.pop(obstacle_id, None)
            position, wxyz = self._obstacle_pose(obstacle)
            color, opacity = self._obstacle_appearance(obstacle)
            path = f"{OBSTACLE_NAMESPACE}/{obstacle_id}"
            scene = self.server.scene
            handles: list[Any] = []
            try:
                if obstacle.obstacle_type == ObstacleType.BOX:
                    dimensions = tuple(float(value) for value in obstacle.dimensions[:3])
                    if len(dimensions) != 3:
                        raise ValueError("box dimensions must contain width, height, and depth")
                    handles.append(
                        scene.add_box(
                            path,
                            dimensions=dimensions,
                            color=color,
                            opacity=opacity,
                            position=position,
                            wxyz=wxyz,
                            visible=self._obstacles_visible,
                        )
                    )
                elif obstacle.obstacle_type == ObstacleType.SPHERE:
                    handles.append(
                        scene.add_icosphere(
                            path,
                            radius=float(obstacle.dimensions[0]),
                            color=color,
                            opacity=opacity,
                            position=position,
                            wxyz=wxyz,
                            visible=self._obstacles_visible,
                        )
                    )
                elif obstacle.obstacle_type == ObstacleType.CYLINDER:
                    handles.append(
                        scene.add_cylinder(
                            path,
                            radius=float(obstacle.dimensions[0]),
                            height=float(obstacle.dimensions[1]),
                            color=color,
                            opacity=opacity,
                            position=position,
                            wxyz=wxyz,
                            visible=self._obstacles_visible,
                        )
                    )
                elif obstacle.obstacle_type == ObstacleType.MESH:
                    handles.append(
                        self._add_mesh(
                            scene,
                            path,
                            obstacle,
                            color,
                            opacity,
                            position,
                            wxyz,
                            self._obstacles_visible,
                        )
                    )
                elif obstacle.obstacle_type == ObstacleType.OCTREE:
                    handles.append(
                        self._add_octree(
                            scene, path, obstacle, color, position, wxyz, self._obstacles_visible
                        )
                    )
                else:
                    raise ValueError(f"unsupported obstacle type: {obstacle.obstacle_type}")
            except Exception as error:
                self._obstacle_render_failures[obstacle_id] = error
                logger.warning(
                    "Could not render obstacle %s; using proxy", obstacle_id, exc_info=True
                )
                handles.extend(
                    (
                        scene.add_box(
                            f"{path}/mesh-failure-proxy",
                            dimensions=(0.25, 0.25, 0.25),
                            color=OBSTACLE_PROXY_COLOR,
                            opacity=0.9,
                            position=position,
                            wxyz=wxyz,
                            visible=self._obstacles_visible,
                        ),
                        scene.add_label(
                            f"{path}/mesh-failure-label",
                            f"MESH RENDER FAILED: {error}",
                            position=position,
                            visible=self._obstacles_visible,
                        ),
                    )
                )
            self._obstacle_handles[obstacle_id] = handles
            self._obstacles[obstacle_id] = replace(deepcopy(obstacle), name=obstacle_id)

    def update_vis_obstacle(self, obstacle: Obstacle) -> None:
        """Replace an obstacle representation while holding the renderer lock."""
        with self._scene_lock:
            self.add_vis_obstacle(obstacle.name, obstacle)
            failure = self._obstacle_render_failures.get(obstacle.name)
            if failure is not None:
                raise RuntimeError(
                    f"renderer used a proxy for obstacle '{obstacle.name}'"
                ) from failure

    def update_vis_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> None:
        """Move one obstacle while preserving its stored render properties."""
        with self._scene_lock:
            obstacle = self._obstacles.get(obstacle_id)
            if obstacle is None:
                return
            self.update_vis_obstacle(replace(obstacle, pose=deepcopy(pose)))

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        """Remove every scene entity belonging to an obstacle ID."""
        with self._scene_lock:
            for handle in self._obstacle_handles.pop(obstacle_id, []):
                self._remove_scene_handle(handle)
            self._obstacles.pop(obstacle_id, None)
            self._obstacle_render_failures.pop(obstacle_id, None)

    def clear_vis_obstacles(self) -> None:
        """Remove every obstacle entity while retaining the robot scene."""
        with self._scene_lock:
            for obstacle_id in list(self._obstacle_handles):
                self.remove_vis_obstacle(obstacle_id)

    def set_grasp_proposals_visible(self, visible: bool) -> None:
        """Toggle the proposal markers without discarding their scene handles."""
        with self._scene_lock:
            if self._closed:
                return
            self._grasp_proposals_visible = bool(visible)
            for handle in self._grasp_proposal_handles:
                self._set_handle_visibility(handle, self._grasp_proposals_visible)

    def show_grasp_proposals(self, candidates: GraspCandidateArray) -> None:
        """Draw the ranked proposals as pose glyphs, best first.

        One line-segment node carries every glyph, plus a label per emphasised
        leader. Scores are only meaningful against each other, so rank drives
        the colour rather than the absolute value. An empty array clears.
        """
        with self._scene_lock:
            if self._closed:
                return
            self.clear_grasp_proposals()
            drawn = list(candidates.candidates[:GRASP_PROPOSAL_DRAW_LIMIT])
            if not drawn:
                return
            emphasis = min(GRASP_PROPOSAL_EMPHASIS_COUNT, len(drawn))
            handles: list[Any] = []
            for start, stop, width in (
                (emphasis, len(drawn), GRASP_PROPOSAL_WIDTH),
                (0, emphasis, GRASP_PROPOSAL_EMPHASIS_WIDTH),
            ):
                if start >= stop:
                    continue
                segments, colors = self._grasp_proposal_segments(drawn, start, stop, len(drawn))
                handles.append(
                    self.server.scene.add_line_segments(
                        f"{GRASP_PROPOSAL_NAMESPACE}/rank-{start}",
                        points=segments,
                        colors=colors,
                        line_width=width,
                        visible=self._grasp_proposals_visible,
                    )
                )
            for rank in range(emphasis):
                pose = drawn[rank].pose
                handles.append(
                    self.server.scene.add_label(
                        f"{GRASP_PROPOSAL_NAMESPACE}/label-{rank}",
                        f"#{rank} {drawn[rank].score:.3f}",
                        position=(
                            float(pose.position.x),
                            float(pose.position.y),
                            float(pose.position.z),
                        ),
                        visible=self._grasp_proposals_visible,
                    )
                )
            self._grasp_proposal_handles = handles

    def clear_grasp_proposals(self) -> None:
        """Remove every proposal marker while retaining the viewer's toggle."""
        with self._scene_lock:
            for handle in self._grasp_proposal_handles:
                self._remove_scene_handle(handle)
            self._grasp_proposal_handles = []

    @staticmethod
    def _grasp_proposal_segments(
        candidates: Sequence[GraspCandidate], start: int, stop: int, total: int
    ) -> tuple[NDArray[np.float32], NDArray[np.uint8]]:
        """Two segments per grasp: the approach axis, and the closing axis across it.

        Both arrays are shaped the way ``add_line_segments`` requires: (N, 2, 3)
        for the endpoints, and one colour per endpoint rather than per segment.
        """
        segments: list[NDArray[np.float32]] = []
        colors: list[tuple[tuple[int, int, int], tuple[int, int, int]]] = []
        local = np.array(
            [
                [0.0, 0.0, -GRASP_PROPOSAL_APPROACH_LENGTH],
                [0.0, 0.0, 0.0],
                [0.0, -GRASP_PROPOSAL_CLOSING_HALF_WIDTH, 0.0],
                [0.0, GRASP_PROPOSAL_CLOSING_HALF_WIDTH, 0.0],
            ],
            dtype=np.float32,
        )
        for rank in range(start, stop):
            pose = candidates[rank].pose
            rotation = np.asarray(pose.orientation.to_rotation_matrix(), dtype=np.float32)
            origin = np.asarray(
                [pose.position.x, pose.position.y, pose.position.z], dtype=np.float32
            )
            points = local @ rotation.T + origin
            segments.append(points.reshape(2, 2, 3))
            color = ViserManipulationScene._grasp_rank_color(rank, total)
            colors.extend([(color, color)] * 2)
        return (
            np.concatenate(segments).astype(np.float32),
            np.asarray(colors, dtype=np.uint8),
        )

    @staticmethod
    def _grasp_rank_color(rank: int, total: int) -> tuple[int, int, int]:
        t = 0.0 if total <= 1 else rank / (total - 1)
        best, worst = GRASP_PROPOSAL_BEST_COLOR, GRASP_PROPOSAL_WORST_COLOR
        return tuple(round(b + (w - b) * t) for b, w in zip(best, worst, strict=True))  # type: ignore[return-value]

    def show_obstacle_warning(self, message: str) -> None:
        """Expose a persistent renderer warning in the frontend when available."""
        with self._scene_lock:
            try:
                if self._obstacle_warning_handle is None:
                    self._obstacle_warning_handle = self.server.gui.add_markdown(message)
                    self._obstacle_gui_handles.append(self._obstacle_warning_handle)
                else:
                    self._obstacle_warning_handle.content = message
            except Exception:
                logger.warning("Could not display obstacle warning in Viser", exc_info=True)

    def _ensure_obstacle_control(self) -> None:
        try:
            folder = self.server.gui.add_folder("Scene", expand_by_default=True)
            self._obstacle_gui_handles.append(folder)
            with folder:
                handle = self.server.gui.add_checkbox("manipulation.obstacles", initial_value=True)
                proposals = self.server.gui.add_checkbox(
                    "manipulation.grasp-proposals", initial_value=True
                )
            handle.on_update(lambda event: self.set_obstacles_visible(event.target.value))
            proposals.on_update(lambda event: self.set_grasp_proposals_visible(event.target.value))
            self._obstacle_gui_handles.extend((handle, proposals))
        except (AttributeError, TypeError):
            self._obstacle_gui_handles.clear()

    @staticmethod
    def _obstacle_pose(
        obstacle: Obstacle,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        pose = obstacle.pose
        return (
            (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
            (
                float(pose.orientation.w),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
            ),
        )

    @staticmethod
    def _obstacle_appearance(obstacle: Obstacle) -> tuple[tuple[int, int, int], float]:
        color = obstacle.color
        if (
            color == OBSTACLE_DEFAULT_RGBA
            or len(color) != 4
            or not all(math.isfinite(float(v)) for v in color)
            or not all(0.0 <= float(v) <= 1.0 for v in color)
        ):
            return OBSTACLE_FALLBACK_COLOR, OBSTACLE_FALLBACK_OPACITY
        return (
            round(float(color[0]) * 255),
            round(float(color[1]) * 255),
            round(float(color[2]) * 255),
        ), float(color[3])

    @staticmethod
    def _add_octree(
        scene: Any,
        path: str,
        obstacle: Obstacle,
        color: tuple[int, int, int],
        position: Any,
        wxyz: Any,
        visible: bool,
    ) -> Any:
        """Draw the occupied cells as a point cloud sized to the cell edge.

        A mapped workspace is tens of thousands of cells, so one box per cell
        would stall the browser. Square points at the cell edge read as the same
        grid and cost one scene node.
        """
        points = np.asarray(obstacle.points, dtype=np.float32).reshape(-1, 3)
        if not len(points):
            raise ValueError("octree obstacle carries no occupied cells")
        colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
        return scene.add_point_cloud(
            path,
            points=points,
            colors=colors,
            point_size=float(obstacle.octree_resolution or 0.05),
            point_shape="square",
            position=position,
            wxyz=wxyz,
            visible=visible,
        )

    @staticmethod
    def _add_mesh(
        scene: Any,
        path: str,
        obstacle: Obstacle,
        color: tuple[int, int, int],
        opacity: float,
        position: tuple[float, float, float],
        wxyz: tuple[float, float, float, float],
        visible: bool,
    ) -> MeshHandle:
        if not obstacle.mesh_path:
            raise ValueError("mesh path is missing")
        mesh = trimesh.load_mesh(obstacle.mesh_path, process=False)
        if hasattr(mesh, "dump") and not hasattr(mesh, "vertices"):
            mesh = mesh.dump(concatenate=True)
        vertices = np.asarray(mesh.vertices, dtype=np.float32)
        faces = np.asarray(mesh.faces, dtype=np.int32)
        if len(vertices) == 0 or len(faces) == 0:
            raise ValueError("mesh contains no renderable triangles")
        return cast(
            "MeshHandle",
            scene.add_mesh_simple(
                path,
                vertices,
                faces,
                color=color,
                opacity=opacity,
                position=position,
                wxyz=wxyz,
                visible=visible,
            ),
        )

    @property
    def robot_display_mode(self) -> RobotDisplayMode:
        """Return the primary robot display mode for this scene session."""
        return self._robot_display_mode

    @robot_display_mode.setter
    def robot_display_mode(self, mode: RobotDisplayMode | str) -> None:
        """Set the primary robot display mode and apply it immediately."""
        try:
            normalized_mode = RobotDisplayMode(mode)
        except ValueError as error:
            raise ValueError(f"Unsupported robot display mode: {mode!r}") from error
        self._robot_display_mode = normalized_mode
        for model_key in self._configs_by_id:
            self._apply_robot_display_mode(model_key)

    @property
    def collision_geometry_available(self) -> bool:
        """Return whether any primary robot has loaded collision geometry."""
        return any(
            self._model_has_collision_geometry(self._models_by_id[model_key])
            for model_key in self._configs_by_id
            if f"{model_key}:current" in self._urdfs
        )

    @staticmethod
    def _model_has_collision_geometry(model: URDF) -> bool:
        collision_scene = model.collision_scene
        return collision_scene is not None and bool(getattr(collision_scene, "geometry", True))

    def _load_robot_model(self, config: RobotModelConfig) -> URDF:
        description = self.loaded_robot_description(config)
        return URDF.load(
            BytesIO(description.xml.encode()),
            mesh_dir=str(description.source_path.parent),
            build_scene_graph=True,
            build_collision_scene_graph=True,
            load_meshes=True,
            load_collision_meshes=True,
        )

    def has_reference_grid(self) -> bool:
        """Return whether the Viser scene accepted the optional reference grid."""
        return self._grid_handle is not None

    def set_reference_grid_visible(self, visible: bool) -> None:
        """Show or hide the optional ground reference grid."""
        self._grid_visible = visible
        self._set_handle_visibility(self._grid_handle, visible)

    def _register_model_key(self, model_key: str, config: RobotModelConfig) -> None:
        self._configs_by_id[model_key] = config
        self._preview_visible.setdefault(model_key, False)
        self._animation_generations.setdefault(model_key, 0)
        self._target_active.setdefault(model_key, False)
        self._target_tracks_current.setdefault(model_key, True)
        if model_key not in self._models_by_id:
            self._models_by_id[model_key] = self._load_robot_model(config)
        self._ensure_robot_urdfs(model_key, config)

    def register_model(self, config: RobotModelConfig) -> None:
        """Register the one configured model."""
        self._register_model_key("model", config)

    def set_target_active(self, model_key: str, active: bool) -> None:
        """Show the target ghost only while a pose-target group is selected."""
        self._target_active[model_key] = active
        if not active:
            self._target_tracks_current[model_key] = True
        self._set_target_visibility(model_key, active)

    def _ensure_reference_grid(self) -> None:
        try:
            scene = self.server.scene
        except AttributeError:
            return
        try:
            self._grid_handle = scene.add_grid(
                REFERENCE_GRID_NAME,
                width=20.0,
                height=20.0,
                plane="xy",
                cell_color=REFERENCE_GRID_CELL_COLOR,
                cell_thickness=0.6,
                cell_size=0.25,
                section_color=REFERENCE_GRID_SECTION_COLOR,
                section_thickness=1.0,
                section_size=1.0,
                infinite_grid=True,
                fade_distance=40.0,
                fade_strength=1.0,
                fade_from="camera",
                shadow_opacity=0.0,
                plane_opacity=0.0,
                visible=self._grid_visible,
            )
        except Exception:
            logger.warning("Could not add Viser reference grid", exc_info=True)
            self._grid_handle = None

    def ensure_target_controls(
        self, model_key: str, on_update: Callable[[TransformControlsHandle], None]
    ) -> TransformControlsHandle | None:
        handle_key = f"{model_key}:ee_control"
        if handle_key in self._handles:
            return self._handles[handle_key]
        handle = self.server.scene.add_transform_controls(
            f"/targets/{model_key}/ee_control", scale=0.25
        )

        def dispatch(event: TransformControlsEvent) -> None:
            on_update(event.target)

        handle.on_update(dispatch)
        self._handles[handle_key] = handle
        return handle

    def remove_target_controls(self, control_id: str) -> None:
        self._remove_handle(f"{control_id}:ee_control")

    def _update_current_model_key(self, model_key: str, joint_state: JointState | None) -> None:
        with self._scene_lock:
            config = self._configs_by_id.get(model_key)
            if config is None or joint_state is None:
                return
            self._ensure_robot_urdfs(model_key, config)
            current = self._urdfs.get(f"{model_key}:current")
            self.set_urdf_joints(current, config.joint_names, joint_state.position)
            if self._target_tracks_current.get(model_key, True):
                self._set_target_joints(model_key, config.joint_names, joint_state.position)
                self._set_target_visibility(model_key, self._target_active.get(model_key, False))
        self.set_urdf_joints(
            self._collision_fallback_urdfs.get(model_key),
            config.joint_names,
            joint_state.position,
        )

    def update_current_model(self, joint_state: JointState | None) -> None:
        """Update the one configured model's canonical joint state."""
        self._update_current_model_key("model", joint_state)

    def cancel_preview_animation(self) -> None:
        """Prevent an old blocking animation from touching replacement handles."""
        with self._scene_lock:
            self._animation_generation += 1
            for model_key in set(self._preview_visible):
                self._animation_generations[model_key] = (
                    self._animation_generations.get(model_key, 0) + 1
                )
                if model_key not in self._preview_visible:
                    continue
                self._preview_visible[model_key] = False
                self._set_preview_visibility(model_key, False)

    def animate_preview(self, preview: PreviewAnimation, duration: float) -> bool:
        """Play the model preview from one normalized tick clock.

        Inputs are fully validated before ghosts become visible; a generation
        replacement, clear, or close stops mutation before the next tick.
        """
        model_key = "model"
        if not preview.frames or model_key not in self._configs_by_id:
            return False
        with self._scene_lock:
            self._animation_generation += 1
            generation = self._animation_generations.get(model_key, 0) + 1
            self._animation_generations[model_key] = generation
            self._preview_visible[model_key] = True
            self._set_preview_visibility(model_key, True)
        try:
            delays = scaled_frame_delays(preview.frames, duration)
            for index, frame in enumerate(preview.frames):
                with self._scene_lock:
                    if self._animation_generations.get(model_key) != generation:
                        return False
                    self._set_preview_ghost_joints(model_key, preview.joint_names, frame.positions)
                if index < len(delays):
                    time.sleep(delays[index])
            return True
        finally:
            with self._scene_lock:
                if self._animation_generations.get(model_key) == generation:
                    self._preview_visible[model_key] = False
                    self._set_preview_visibility(model_key, False)

    def set_target_joints(
        self, model_key: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> bool:
        target = self._urdfs.get(f"{model_key}:target")
        if target is None:
            return False
        self._target_tracks_current[model_key] = False
        self._set_target_joints(model_key, joint_names, joints)
        self._set_target_visibility(model_key, True)
        return True

    def clear_target(self, model_key: str) -> None:
        """Return the persistent target ghost to current-state tracking."""
        self._target_tracks_current[model_key] = True

    def _set_target_joints(
        self, model_key: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        target = self._urdfs.get(f"{model_key}:target")
        self.set_urdf_joints(target, joint_names, joints)

    def _set_preview_ghost_joints(
        self, model_key: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        ghost = self._urdfs.get(f"{model_key}:preview")
        self.set_urdf_joints(ghost, joint_names, joints)

    def set_target_pose(self, model_key: str, pose: Pose | None) -> None:
        handle = self._handles.get(f"{model_key}:ee_control")
        if handle is None or pose is None:
            return
        handle.position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        handle.wxyz = (
            float(pose.orientation.w),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
        )

    def set_target_visual_state(self, model_key: str, feasible: bool) -> None:
        """Set the matching model and control target visual state."""
        self.set_target_control_visual_state(model_key, feasible)
        self.set_target_robot_visual_state(model_key, feasible)

    def set_target_control_visual_state(self, control_id: str, feasible: bool) -> None:
        """Set feasibility color for one planning-group keyed target control."""
        color = TARGET_CONTROL_FEASIBLE_COLOR if feasible else TARGET_CONTROL_INFEASIBLE_COLOR
        handle = self._handles.get(f"{control_id}:ee_control")
        if handle is not None:
            cast("_ColorHandle", handle).color = color

    def set_target_robot_visual_state(self, model_key: str, feasible: bool) -> None:
        """Set feasibility material for one robot-ID keyed target ghost."""
        mesh_color = GOAL_ROBOT_FEASIBLE_COLOR if feasible else GOAL_ROBOT_INFEASIBLE_COLOR
        mesh_opacity = GOAL_ROBOT_FEASIBLE_OPACITY if feasible else GOAL_ROBOT_INFEASIBLE_OPACITY
        target = self._urdfs.get(f"{model_key}:target")
        self._set_urdf_mesh_material(target, mesh_color, mesh_opacity)

    def close(self) -> None:
        with self._scene_lock:
            if self._closed:
                return
            self._closed = True
            self.cancel_preview_animation()
            for handles in self._obstacle_handles.values():
                for handle in handles:
                    self._remove_scene_handle(handle)
            self._obstacle_handles.clear()
            self._obstacles.clear()
            self._obstacle_render_failures.clear()
            for key in list(self._handles):
                self._remove_handle(key)
            if self._grid_handle is not None:
                self._remove_scene_handle(self._grid_handle)
                self._grid_handle = None
            for urdf in self._urdfs.values():
                self._remove_scene_handle(urdf)
            for frame in self._root_frames.values():
                self._remove_scene_handle(frame)
            for urdf in self._collision_fallback_urdfs.values():
                self._remove_scene_handle(urdf)
            for handle in self._obstacle_gui_handles:
                self._remove_scene_handle(handle)
            self._obstacle_gui_handles.clear()
            self._urdfs.clear()
            self._root_frames.clear()
            self._models_by_id.clear()
            self._joint_names_by_urdf.clear()
            self._collision_fallback_urdfs.clear()
            self._configs_by_id.clear()
            self._preview_visible.clear()
            self._target_active.clear()
            self._target_tracks_current.clear()
            self._robot_display_mode = RobotDisplayMode.VISUAL

    def _ensure_robot_urdfs(self, model_key: str, config: RobotModelConfig) -> None:
        model = self._models_by_id.get(model_key)
        if model is None:
            return
        for kind in ("current", "target", "preview"):
            key = f"{model_key}:{kind}"
            if key in self._urdfs:
                continue
            root_node_name = self._urdf_root_node_name(model_key, kind, config)
            mesh_color_override = {
                "current": None,
                "target": GOAL_ROBOT_MESH_COLOR,
                "preview": PREVIEW_ROBOT_MESH_COLOR,
            }[kind]
            if kind == "current":
                # Keep both representations resident so changing the diagnostic
                # view does not reload or replace the primary robot.
                old_fallback = self._collision_fallback_urdfs.pop(model_key, None)
                if old_fallback is not None:
                    self._remove_scene_handle(old_fallback)
                urdf = self.viser_urdf(
                    self.server,
                    urdf_or_path=model,
                    root_node_name=root_node_name,
                    mesh_color_override=mesh_color_override,
                    load_meshes=True,
                    load_collision_meshes=True,
                    collision_mesh_color_override=(
                        *COLLISION_MESH_COLOR,
                        COLLISION_MESH_OPACITY,
                    ),
                )
            else:
                urdf = self.viser_urdf(
                    self.server,
                    urdf_or_path=model,
                    root_node_name=root_node_name,
                    mesh_color_override=mesh_color_override,
                )
            self._urdfs[key] = urdf
            self._joint_names_by_urdf[id(urdf)] = tuple(
                str(name) for name in model.actuated_joint_names
            )
            if kind == "current":
                if not self._model_has_collision_geometry(model):
                    fallback = self.viser_urdf(
                        self.server,
                        urdf_or_path=model,
                        root_node_name=f"/robots/{model_key}/collision_fallback",
                        mesh_color_override=(
                            *COLLISION_MESH_COLOR,
                            COLLISION_MESH_OPACITY,
                        ),
                        load_meshes=True,
                        load_collision_meshes=False,
                    )
                    self._collision_fallback_urdfs[model_key] = fallback
                    self._joint_names_by_urdf[id(fallback)] = tuple(
                        str(name) for name in model.actuated_joint_names
                    )
                    self._set_urdf_mesh_material(
                        fallback, COLLISION_MESH_COLOR, COLLISION_MESH_OPACITY
                    )
                self._apply_robot_display_mode(model_key)
            if kind == "target":
                self._set_urdf_mesh_material(
                    self._urdfs[key], GOAL_ROBOT_FEASIBLE_COLOR, GOAL_ROBOT_FEASIBLE_OPACITY
                )
                self._set_handle_visibility(
                    self._urdfs[key], self._target_active.get(model_key, False)
                )
            elif kind == "preview":
                self._set_urdf_mesh_material(
                    self._urdfs[key], PREVIEW_ROBOT_COLOR, PREVIEW_ROBOT_OPACITY
                )
                self._set_handle_visibility(
                    self._urdfs[key], self._preview_visible.get(model_key, False)
                )

    def _apply_robot_display_mode(self, model_key: str) -> None:
        current = self._urdfs.get(f"{model_key}:current")
        if current is None:
            return
        model = self._models_by_id.get(model_key)
        if model is None:
            return
        has_collision = self._model_has_collision_geometry(model)
        mode = self._robot_display_mode
        # Viser's public flags manage all links, including links whose mesh
        # handles are not exposed by the helper.  A model without collision
        # geometry falls back to its visual representation.
        current.show_visual = mode in {RobotDisplayMode.VISUAL, RobotDisplayMode.BOTH}
        current.show_collision = has_collision and mode in {
            RobotDisplayMode.COLLISION,
            RobotDisplayMode.BOTH,
        }
        fallback = self._collision_fallback_urdfs.get(model_key)
        if fallback is not None:
            fallback.show_visual = mode in {
                RobotDisplayMode.COLLISION,
                RobotDisplayMode.BOTH,
            }
            fallback.show_collision = False
            self._set_handle_visibility(
                fallback,
                mode in {RobotDisplayMode.COLLISION, RobotDisplayMode.BOTH},
            )

    def loaded_robot_description(self, config: RobotModelConfig) -> LoadedRobotModel:
        description = prepare_urdf_for_drake(
            config.model.load(),
            convert_meshes=bool(config.auto_convert_meshes),
        )
        description = self._strip_visualization_world_root_attachment(config, description)
        self._assert_base_link_is_urdf_root(config, description)
        return description

    @staticmethod
    def _strip_visualization_world_root_attachment(
        config: RobotModelConfig, description: LoadedRobotModel
    ) -> LoadedRobotModel:
        """Detach a model-owned world root only for Viser base-pose rendering."""
        urdf_content = description.xml
        try:
            root = ET.fromstring(urdf_content)
        except ET.ParseError:
            return description

        attachments = [
            joint
            for joint in root.findall("joint")
            if joint.attrib.get("type") == "fixed"
            and (parent := joint.find("parent")) is not None
            and parent.attrib.get("link") == "world"
            and (child := joint.find("child")) is not None
            and child.attrib.get("link") == config.base_link
        ]
        if len(attachments) != 1:
            return description

        root.remove(attachments[0])
        referenced_links = {
            link
            for joint in root.findall("joint")
            for element in (joint.find("parent"), joint.find("child"))
            if (link := element.get("link") if element is not None else None) is not None
        }
        if "world" in referenced_links:
            return description
        world_links = [link for link in root.findall("link") if link.attrib.get("name") == "world"]
        if len(world_links) != 1:
            return description
        root.remove(world_links[0])

        return LoadedRobotModel(
            xml=ET.tostring(root, encoding="unicode"),
            source_path=description.source_path,
            package_paths=description.package_paths,
        )

    @staticmethod
    def _assert_base_link_is_urdf_root(
        config: RobotModelConfig, description: LoadedRobotModel
    ) -> None:
        root_link = description.root_link
        if root_link == config.base_link:
            return
        raise ValueError(
            f"Viser visualization requires base_link '{config.base_link}' to match "
            f"the prepared URDF root '{root_link}' because base_pose is applied to the URDF root"
        )

    def _urdf_root_node_name(self, model_key: str, kind: str, config: RobotModelConfig) -> str:
        root_node_name = {
            "current": f"/robots/{model_key}/current",
            "target": f"/targets/{model_key}/target",
            "preview": f"/previews/{model_key}/ghost",
        }[kind]
        if not self._has_non_identity_base_pose(config):
            return root_node_name
        self._ensure_base_pose_frame(model_key, kind, config)
        return f"{root_node_name}/base_pose/urdf"

    def _ensure_base_pose_frame(self, model_key: str, kind: str, config: RobotModelConfig) -> None:
        key = f"{model_key}:{kind}:base_pose"
        if key in self._root_frames:
            return
        pose = config.base_pose
        frame_name = {
            "current": f"/robots/{model_key}/current/base_pose",
            "target": f"/targets/{model_key}/target/base_pose",
            "preview": f"/previews/{model_key}/ghost/base_pose",
        }[kind]
        self._root_frames[key] = self.server.scene.add_frame(
            frame_name,
            show_axes=False,
            position=(
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ),
            wxyz=(
                float(pose.orientation.w),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
            ),
        )

    @staticmethod
    def _has_non_identity_base_pose(config: RobotModelConfig) -> bool:
        pose = getattr(config, "base_pose", None)
        if pose is None:
            return False
        return any(
            abs(value) > 1e-12
            for value in (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w) - 1.0,
            )
        )

    def set_urdf_joints(
        self, urdf: ViserUrdf | None, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        if urdf is None:
            return
        cfg = self.viser_joint_configuration(urdf, joint_names, joints)
        if not cfg:
            return
        update_cfg = getattr(urdf, "update_cfg", None)
        if callable(update_cfg):
            update_cfg(cfg)
            return
        update_configuration = getattr(urdf, "update_configuration", None)
        if callable(update_configuration):
            update_configuration(cfg)

    def viser_joint_configuration(
        self, urdf: ViserUrdf, joint_names: Sequence[str], joints: Sequence[float]
    ) -> list[float]:
        allowed_names = list(self.viser_actuated_joint_names(urdf))
        if not allowed_names:
            return []
        values_by_name: dict[str, float] = {}
        for name, value in zip(joint_names, joints, strict=False):
            values_by_name[str(name)] = float(value)
        return [values_by_name.get(name, 0.0) for name in allowed_names]

    def viser_actuated_joint_names(self, urdf: ViserUrdf) -> tuple[str, ...]:
        return self._joint_names_by_urdf.get(id(urdf), ())

    def _set_preview_visibility(self, model_key: str, visible: bool) -> None:
        self._set_handle_visibility(self._urdfs.get(f"{model_key}:preview"), visible)

    def _set_target_visibility(self, model_key: str, visible: bool) -> None:
        self._set_handle_visibility(self._urdfs.get(f"{model_key}:target"), visible)

    def _set_handle_visibility(self, handle: SceneHandle | None, visible: bool) -> None:
        if handle is None:
            return
        if not isinstance(handle, ViserUrdf):
            handle.visible = visible
        for mesh in self._meshes(handle):
            mesh.visible = visible

    def _set_urdf_mesh_material(
        self, urdf: ViserUrdf | None, color: tuple[int, int, int], opacity: float
    ) -> None:
        if urdf is None:
            return
        for mesh in self._meshes(urdf):
            mesh.color = color
            mesh.opacity = opacity

    def _meshes(self, handle: SceneHandle) -> tuple[MeshHandle, ...]:
        # Depends on viser internals: ViserUrdf exposes no public accessor for the
        # per-link mesh handles, so we read the private `_meshes` attribute here.
        # Keep this the single place that touches it.
        meshes = getattr(handle, "_meshes", ())
        return tuple(meshes)

    def _remove_handle(self, key: str) -> None:
        handle = self._handles.pop(key, None)
        if handle is None:
            return
        self._remove_scene_handle(handle)

    @staticmethod
    def _remove_scene_handle(handle: object) -> None:
        remove = getattr(handle, "remove", None)
        if callable(remove):
            remove()
