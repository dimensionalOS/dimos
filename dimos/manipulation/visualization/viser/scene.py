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
import math
from pathlib import Path
import threading
from typing import Any, Protocol, TypeAlias, cast

import numpy as np
import trimesh

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.visualization.viser.animation import PreviewAnimator
from dimos.manipulation.visualization.viser.runtime import (
    VISER_INSTALL_HINT,
    VISER_URDF_INSTALL_HINT,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

try:
    from viser import (
        GridHandle,
        MeshHandle,
        SceneNodeHandle,
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

SceneHandle: TypeAlias = ViserUrdf | SceneNodeHandle

OBSTACLE_NAMESPACE = "/manipulation/obstacles"
# The planner model's implicit red default is reserved for collision state. Use a
# cool cyan-teal so obstacles remain readable without being confused for danger.
OBSTACLE_DEFAULT_RGBA = (0.8, 0.2, 0.2, 0.8)
OBSTACLE_FALLBACK_COLOR = (55, 190, 210)
OBSTACLE_FALLBACK_OPACITY = 0.55
OBSTACLE_PROXY_COLOR = (255, 45, 25)


class _ColorHandle(Protocol):
    color: tuple[int, int, int]


class ViserManipulationScene:
    """Viser scene graph helpers for current robot, ghost robot, and path rendering."""

    def __init__(
        self, server: ViserServer, viser_urdf: type[ViserUrdf], *, preview_fps: float
    ) -> None:
        self.server = server
        self.viser_urdf = viser_urdf
        self.preview_fps = preview_fps
        self._configs_by_id: dict[str, RobotModelConfig] = {}
        self._urdfs: dict[str, ViserUrdf] = {}
        self._handles: dict[str, TransformControlsHandle] = {}
        self._grid_handle: GridHandle | None = None
        self._grid_visible = True
        self._preview_visible: dict[str, bool] = {}
        self._target_tracks_current: dict[str, bool] = {}
        self._obstacle_handles: dict[str, list[SceneHandle]] = {}
        self._obstacles_visible = True
        self._obstacle_gui_handles: list[object] = []
        self._lock = threading.RLock()
        self._closed = False
        self._ensure_obstacle_control()
        self._ensure_reference_grid()

    def set_obstacles_visible(self, visible: bool) -> None:
        """Toggle obstacle entities without discarding their scene handles."""
        with self._lock:
            if self._closed:
                return
            self._obstacles_visible = bool(visible)
            for handles in self._obstacle_handles.values():
                for handle in handles:
                    self._set_handle_visibility(handle, self._obstacles_visible)

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        """Render one accepted planner obstacle under the local obstacle namespace."""
        with self._lock:
            if self._closed:
                return
            self.remove_vis_obstacle(obstacle_id)
            position, wxyz = self._obstacle_pose(obstacle)
            color, opacity = self._obstacle_appearance(obstacle)
            path = f"{OBSTACLE_NAMESPACE}/{obstacle_id}"
            handles: list[SceneHandle] = []
            scene = self.server.scene
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
                    radius = float(obstacle.dimensions[0])
                    handles.append(
                        scene.add_icosphere(
                            path,
                            radius=radius,
                            color=color,
                            opacity=opacity,
                            position=position,
                            wxyz=wxyz,
                            visible=self._obstacles_visible,
                        )
                    )
                elif obstacle.obstacle_type == ObstacleType.CYLINDER:
                    radius, height = (float(value) for value in obstacle.dimensions[:2])
                    handles.append(
                        scene.add_cylinder(
                            path,
                            radius=radius,
                            height=height,
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
                else:
                    raise ValueError(f"unsupported obstacle type: {obstacle.obstacle_type}")
            except Exception as error:
                logger.warning(
                    "Could not render obstacle %s; using proxy", obstacle_id, exc_info=True
                )
                proxy = scene.add_box(
                    f"{path}/mesh-failure-proxy",
                    dimensions=(0.25, 0.25, 0.25),
                    color=OBSTACLE_PROXY_COLOR,
                    opacity=0.9,
                    position=position,
                    wxyz=wxyz,
                    visible=self._obstacles_visible,
                )
                label = scene.add_label(
                    f"{path}/mesh-failure-label",
                    f"MESH RENDER FAILED: {error}",
                    position=position,
                    visible=self._obstacles_visible,
                )
                handles.extend((proxy, label))
            self._obstacle_handles[obstacle_id] = handles

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        """Remove every scene entity belonging to an obstacle ID."""
        with self._lock:
            if self._closed:
                return
            for handle in self._obstacle_handles.pop(obstacle_id, []):
                self._remove_scene_handle(handle)

    def clear_vis_obstacles(self) -> None:
        """Remove every obstacle entity while retaining the robot scene."""
        with self._lock:
            for obstacle_id in list(self._obstacle_handles):
                self.remove_vis_obstacle(obstacle_id)

    def _ensure_obstacle_control(self) -> None:
        """Create the one stable local obstacle control, independent of the panel."""
        try:
            gui = self.server.gui
            folder = gui.add_folder("Scene", expand_by_default=True)
            self._obstacle_gui_handles.append(folder)
            with folder:
                handle = gui.add_checkbox("manipulation.obstacles", initial_value=True)
            handle.on_update(lambda event: self.set_obstacles_visible(event.target.value))
            self._obstacle_gui_handles.append(handle)
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
        if color == OBSTACLE_DEFAULT_RGBA:
            return OBSTACLE_FALLBACK_COLOR, OBSTACLE_FALLBACK_OPACITY
        if len(color) != 4 or not all(math.isfinite(float(value)) for value in color):
            return OBSTACLE_FALLBACK_COLOR, OBSTACLE_FALLBACK_OPACITY
        if not all(0.0 <= float(value) <= 1.0 for value in color):
            return OBSTACLE_FALLBACK_COLOR, OBSTACLE_FALLBACK_OPACITY
        return (
            round(float(color[0]) * 255),
            round(float(color[1]) * 255),
            round(float(color[2]) * 255),
        ), float(color[3])

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
            dumped = mesh.dump(concatenate=True)
            mesh = dumped
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

    def has_reference_grid(self) -> bool:
        """Return whether the Viser scene accepted the optional reference grid."""
        return self._grid_handle is not None

    def set_reference_grid_visible(self, visible: bool) -> None:
        """Show or hide the optional ground reference grid."""
        self._grid_visible = visible
        self._set_handle_visibility(self._grid_handle, visible)

    def register_robot(self, robot_id: str, config: RobotModelConfig) -> None:
        self._configs_by_id[robot_id] = config
        self._preview_visible.setdefault(robot_id, False)
        self._target_tracks_current.setdefault(robot_id, True)
        self._ensure_robot_urdfs(robot_id, config)

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
        self, robot_id: str, on_update: Callable[[TransformControlsHandle], None]
    ) -> TransformControlsHandle | None:
        handle_key = f"{robot_id}:ee_control"
        if handle_key in self._handles:
            return self._handles[handle_key]
        handle = self.server.scene.add_transform_controls(
            f"/targets/{robot_id}/ee_control", scale=0.25
        )

        def dispatch(event: TransformControlsEvent) -> None:
            on_update(event.target)

        handle.on_update(dispatch)
        self._handles[handle_key] = handle
        return handle

    def update_current_robot(self, robot_id: str, joint_state: JointState | None) -> None:
        config = self._configs_by_id.get(robot_id)
        if config is None or joint_state is None:
            return
        self._ensure_robot_urdfs(robot_id, config)
        current = self._urdfs.get(f"{robot_id}:current")
        self.set_urdf_joints(current, config.joint_names, joint_state.position)
        if self._target_tracks_current.get(robot_id, True):
            self._set_target_joints(robot_id, config.joint_names, joint_state.position)
            self._set_target_visibility(robot_id, True)

    def show_preview(self, robot_id: str) -> None:
        """Show the transient preview-animation ghost.

        Target editing uses the separate target ghost and must not call this path.
        """
        self._preview_visible[robot_id] = True
        self._set_preview_visibility(robot_id, True)

    def hide_preview(self, robot_id: str) -> None:
        """Hide the transient preview-animation ghost."""
        self._preview_visible[robot_id] = False
        self._set_preview_visibility(robot_id, False)

    def animate_path(self, robot_id: str, path: Sequence[JointState], duration: float) -> bool:
        config = self._configs_by_id.get(robot_id)
        if config is None:
            return False
        self.show_preview(robot_id)
        try:
            return PreviewAnimator(
                lambda joints: self._set_preview_ghost_joints(robot_id, config.joint_names, joints)
            ).animate(path, duration, self.preview_fps)
        finally:
            self.hide_preview(robot_id)

    def set_target_joints(
        self, robot_id: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> bool:
        target = self._urdfs.get(f"{robot_id}:target")
        if target is None:
            return False
        self._target_tracks_current[robot_id] = False
        self._set_target_joints(robot_id, joint_names, joints)
        self._set_target_visibility(robot_id, True)
        return True

    def clear_target(self, robot_id: str) -> None:
        """Return the persistent target ghost to current-state tracking."""
        self._target_tracks_current[robot_id] = True

    def _set_target_joints(
        self, robot_id: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        target = self._urdfs.get(f"{robot_id}:target")
        self.set_urdf_joints(target, joint_names, joints)

    def _set_preview_ghost_joints(
        self, robot_id: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        ghost = self._urdfs.get(f"{robot_id}:preview")
        self.set_urdf_joints(ghost, joint_names, joints)

    def set_target_pose(self, robot_id: str, pose: Pose | None) -> None:
        handle = self._handles.get(f"{robot_id}:ee_control")
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

    def set_target_visual_state(self, robot_id: str, feasible: bool) -> None:
        color = TARGET_CONTROL_FEASIBLE_COLOR if feasible else TARGET_CONTROL_INFEASIBLE_COLOR
        mesh_color = GOAL_ROBOT_FEASIBLE_COLOR if feasible else GOAL_ROBOT_INFEASIBLE_COLOR
        mesh_opacity = GOAL_ROBOT_FEASIBLE_OPACITY if feasible else GOAL_ROBOT_INFEASIBLE_OPACITY
        handle = self._handles.get(f"{robot_id}:ee_control")
        if handle is not None:
            cast("_ColorHandle", handle).color = color
        target = self._urdfs.get(f"{robot_id}:target")
        self._set_urdf_mesh_material(target, mesh_color, mesh_opacity)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for handles in self._obstacle_handles.values():
                for handle in handles:
                    self._remove_scene_handle(handle)
            self._obstacle_handles.clear()
            for key in list(self._handles):
                self._remove_handle(key)
            if self._grid_handle is not None:
                self._remove_scene_handle(self._grid_handle)
                self._grid_handle = None
            for urdf in self._urdfs.values():
                self._remove_scene_handle(urdf)
            self._urdfs.clear()
            self._configs_by_id.clear()
            self._preview_visible.clear()
            self._target_tracks_current.clear()
            for gui_handle in self._obstacle_gui_handles:
                self._remove_scene_handle(gui_handle)
            self._obstacle_gui_handles.clear()

    def _ensure_robot_urdfs(self, robot_id: str, config: RobotModelConfig) -> None:
        if not config.model_path:
            return
        for kind in ("current", "target", "preview"):
            key = f"{robot_id}:{kind}"
            if key in self._urdfs:
                continue
            root_node_name = {
                "current": f"/robots/{robot_id}/current",
                "target": f"/targets/{robot_id}/target",
                "preview": f"/previews/{robot_id}/ghost",
            }[kind]
            mesh_color_override = {
                "current": None,
                "target": GOAL_ROBOT_MESH_COLOR,
                "preview": PREVIEW_ROBOT_MESH_COLOR,
            }[kind]
            self._urdfs[key] = self.viser_urdf(
                self.server,
                self.prepared_urdf_path(config),
                root_node_name=root_node_name,
                mesh_color_override=mesh_color_override,
            )
            if kind == "target":
                self._set_urdf_mesh_material(
                    self._urdfs[key], GOAL_ROBOT_FEASIBLE_COLOR, GOAL_ROBOT_FEASIBLE_OPACITY
                )
                self._set_handle_visibility(self._urdfs[key], True)
            elif kind == "preview":
                self._set_urdf_mesh_material(
                    self._urdfs[key], PREVIEW_ROBOT_COLOR, PREVIEW_ROBOT_OPACITY
                )
                self._set_handle_visibility(
                    self._urdfs[key], self._preview_visible.get(robot_id, False)
                )

    def prepared_urdf_path(self, config: RobotModelConfig) -> Path:
        package_paths = {package: Path(path) for package, path in config.package_paths.items()}
        return Path(
            prepare_urdf_for_drake(
                Path(str(config.model_path)),
                package_paths=package_paths,
                xacro_args={str(key): str(value) for key, value in config.xacro_args.items()},
                convert_meshes=bool(config.auto_convert_meshes),
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
            values_by_name[name] = float(value)
            values_by_name[name.rsplit("/", 1)[-1]] = float(value)
        return [values_by_name.get(name, 0.0) for name in allowed_names]

    def viser_actuated_joint_names(self, urdf: ViserUrdf) -> tuple[str, ...]:
        # Depends on viser internals: ViserUrdf exposes no public accessor for its
        # wrapped yourdfpy model, so we reach for the private `_urdf` attribute here.
        # Keep this the single place that touches it.
        return tuple(str(name) for name in urdf._urdf.actuated_joint_names)

    def _set_preview_visibility(self, robot_id: str, visible: bool) -> None:
        self._set_handle_visibility(self._urdfs.get(f"{robot_id}:preview"), visible)

    def _set_target_visibility(self, robot_id: str, visible: bool) -> None:
        self._set_handle_visibility(self._urdfs.get(f"{robot_id}:target"), visible)

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
