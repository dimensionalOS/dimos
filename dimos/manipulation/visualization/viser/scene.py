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
from enum import StrEnum
from pathlib import Path
from typing import Protocol, TypeAlias, cast

from yourdfpy import URDF  # type: ignore[import-untyped]

from dimos.manipulation.planning.spec.config import RobotModelConfig
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


class RobotDisplayMode(StrEnum):
    VISUAL = "visual"
    COLLISION = "collision"
    BOTH = "both"


SceneHandle: TypeAlias = ViserUrdf | TransformControlsHandle | GridHandle | MeshHandle


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
        self._models_by_id: dict[str, URDF] = {}
        self._urdfs: dict[str, ViserUrdf] = {}
        self._joint_names_by_urdf: dict[int, tuple[str, ...]] = {}
        self._handles: dict[str, TransformControlsHandle] = {}
        self._grid_handle: GridHandle | None = None
        self._grid_visible = True
        self._preview_visible: dict[str, bool] = {}
        self._target_tracks_current: dict[str, bool] = {}
        self._collision_fallback_urdfs: dict[str, ViserUrdf] = {}
        self._robot_display_mode = RobotDisplayMode.VISUAL
        self._ensure_reference_grid()

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
        for robot_id in self._configs_by_id:
            self._apply_robot_display_mode(robot_id)

    @property
    def collision_geometry_available(self) -> bool:
        """Return whether any primary robot has loaded collision geometry."""
        return any(
            self._model_has_collision_geometry(self._models_by_id[robot_id])
            for robot_id in self._configs_by_id
            if f"{robot_id}:current" in self._urdfs
        )

    @staticmethod
    def _model_has_collision_geometry(model: URDF) -> bool:
        collision_scene = model.collision_scene
        return collision_scene is not None and bool(getattr(collision_scene, "geometry", True))

    def _load_robot_model(self, config: RobotModelConfig) -> URDF:
        return URDF.load(
            self.prepared_urdf_path(config),
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

    def register_robot(self, robot_id: str, config: RobotModelConfig) -> None:
        self._configs_by_id[robot_id] = config
        self._preview_visible.setdefault(robot_id, False)
        self._target_tracks_current.setdefault(robot_id, True)
        if config.model_path and robot_id not in self._models_by_id:
            self._models_by_id[robot_id] = self._load_robot_model(config)
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
        self.set_urdf_joints(
            self._collision_fallback_urdfs.get(robot_id),
            config.joint_names,
            joint_state.position,
        )
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
        for key in list(self._handles):
            self._remove_handle(key)
        if self._grid_handle is not None:
            self._remove_scene_handle(self._grid_handle)
            self._grid_handle = None
        for urdf in self._urdfs.values():
            self._remove_scene_handle(urdf)
        for urdf in self._collision_fallback_urdfs.values():
            self._remove_scene_handle(urdf)
        self._urdfs.clear()
        self._models_by_id.clear()
        self._joint_names_by_urdf.clear()
        self._collision_fallback_urdfs.clear()
        self._configs_by_id.clear()
        self._preview_visible.clear()
        self._target_tracks_current.clear()
        self._robot_display_mode = RobotDisplayMode.VISUAL

    def _ensure_robot_urdfs(self, robot_id: str, config: RobotModelConfig) -> None:
        if not config.model_path:
            return
        model = self._models_by_id.get(robot_id)
        if model is None:
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
            if kind == "current":
                # Keep both representations resident so changing the diagnostic
                # view does not reload or replace the primary robot.
                old_fallback = self._collision_fallback_urdfs.pop(robot_id, None)
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
                        root_node_name=f"/robots/{robot_id}/collision_fallback",
                        mesh_color_override=(
                            *COLLISION_MESH_COLOR,
                            COLLISION_MESH_OPACITY,
                        ),
                        load_meshes=True,
                        load_collision_meshes=False,
                    )
                    self._collision_fallback_urdfs[robot_id] = fallback
                    self._joint_names_by_urdf[id(fallback)] = tuple(
                        str(name) for name in model.actuated_joint_names
                    )
                    self._set_urdf_mesh_material(
                        fallback, COLLISION_MESH_COLOR, COLLISION_MESH_OPACITY
                    )
                self._apply_robot_display_mode(robot_id)
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

    def _apply_robot_display_mode(self, robot_id: str) -> None:
        current = self._urdfs.get(f"{robot_id}:current")
        if current is None:
            return
        model = self._models_by_id.get(robot_id)
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
        fallback = self._collision_fallback_urdfs.get(robot_id)
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
        return self._joint_names_by_urdf.get(id(urdf), ())

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
