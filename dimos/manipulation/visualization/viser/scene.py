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

from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Protocol, cast

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.visualization.viser.animation import PreviewAnimator
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

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


class _ViserUrdfFactory(Protocol):
    def __call__(
        self,
        server: object,
        urdf_path: Path,
        *,
        root_node_name: str,
        mesh_color_override: tuple[int, int, int, float] | None,
    ) -> object: ...


class _SceneServer(Protocol):
    scene: object


class _TransformScene(Protocol):
    def add_transform_controls(self, name: str, *, scale: float) -> object: ...


class _GridScene(Protocol):
    def add_grid(self, name: str, **kwargs: object) -> object: ...


class _UpdateHandle(Protocol):
    def on_update(self, callback: object) -> object: ...


class _EventWithTarget(Protocol):
    target: object


class _PoseLike(Protocol):
    position: object
    orientation: object


class _PointLike(Protocol):
    x: float
    y: float
    z: float


class _QuaternionLike(Protocol):
    w: float
    x: float
    y: float
    z: float


class _TransformHandle(Protocol):
    position: tuple[float, float, float]
    wxyz: tuple[float, float, float, float]


class _UrdfHandle(Protocol):
    _urdf: object


class _UpdateCfgUrdf(Protocol):
    def update_cfg(self, cfg: Sequence[float]) -> None: ...


class _UpdateConfigurationUrdf(Protocol):
    def update_configuration(self, cfg: Sequence[float]) -> None: ...


class _ActuatedNames(Protocol):
    actuated_joint_names: Sequence[str]


class _JointMap(Protocol):
    joint_map: Mapping[str, object]


class _MeshContainer(Protocol):
    _meshes: Sequence[object]


class _VisibleHandle(Protocol):
    visible: bool


class _ColorHandle(Protocol):
    color: tuple[int, int, int]


class _MaterialColorHandle(Protocol):
    material_color: tuple[int, int, int]


class _OpacityHandle(Protocol):
    opacity: float


class _RemovableHandle(Protocol):
    def remove(self) -> None: ...


class ViserManipulationScene:
    """Viser scene graph helpers for current robot, ghost robot, and path rendering."""

    def __init__(
        self, server: object, viser_urdf: _ViserUrdfFactory, *, preview_fps: float
    ) -> None:
        self.server = server
        self.viser_urdf = viser_urdf
        self.preview_fps = preview_fps
        self._configs_by_id: dict[str, RobotModelConfig] = {}
        self._urdfs: dict[str, object] = {}
        self._handles: dict[str, object] = {}
        self._grid_handle: object | None = None
        self._grid_visible = True
        self._preview_visible: dict[str, bool] = {}
        self._target_tracks_current: dict[str, bool] = {}
        self._ensure_reference_grid()

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
            scene = cast("_SceneServer", self.server).scene
        except AttributeError:
            return
        if scene is None or not hasattr(scene, "add_grid"):
            return
        try:
            self._grid_handle = cast("_GridScene", scene).add_grid(
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
        self, robot_id: str, on_update: Callable[[object], None]
    ) -> object | None:
        handle_key = f"{robot_id}:ee_control"
        if handle_key in self._handles:
            return self._handles[handle_key]
        try:
            scene = cast("_SceneServer", self.server).scene
        except AttributeError:
            return None
        if scene is None or not hasattr(scene, "add_transform_controls"):
            return None
        handle = cast("_TransformScene", scene).add_transform_controls(
            f"/targets/{robot_id}/ee_control", scale=0.25
        )
        if hasattr(handle, "on_update"):

            def dispatch(event: object) -> None:
                try:
                    target = cast("_EventWithTarget", event).target
                except AttributeError:
                    target = handle
                on_update(target)

            cast("_UpdateHandle", handle).on_update(dispatch)
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

    def set_target_pose(self, robot_id: str, pose: object | None) -> None:
        handle = self._handles.get(f"{robot_id}:ee_control")
        if handle is None or pose is None:
            return
        try:
            pose_like = cast("_PoseLike", pose)
            position = cast("_PointLike", pose_like.position)
            cast("_TransformHandle", handle).position = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
        except (AttributeError, TypeError):
            pass
        try:
            orientation = cast("_QuaternionLike", cast("_PoseLike", pose).orientation)
            cast("_TransformHandle", handle).wxyz = (
                float(orientation.w),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
            )
        except (AttributeError, TypeError):
            pass

    def set_target_visual_state(self, robot_id: str, feasible: bool) -> None:
        color = TARGET_CONTROL_FEASIBLE_COLOR if feasible else TARGET_CONTROL_INFEASIBLE_COLOR
        mesh_color = GOAL_ROBOT_FEASIBLE_COLOR if feasible else GOAL_ROBOT_INFEASIBLE_COLOR
        mesh_opacity = GOAL_ROBOT_FEASIBLE_OPACITY if feasible else GOAL_ROBOT_INFEASIBLE_OPACITY
        handles = [self._handles.get(f"{robot_id}:ee_control")]
        target = self._urdfs.get(f"{robot_id}:target")
        handles.append(target)
        self._set_urdf_mesh_material(target, mesh_color, mesh_opacity)
        for handle in handles:
            if handle is None:
                continue
            for attr in ("color", "material_color"):
                if hasattr(handle, attr):
                    try:
                        if attr == "color":
                            cast("_ColorHandle", handle).color = color
                        else:
                            cast("_MaterialColorHandle", handle).material_color = color
                    except Exception:
                        logger.warning("Could not set Viser handle %s", attr, exc_info=True)

    def close(self) -> None:
        for key in list(self._handles):
            self._remove_handle(key)
        self._urdfs.clear()
        self._configs_by_id.clear()
        self._preview_visible.clear()
        self._target_tracks_current.clear()

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
        self, urdf: object | None, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        if urdf is None:
            return
        cfg = self.viser_joint_configuration(urdf, joint_names, joints)
        if not cfg:
            return
        if hasattr(urdf, "update_cfg"):
            cast("_UpdateCfgUrdf", urdf).update_cfg(cfg)
        elif hasattr(urdf, "update_configuration"):
            cast("_UpdateConfigurationUrdf", urdf).update_configuration(cfg)

    def viser_joint_configuration(
        self, urdf: object, joint_names: Sequence[str], joints: Sequence[float]
    ) -> list[float]:
        allowed_names = list(self.viser_actuated_joint_names(urdf))
        if not allowed_names:
            return []
        values_by_name: dict[str, float] = {}
        for name, value in zip(joint_names, joints, strict=False):
            values_by_name[name] = float(value)
            values_by_name[name.rsplit("/", 1)[-1]] = float(value)
        return [values_by_name.get(name, 0.0) for name in allowed_names]

    def viser_actuated_joint_names(self, urdf: object) -> tuple[str, ...]:
        # Depends on viser internals: ViserUrdf exposes no public accessor for its
        # wrapped yourdfpy model, so we reach for the private `_urdf` attribute here.
        # Keep this the single place that touches it.
        try:
            wrapped_urdf = cast("_UrdfHandle", urdf)._urdf
        except AttributeError:
            return ()
        try:
            return tuple(cast("_ActuatedNames", wrapped_urdf).actuated_joint_names)
        except AttributeError:
            pass
        try:
            return tuple(cast("_JointMap", wrapped_urdf).joint_map)
        except AttributeError:
            pass
        return ()

    def _set_preview_visibility(self, robot_id: str, visible: bool) -> None:
        self._set_handle_visibility(self._urdfs.get(f"{robot_id}:preview"), visible)

    def _set_target_visibility(self, robot_id: str, visible: bool) -> None:
        self._set_handle_visibility(self._urdfs.get(f"{robot_id}:target"), visible)

    def _set_handle_visibility(self, handle: object | None, visible: bool) -> None:
        if handle is None:
            return
        for candidate in (handle, *self._meshes(handle)):
            if hasattr(candidate, "visible"):
                try:
                    cast("_VisibleHandle", candidate).visible = visible
                except Exception:
                    logger.warning("Could not set Viser handle visibility", exc_info=True)

    def _set_urdf_mesh_material(
        self, urdf: object | None, color: tuple[int, int, int], opacity: float
    ) -> None:
        if urdf is None:
            return
        for mesh in self._meshes(urdf):
            for attr in ("color", "material_color"):
                if hasattr(mesh, attr):
                    try:
                        if attr == "color":
                            cast("_ColorHandle", mesh).color = color
                        else:
                            cast("_MaterialColorHandle", mesh).material_color = color
                    except Exception:
                        logger.warning("Could not set Viser mesh %s", attr, exc_info=True)
            if hasattr(mesh, "opacity"):
                try:
                    cast("_OpacityHandle", mesh).opacity = opacity
                except Exception:
                    logger.warning("Could not set Viser mesh opacity", exc_info=True)

    def _meshes(self, handle: object) -> Sequence[object]:
        # Depends on viser internals: ViserUrdf exposes no public accessor for the
        # per-link mesh handles, so we read the private `_meshes` attribute here.
        # Keep this the single place that touches it.
        try:
            return cast("_MeshContainer", handle)._meshes
        except AttributeError:
            return ()

    def _remove_handle(self, key: str) -> None:
        handle = self._handles.pop(key, None)
        if handle is not None and hasattr(handle, "remove"):
            cast("_RemovableHandle", handle).remove()
