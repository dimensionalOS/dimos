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

from collections.abc import Sequence
from contextlib import suppress
from typing import TYPE_CHECKING

from dimos.manipulation.planning.groups.identifiers import make_global_joint_name
from dimos.manipulation.visualization.viser.animation import GroupPreviewAnimation, PreviewTrack
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.runtime import (
    VISER_URDF_INSTALL_HINT,
    ViserRuntime,
    ViserServer,
)
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.manipulation.visualization.viser.theme import apply_dimos_theme
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

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

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        GeneratedPlan,
        PlanningGroupID,
        PlanningSceneInfo,
    )

logger = setup_logger()


class ViserManipulationVisualizer:
    """In-process Viser implementation of the manipulation VisualizationSpec."""

    def __init__(
        self,
        *,
        world_monitor: WorldMonitor,
        manipulation_module: ManipulationModule,
        config: ViserVisualizationConfig | None = None,
    ) -> None:
        self._world_monitor = world_monitor
        self._manipulation_module = manipulation_module
        self.config = config or ViserVisualizationConfig()
        self._runtime: ViserRuntime | None = None
        self._server: ViserServer | None = None
        self._scene: ViserManipulationScene | None = None
        self._gui: ViserPanelGui | None = None
        self._closed = False

    def _ensure_started(self) -> None:
        if self._closed or self._runtime is not None:
            return
        runtime = ViserRuntime(self.config)
        scene: ViserManipulationScene | None = None
        gui: ViserPanelGui | None = None
        try:
            server = runtime.start()
            apply_dimos_theme(server)
            scene = ViserManipulationScene(
                server,
                ViserUrdf,
                preview_fps=self.config.preview_fps,
            )
            gui = (
                ViserPanelGui(
                    server,
                    self._world_monitor,
                    self._manipulation_module,
                    self.config,
                    scene,
                )
                if self.config.panel_enabled
                else None
            )
            if gui is not None:
                gui.start()
        except Exception:
            if gui is not None:
                with suppress(Exception):
                    gui.close()
            if scene is not None:
                with suppress(Exception):
                    scene.close()
            with suppress(Exception):
                runtime.close()
            self._runtime = None
            self._server = None
            self._scene = None
            self._gui = None
            self._closed = True
            raise
        self._runtime = runtime
        self._server = server
        self._scene = scene
        self._gui = gui
        self._closed = False
        logger.info(f"Viser manipulation visualization: {self.get_visualization_url()}")

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        """Initialize Viser robot visuals from planning-scene metadata."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        try:
            for robot_id, config in scene.robots.items():
                self._scene.register_robot(str(robot_id), config)
            if self._gui is not None:
                self._gui.refresh()
        except Exception:
            self.close()
            raise

    def get_visualization_url(self) -> str | None:
        return None if self._runtime is None else self._runtime.url

    def publish_visualization(self, ctx: None = None) -> None:
        """Update current robot render state. ctx is accepted for protocol compatibility."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        for _robot_name, robot_id, _config in self._manipulation_module.robot_items():
            current = self._world_monitor.get_current_joint_state(robot_id)
            self._scene.update_current_robot(str(robot_id), current)
        if self._gui is not None:
            self._gui.refresh()

    def show_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        if not self._closed:
            self._ensure_started()
            if self._scene is None:
                return
            for robot_id in self._robot_ids_for_groups(group_ids):
                self._scene.show_preview(robot_id)

    def hide_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        if not self._closed:
            self._ensure_started()
            if self._scene is None:
                return
            self.cancel_preview_animation()
            for robot_id in self._robot_ids_for_groups(group_ids):
                self._scene.hide_preview(robot_id)

    def animate_plan(self, plan: GeneratedPlan, duration: float = 3.0) -> None:
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        preview = self._build_group_preview_animation(plan)
        if preview is not None:
            self._scene.animate_preview(preview, duration)

    def cancel_preview_animation(self) -> None:
        """Cancel preview playback without starting a renderer or waiting for it.

        The world monitor deliberately invokes this outside its visualization
        lock, so a renderer sleeping between frames can observe the scene
        generation change immediately.  Do not call ``_ensure_started()``:
        cancelling before Viser has started must remain a no-op.  Likewise,
        retain the scene reference while ``close()`` is unwinding so a
        concurrent cancellation can still invalidate an in-flight frame.
        """
        scene = self._scene
        if scene is not None:
            scene.cancel_preview_animation()

    def _build_group_preview_animation(self, plan: GeneratedPlan) -> GroupPreviewAnimation | None:
        """Validate the complete canonical plan before exposing any preview ghost."""
        if not plan.group_ids or len(set(plan.group_ids)) != len(plan.group_ids) or not plan.path:
            logger.warning("Cannot preview malformed plan with empty or duplicate groups/path")
            return None
        try:
            selection = self._world_monitor.planning_groups.select(plan.group_ids)
        except (KeyError, ValueError):
            logger.warning("Cannot preview plan with unknown or overlapping planning groups")
            return None
        expected_joints = {str(name) for name in selection.joint_names}
        for waypoint in plan.path:
            if len(waypoint.name) != len(waypoint.position):
                logger.warning("Cannot preview malformed plan waypoint")
                return None
            names = [str(name) for name in waypoint.name]
            if len(names) != len(set(names)) or set(names) != expected_joints:
                logger.warning("Cannot preview plan with missing or extraneous selected joints")
                return None

        tracks: list[PreviewTrack] = []
        for robot_name in selection.robot_names:
            robot_id = self._manipulation_module.robot_id_for_name(robot_name)
            config = self._manipulation_module.get_robot_config(robot_name)
            current = (
                self._world_monitor.get_current_joint_state(robot_id)
                if robot_id is not None
                else None
            )
            if robot_id is None or config is None or current is None:
                logger.warning("Cannot construct complete preview state for '%s'", robot_name)
                return None
            path = self._project_plan_path(robot_name, config, current, plan)
            if path is None:
                logger.warning("Cannot project generated plan for robot '%s'", robot_name)
                return None
            tracks.append(PreviewTrack(str(robot_id), tuple(config.joint_names), tuple(path)))
        return GroupPreviewAnimation(tuple(tracks))

    def _robot_ids_for_groups(self, group_ids: Sequence[PlanningGroupID]) -> tuple[str, ...]:
        try:
            names = self._world_monitor.planning_groups.select(group_ids).robot_names
        except (KeyError, ValueError):
            return ()
        return tuple(
            str(robot_id)
            for name in names
            if (robot_id := self._manipulation_module.robot_id_for_name(name)) is not None
        )

    @staticmethod
    def _project_plan_path(
        robot_name: str, config: RobotModelConfig, current: JointState, plan: GeneratedPlan
    ) -> list[JointState] | None:
        if current.name:
            if len(current.name) != len(current.position):
                return None
            baseline = {
                str(name).rsplit("/", 1)[-1]: float(position)
                for name, position in zip(current.name, current.position, strict=True)
            }
        elif len(current.position) == len(config.joint_names):
            # Some joint-state publishers omit names.  A complete current
            # state still supplies the source-compatible unselected baseline.
            baseline = {
                str(name): float(position)
                for name, position in zip(config.joint_names, current.position, strict=True)
            }
        else:
            return None
        result: list[JointState] = []
        for waypoint in plan.path:
            if len(waypoint.name) != len(waypoint.position):
                return None
            selected = dict(zip(waypoint.name, waypoint.position, strict=True))
            positions: list[float] = []
            for local_name in config.joint_names:
                value = selected.get(
                    make_global_joint_name(robot_name, local_name), baseline.get(local_name)
                )
                if value is None:
                    return None
                positions.append(float(value))
            result.append(JointState({"name": list(config.joint_names), "position": positions}))
        return result

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        errors: list[BaseException] = []
        try:
            if self._gui is not None:
                try:
                    self._gui.close()
                except Exception as e:
                    errors.append(e)
            if self._scene is not None:
                try:
                    self._scene.close()
                except Exception as e:
                    errors.append(e)
        finally:
            if self._runtime is not None:
                try:
                    self._runtime.close()
                except Exception as e:
                    errors.append(e)
            self._runtime = None
            self._server = None
            self._scene = None
            self._gui = None
        if errors:
            raise errors[0]
