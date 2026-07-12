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

from dimos.manipulation.visualization.viser.animation import (
    GroupPreviewAnimation,
    PreviewFrame,
    PreviewTrack,
)
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
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
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
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        PlanningSceneInfo,
        VisualizationSession,
        VisualizationStateFrame,
    )

logger = setup_logger()


class ViserManipulationVisualizer:
    """In-process Viser implementation of the manipulation VisualizationSpec."""

    def __init__(
        self,
        *,
        config: ViserVisualizationConfig | None = None,
    ) -> None:
        self.config = config or ViserVisualizationConfig()
        self._runtime: ViserRuntime | None = None
        self._server: ViserServer | None = None
        self._scene: ViserManipulationScene | None = None
        self._gui: ViserPanelGui | None = None
        self._session_scene: PlanningSceneInfo | None = None
        self._operator: object | None = None
        self._current_states: dict[str, JointState] = {}
        self._robot_names_by_id: dict[str, str] = {}
        self._robot_ids_by_name: dict[str, str] = {}
        self._configs_by_name: dict[str, RobotModelConfig] = {}
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
            scene = ViserManipulationScene(server, ViserUrdf)
            gui = (
                ViserPanelGui(
                    server,
                    self._session_scene,
                    self._operator,
                    self._current_states,
                    self.config,
                    scene,
                )
                if self.config.panel_enabled
                and self._session_scene is not None
                and self._operator is not None
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

    def initialize(self, session: VisualizationSession) -> None:
        """Initialize Viser robot visuals from a one-shot visualization session."""
        self._operator = session.operator
        self._session_scene = session.scene
        self._robot_names_by_id = {
            str(robot_id): config.name for robot_id, config in session.scene.robots.items()
        }
        self._robot_ids_by_name = {
            config.name: str(robot_id) for robot_id, config in session.scene.robots.items()
        }
        self._configs_by_name = {config.name: config for config in session.scene.robots.values()}
        self._initialize_scene(session.scene)

    def _initialize_scene(self, scene: PlanningSceneInfo) -> None:
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

    def update_state(self, frame: VisualizationStateFrame) -> None:
        """Update current robot render state from a pushed state frame."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        for robot_id, current in frame.joint_states.items():
            robot_id_string = str(robot_id)
            self._current_states[robot_id_string] = JointState(current)
            self._scene.update_current_robot(robot_id_string, current)
        if self._gui is not None:
            self._gui.refresh()

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        preview = self._raw_preview_animation(trajectory)
        if preview is not None:
            self._scene.animate_preview(
                preview, duration if duration is not None else max(float(trajectory.duration), 0.0)
            )

    def cancel_preview_animation(self, robot_ids: Sequence[str] | None = None) -> None:
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
            if robot_ids is None:
                scene.cancel_preview_animation()
            else:
                scene.cancel_preview_animation(robot_ids)

    def _raw_preview_animation(self, trajectory: JointTrajectory) -> GroupPreviewAnimation | None:
        robot_indices: dict[str, list[tuple[int, str]]] = {}
        for index, global_name in enumerate(trajectory.joint_names):
            if "/" not in str(global_name):
                return None
            robot_name, local_name = str(global_name).split("/", 1)
            if robot_name not in self._robot_ids_by_name:
                return None
            robot_indices.setdefault(robot_name, []).append((index, local_name))
        tracks: list[PreviewTrack] = []
        for robot_name, indexed_names in robot_indices.items():
            robot_id = self._robot_ids_by_name[robot_name]
            config = self._configs_by_name[robot_name]
            current = self._current_states.get(robot_id)
            baseline = self._baseline_values(config, current)
            if baseline is None:
                return None
            frames: list[PreviewFrame] = []
            for point in trajectory.points:
                selected = {
                    local_name: float(point.positions[index]) for index, local_name in indexed_names
                }
                positions: list[float] = []
                for local_name in config.joint_names:
                    value = selected.get(local_name, baseline.get(local_name))
                    if value is None:
                        return None
                    positions.append(float(value))
                frames.append(PreviewFrame(float(point.time_from_start), tuple(positions)))
            tracks.append(PreviewTrack(robot_id, tuple(config.joint_names), tuple(frames)))
        return GroupPreviewAnimation(tuple(tracks)) if tracks else None

    @staticmethod
    def _baseline_values(
        config: RobotModelConfig, current: JointState | None
    ) -> dict[str, float] | None:
        if current is None or len(current.name) != len(current.position):
            return None
        values = {
            str(name): float(value)
            for name, value in zip(current.name, current.position, strict=True)
        }
        return values if all(name in values for name in config.joint_names) else None

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
