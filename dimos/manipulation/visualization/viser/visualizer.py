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

from contextlib import suppress
import time
from typing import TYPE_CHECKING

from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.runtime import (
    VISER_URDF_INSTALL_HINT,
    ViserRuntime,
    ViserServer,
)
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.manipulation.visualization.viser.theme import apply_dimos_theme
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
        JointPath,
        PlanningSceneInfo,
        WorldRobotID,
    )

logger = setup_logger()

# Gripper state is a coordinator round-trip; poll it far below the render rate.
_GRIPPER_POLL_INTERVAL = 0.2


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
        self._adapter: InProcessViserAdapter | None = None
        self._scene: ViserManipulationScene | None = None
        self._gui: ViserPanelGui | None = None
        self._gripper_positions: dict[str, float] = {}
        self._gripper_polled_at: dict[str, float] = {}
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
            adapter = InProcessViserAdapter(
                world_monitor=self._world_monitor,
                manipulation_module=self._manipulation_module,
            )
            scene = ViserManipulationScene(
                server,
                ViserUrdf,
                preview_fps=self.config.preview_fps,
            )
            gui = (
                ViserPanelGui(server, adapter, self.config, scene)
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
            self._adapter = None
            self._scene = None
            self._gui = None
            self._closed = True
            raise
        self._runtime = runtime
        self._server = server
        self._adapter = adapter
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
        if self._adapter is None or self._scene is None:
            return
        for _robot_name, robot_id, _config in self._adapter.robot_items():
            current = self._adapter.get_current_joint_state(_robot_name)
            gripper = self._gripper_position(_robot_name, _config)
            self._scene.update_current_robot(str(robot_id), current, gripper)
        if self._gui is not None:
            self._gui.refresh()

    def _gripper_position(self, robot_name: str, config: RobotModelConfig) -> float | None:
        """Last known jaw opening, polled well below the render rate.

        Unlike joint state, this is a coordinator round-trip, so it is throttled
        and never allowed to raise: this loop also refreshes the panel controls,
        and stalling it freezes the Plan/Preview/Execute buttons.
        """
        if config.gripper is None or self._adapter is None:
            return None
        now = time.monotonic()
        if now - self._gripper_polled_at.get(robot_name, 0.0) >= _GRIPPER_POLL_INTERVAL:
            self._gripper_polled_at[robot_name] = now
            try:
                value = self._adapter.get_gripper(robot_name)
            except Exception:
                logger.warning("Could not read gripper position", exc_info=True)
                value = None
            if value is not None:
                self._gripper_positions[robot_name] = float(value)
        return self._gripper_positions.get(robot_name)

    def show_preview(self, robot_id: WorldRobotID) -> None:
        if not self._closed:
            self._ensure_started()
            if self._scene is None:
                return
            self._scene.show_preview(str(robot_id))

    def hide_preview(self, robot_id: WorldRobotID) -> None:
        if not self._closed:
            self._ensure_started()
            if self._scene is None:
                return
            self._scene.hide_preview(str(robot_id))

    def animate_path(
        self,
        robot_id: WorldRobotID,
        path: JointPath,
        duration: float = 3.0,
    ) -> None:
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        self._scene.animate_path(str(robot_id), list(path), duration)

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
            self._adapter = None
            self._scene = None
            self._gui = None
        if errors:
            raise errors[0]
