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

from typing import TYPE_CHECKING, cast

from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.runtime import ViserRuntime, import_viser_urdf
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene, _ViserUrdfFactory
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.models import (
        JointPath,
        PlanningSceneInfo,
        WorldRobotID,
    )
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.JointState import JointState

logger = setup_logger()


class ViserManipulationVisualizer:
    """In-process Viser implementation of the manipulation VisualizationSpec."""

    def __init__(
        self,
        *,
        world_monitor: WorldMonitor,
        manipulation_module: ManipulationModule | None,
        config: ViserVisualizationConfig | None = None,
    ) -> None:
        self.config = config or ViserVisualizationConfig()
        self._runtime = ViserRuntime(self.config)
        self._server = self._runtime.start()
        self._viser_urdf = import_viser_urdf()
        self._adapter = InProcessViserAdapter(
            world_monitor=world_monitor,
            manipulation_module=manipulation_module,
        )
        self._scene = ViserManipulationScene(
            self._server,
            cast("_ViserUrdfFactory", self._viser_urdf),
            preview_fps=self.config.preview_fps,
        )
        self._gui = (
            ViserPanelGui(self._server, self._adapter, self.config, self._scene)
            if self.config.panel_enabled
            else None
        )
        if self._gui is not None:
            self._gui.start()
        self._closed = False
        logger.info(f"Viser manipulation visualization: {self.get_visualization_url()}")

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        """Initialize Viser robot visuals from planning-scene metadata."""
        if self._closed:
            return
        for robot_id, config in scene.robots.items():
            self._scene.register_robot(str(robot_id), config)
        if self._gui is not None:
            self._gui.refresh()

    def get_visualization_url(self) -> str | None:
        return self._runtime.url

    def publish_visualization(self, ctx: object | None = None) -> None:
        """Update current robot render state. ctx is accepted for protocol compatibility."""
        if self._closed:
            return
        for _robot_name, robot_id, _config in self._adapter.robot_items():
            current = self._adapter.get_current_joint_state(_robot_name)
            self._scene.update_current_robot(str(robot_id), current)
        if self._gui is not None:
            self._gui.refresh()

    def show_preview(self, robot_id: WorldRobotID) -> None:
        if not self._closed:
            self._scene.show_preview(str(robot_id))

    def hide_preview(self, robot_id: WorldRobotID) -> None:
        if not self._closed:
            self._scene.hide_preview(str(robot_id))

    def animate_path(
        self,
        robot_id: WorldRobotID,
        path: JointPath,
        duration: float = 3.0,
    ) -> None:
        if self._closed:
            return
        self._scene.animate_path(str(robot_id), list(path), duration)

    def set_planning_target(
        self,
        robot_id: WorldRobotID,
        joints: JointState,
        pose: PoseStamped | None = None,
        feasible: bool | None = None,
    ) -> None:
        """Render the planner-selected persistent target."""
        if self._closed:
            return
        self._scene.set_target_joints(str(robot_id), joints.name, joints.position)
        self._scene.set_target_pose(str(robot_id), pose)
        if feasible is not None:
            self._scene.set_target_visual_state(str(robot_id), feasible)

    def clear_planning_target(self, robot_id: WorldRobotID) -> None:
        """Clear the planner-selected persistent target."""
        if not self._closed:
            self._scene.clear_target(str(robot_id))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._gui is not None:
                self._gui.close()
            self._scene.close()
        finally:
            self._runtime.close()
