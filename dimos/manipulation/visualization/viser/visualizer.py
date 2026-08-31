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
from typing import TYPE_CHECKING

from dimos.manipulation.visualization.viser.animation import (
    PreviewAnimation,
    PreviewFrame,
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
        Obstacle,
        PlanningSceneInfo,
        VisualizationSession,
        VisualizationStateFrame,
    )
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

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
        self._current_state: JointState | None = None
        self._model_config: RobotModelConfig | None = None
        self._last_ground_truth_report: tuple[tuple[str, str | None, float | None], ...] = ()
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
                ground_truth_overlay=self.config.ground_truth_overlay,
            )
            gui = (
                ViserPanelGui(
                    server,
                    self._session_scene,
                    self._operator,
                    lambda: self._current_state,
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
        self._model_config = session.scene.model
        self._initialize_scene(session.scene)

    def _initialize_scene(self, scene: PlanningSceneInfo) -> None:
        """Initialize Viser robot visuals from planning-scene metadata."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        try:
            self._scene.register_model(scene.model)
            if self._gui is not None:
                self._gui.refresh()
        except Exception:
            self.close()
            raise

    def get_visualization_url(self) -> str | None:
        return None if self._runtime is None else self._runtime.url

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        """Render an accepted planning obstacle."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is not None:
            self._scene.add_vis_obstacle(obstacle_id, obstacle)

    def update_vis_obstacle(self, obstacle: Obstacle) -> None:
        """Replace an obstacle representation without affecting planning state."""
        if self._closed:
            return
        try:
            self._ensure_started()
            if self._scene is None:
                return
            self._scene.update_vis_obstacle(obstacle)
        except Exception as error:
            logger.warning(
                "Obstacle visualization update failed for '%s': %s",
                obstacle.name,
                error,
                exc_info=True,
            )
            if self._scene is not None:
                self._scene.show_obstacle_warning(
                    f"⚠️ Obstacle visualization is stale for `{obstacle.name}`: {error}"
                )

    def update_vis_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> None:
        """Move an obstacle representation without affecting planning state."""
        if self._closed:
            return
        try:
            self._ensure_started()
            if self._scene is None:
                return
            self._scene.update_vis_obstacle_pose(obstacle_id, pose)
        except Exception as error:
            logger.warning(
                "Obstacle pose visualization update failed for '%s': %s",
                obstacle_id,
                error,
                exc_info=True,
            )
            if self._scene is not None:
                self._scene.show_obstacle_warning(
                    f"⚠️ Obstacle visualization is stale for `{obstacle_id}`: {error}"
                )

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        """Remove an obstacle representation."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is not None:
            self._scene.remove_vis_obstacle(obstacle_id)

    def clear_vis_obstacles(self) -> None:
        """Remove all obstacle representations."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is not None:
            self._scene.clear_vis_obstacles()

    def set_ground_truth_poses(
        self,
        poses: dict[str, PoseStamped],
        belief: dict[str, PoseStamped],
    ) -> None:
        """Replace the live sim-truth ghosts and compare them with planner belief."""
        if self._closed or not self.config.ground_truth_overlay:
            return
        self._ensure_started()
        if self._scene is None:
            return
        rows = tuple(self._scene.set_ground_truth_poses(poses, belief))
        rounded = tuple(
            (truth, matched, None if delta is None else round(delta, 4))
            for truth, matched, delta in rows
        )
        if rounded != self._last_ground_truth_report:
            self._last_ground_truth_report = rounded
            logger.info(
                "Truth-vs-belief deltas",
                deltas=[
                    {
                        "truth": truth,
                        "belief": matched,
                        "delta_m": delta,
                    }
                    for truth, matched, delta in rounded
                ],
            )

    def update_state(self, frame: VisualizationStateFrame) -> None:
        """Update current robot render state from a pushed state frame."""
        if self._closed:
            return
        self._ensure_started()
        if self._scene is None:
            return
        if frame.joint_state is not None:
            self._current_state = JointState(frame.joint_state)
            self._scene.update_current_model(frame.joint_state)
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

    def _raw_preview_animation(self, trajectory: JointTrajectory) -> PreviewAnimation | None:
        config = self._model_config
        if config is None or len(trajectory.joint_names) != len(set(trajectory.joint_names)):
            return None
        indices = {str(name): index for index, name in enumerate(trajectory.joint_names)}
        if not set(indices).issubset(config.joint_names):
            return None
        baseline = self._baseline_values(config, self._current_state)
        if baseline is None:
            return None
        frames: list[PreviewFrame] = []
        for point in trajectory.points:
            selected = {name: float(point.positions[index]) for name, index in indices.items()}
            positions = [float(selected.get(name, baseline[name])) for name in config.joint_names]
            frames.append(PreviewFrame(float(point.time_from_start), tuple(positions)))
        return PreviewAnimation(tuple(config.joint_names), tuple(frames)) if frames else None

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
