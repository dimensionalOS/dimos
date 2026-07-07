# Copyright 2026 Dimensional Inc.
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

"""Visualization-only OpenArm JointState renderer for OpenArm Mini teleop bring-up."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from contextlib import suppress
import importlib
from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.visualization.viser.runtime import ViserRuntime
    from dimos.manipulation.visualization.viser.scene import ViserManipulationScene

logger = setup_logger()


class OpenArmJointStateViserModule(Module):
    """Render named OpenArm arm JointState commands in Viser without follower hardware."""

    joint_command: In[JointState]

    def __init__(
        self,
        robot: RobotModelConfig,
        *,
        visualization: ViserVisualizationConfig | None = None,
        robot_id: str | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._robot = robot
        self._visualization = visualization or ViserVisualizationConfig(panel_enabled=False)
        self._robot_id = robot_id or robot.name
        self._runtime: ViserRuntime | None = None
        self._scene: ViserManipulationScene | None = None
        self._joint_command_unsub: Callable[[], None] | None = None

    @classmethod
    def resolve_config(cls, config_args: Mapping[str, Any]) -> ModuleConfig:
        filtered_args = {
            k: v for k, v in config_args.items() if k not in {"robot", "visualization", "robot_id"}
        }
        return super().resolve_config(filtered_args)

    @rpc
    def start(self) -> None:
        super().start()
        self._ensure_started()
        self._joint_command_unsub = self.joint_command.subscribe(self._on_joint_command)

    @rpc
    def stop(self) -> None:
        if self._joint_command_unsub is not None:
            self._joint_command_unsub()
            self._joint_command_unsub = None
        if self._scene is not None:
            with suppress(Exception):
                self._scene.close()
            self._scene = None
        if self._runtime is not None:
            with suppress(Exception):
                self._runtime.close()
            self._runtime = None
        super().stop()

    def render_joint_command(self, joint_state: JointState) -> bool:
        """Render one command synchronously; exposed for tests and smoke drivers."""
        ordered = ordered_joint_state(joint_state, self._robot.joint_names)
        if ordered is None:
            missing = missing_joint_names(joint_state, self._robot.joint_names)
            logger.warning(
                "Skipping incomplete OpenArm Viser joint command",
                robot_id=self._robot_id,
                missing_joints=missing,
            )
            return False
        self._ensure_started()
        if self._scene is None:
            return False
        self._scene.update_current_robot(self._robot_id, ordered)
        return True

    def _ensure_started(self) -> None:
        if self._runtime is not None and self._scene is not None:
            return
        from dimos.manipulation.visualization.viser.runtime import ViserRuntime
        from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
        from dimos.manipulation.visualization.viser.theme import apply_dimos_theme

        try:
            viser_extras = importlib.import_module("viser.extras")
            viser_urdf = viser_extras.ViserUrdf
        except ModuleNotFoundError as e:
            from dimos.manipulation.visualization.viser.runtime import VISER_URDF_INSTALL_HINT

            if e.name not in {"viser", "viser.extras", "yourdfpy"}:
                raise
            raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
        except ImportError as e:
            from dimos.manipulation.visualization.viser.runtime import VISER_URDF_INSTALL_HINT

            if "ViserUrdf" not in str(e):
                raise
            raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
        except AttributeError as e:
            from dimos.manipulation.visualization.viser.runtime import VISER_URDF_INSTALL_HINT

            raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e

        runtime = ViserRuntime(self._visualization)
        scene: ViserManipulationScene | None = None
        try:
            server = runtime.start()
            apply_dimos_theme(server)
            scene = ViserManipulationScene(
                server,
                viser_urdf,
                preview_fps=self._visualization.preview_fps,
            )
            scene.register_robot(self._robot_id, self._robot)
        except Exception:
            if scene is not None:
                with suppress(Exception):
                    scene.close()
            with suppress(Exception):
                runtime.close()
            raise
        self._runtime = runtime
        self._scene = scene
        logger.info(f"OpenArm Mini Viser teleop visualization: {runtime.url}")

    def _on_joint_command(self, joint_state: JointState) -> None:
        self.render_joint_command(joint_state)


def ordered_joint_state(
    joint_state: JointState, required_joint_names: Sequence[str]
) -> JointState | None:
    """Return a JointState ordered by required names, or None if any are missing."""
    if len(joint_state.name) != len(joint_state.position):
        return None
    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    if any(name not in positions_by_name for name in required_joint_names):
        return None
    return JointState(
        {
            "name": list(required_joint_names),
            "position": [positions_by_name[name] for name in required_joint_names],
        }
    )


def missing_joint_names(
    joint_state: JointState, required_joint_names: Sequence[str]
) -> tuple[str, ...]:
    """Return required joint names absent from a command."""
    available = set(joint_state.name)
    return tuple(name for name in required_joint_names if name not in available)
