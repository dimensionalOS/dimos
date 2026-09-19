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

"""Host-visible contract for the isolated OmniGibson R1 Pro runtime."""

import json
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUTS
from dimos.simulation.behavior.setup import MARKER, PROJECT_DIR, runtime_environment
from dimos.simulation.behavior.types import BehaviorStatus, ControlMode, Operation, TaskSelection


class BehaviorConfig(IsolatedPythonModuleConfig):
    scene: str = "Rs_int"
    task: TaskSelection | None = None
    allow_task_changes: bool = True
    headless: bool = True
    image_width: int = Field(default=320, ge=16)
    image_height: int = Field(default=240, ge=16)
    action_hz: float = Field(default=30, gt=0)
    physics_hz: float = Field(default=120, gt=0)
    command_timeout: float = Field(default=0.2, gt=0)
    publish_scan: bool = False
    publish_semantic: bool = False
    max_depth: float = Field(default=5, gt=0)
    spawn_position: tuple[float, float, float] | None = None
    spawn_yaw: float = 0.0
    seed: int = 0
    max_episode_steps: int = Field(default=30000, ge=1)
    shutdown_timeout: float = 10.0
    # Engine initialization happens after the process handshake, in build().
    rpc_timeouts: dict[str, float] = Field(
        default_factory=lambda: {**DEFAULT_RPC_TIMEOUTS, "build": 1800.0}
    )


class BehaviorConnection(IsolatedPythonModule):
    """A continuous R1 Pro simulator with explicit task and control ownership."""

    project_dir = PROJECT_DIR
    implementation = "dimos_behavior.runtime:BehaviorRuntime"
    config: BehaviorConfig

    cmd_vel: In[Twist]
    joint_command: In[JointState]
    native_action: In[list[float]]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    left_wrist_image: Out[Image]
    left_wrist_depth: Out[Image]
    left_wrist_camera_info: Out[CameraInfo]
    right_wrist_image: Out[Image]
    right_wrist_depth: Out[Image]
    right_wrist_camera_info: Out[CameraInfo]
    semantic_image: Out[Image]
    joint_state: Out[JointState]
    odometry: Out[Odometry]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    registered_scan: Out[PointCloud2]
    left_wrist_scan: Out[PointCloud2]
    right_wrist_scan: Out[PointCloud2]
    status: Out[BehaviorStatus]

    def _runtime_env(self) -> dict[str, str]:
        env = runtime_environment(self.runtime_project)
        env.update(self.config.extra_env)
        return env

    def _run_prepare(self) -> None:
        marker = self.runtime_project / MARKER
        if not marker.exists() or not json.loads(marker.read_text()).get("complete"):
            raise RuntimeError(
                "BEHAVIOR needs one-time setup: python -m dimos.simulation.behavior.setup; "
                "see docs/capabilities/simulation/behavior.md"
            )
        super()._run_prepare()

    @rpc
    def describe(self) -> dict[str, Any]:
        """Describe robot controls, cameras, and physical/symbolic capabilities."""
        raise NotImplementedError

    @rpc
    def list_tasks(self) -> list[TaskSelection]:
        """List installed, pre-sampled task instances with their scenes."""
        raise NotImplementedError

    @rpc
    def list_scenes(self) -> list[str]:
        """List installed scene identifiers."""
        raise NotImplementedError

    @rpc
    def get_status(self) -> BehaviorStatus:
        """Read runtime, episode, operation, and control status."""
        raise NotImplementedError

    @rpc
    def get_observation(self) -> dict[str, Any]:
        """Read the last complete observation, tagged with episode and step."""
        raise NotImplementedError

    @rpc
    def get_ground_truth(self) -> dict[str, Any]:
        """Read simulator object poses and task goal evaluation, not perception estimates."""
        raise NotImplementedError

    @rpc
    def load_task(self, task: TaskSelection) -> str:
        """Start loading a pre-sampled task and return its operation ID."""
        raise NotImplementedError

    @rpc
    def reset_task(self) -> str:
        """Start resetting the current scene/task and return its operation ID."""
        raise NotImplementedError

    @rpc
    def get_operation(self, operation_id: str) -> Operation:
        """Read an operation result; IDs remain valid until process restart."""
        raise NotImplementedError

    @rpc
    def cancel_operation(self, operation_id: str) -> None:
        """Request cancellation; completion is acknowledged at an engine boundary."""
        raise NotImplementedError

    @rpc
    def take_control(self, mode: ControlMode) -> str:
        """Cancel active motion and explicitly transfer whole-robot control."""
        raise NotImplementedError

    @rpc
    def stop_motion(self) -> str:
        """Cancel motion and hold the robot in primitive mode."""
        raise NotImplementedError

    @rpc
    def pause(self) -> str:
        """Cancel motion and pause simulation at its next execution boundary."""
        raise NotImplementedError

    @rpc
    def resume(self) -> str:
        """Resume a paused, nonterminal episode without replaying commands."""
        raise NotImplementedError

    @rpc
    def start_primitive(self, kind: str, primitive: str, target: str = "") -> str:
        """Start a physical or symbolic action in primitive control mode."""
        raise NotImplementedError
