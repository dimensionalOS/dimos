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

"""One R1 control owner with explicit leader/WebXR input handover."""

from dataclasses import replace
import threading
import time
from typing import Any, Literal, Protocol

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import _teleop_task
from dimos.robot.galaxea.r1pro.config import R1PRO_DESCRIPTION_SOURCE
from dimos.robot.galaxea.r1pro.joints import PASSIVE_JOINTS
from dimos.robot.galaxea.r1pro.joylo import UPPER
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator, joylo_tasks
from dimos.robot.galaxea.r1pro.torso_height_task import TORSO_LIFT_JOINTS
from dimos.spec.utils import Spec
from dimos.teleop.webxr.controller_types import Buttons

TeleopSource = Literal["leader", "webxr"]
_LEADER_PORTS = frozenset(
    ("leader_joint_command", "joylo_twist_command", "left_gripper_command", "right_gripper_command")
)
_WEBXR_PORTS = frozenset(
    (
        "left_cartesian_command",
        "right_cartesian_command",
        "teleop_buttons",
        "lift_rate",
        "webxr_twist_command",
        "webxr_left_gripper",
        "webxr_right_gripper",
    )
)


def shared_teleop_tasks(velocity_limits: dict[str, float]) -> list[TaskConfig]:
    """Reuse CC's IK with the 2025 hardware geometry already used by our leader stack.

    Pink consumes kinematics, without loading the public URDF's unavailable meshes.
    The portable source resolves independently on the robot host.
    """
    model = (
        RobotModel.from_file(R1PRO_DESCRIPTION_SOURCE / "R1Pro" / "urdf_r1pro_2025.urdf")
        .with_default_joint_acceleration_limit(2.0)
        .with_fixed_joints(*PASSIVE_JOINTS)
        .with_renamed_joints({name.removeprefix("r1pro/"): name for name in UPPER})
    )
    ik = _teleop_task()
    return [
        *joylo_tasks(velocity_limits),
        replace(
            ik,
            params={
                **ik.params,
                "robot_model": RobotModelConfig(model=model, joint_names=list(UPPER)),
                "max_joint_velocity_rad_s": 1.0,
            },
        ),
        TaskConfig(
            name="torso_height",
            type="r1pro_torso_height",
            priority=20,
            joint_names=list(TORSO_LIFT_JOINTS),
            params={"link_lengths": (0.4, 0.3)},
        ),
    ]


class LeaderPauseSpec(Spec, Protocol):
    def pause(self) -> None: ...


class R1ProTeleopCoordinatorConfig(ControlCoordinatorConfig):
    teleop_source: TeleopSource = "leader"


class R1ProTeleopCoordinator(R1ProJoyloCoordinator):
    """Gate device inputs before shared tasks, arbitration, limits and recording.

    Switching clears old targets and requires a new leader enable or released
    headset clutches. Browser RPC control retains its existing task interfaces.
    """

    config: R1ProTeleopCoordinatorConfig
    _leader: LeaderPauseSpec
    left_cartesian_command: In[PoseStamped]
    right_cartesian_command: In[PoseStamped]
    teleop_buttons: In[Buttons]
    lift_rate: In[Float32]
    webxr_twist_command: In[Twist]
    webxr_left_gripper: In[Float32]
    webxr_right_gripper: In[Float32]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._input_lock = threading.RLock()
        self._teleop_source = self.config.teleop_source
        self._webxr_ready = False
        self._webxr_buttons = Buttons()
        self._leader_ready = False
        self._selected_at = time.time()

    def _setup_from_config(self) -> None:
        super()._setup_from_config()
        # Both devices feed the same jaw/base tasks. Device streams stay distinct
        # until dispatch, so an unselected device cannot overwrite a target.
        with self._task_lock:
            for name, task_type, mapping in (
                ("vel_chassis", "twist_velocity", {"twist_command": "webxr_twist_command"}),
                ("left_gripper", "gripper", {"gripper_command": "webxr_left_gripper"}),
                ("right_gripper", "gripper", {"gripper_command": "webxr_right_gripper"}),
            ):
                self._register_routes(self._tasks[name], task_type, mapping)

    @rpc
    def get_teleop_source(self) -> dict[str, Any]:
        with self._input_lock:
            return {"source": self._teleop_source, "webxr_ready": self._webxr_ready}

    @rpc
    def select_teleop_source(self, source: TeleopSource) -> dict[str, Any]:
        """Select leader or WebXR; clear intent and require fresh device engagement."""
        if source not in ("leader", "webxr"):
            raise ValueError("teleop source must be 'leader' or 'webxr'")
        with self._input_lock, self._estop_lock:
            if source == self._teleop_source:
                return self.get_teleop_source()
            self._leader.pause()
            loop = self._tick_loop
            running = bool(loop and loop.is_running)
            # A handover is rare. Finish the current tick before resetting tasks
            # and holds, so an already-computed old command cannot escape later.
            if running and loop is not None:
                loop.stop()
            try:
                self._set_tasks_estop(True)
                with self._hardware_lock:
                    for connection in self._connections.values():
                        connection.drop_holds()
                    self._motion_limiter.reset()
                self._teleop_source = source
                self._selected_at = time.time()
                self._webxr_ready = False
                self._webxr_buttons = Buttons()
                self._leader_ready = False
                self._set_tasks_estop(self._estopped)
            finally:
                if running and loop is not None:
                    loop.start()
            return self.get_teleop_source()

    def _dispatch(self, stream: str, msg: Any) -> None:
        with self._input_lock:
            if stream in _LEADER_PORTS | _WEBXR_PORTS:
                if isinstance(msg, (JointState, PoseStamped)) and msg.ts < self._selected_at:
                    return
            if stream in _LEADER_PORTS:
                if self._teleop_source != "leader":
                    return
                if stream == "leader_joint_command":
                    self._leader_ready = True
                # Jaw/Twist messages have no timestamp. Require a fresh, stamped
                # leader target before accepting these after a handover.
                if not self._leader_ready:
                    return
            if stream in _WEBXR_PORTS:
                if self._teleop_source != "webxr":
                    return
                if stream == "teleop_buttons":
                    self._webxr_buttons = msg
                    if not self._webxr_ready:
                        self._webxr_ready = not (
                            msg.left_grip or msg.right_grip or msg.right_thumbstick
                        )
                    if not msg.right_thumbstick:
                        super()._dispatch("webxr_twist_command", Twist.zero())
                        super()._dispatch("lift_rate", Float32(data=0.0))
                if not self._webxr_ready:
                    return
                if stream in ("webxr_twist_command", "lift_rate"):
                    if not self._webxr_buttons.right_thumbstick:
                        return
                if stream == "webxr_left_gripper" and not self._webxr_buttons.left_grip:
                    return
                if stream == "webxr_right_gripper" and not self._webxr_buttons.right_grip:
                    return
            super()._dispatch(stream, msg)
