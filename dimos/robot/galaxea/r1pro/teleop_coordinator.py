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

"""One R1 control owner with explicit input handover.

Three devices can move the arms, one at a time: the leader arms with their
Joy-Cons ("leader"), a headset's tracked controllers ("webxr"), and a headset's
hands with the Joy-Cons ("hands"). With the hands, JoyLo keeps every Joy-Con
function and gives the headset arm task the hands to follow.
"""

from dataclasses import replace
import math
import threading
import time
from typing import Any, Literal, Protocol

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import (
    R1PRO_VR_TASK_NAME,
    _teleop_task,
)
from dimos.robot.galaxea.r1pro.config import R1PRO_DESCRIPTION_SOURCE, R1PRO_HOME_POSE
from dimos.robot.galaxea.r1pro.joints import PASSIVE_JOINTS
from dimos.robot.galaxea.r1pro.joylo import TORSO, UPPER, ArmSource
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator, joylo_tasks
from dimos.robot.galaxea.r1pro.torso_height_task import TORSO_LIFT_JOINTS
from dimos.spec.utils import Spec
from dimos.teleop.webxr.controller_types import Buttons

TeleopSource = Literal["leader", "webxr", "hands"]
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
# JoyLo with the hands as its arm source: the hands to follow and the task's deadman.
_HANDS_PORTS = frozenset(("hand_left_pose", "hand_right_pose", "hand_buttons"))


def shared_teleop_tasks(
    velocity_limits: dict[str, float], *, command_model: RobotModel | None = None
) -> list[TaskConfig]:
    """Reuse CC's IK with the 2025 hardware geometry already used by our leader stack.

    Pink consumes kinematics, without loading the public URDF's unavailable meshes.
    The portable source resolves independently on the robot host.
    """
    model = (
        RobotModel.from_file(R1PRO_DESCRIPTION_SOURCE / "R1Pro" / "urdf_r1pro_2025.urdf")
        .with_default_joint_acceleration_limit(2.0)
        .with_fixed_joints(*PASSIVE_JOINTS)
    )
    if command_model is not None:
        # Keep this robot's kinematics, but solve inside the connection's accepted
        # range too. The hardware contract includes narrower joint soft stops.
        command_joints = {joint.name: joint for joint in command_model.load().joints}
        for joint in model.load().joints:
            name = f"r1pro/{joint.name}"
            if name not in UPPER:
                continue
            limit = command_joints[name]
            if joint.lower is None or joint.upper is None:
                continue
            model = model.with_joint_position_limits(
                joint.name,
                lower=max(joint.lower, limit.lower) if limit.lower is not None else joint.lower,
                upper=min(joint.upper, limit.upper) if limit.upper is not None else joint.upper,
            )
    model = model.with_renamed_joints({name.removeprefix("r1pro/"): name for name in UPPER})
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
    def set_arm_source(self, source: ArmSource) -> str: ...


class R1ProTeleopCoordinatorConfig(ControlCoordinatorConfig):
    teleop_source: TeleopSource = "leader"


class R1ProTeleopCoordinator(R1ProJoyloCoordinator):
    """Gate device inputs before shared tasks, arbitration, limits and recording.

    Switching clears old targets and requires a new enable press on the Joy-Con
    or released headset clutches. Browser RPC control retains its existing task
    interfaces.
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
    hand_left_pose: In[PoseStamped]
    hand_right_pose: In[PoseStamped]
    hand_buttons: In[Buttons]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._input_lock = threading.RLock()
        self._teleop_source = self.config.teleop_source
        self._webxr_ready = False
        self._webxr_buttons = Buttons()
        self._leader_ready = False
        self._selected_at = time.time()
        self._calibration_hold = False
        self._home_hold = False
        self._home_state = "idle"

    @rpc
    def home_arms(self) -> dict[str, Any]:
        """Pause teleop and return both arms to the bent-elbow home pose.

        Torso and grippers stay in place. This is an operator-supervised joint-space
        move, not an obstacle-avoiding plan: keep the arm paths clear. Cancel with
        cancel_arm_home. Tracking stays paused until the operator selects it again.
        """
        with self._input_lock, self._estop_lock:
            if self._estopped or self._calibration_hold:
                raise RuntimeError("Cannot home while emergency-stopped or calibrating")
            with self._hardware_lock:
                if any(not source.ready_for_control() for source in self._connections.values()):
                    raise RuntimeError("Fresh robot feedback is required to home")
            names = [name for name in UPPER if name not in TORSO]
            measured = self.get_joint_positions()
            if any(name not in measured or not math.isfinite(measured[name]) for name in names):
                raise RuntimeError("Fresh feedback from both arms is required to home")
            target = [R1PRO_HOME_POSE[name.split("/")[-1]] for name in names]
            duration = max(
                2.0, max(abs(q - measured[n]) for n, q in zip(names, target, strict=True)) / 0.35
            )
            self._leader.pause()
            loop = self._tick_loop
            running = bool(loop and loop.is_running)
            if running and loop is not None:
                loop.stop()
            try:
                self._set_tasks_estop(True)
                with self._hardware_lock:
                    for connection in self._connections.values():
                        connection.drop_holds()
                    self._motion_limiter.reset()
                self._home_hold = True
                self._home_state = "moving"
                self._webxr_ready = self._leader_ready = False
                self._selected_at = time.time()
                self._set_tasks_estop(False)
                result = self.execute_trajectory(
                    JointTrajectory(
                        joint_names=names,
                        points=[
                            TrajectoryPoint(positions=[measured[n] for n in names]),
                            TrajectoryPoint(time_from_start=duration, positions=target),
                        ],
                    )
                )
                if result.status is not TrajectoryExecutionStatus.ACCEPTED:
                    self._home_state = "refused"
                    raise RuntimeError(result.message)
            finally:
                if running and loop is not None:
                    loop.start()
            return self.get_arm_home_status()

    @rpc
    def cancel_arm_home(self) -> dict[str, Any]:
        """Cancel the home motion and hold; teleop requires a fresh enable action."""
        with self._input_lock, self._estop_lock:
            if self._home_hold:
                loop = self._tick_loop
                running = bool(loop and loop.is_running)
                if running and loop is not None:
                    loop.stop()
                try:
                    self._set_tasks_estop(True)
                    with self._hardware_lock:
                        for connection in self._connections.values():
                            connection.drop_holds()
                        self._motion_limiter.reset()
                    self._home_state = "cancelled"
                    self._set_tasks_estop(self._estopped)
                finally:
                    if running and loop is not None:
                        loop.start()
            return self.get_arm_home_status()

    @rpc
    def get_arm_home_status(self) -> dict[str, Any]:
        """Report measured home error and whether teleop needs explicit re-enabling."""
        with self._input_lock:
            measured = self.get_joint_positions()
            names = [name for name in UPPER if name not in TORSO]
            error = (
                max(abs(measured[n] - R1PRO_HOME_POSE[n.split("/")[-1]]) for n in names)
                if all(n in measured and math.isfinite(measured[n]) for n in names)
                else None
            )
            state = self._home_state
            completed = self._trajectory_task is not None and not self._trajectory_task.is_active()
            if state == "moving" and completed and error is not None and error < 0.03:
                state = "reached"
            if state == "moving" and self._estopped:
                state = "stopped"
            return {"state": state, "max_error_rad": error, "tracking_paused": self._home_hold}

    def _set_tasks_estop(self, estopped: bool) -> None:
        super()._set_tasks_estop(estopped or getattr(self, "_calibration_hold", False))

    @rpc
    def hold_for_arm_calibration(self, held: bool) -> None:
        """Hold all task outputs while calibrating, then require fresh engagement."""
        with self._input_lock, self._estop_lock:
            self._leader.pause()
            self._calibration_hold = held
            self._webxr_ready = self._leader_ready = False
            self._selected_at = time.time()
            self._set_tasks_estop(True)
            with self._hardware_lock:
                for connection in self._connections.values():
                    connection.drop_holds()
                self._motion_limiter.reset()
            self._set_tasks_estop(self._estopped)

    def _setup_from_config(self) -> None:
        super()._setup_from_config()
        # Both devices feed the same jaw/base tasks. Device streams stay distinct
        # until dispatch, so an unselected device cannot overwrite a target.
        with self._task_lock:
            for name, task_type, mapping in (
                ("vel_chassis", "twist_velocity", {"twist_command": "webxr_twist_command"}),
                ("left_gripper", "gripper", {"gripper_command": "webxr_left_gripper"}),
                ("right_gripper", "gripper", {"gripper_command": "webxr_right_gripper"}),
                # The headset arm task takes the hands from JoyLo as well.
                (
                    R1PRO_VR_TASK_NAME,
                    "teleop_ik",
                    {
                        "left_cartesian_command": "hand_left_pose",
                        "right_cartesian_command": "hand_right_pose",
                        "teleop_buttons": "hand_buttons",
                    },
                ),
            ):
                self._register_routes(self._tasks[name], task_type, mapping)

    @rpc
    def get_teleop_source(self) -> dict[str, Any]:
        with self._input_lock:
            return {"source": self._teleop_source, "webxr_ready": self._webxr_ready}

    @rpc
    def select_teleop_source(self, source: TeleopSource) -> dict[str, Any]:
        """Select the device that moves the arms; clear intent and require fresh engagement.

        Args:
            source: "leader" for the leader arms with their Joy-Cons, "webxr" for a
                headset's tracked controllers, "hands" for a headset's hands with
                the Joy-Cons.
        """
        if source not in ("leader", "webxr", "hands"):
            raise ValueError("teleop source must be 'leader', 'webxr' or 'hands'")
        with self._input_lock, self._estop_lock:
            if source == self._teleop_source and not self._home_hold:
                return self.get_teleop_source()
            self._leader.pause()
            # Before anything is reset: a profile that cannot take the hands refuses here.
            self._leader.set_arm_source("hands" if source == "hands" else "leader")
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
                self._home_hold = False
                self._home_state = "idle"
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
            if self._calibration_hold:
                return
            if self._home_hold and stream in _LEADER_PORTS | _WEBXR_PORTS | _HANDS_PORTS:
                return
            source = self._teleop_source
            if stream in _LEADER_PORTS | _WEBXR_PORTS | _HANDS_PORTS:
                if isinstance(msg, (JointState, PoseStamped)) and msg.ts < self._selected_at:
                    return
            if stream in _LEADER_PORTS:
                if source == "webxr":
                    return
                if stream == "leader_joint_command":
                    if source == "hands":
                        # Hands own the arms; JoyLo owns explicit torso commands.
                        # Reject mixed/old leader packets rather than letting them
                        # compete with the Cartesian task after a source switch.
                        if not msg.name or not set(msg.name) <= set(TORSO):
                            return
                    else:
                        self._leader_ready = True
                # Jaw/Twist messages have no timestamp. Require a fresh, stamped
                # leader target or hand pose before accepting these after a handover.
                if not self._leader_ready:
                    return
            if stream in _HANDS_PORTS:
                if source != "hands":
                    return
                if stream in ("hand_left_pose", "hand_right_pose"):
                    self._leader_ready = True
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
