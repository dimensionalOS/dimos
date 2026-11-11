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

"""Closed-loop R1 Pro simulation on canonical JointState command/feedback ports.

The declaration and strict joint bounds match the real connection. Upper-body
positions follow accepted targets at tracking_speed; jaws follow at the declared
opening speed. Base actions and measured actuator rates stay in the body frame;
pose is integrated and published separately as Odometry. No ROS, vendor stack,
CAN, camera or robot is started. Outage hooks suppress measured publications;
they never fabricate a recovered reading or resend a cached jaw target.
"""

from __future__ import annotations

import math
import threading
from threading import Thread
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import (
    joint_state_to_values,
)
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_GRIPPER_SIDES,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnectionConfig,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 18


class FakeR1ProConnectionConfig(R1ProConnectionConfig):
    """The contract fields ``R1ProConnectionConfig`` defaults to, minus every
    knob that only talks to the vendor stack, plus the simulation's rates.

    Attributes:
        tracking_speed: rad/s the simulated joints follow targets at (the
            vendor tracker's cap).
        gripper_speed_m_s: m/s of jaw opening the simulated jaws follow
            targets at (0.1 = the full 0-100 mm range in one second).
        publish_rate_hz: How often the simulated joint_state goes out.
        frame_id: The robot's root frame, stamped on feedback.
    """

    tracking_speed: float = Field(default=0.5)
    gripper_speed_m_s: float = Field(default=0.1)
    publish_rate_hz: float = Field(default=100.0)
    frame_id: str = Field(default="base_link")
    #: Joints the simulation starts at, by coordinator name
    #: (``r1pro/<joint>``); unlisted joints start at zero. Used to begin
    #: teleop from a reachable posture instead of the stowed all-zero pose.
    initial_positions: dict[str, float] = Field(default_factory=dict)


class FakeR1ProConnection(ConnectionModule):
    """Simulated R1 Pro behind the same ports as ``R1ProConnection``.

    ``position_command`` carries the upper-body targets and jaw openings;
    ``velocity_command`` carries canonical world-frame planar velocities. Feedback
    is published at ``config.publish_rate_hz``: one message for the 18
    joints, one position-only message per jaw whose feedback is enabled —
    the same two-message layout the real module produces.
    """

    config: FakeR1ProConnectionConfig

    # Control inputs.
    position_command: In[JointState]
    velocity_command: In[JointState]

    # Whole-body feedback (same contract as the real module).
    joint_state: Out[JointState]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        # Simulated state: measured positions, and the targets they follow.
        self._q = [0.0] * _NUM_MOTORS
        bare = [name.split("/", 1)[-1] for name in R1PRO_UPPER_BODY_JOINTS]
        for joint, value in (self.config.initial_positions or {}).items():
            name = joint.split("/", 1)[-1]
            if name not in bare:
                raise ValueError(f"initial_positions: unknown joint {joint!r}")
            self._q[bare.index(name)] = float(value)
        initial = {
            f"{name}/position": q for name, q in zip(R1PRO_UPPER_BODY_JOINTS, self._q, strict=True)
        }
        from dimos.control.contract.validate import Rejected, validate_command

        result = validate_command(initial, self.limits)
        if isinstance(result, Rejected):
            raise ValueError(f"initial_positions: {result.detail}")
        self._target_q: list[float] | None = None
        self._speed = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        #: Planar chassis pose (x, y, yaw) integrated from ``_speed``, for
        #: Published as canonical planar position feedback.
        self._base_pose = [0.0, 0.0, 0.0]
        self._gripper_measured: dict[str, float] = {
            side: 0.0 for side in R1PRO_GRIPPER_SIDES.values()
        }
        self._gripper_target: dict[str, float | None] = {
            side: None for side in R1PRO_GRIPPER_SIDES.values()
        }
        self._gripper_last_intent: dict[str, float | None] = dict.fromkeys(self._gripper_target)
        self._gripper_blocked: dict[str, float | None] = dict.fromkeys(self._gripper_target)
        # Feedback suppression knobs for outage tests.
        self._feedback_enabled = True
        self._gripper_feedback_enabled: dict[str, bool] = {
            side: True for side in R1PRO_GRIPPER_SIDES.values()
        }
        self._stop_event = threading.Event()
        self._sim_thread: Thread | None = None

    # Lifecycle

    def connect(self) -> None:
        """Nothing to open: the stand-in is already 'connected'."""

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.velocity_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.VELOCITY,)))
                )
            )
        )
        self._stop_event.clear()
        self._sim_thread = Thread(target=self._sim_loop, name="fake-r1pro-sim", daemon=True)
        self._sim_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._sim_thread is not None and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._sim_thread = None
        # Halts the stand-in (zero chassis, arms held) like the real module.
        super().stop()

    # Outage simulation (test/observability hooks; harmless in production)

    @rpc
    def base_state(self) -> dict[str, float]:
        """The simulated chassis: pose x/y/yaw plus the speed it is following.

        Integrated from accepted canonical world velocity commands; returned
        speed fields are the simulated body-frame vendor state.
        """
        with self._lock:
            x, y, yaw = self._base_pose
            return {
                "x": x,
                "y": y,
                "yaw": yaw,
                "vx": self._speed.linear.x,
                "vy": self._speed.linear.y,
                "wz": self._speed.angular.z,
            }

    @rpc
    def set_feedback_enabled(self, enabled: bool) -> None:
        """Suppress or resume the whole-body joint_state publication."""
        with self._lock:
            self._feedback_enabled = enabled

    @rpc
    def set_gripper_feedback_enabled(self, side: str, enabled: bool) -> None:
        """Suppress or resume one jaw's feedback message ("left"/"right")."""
        if side not in self._gripper_feedback_enabled:
            raise ValueError(f"unknown gripper side {side!r}")
        with self._lock:
            self._gripper_feedback_enabled[side] = enabled

    # The control contract

    def write(self, values: Values) -> None:
        """Latch one checked command: 18-joint frames, chassis speeds, jaws."""
        positions = [
            values.get(f"{joint}{SEPARATOR}{Interface.POSITION}")
            for joint in R1PRO_UPPER_BODY_JOINTS
        ]
        with self._lock:
            if any(target is not None for target in positions):
                if not all(target is not None for target in positions):
                    raise ValueError(
                        "a position command must cover all "
                        f"{_NUM_MOTORS} joints, got "
                        f"{sum(target is not None for target in positions)}"
                    )
                self._target_q = [float(q) for q in positions if q is not None]

            speeds = [values.get(f"{joint}/velocity") for joint in R1PRO_BASE_JOINTS]
            if any(speed is not None for speed in speeds):
                vx, vy, wz = (0.0 if speed is None else speed for speed in speeds)
                self._speed = Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))

            for joint, side in R1PRO_GRIPPER_SIDES.items():
                value = values.get(f"{joint}{SEPARATOR}{Interface.POSITION}")
                if value is None:
                    continue
                if not self._feedback_enabled or not self._gripper_feedback_enabled[side]:
                    self._gripper_blocked[side] = value
                    continue
                if value == self._gripper_blocked[side] or value == self._gripper_last_intent[side]:
                    continue
                initial_hold = (
                    self._gripper_last_intent[side] is None
                    and value == self._gripper_measured[side]
                )
                self._gripper_blocked[side] = None
                self._gripper_last_intent[side] = value
                if not initial_hold:
                    self._gripper_target[side] = value

    def halt(self) -> None:
        """Zero the chassis and hold the arms where they are; jaws untouched.

        Mirrors the real module: there is no verified vendor cancel, so a
        jaw keeps the target it was last told, and the arms are held at
        their last measured positions (safe before any command: the target
        simply becomes the current state).
        """
        with self._lock:
            self._speed = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            self._target_q = list(self._q)

    # Simulation loop

    def _sim_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            ts = time.time()
            with self._lock:
                previous_q = list(self._q)
                step = self.config.tracking_speed * period
                if self._target_q is not None:
                    self._q = [
                        _approach(q, target, step)
                        for q, target in zip(self._q, self._target_q, strict=True)
                    ]
                g_step = self.config.gripper_speed_m_s * period
                for side, target in self._gripper_target.items():
                    if target is not None:
                        self._gripper_measured[side] = _approach(
                            self._gripper_measured[side], target, g_step
                        )
                _integrate_base(self._base_pose, self._speed, period)
                q = list(self._q)
                q_velocity = [
                    (now - previous) / period for now, previous in zip(q, previous_q, strict=True)
                ]
                pose = list(self._base_pose)
                body_velocity = [self._speed.linear.x, self._speed.linear.y, self._speed.angular.z]
                grippers = [
                    (joint, self._gripper_measured[side])
                    for joint, side in R1PRO_GRIPPER_SIDES.items()
                    if self._gripper_feedback_enabled[side]
                ]
                body_enabled = self._feedback_enabled
                frame_id = self.config.frame_id

            if body_enabled:
                self.joint_state.publish(
                    JointState(
                        ts=ts,
                        frame_id=frame_id,
                        name=R1PRO_BASE_JOINTS,
                        velocity=body_velocity,
                    )
                )
                self.odometry.publish(
                    Odometry(
                        ts=ts,
                        frame_id=self.config.odom_frame_id,
                        child_frame_id=frame_id,
                        pose=Pose(
                            position=Vector3(pose[0], pose[1], 0.0),
                            orientation=Quaternion(
                                0.0, 0.0, math.sin(pose[2] / 2), math.cos(pose[2] / 2)
                            ),
                        ),
                        twist=Twist(
                            linear=Vector3(*body_velocity[:2], 0.0),
                            angular=Vector3(0.0, 0.0, body_velocity[2]),
                        ),
                    )
                )
                self.joint_state.publish(
                    JointState(
                        ts=ts,
                        frame_id=frame_id,
                        name=list(R1PRO_UPPER_BODY_JOINTS),
                        position=q,
                        velocity=q_velocity,
                    )
                )
            if grippers:
                self.joint_state.publish(
                    JointState(
                        ts=ts,
                        frame_id=frame_id,
                        name=[joint for joint, _ in grippers],
                        position=[measured for _, measured in grippers],
                    )
                )

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()


def _approach(current: float, target: float, max_step: float) -> float:
    """Move ``current`` toward ``target`` by at most ``max_step``."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


def _integrate_base(pose: list[float], speed: Twist, dt: float) -> None:
    """Integrate one body-frame ``Twist`` into a planar pose ``[x, y, yaw]``.

    Holonomic in the body frame: vx is forward, vy is left, wz is yaw rate.
    Yaw is kept in ``[-pi, pi]``.
    """
    x, y, yaw = pose
    x += (speed.linear.x * math.cos(yaw) - speed.linear.y * math.sin(yaw)) * dt
    y += (speed.linear.x * math.sin(yaw) + speed.linear.y * math.cos(yaw)) * dt
    yaw = math.atan2(math.sin(yaw + speed.angular.z * dt), math.cos(yaw + speed.angular.z * dt))
    pose[:] = [x, y, yaw]
