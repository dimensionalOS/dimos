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

"""In-memory R1 Pro stand-in on the same control contract as ``R1ProConnection``.

No ROS 2, no vendor stack, no cameras, no lidar, no CAN, no robot. The same
``position_command``/``cmd_vel`` inputs and ``joint_state`` output, the same
contract names (``r1pro/<joint>``), the same limits table
(``R1PRO_CONNECTION_MODEL``), and the same command semantics: a position
command must cover all 18 upper-body joints, chassis speeds are clamped to
the same ``base_range``, and once a command was accepted, going
``deadman_timeout_s`` without another halts the stand-in (chassis zeroed,
arms held where they were last measured, grippers left alone — there is no
verified vendor cancel, exactly as on the real module).

The feedback loop is a real closed loop, not a conversion stub: accepted
targets are integrated toward at ``tracking_speed`` (rad/s, the vendor
tracker's cap) and ``gripper_speed_m_s`` (m/s of jaw opening), and the
integrated state is published back on ``joint_state`` at
``publish_rate_hz`` — the 18 upper-body joints on every frame, and each jaw
on its own position-only frame, mirroring the real module's two-message
layout. Chassis speeds are integrated into an in-memory planar pose in the
body frame (x, y, yaw), readable with ``base_state()``; nothing is published
for it, so a thumbstick command is observable without adding a dependency on
a real odometry consumer. ``set_feedback_enabled`` and
``set_gripper_feedback_enabled`` suppress those publications so tests can
drive feedback-outage behavior through the coordinator exactly as the bus
would carry it.
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
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import (
    TWIST_INTERFACES,
    joint_state_to_values,
    twist_to_values,
)
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_CONNECTION_MODEL, R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE,
    R1PRO_GRIPPER_SIDES,
    R1PRO_UPPER_BODY_JOINTS,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 18


def _r1pro_base_range() -> Twist:
    vx, vy, wz = R1PRO_PLANAR_BASE.velocity_limits
    return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))


class FakeR1ProConnectionConfig(ConnectionConfig):
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

    source: str = "r1pro"
    model: RobotModel | None = R1PRO_CONNECTION_MODEL
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    base_range: Twist | None = Field(default_factory=_r1pro_base_range)
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

    ``position_command`` carries the 18 upper-body targets in rad named
    ``r1pro/<joint>`` plus each jaw's opening in metres named
    ``r1pro/<side>_gripper``; ``cmd_vel`` carries chassis speeds. Feedback
    is published at ``config.publish_rate_hz``: one message for the 18
    joints, one position-only message per jaw whose feedback is enabled —
    the same two-message layout the real module produces.
    """

    config: FakeR1ProConnectionConfig

    # Control inputs.
    position_command: In[JointState]
    cmd_vel: In[Twist]

    # Whole-body feedback (same contract as the real module).
    joint_state: Out[JointState]

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
        self._target_q: list[float] | None = None
        self._speed = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        #: Planar chassis pose (x, y, yaw) integrated from ``_speed``, for
        #: observation only: it is not published on any port.
        self._base_pose = [0.0, 0.0, 0.0]
        self._gripper_measured: dict[str, float] = {
            side: 0.0 for side in R1PRO_GRIPPER_SIDES.values()
        }
        self._gripper_target: dict[str, float | None] = {
            side: None for side in R1PRO_GRIPPER_SIDES.values()
        }
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
                self.cmd_vel.subscribe(lambda msg: self.command(twist_to_values(msg, R1PRO_BASE)))
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

        Integrated in memory from the ``cmd_vel`` commands the stand-in
        accepted; for observing the thumbstick chain without a real base.
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

            speeds = [values.get(f"{R1PRO_BASE}{SEPARATOR}{axis}") for axis in TWIST_INTERFACES]
            if any(speed is not None for speed in speeds):
                vx, vy, wz = (0.0 if speed is None else speed for speed in speeds)
                self._speed = Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))

            for joint, side in R1PRO_GRIPPER_SIDES.items():
                value = values.get(f"{joint}{SEPARATOR}{Interface.POSITION}")
                if value is not None:
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
                        name=list(R1PRO_UPPER_BODY_JOINTS),
                        position=q,
                        velocity=[0.0] * _NUM_MOTORS,
                        effort=[0.0] * _NUM_MOTORS,
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
