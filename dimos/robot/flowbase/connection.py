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

"""A FlowBase holonomic base as a connection module, over Portal RPC."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_ADDRESS = "172.6.2.20:11323"

FLOWBASE_BASE_JOINTS: tuple[str, str, str] = (
    "base/base_forward",
    "base/base_lateral",
    "base/base_turn",
)
# The base firmware's own limiter: top speeds in m/s, m/s, rad/s, and the
# same numbers as accelerations in m/s², m/s², rad/s².
_TOP_SPEEDS = (0.8, 0.8, 3.0)
FLOWBASE_MODEL = RobotModel.from_root_link("base_link", name="flowbase").with_added_joints(
    *(
        JointDescription(
            name=name,
            type="continuous" if i == 2 else "prismatic",
            parent_link="base_link",
            child_link=f"flowbase_body_control_{i}",
            axis=axis,
            velocity=speed,
            acceleration=speed,
        )
        for i, (name, axis, speed) in enumerate(
            zip(
                FLOWBASE_BASE_JOINTS,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                _TOP_SPEEDS,
                strict=True,
            )
        )
    )
)
FLOWBASE_JOINT_COMMANDS: dict[str, tuple[Interface, ...]] = {
    name: (Interface.VELOCITY,) for name in FLOWBASE_BASE_JOINTS
}

_ODOMETRY_HZ = 50.0
_CONNECT_TIMEOUT_S = 5.0
_RPC_TIMEOUT_S = 0.5


class FlowBaseConfig(ConnectionConfig):
    """Where the FlowBase's controller listens.

    Attributes:
        address: The controller's Portal RPC address, "host:port".
        source: The robot's name, in front of each joint's name ("base/base_forward").
        odom_frame: The frame odometry is stamped in, e.g. "wheel_odom" when
            something else already publishes "odom".
    """

    address: str = DEFAULT_ADDRESS
    source: str = "base"
    odom_frame: str = "odom"
    model: RobotModel | None = FLOWBASE_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = FLOWBASE_JOINT_COMMANDS


def flowbase_joint_connection() -> ConnectionConfig:
    """The FlowBase as a coordinator sees it: robot name, model and velocity-only body joints.

    For a coordinator's ``connections``, which take only these fields, not the
    address or odometry frame in ``FlowBaseConfig``.
    """
    return ConnectionConfig(
        source="base", model=FLOWBASE_MODEL, joint_commands=FLOWBASE_JOINT_COMMANDS
    )


class FlowBaseConnection(ConnectionModule):
    """A FlowBase driven over Portal RPC at ``config.address``.

    Body speeds come in on ``velocity_command``: ``base/base_forward`` and
    ``base/base_lateral`` in m/s, ``base/base_turn`` in rad/s, x forward and
    y left. A Twist on ``cmd_vel`` does the same, with the same limits.
    ``odometry`` and ``joint_state`` (the measured body speeds) go out 50
    times a second. When commands stop, the base is sent zero speed.
    """

    required_imports = ("portal",)
    config: FlowBaseConfig

    cmd_vel: In[Twist]
    velocity_command: In[JointState]
    joint_state: Out[JointState]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._client: Any = None
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.velocity_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.VELOCITY,)))
                )
            )
        )
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_twist)))
        self._stop_reading.clear()
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="FlowBaseConnection-read"
        )
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None:
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        client, self._client = self._client, None
        if client is not None:
            client.close(timeout=_RPC_TIMEOUT_S)

    def connect(self) -> None:
        address = self.config.address
        client = _open_client(address)
        if not client.connect(timeout=_CONNECT_TIMEOUT_S):
            client.close(timeout=_RPC_TIMEOUT_S)
            raise ConnectionError(f"no FlowBase answers at {address}")
        self._client = client
        logger.info(f"Connected to FlowBase at {address}")

    def _on_twist(self, twist: Twist) -> None:
        """Drive at a Twist's forward and left speeds (m/s) and turn rate (rad/s)."""
        speeds = (twist.linear.x, twist.linear.y, twist.angular.z)
        self.command(
            {
                f"{joint}/{Interface.VELOCITY}": speed
                for joint, speed in zip(FLOWBASE_BASE_JOINTS, speeds, strict=True)
            }
        )

    def write(self, values: Values) -> None:
        vx, vy, wz = (
            values.get(f"{joint}/{Interface.VELOCITY}", 0.0) for joint in FLOWBASE_BASE_JOINTS
        )
        # The FlowBase's y axis points right, so lateral speed and turn rate flip sign.
        self._send(vx, -vy, -wz)

    def halt(self) -> None:
        self._send(0.0, 0.0, 0.0)

    def _send(self, vx: float, vy: float, wz: float) -> None:
        """Send body speeds (m/s, m/s, rad/s) in the FlowBase's own frame."""
        command = {"target_velocity": np.array([vx, vy, wz]), "frame": "local"}
        self._client.set_target_velocity(command).result(timeout=_RPC_TIMEOUT_S)

    def _read_loop(self) -> None:
        period = 1.0 / _ODOMETRY_HZ
        due = time.monotonic()
        previous: tuple[float, float, float, float] | None = None
        while not self._stop_reading.wait(max(0.0, due - time.monotonic())):
            due = max(due + period, time.monotonic())
            try:
                # Stamped before the call, so a slow answer does not move the stamp.
                ts = time.time()
                odom = self._client.get_odometry({}).result(timeout=_RPC_TIMEOUT_S)
                x, y = (float(v) for v in odom["translation"])
                # Odometry comes back in the same flipped frame the speeds go out in.
                y, yaw = -y, -float(odom["rotation"])
            except Exception as error:
                self._report(f"reading odometry failed: {type(error).__name__}: {error}")
                continue
            if previous is not None and ts > previous[0]:
                self._publish(ts, (x, y, yaw), previous)
            previous = (ts, x, y, yaw)

    def _publish(
        self,
        ts: float,
        pose: tuple[float, float, float],
        previous: tuple[float, float, float, float],
    ) -> None:
        """Publish the pose (x m, y m, yaw rad) and the body speeds since ``previous`` (ts, x, y, yaw)."""
        forward, left, turn = _body_speeds(previous[1:], pose, ts - previous[0])
        self.joint_state.publish(
            JointState(ts=ts, name=list(FLOWBASE_BASE_JOINTS), velocity=[forward, left, turn])
        )
        x, y, yaw = pose
        self.odometry.publish(
            Odometry(
                ts=ts,
                frame_id=self.frame(self.config.odom_frame),
                child_frame_id=self.frame("base_link"),
                pose=Pose(
                    position=Vector3(x, y, 0.0),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                ),
                twist=Twist(linear=Vector3(forward, left, 0.0), angular=Vector3(0.0, 0.0, turn)),
            )
        )


def _body_speeds(
    start: tuple[float, ...], end: tuple[float, float, float], dt: float
) -> tuple[float, float, float]:
    """The forward and left speeds (m/s) and turn rate (rad/s) that took the
    base from pose ``start`` to pose ``end`` (x m, y m, yaw rad) in ``dt`` s."""
    (x0, y0, yaw0), (x1, y1, yaw1) = start, end
    turn = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    # The heading halfway through, so turning does not tilt forward into left.
    heading = yaw0 + turn / 2.0
    dx, dy = x1 - x0, y1 - y0
    forward = math.cos(heading) * dx + math.sin(heading) * dy
    left = -math.sin(heading) * dx + math.cos(heading) * dy
    return forward / dt, left / dt, turn / dt


def _open_client(address: str) -> Any:
    """A Portal RPC client for the controller at ``address``, not yet connected."""
    # Imported here so blueprints that use this module load without portal installed.
    import portal

    return portal.Client(address)
