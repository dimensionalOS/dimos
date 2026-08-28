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

"""Guarded high-level velocity connection for the Deep Robotics Lynx M20."""

from __future__ import annotations

import math
from threading import RLock
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.deeprobotics.m20.constants import (
    MAX_ANGULAR_Z_RAD_S,
    MAX_LINEAR_X_M_S,
    MAX_LINEAR_Y_M_S,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class M20ConnectionConfig(ModuleConfig):
    """Limits for commands sent to the M20's high-level navigation interface."""

    max_linear_x: float = Field(default=MAX_LINEAR_X_M_S, gt=0.0)
    max_linear_y: float = Field(default=MAX_LINEAR_Y_M_S, gt=0.0)
    max_angular_z: float = Field(default=MAX_ANGULAR_Z_RAD_S, gt=0.0)
    require_command_ready: bool = True


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def sanitize_twist(twist: Twist, config: M20ConnectionConfig) -> Twist:
    """Return a finite, planar Twist bounded by the configured M20 limits."""
    values = (twist.linear.x, twist.linear.y, twist.angular.z)
    if not all(math.isfinite(value) for value in values):
        return Twist.zero()
    return Twist(
        linear=Vector3(
            _clamp(twist.linear.x, config.max_linear_x),
            _clamp(twist.linear.y, config.max_linear_y),
            0.0,
        ),
        angular=Vector3(0.0, 0.0, _clamp(twist.angular.z, config.max_angular_z)),
    )


class M20Connection(Module):
    """Expose the planner-facing M20 command surface with an explicit safety gate.

    The hardware bridge owns ROS 2/DrDDS and the command watchdog. This module
    remains transport-agnostic: it accepts the standard DimOS ``cmd_vel`` stream,
    rejects it while disarmed, bounds planar commands while armed, and emits
    ``safe_cmd_vel`` for the robot-local bridge.

    Arming never changes robot motion state, gait, vendor services, or charging
    state. Those remain explicit deployment/operator responsibilities.
    """

    config: M20ConnectionConfig

    cmd_vel: In[Twist]
    command_ready: In[Bool]
    lidar_ready: In[Bool]
    safe_cmd_vel: Out[Twist]
    armed: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._armed = False
        self._command_ready = False
        self._lidar_ready = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.command_ready.subscribe(self._on_command_ready)))
        self.register_disposable(Disposable(self.lidar_ready.subscribe(self._on_lidar_ready)))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.safe_cmd_vel.publish(Twist.zero())
        self.armed.publish(Bool(False))

    @rpc
    def stop(self) -> None:
        self.disarm()
        super().stop()

    @rpc
    def arm(self) -> bool:
        """Allow bounded planner commands to reach the M20 ROS bridge."""
        with self._lock:
            if self.config.require_command_ready and (
                not self._command_ready or not self._lidar_ready
            ):
                logger.warning("M20 command gate refused arm: native bridge or lidar is not ready")
                return False
            self._armed = True
        self.armed.publish(Bool(True))
        logger.warning("M20 command gate armed")
        return True

    @rpc
    def disarm(self) -> bool:
        """Block commands and publish an immediate zero velocity."""
        with self._lock:
            self._armed = False
        self.safe_cmd_vel.publish(Twist.zero())
        self.armed.publish(Bool(False))
        logger.info("M20 command gate disarmed")
        return True

    @rpc
    def is_armed(self) -> bool:
        """Return whether nonzero commands may pass through the gate."""
        with self._lock:
            return self._armed

    @rpc
    def is_command_ready(self) -> bool:
        """Return whether the native bridge reports a fresh, lidar-safe command path."""
        with self._lock:
            return self._command_ready

    @rpc
    def is_lidar_ready(self) -> bool:
        """Return whether the native bridge has received a valid cloud within its timeout."""
        with self._lock:
            return self._lidar_ready

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Forward a bounded planar velocity when armed.

        ``duration`` is accepted for connection compatibility. Command lifetime
        is enforced by the native bridge's monotonic watchdog.
        """
        del duration
        with self._lock:
            enabled = self._armed and (self._command_ready or not self.config.require_command_ready)
        command = sanitize_twist(twist, self.config) if enabled else Twist.zero()
        self.safe_cmd_vel.publish(command)
        return enabled

    @rpc
    def stop_movement(self) -> None:
        """Publish an immediate zero velocity without changing the arm state."""
        self.safe_cmd_vel.publish(Twist.zero())

    def _on_command_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        with self._lock:
            was_armed = self._armed
            self._command_ready = ready
            if not ready:
                self._armed = False
        if was_armed and not ready:
            self.safe_cmd_vel.publish(Twist.zero())
            self.armed.publish(Bool(False))
            logger.warning("M20 command gate disarmed: native bridge lost readiness")

    def _on_lidar_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        with self._lock:
            was_armed = self._armed
            changed = self._lidar_ready != ready
            self._lidar_ready = ready
            if not ready:
                self._armed = False
        if was_armed and not ready:
            self.safe_cmd_vel.publish(Twist.zero())
            self.armed.publish(Bool(False))
            logger.warning("M20 command gate disarmed: lidar stream became stale")
        elif changed and ready:
            logger.info("M20 lidar stream became ready")
