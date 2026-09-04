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

"""Teleop-over-navigation arbitration for Alfred's base, as a ControlCoordinator.

``AlfredHighLevel`` owns the Portal connection and is the only publisher of the wheel
odometry dimSLAM fuses, so this coordinator arbitrates rather than drives: navigation and
teleop each land on their own velocity task, per-joint priority picks a winner every tick,
and the winning velocities go back out on ``cmd_vel`` for AlfredHighLevel to send.

Teleop outranks navigation while the operator is driving. Its task goes inactive one
``tele_cooldown`` after the last teleop message, and that is what hands the base back.

Neither task holds zeros when it falls idle: a task that zeroes stays active forever and
would turn every tick into a Portal RPC. Going inactive stops the writes instead, and
``AlfredHighLevel``'s own cmd_vel watchdog is what actually stops the platform.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

BASE_HARDWARE_ID = "base"
BASE_JOINTS = make_twist_base_joints(BASE_HARDWARE_ID)

NAV_PRIORITY = 10
TELE_PRIORITY = 20

# Matches MovementManagerConfig.tele_cooldown_sec, the behaviour this replaces.
DEFAULT_TELE_COOLDOWN = 1.0


class PublishingTwistBase:
    """A twist base whose "hardware" is a stream, for platforms an existing module drives."""

    def __init__(self, publish: Callable[[Twist], None], dof: int = 3, **_: object) -> None:
        self._publish = publish
        self._dof = dof
        self._velocities = [0.0] * dof
        self._connected = False
        self._enabled = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self.write_stop()
        self._connected = False
        self._enabled = False

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return self._dof

    def read_velocities(self) -> list[float]:
        return self._velocities.copy()

    def read_odometry(self) -> list[float] | None:
        """None: the driving module publishes real odometry on its own stream."""
        return None

    def write_velocities(self, velocities: list[float]) -> bool:
        if len(velocities) != self._dof:
            return False
        self._velocities = list(velocities)
        self._publish(
            Twist(
                linear=Vector3(velocities[0], velocities[1], 0.0),
                angular=Vector3(0.0, 0.0, velocities[2]),
            )
        )
        return True

    def write_stop(self) -> bool:
        return self.write_velocities([0.0] * self._dof)

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled


class AlfredBaseArbiter(ControlCoordinator):
    """Muxes navigation and teleop twists onto Alfred's single ``cmd_vel``."""

    tele_twist_command: In[Twist]

    # Dispatch targets for the twist mapper, one per source: sharing a port would hand
    # both tasks the same velocities and leave priority nothing to pick from.
    nav_joint_command: In[JointState]
    tele_joint_command: In[JointState]

    cmd_vel: Out[Twist]

    twist_ports = {
        "twist_command": "nav_joint_command",
        "tele_twist_command": "tele_joint_command",
    }

    def _create_twist_base_adapter(self, component: HardwareComponent) -> PublishingTwistBase:
        return PublishingTwistBase(self.cmd_vel.publish, dof=len(component.joints))


def base_hardware() -> HardwareComponent:
    """Alfred's holonomic base, commanded through ``AlfredBaseArbiter.cmd_vel``."""
    return HardwareComponent(
        hardware_id=BASE_HARDWARE_ID,
        hardware_type=HardwareType.BASE,
        joints=BASE_JOINTS,
        # AlfredBaseArbiter builds the adapter itself; no registry entry to name here.
        adapter_type="publishing_twist_base",
    )


def arbiter_tasks(tele_cooldown: float = DEFAULT_TELE_COOLDOWN) -> list[TaskConfig]:
    """One velocity task per twist source, teleop above navigation."""
    idle = {"zero_on_timeout": False}
    return [
        TaskConfig(
            name="nav",
            type="velocity",
            joint_names=BASE_JOINTS,
            priority=NAV_PRIORITY,
            stream_bind={"joint_command": "nav_joint_command"},
            params=idle,
        ),
        TaskConfig(
            name="tele",
            type="velocity",
            joint_names=BASE_JOINTS,
            priority=TELE_PRIORITY,
            stream_bind={"joint_command": "tele_joint_command"},
            params={**idle, "timeout": tele_cooldown},
        ),
    ]


def alfred_base_arbiter_blueprint(tele_cooldown: float = DEFAULT_TELE_COOLDOWN) -> Blueprint:
    """Blueprint for the arbiter, wired to Alfred's holonomic base."""
    return AlfredBaseArbiter.blueprint(
        hardware=[base_hardware()],
        tasks=arbiter_tasks(tele_cooldown),
    ).remappings(
        [
            (AlfredBaseArbiter, "twist_command", "nav_cmd_vel"),
            (AlfredBaseArbiter, "tele_twist_command", "tele_cmd_vel"),
        ]
    )
