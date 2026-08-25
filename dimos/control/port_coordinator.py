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

"""Coordinator subclasses for one manipulator connection module."""

from __future__ import annotations

from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.stream import In, Out
from dimos.hardware.manipulators.port_adapter import PortManipulatorAdapter
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.spec import JointLimits
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class _PortManipulatorCoordinatorMixin:
    motor_states: In[JointState]
    motor_command: Out[MotorCommandArray]

    def _create_adapter(self, component: HardwareComponent) -> ManipulatorAdapter:
        limits: JointLimits | None = None
        configured_limits: Any = component.adapter_kwargs.get("limits")
        if isinstance(configured_limits, JointLimits):
            limits = configured_limits
        return PortManipulatorAdapter(
            dof=len(component.joints),
            motor_states=self.motor_states,
            motor_command=self.motor_command,
            limits=limits,
        )


class PortControlCoordinator(_PortManipulatorCoordinatorMixin, ControlCoordinator):
    """ControlCoordinator whose single manipulator is a connection module."""


class PortTeleopControlCoordinator(
    _PortManipulatorCoordinatorMixin,
    TeleopControlCoordinator,
):
    """Teleop coordinator whose single manipulator is a connection module."""
