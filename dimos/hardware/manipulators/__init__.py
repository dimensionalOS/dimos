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

"""Manipulator drivers for robotic arms.

Architecture: Protocol-based adapters for different manipulator hardware.
- spec.py: ManipulatorAdapter Protocol and shared types
- xarm/: XArm adapter
- piper/: Piper adapter
- mock/: Mock adapter for testing

Usage:
    >>> from dimos.hardware.manipulators.xarm import XArm
    >>> arm = XArm(ip="192.168.1.185")
    >>> arm.start()
    >>> arm.enable_servos()
    >>> arm.move_joint([0, 0, 0, 0, 0, 0])

Testing:
    >>> from dimos.hardware.manipulators.xarm import XArm
    >>> from dimos.hardware.manipulators.mock import MockAdapter
    >>> arm = XArm(adapter=MockAdapter())
    >>> arm.start()  # No hardware needed!
"""

from dimos.hardware.manipulators.spec import (
    ControlMode,
    DriverStatus,
    JointLimits,
    ManipulatorAdapter,
    ManipulatorInfo,
)

__all__ = [
    "ControlMode",
    "DriverStatus",
    "JointLimits",
    "ManipulatorAdapter",
    "ManipulatorInfo",
]
