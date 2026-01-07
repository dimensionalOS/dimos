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

"""XArm manipulator driver.

Usage:
    >>> from dimos.hardware.manipulators.xarm import XArm
    >>> arm = XArm(ip="192.168.1.185", dof=6)
    >>> arm.start()
    >>> arm.enable_servos()
    >>> arm.move_joint([0, 0, 0, 0, 0, 0])
"""

from dimos.hardware.manipulators.xarm.arm import XArm, XArmConfig, xarm
from dimos.hardware.manipulators.xarm.backend import XArmBackend
from dimos.hardware.manipulators.xarm.blueprints import (
    xarm5_servo,
    xarm7_servo,
    xarm7_trajectory,
    xarm_cartesian,
    xarm_servo,
    xarm_trajectory,
)

__all__ = [
    "XArm",
    "XArmBackend",
    "XArmConfig",
    "xarm",
    "xarm5_servo",
    "xarm7_servo",
    "xarm7_trajectory",
    "xarm_cartesian",
    # Blueprints
    "xarm_servo",
    "xarm_trajectory",
]
