# Copyright 2025 Dimensional Inc.
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

"""Base framework for generalized manipulator drivers.

This package provides the foundation for building manipulator drivers
that work with any robotic arm (XArm, Piper, UR, Franka, etc.).
"""

from .components import StandardMotionComponent, StandardServoComponent, StandardStatusComponent
from .driver import BaseManipulatorDriver, Command
from .sdk_interface import BaseManipulatorSDK, ManipulatorInfo
from .spec import ManipulatorCapabilities, ManipulatorDriverSpec, RobotState
from .utils import SharedState

__all__ = [
    # Driver
    "BaseManipulatorDriver",
    # SDK Interface
    "BaseManipulatorSDK",
    "Command",
    "ManipulatorCapabilities",
    # Spec
    "ManipulatorDriverSpec",
    "ManipulatorInfo",
    "RobotState",
    # Utils
    "SharedState",
    # Components
    "StandardMotionComponent",
    "StandardServoComponent",
    "StandardStatusComponent",
]
