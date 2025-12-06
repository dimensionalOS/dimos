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

"""
xArm Manipulator Driver Module

Real-time driver and components for xArm5/6/7 manipulators.
"""

from dimos.hardware.manipulators.xarm.spec import ArmDriverSpec
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.hardware.manipulators.xarm.xarm_wrapper import XArmSDKWrapper

__all__ = [
    "ArmDriverSpec",
    "XArmDriver",
    "XArmSDKWrapper",
]
