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

"""Pinned MakerMods Metal geometry and arm planning coordinates."""

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

METAL_SOURCE = RobotDescriptionSource(
    url="https://github.com/makermods-robotics/metal-python-ros",
    ref="bccf3c82a21453a7b1784a94966456e16ff0f008",
)
METAL_MODEL = RobotModel.from_file(
    METAL_SOURCE / "metal_ros2/src/metal_description/urdf/metal_with_gripper.urdf",
    package_paths={"metal_description": METAL_SOURCE / "metal_ros2/src/metal_description"},
).with_fixed_joints("joint7", "joint8")
METAL_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
METAL_HARDWARE_ID = "metal"
