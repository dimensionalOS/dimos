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

"""The vendor's M20 URDF."""

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

M20_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url="https://github.com/DeepRoboticsLab/deep_robotics_model",
    ref="52352f5bbf2a30b5d02ece91900415188e7abc74",
)
# M20Connection's camera relay owns the camera frames (with their optical and rear yaw rotations).
M20_MODEL = RobotModel.from_file(
    M20_DESCRIPTION_SOURCE / "M20" / "urdf" / "M20.urdf"
).without_joint_subtrees("front_camera_joint", "rear_camera_joint")
