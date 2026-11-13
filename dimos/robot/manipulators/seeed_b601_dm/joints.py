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

"""Seeed B601-DM joint names without hardware or model imports."""

B601_DM_DOF = 6
# The vendor URDF's joint names, so the connection's gravity model and the
# planner share them without renaming.
B601_DM_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, B601_DM_DOF + 1))
# The jaw's URDF finger joint, named under the robot "arm".
B601_DM_GRIPPER_JOINT = "arm/finger_left"
B601_DM_JOINTS = (*B601_DM_ARM_JOINTS, B601_DM_GRIPPER_JOINT)
