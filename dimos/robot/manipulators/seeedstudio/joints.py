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

"""Seeed Studio reBot B601-DM joint names without hardware or model imports."""

SEEEDSTUDIO_HARDWARE_ID = "arm"
SEEEDSTUDIO_DOF = 6
# The vendor URDF's joint names, so the adapter's gravity model and the
# planner share them without renaming.
SEEEDSTUDIO_ARM_JOINTS = tuple(f"joint{i}" for i in range(1, SEEEDSTUDIO_DOF + 1))
SEEEDSTUDIO_GRIPPER_JOINT = f"{SEEEDSTUDIO_HARDWARE_ID}/gripper"
SEEEDSTUDIO_JOINTS = [*SEEEDSTUDIO_ARM_JOINTS, SEEEDSTUDIO_GRIPPER_JOINT]
