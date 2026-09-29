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

"""OpenYAM joint names without hardware or model imports."""

OPENYAM_DOF = 6
OPENYAM_ARM_JOINTS = [f"yam_joint{i}" for i in range(1, OPENYAM_DOF + 1)]
OPENYAM_GRIPPER_JOINT = "arm/gripper"
OPENYAM_JOINTS = [*OPENYAM_ARM_JOINTS, OPENYAM_GRIPPER_JOINT]
