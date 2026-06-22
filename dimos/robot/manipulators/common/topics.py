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

"""Shared control naming conventions for manipulator blueprints."""

from __future__ import annotations

from typing import TypeAlias

FrameId: TypeAlias = str
TaskName: TypeAlias = str

COORDINATOR_FRAME_ID: FrameId = "coordinator"
CARTESIAN_IK_TASK_NAME: TaskName = "cartesian_ik_arm"
DEFAULT_TRAJECTORY_TASK_NAME: TaskName = "traj_arm"


def trajectory_task_name(hardware_id: str) -> TaskName:
    return f"traj_{hardware_id}"
