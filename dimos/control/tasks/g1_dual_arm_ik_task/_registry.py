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

TASK_FACTORIES = {
    "g1_dual_arm_ik": "dimos.control.tasks.g1_dual_arm_ik_task.g1_dual_arm_ik_task:create_task",
}

# Broadcast, not by_task_name: the quest module addresses hands as
# "dual_arm_ik/left|right" and the handler routes on the suffix itself.
TASK_CONSUMES = {
    "g1_dual_arm_ik": {
        "coordinator_cartesian_command": ("on_cartesian_command", "broadcast"),
        "teleop_buttons": ("on_teleop_buttons", "broadcast"),
    },
}
