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

"""Registry manifest for the dual Metal PPO policy task."""

TASK_FACTORIES = {
    "metal_arm_policy": (
        "dimos.control.tasks.metal_arm_policy_task.metal_arm_policy_task:create_task"
    ),
}

TASK_CONSUMES: dict[str, dict[str, tuple[str, str]]] = {
    "metal_arm_policy": {
        "teleop_buttons": ("on_teleop_buttons", "broadcast"),
        "left_cartesian_command": ("on_left_cartesian_command", "direct"),
        "right_cartesian_command": ("on_right_cartesian_command", "direct"),
    },
}

TASK_EXPOSES: dict[str, list[str]] = {
    "metal_arm_policy": [
        "disarm",
        "engage",
        "release",
        "reset_runtime_state",
        "set_dry_run",
        "set_goal",
        "set_goal_pose24",
        "state_snapshot",
        "stop",
    ],
}
