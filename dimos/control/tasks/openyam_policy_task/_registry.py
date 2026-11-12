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

"""Registry manifest for the OpenYAM history policy task."""

TASK_FACTORIES = {
    "openyam_policy": ("dimos.control.tasks.openyam_policy_task.openyam_policy_task:create_task"),
}

TASK_CONSUMES: dict[str, dict[str, tuple[str, str]]] = {
    "openyam_policy": {
        "teleop_buttons": ("on_teleop_buttons", "broadcast"),
        "right_cartesian_command": ("on_right_cartesian_command", "direct"),
    },
}

TASK_EXPOSES: dict[str, list[str]] = {
    "openyam_policy": [
        "disarm",
        "engage",
        "release",
        "reset_runtime_state",
        "set_dry_run",
        "state_snapshot",
        "stop",
    ],
}
