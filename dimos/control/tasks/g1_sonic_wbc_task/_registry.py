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
    "g1_sonic_wbc": "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task:create_task",
}

TASK_CONSUMES: dict[str, dict[str, tuple[str, str]]] = {
    "g1_sonic_wbc": {"twist_command": ("on_twist_command", "broadcast")},
}

TASK_EXPOSES: dict[str, list[str]] = {
    "g1_sonic_wbc": [
        "arm",
        "disarm",
        "set_dry_run",
        "reset_runtime_state",
        "start",
        "set_velocity_command",
        "set_locomotion_mode",
        "list_locomotion_modes",
        "set_base_height",
        "set_upper_body",
        "clear_upper_body",
        "state_snapshot",
        "play_motion_clip",
        "set_vr_3point",
        "clear_vr_3point",
        "stop_motion_clip",
        "list_motion_clips",
    ],
}
