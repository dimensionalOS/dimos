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

TASK_FACTORIES = {
    "operator_hold": "dimos.control.tasks.operator_hold_task.operator_hold_task:create_task",
}

# The coordinator needs an operator_hold_command input port for this stream;
# OperatorHoldCoordinator in the hybrid-teleop-sim blueprint declares it.
TASK_CONSUMES: dict[str, dict[str, tuple[str, str]]] = {
    "operator_hold": {"operator_hold_command": ("on_command", "direct")},
}

TASK_EXPOSES: dict[str, list[str]] = {
    "operator_hold": ["request", "acknowledge", "get_status"],
}
