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
    "planar_base_trajectory": (
        "dimos.control.tasks.planar_base_trajectory_task.planar_base_trajectory_task:create_task"
    ),
}

TASK_EXPOSES: dict[str, list[str]] = {
    "planar_base_trajectory": ["execute", "cancel", "get_status"],
}
