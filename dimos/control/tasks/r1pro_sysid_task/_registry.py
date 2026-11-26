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

"""R1 Pro diagnostic excitation task card."""

TASK_FACTORIES = {"r1pro_sysid": "dimos.robot.galaxea.r1pro.sysid:create_task"}
TASK_CONSUMES = {"r1pro_sysid": {"r1pro_diagnostics": ("on_diagnostics", "broadcast")}}
TASK_EXPOSES = {"r1pro_sysid": ("run_trial", "abort", "status")}
