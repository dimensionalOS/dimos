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

"""Manipulation-domain failure codes for ``ToolResult``.

Cross-domain codes (``ROBOT_NOT_FOUND``, ``INVALID_INPUT``, ``EXECUTION_FAILED``,
``EXECUTION_TIMEOUT``, ...) live in ``dimos.agents.tool_result.CommonToolError``.
This module owns codes specific to manipulation tools.

Both aliases are plain ``Literal`` types — strings at runtime, constrained by
the type checker. Use ``ToolResult[ManipulationToolError]`` on a tool to
allow either common or manipulation codes.
"""

from typing import Literal

from dimos.agents.tool_result import CommonToolError

ManipulationError = Literal[
    "NO_PRIOR_POSE",
    "OBJECT_NOT_DETECTED",
    "IK_FAILED",
    "PLANNING_FAILED",
    "COLLISION_AT_START",
    "GRASP_GENERATION_FAILED",
    "GRASP_ATTEMPTS_EXHAUSTED",
    "GRIPPER_FAILED",
    "WORLD_MONITOR_UNAVAILABLE",
]

# Union of codes a manipulation tool may emit (common + manipulation-specific).
ManipulationToolError = CommonToolError | ManipulationError
