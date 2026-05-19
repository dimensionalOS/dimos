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

"""Manipulation-domain failure codes for ``SkillResult``.

Cross-domain codes (``ROBOT_NOT_FOUND``, ``INVALID_INPUT``, ``EXECUTION_FAILED``,
``EXECUTION_TIMEOUT``, ...) live in ``dimos.agents.skill_result.CommonSkillError``.
This module owns codes specific to manipulation skills.

Both aliases are plain ``Literal`` types — strings at runtime, constrained by
the type checker. Use ``SkillResult[ManipulationSkillError]`` on a skill to
allow either common or manipulation codes.
"""

from typing import Literal

from dimos.agents.skill_result import CommonSkillError

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

# Union of codes a manipulation skill may emit (common + manipulation-specific).
ManipulationSkillError = CommonSkillError | ManipulationError
