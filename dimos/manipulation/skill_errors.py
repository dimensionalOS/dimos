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
This module owns codes that are specific to manipulation skills.
"""

from enum import Enum, auto


class ManipulationError(Enum):
    """Manipulation-specific skill failure modes."""

    NO_PRIOR_POSE = auto()
    OBJECT_NOT_DETECTED = auto()
    IK_FAILED = auto()
    PLANNING_FAILED = auto()
    COLLISION_AT_START = auto()
    GRASP_GENERATION_FAILED = auto()
    GRASP_ATTEMPTS_EXHAUSTED = auto()
    GRIPPER_FAILED = auto()
    WORLD_MONITOR_UNAVAILABLE = auto()
