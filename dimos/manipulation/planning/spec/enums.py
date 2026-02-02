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

"""Enumerations for manipulation planning."""

from enum import Enum, auto


class ObstacleType(Enum):
    """Type of obstacle geometry."""

    BOX = auto()
    SPHERE = auto()
    CYLINDER = auto()
    MESH = auto()


class IKStatus(Enum):
    """Status of IK solution."""

    SUCCESS = auto()
    NO_SOLUTION = auto()
    SINGULARITY = auto()
    JOINT_LIMITS = auto()
    COLLISION = auto()
    TIMEOUT = auto()


class PlanningStatus(Enum):
    """Status of motion planning."""

    SUCCESS = auto()
    NO_SOLUTION = auto()
    TIMEOUT = auto()
    INVALID_START = auto()
    INVALID_GOAL = auto()
    COLLISION_AT_START = auto()
    COLLISION_AT_GOAL = auto()
