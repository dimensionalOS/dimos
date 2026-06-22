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

"""OpenArm planner + coordinator blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator, planner
from dimos.robot.manipulators.openarm.blueprints.basic import (
    left_hw,
    mock_left,
    mock_right,
    openarm_task,
    right_hw,
)
from dimos.robot.manipulators.openarm.config import openarm_model_config

openarm_mock_planner_coordinator = autoconnect(
    planner(
        robots=[
            openarm_model_config("left"),
            openarm_model_config("right"),
        ],
    ),
    coordinator(
        hardware=[mock_left, mock_right],
        tasks=[
            openarm_task(mock_left),
            openarm_task(mock_right),
        ],
    ),
)

openarm_planner_coordinator = autoconnect(
    planner(
        robots=[
            openarm_model_config("left"),
            openarm_model_config("right"),
        ],
    ),
    coordinator(
        hardware=[left_hw, right_hw],
        tasks=[
            openarm_task(left_hw),
            openarm_task(right_hw),
        ],
    ),
)
