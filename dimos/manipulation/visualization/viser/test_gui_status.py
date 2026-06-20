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

from __future__ import annotations

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.visualization.types import TargetEvaluation
from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.state import FeasibilityStatus


class StatusOnlyServer:
    pass


class StatusOnlyAdapter(InProcessViserAdapter):
    def __init__(self) -> None:
        pass


@pytest.mark.parametrize(
    ("result", "success", "collision_free", "expected"),
    [
        ({"status": "FEASIBLE"}, True, True, FeasibilityStatus.FEASIBLE),
        ({"status": "COLLISION"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "COLLISION_AT_START"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "COLLISION_AT_GOAL"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "NO_SOLUTION"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "SINGULARITY"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "JOINT_LIMITS"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "TIMEOUT"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "IK_SUCCEEDED"}, False, False, FeasibilityStatus.INVALID),
    ],
)
def test_gui_feasibility_status_uses_exact_status_mapping(
    result: TargetEvaluation,
    success: bool,
    collision_free: bool,
    expected: FeasibilityStatus,
) -> None:
    gui = ViserPanelGui(
        StatusOnlyServer(),
        StatusOnlyAdapter(),
        ViserVisualizationConfig(),
    )

    assert gui._feasibility_status(result, success, collision_free) == expected
