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

from dimos.manipulation.visualization.viser.panel_backend import feasibility_status
from dimos.manipulation.visualization.viser.state import FeasibilityStatus


@pytest.mark.parametrize(
    ("status", "success", "collision_free", "expected"),
    [
        ("FEASIBLE", True, True, FeasibilityStatus.FEASIBLE),
        ("COLLISION", False, False, FeasibilityStatus.COLLISION),
        ("COLLISION_AT_START", False, False, FeasibilityStatus.COLLISION),
        ("COLLISION_AT_GOAL", False, False, FeasibilityStatus.COLLISION),
        ("NO_SOLUTION", False, False, FeasibilityStatus.IK_FAILED),
        ("SINGULARITY", False, False, FeasibilityStatus.IK_FAILED),
        ("JOINT_LIMITS", False, False, FeasibilityStatus.IK_FAILED),
        ("TIMEOUT", False, False, FeasibilityStatus.IK_FAILED),
        ("IK_SUCCEEDED", False, False, FeasibilityStatus.INVALID),
    ],
)
def test_gui_feasibility_status_uses_exact_status_mapping(
    status: str,
    success: bool,
    collision_free: bool,
    expected: FeasibilityStatus,
) -> None:
    assert feasibility_status(status, success, collision_free) == expected
