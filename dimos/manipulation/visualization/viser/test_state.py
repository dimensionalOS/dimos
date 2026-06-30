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

from dimos.manipulation.visualization.viser.state import (
    BackendConnectionStatus,
    PanelRuntime,
    PanelState,
    TargetStatus,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def test_panel_can_plan_from_fault_after_planning_failure() -> None:
    state = PanelState(
        selected_robot="arm",
        selected_group_ids=("arm:manipulator",),
        target_joints=JointState({"name": ["arm/j1"], "position": [1.0]}),
        runtime=PanelRuntime.RUNNING,
        backend_status=BackendConnectionStatus.READY,
        target_status=TargetStatus.FEASIBLE,
        manipulation_state="FAULT",
    )

    assert state.can_plan() is True
