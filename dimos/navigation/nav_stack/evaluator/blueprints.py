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

"""Simulate various inputs and outputs to test path planners.

dimos run path-planner-eval
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.nav_stack.evaluator.evaluator import Evaluator
from dimos.navigation.nav_stack.evaluator.straight_line_planner import StraightLinePlanner
from dimos.visualization.rerun.bridge import RerunBridgeModule

_POSE_MARKER_RADIUS = 0.4


def _render_start_pose(msg: Any) -> Any:
    import rerun as rr

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[0, 255, 0]],
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_goal_pose(msg: Any) -> Any:
    import rerun as rr

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[255, 0, 0]],
        radii=[_POSE_MARKER_RADIUS],
    )


path_planner_eval = autoconnect(
    Evaluator.blueprint(),
    StraightLinePlanner.blueprint(),
    RerunBridgeModule.blueprint(
        visual_override={
            "world/start_pose": _render_start_pose,
            "world/goal_pose": _render_goal_pose,
        }
    ),
)


__all__ = ["path_planner_eval"]
