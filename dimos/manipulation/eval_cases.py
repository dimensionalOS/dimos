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

"""Per-arm evaluation case data for ``dimos.manipulation.eval``.

This is pure data. Tune existing arms or add a new one by editing this
file — the framework in ``eval.py`` does not need to change.

Each entry in ``CASES_BY_ARM`` is a list of ``Case`` objects. To add a
new arm:

1. Find the arm's "gripper-down" quaternion for its URDF convention
   (the EE link's local axes may differ between vendors).
2. Pick target poses inside the arm's reachable workspace.
3. Add an entry to ``CASES_BY_ARM`` under the arm's canonical name.

Customers with a specific cell layout can also write their own ``Case``
list at the call site and skip this registry entirely::

    from dimos.manipulation.eval import Case, evaluate
    my_cases = [Case(...), ...]
    evaluate(my_arm, my_cases)
"""

from __future__ import annotations

from typing import Callable

from dimos.manipulation.eval import Case, reach_case
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


# Quaternion (x, y, z, w) for "gripper pointing down" — depends on the
# EE link's URDF convention. Measure empirically by running an IK probe
# against a known-reachable pose and trying common orientations.
XARM_GRIPPER_DOWN = (1.0, 0.0, 0.0, 0.0)        # xArm: roll=180°, EE +Z up
PIPER_GRIPPER_DOWN = (0.0, 0.7071, 0.0, 0.7071)  # Piper: pitch=90°, EE +X forward


def _xarm6_obstacle_blocker() -> Obstacle:
    return Obstacle(
        name="blocker",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(
            frame_id="world",
            position=Vector3(0.30, 0.0, 0.40),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        dimensions=(0.06, 0.25, 0.06),
    )


def _xarm6_cases() -> list[Case]:
    """xArm6 — 6 reaches across the workspace + 1 obstacle case."""
    return [
        reach_case("reach_center", 0.35, 0.0, 0.25),
        reach_case("reach_left", 0.30, 0.25, 0.25),
        reach_case("reach_right", 0.30, -0.25, 0.25),
        reach_case("reach_high", 0.25, 0.0, 0.42),
        reach_case("reach_low", 0.35, 0.0, 0.13),
        reach_case("reach_extended", 0.48, 0.0, 0.20),
        reach_case("reach_obstacle_box", 0.42, 0.0, 0.28, obstacles=[_xarm6_obstacle_blocker()]),
    ]


def _xarm7_cases() -> list[Case]:
    """xArm7 — same reaches as xArm6, no obstacle (extra DOF causes goal-collision)."""
    return _xarm6_cases()[:-1]


def _piper_cases() -> list[Case]:
    """Piper — 6 reaches tuned to its smaller workspace and gripper convention."""
    q = PIPER_GRIPPER_DOWN
    return [
        reach_case("reach_center", 0.25, 0.0, 0.20, orientation=q),
        reach_case("reach_left", 0.25, 0.15, 0.20, orientation=q),
        reach_case("reach_right", 0.25, -0.15, 0.20, orientation=q),
        reach_case("reach_high", 0.20, 0.0, 0.30, orientation=q),
        reach_case("reach_extended", 0.30, 0.0, 0.20, orientation=q),
        reach_case("reach_diagonal", 0.25, 0.15, 0.30, orientation=q),
    ]


# Registry of arm name → case-list factory. Add an entry to support a new arm.
# Factories are zero-arg so cases are constructed lazily (avoids heavy imports
# at module load time).
CASES_BY_ARM: dict[str, Callable[[], list[Case]]] = {
    "xarm6": _xarm6_cases,
    "xarm7": _xarm7_cases,
    "piper": _piper_cases,
}
