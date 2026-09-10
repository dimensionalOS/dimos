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

"""xArm7 tabletop regression — the canonical case from issue #3594.

"pick up the cup" on ``xarm-perception-sim-agent``: the grasp skill can
report success while the approach knocks the other objects off the table.
Agent-visible memory scores that a pass; the GT oracle fails the episode —
the cup must actually rise, and nothing else may move.

The scene (LFS ``xarm7/scene.xml``) has three free-joint tabletop bodies —
``apple``, ``orange``, ``cup`` — which is what MujocoSimModule's
``publish_ground_truth`` stream (enabled here via the runner passing
``--mujoco-publish-ground-truth``) publishes world poses for.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.evals.predicates import displaced, lifted
from dimos.evals.scorers import final
from dimos.evals.types import InteractiveEval, Suite

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

# Semantic role -> GT body name (the pose's frame_id in the GT stream).
ROLES = {"cup": "cup", "apple": "apple", "orange": "orange"}

_LIFT_MIN = 0.05  # 5 cm — a held cup, not a nudge
_COLLATERAL_THRESHOLD = 0.10  # 10 cm xy — anything past this was struck


def _pick_without_collateral(store: Store, gt: Store) -> float:
    """Full credit only when the cup lifted and every bystander stayed put."""
    picked = lifted(gt, ROLES, "cup", min_delta=_LIFT_MIN)
    collateral = max(
        displaced(gt, ROLES, bystander, threshold=_COLLATERAL_THRESHOLD)
        for bystander in ("apple", "orange")
    )
    return picked * (1.0 - collateral)


pick_up_cup = InteractiveEval(
    id="xarm7_pick_up_cup_gt",
    inputs="pick up the cup",
    score=_pick_without_collateral,
    aggregate=final,
    ground_truth=True,
    blueprint="xarm-perception-sim-agent",
    simulator="mujoco",
    interval_s=2.0,
    timeout_s=300.0,
    tags=frozenset({"manipulation", "gt"}),
)

SUITE: Suite = [pick_up_cup]
