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

"""R1 Pro in the MuJoCo open-space arena at seed 5000: classical picks, placing and the
tray, graded on the objects' recorded poses.

Seed 5000 puts a gray cup on the display table, a blue bottle on the low bench, an
orange toy block on the tall table, a purple drink carton on the worktable beside the
tray and a green glue stick on the high counter.

    dimos evals run dimos.evals.suites.r1pro_open_space --agent dimos.evals.agents.mcp_client_adapter \\
        --set 'modules=["r1pro-classical-open-space-sim-agent"]'
"""

from __future__ import annotations

from collections.abc import Callable
import json
import math
import os

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.types import EvalCase, Outcome, Suite, recording

CUP, BOTTLE, CARTON, GLUE_STICK = "task_object_1", "task_object_2", "task_object_4", "task_object_5"
TRAY = "task_bin"
TRACKED = (
    "task_object_1",
    "task_object_2",
    "task_object_3",
    "task_object_4",
    "task_object_5",
    TRAY,
)

# Platform centre x, y and top height, metres (open_space_scene.OPEN_PLATFORMS).
PLATFORMS = {
    "worktable": (0.49, 0.0, 0.70),
    "low_bench": (-3.0, 2.5, 0.60),
    "display_table": (0.0, 3.5, 0.80),
    "tall_table": (3.5, 2.5, 0.90),
    "high_counter": (3.5, -2.5, 0.85),
}
PLATFORM_HALF_M = (0.35, 0.60)
TRAY_HALF_M = (0.16, 0.235)


def environment() -> MujocoEnvironment:
    sim = "R1PROOPENSPACESIM"
    return MujocoEnvironment(
        blueprint=["r1pro-classical-open-space-sim"],
        tracked_bodies=TRACKED,
        module_env={
            f"{sim}__SEED": "5000",
            f"{sim}__HEADLESS": os.environ.get(f"{sim}__HEADLESS", "true"),
            f"{sim}__TRACKED_BODIES": json.dumps(list(TRACKED)),
        },
    )


def lifted(body: str, *, by_m: float) -> Callable[[Outcome], float]:
    """How far the body ended above where it started, full credit at ``by_m``."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation.z
                end = last_body_transform(store, body).translation.z
            except LookupError:
                return 0.0
        return min(max((end - start) / by_m, 0.0), 1.0)

    return grade


def on_platform(body: str, platform: str) -> Callable[[Outcome], float]:
    """1.0 when the body ended resting on the platform's top."""
    x, y, top = PLATFORMS[platform]

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                t = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return float(
            abs(t.x - x) <= PLATFORM_HALF_M[0]
            and abs(t.y - y) <= PLATFORM_HALF_M[1]
            and 0.0 <= t.z - top <= 0.3
        )

    return grade


def in_tray(body: str) -> Callable[[Outcome], float]:
    """1.0 when the body ended inside the tray's footprint, wherever the tray is."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                item = last_body_transform(store, body).translation
                tray = last_body_transform(store, TRAY)
            except LookupError:
                return 0.0
        q = tray.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        dx, dy = item.x - tray.translation.x, item.y - tray.translation.y
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        across = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return float(
            abs(along) <= TRAY_HALF_M[0]
            and abs(across) <= TRAY_HALF_M[1]
            and 0.0 <= item.z - tray.translation.z <= 0.2
        )

    return grade


def all_of(*grades: Callable[[Outcome], float]) -> Callable[[Outcome], float]:
    """The mean of several checks, so a partly done task gets partial credit."""
    return lambda outcome: sum(grade(outcome) for grade in grades) / len(grades)


SUITE: Suite = [
    EvalCase(
        id="r1pro_pick_cup",
        inputs="Go to the display table and pick up the gray cup with your left hand.",
        environment=environment(),
        grade=lifted(CUP, by_m=0.05),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "manipulation", "pick"}),
    ),
    EvalCase(
        id="r1pro_move_bottle",
        inputs=(
            "Take the blue bottle from the low bench to the display table and put it down there."
        ),
        environment=environment(),
        grade=on_platform(BOTTLE, "display_table"),
        timeout_s=1500.0,
        tags=frozenset({"mujoco", "manipulation", "pick", "place"}),
    ),
    EvalCase(
        id="r1pro_tray_to_low_bench",
        inputs=(
            "Put the purple drink carton into the tray, then carry the tray to the low bench "
            "and put it down there."
        ),
        environment=environment(),
        grade=all_of(in_tray(CARTON), on_platform(TRAY, "low_bench")),
        timeout_s=1800.0,
        tags=frozenset({"mujoco", "manipulation", "tray"}),
    ),
    EvalCase(
        id="r1pro_full_tray_delivery",
        inputs=(
            "Go to the display table and pick up the gray cup with your left hand. Then go to "
            "the high counter and pick up the green glue stick with your right hand. Then go to "
            "the low bench and put down what is in your left hand there. Pick up the blue bottle "
            "with your right hand, then pick up the green glue stick again with your left hand. "
            "Bring both items to the tray and put them in it, then carry the tray to the display "
            "table and put it down there."
        ),
        environment=environment(),
        grade=all_of(
            on_platform(CUP, "low_bench"),
            in_tray(BOTTLE),
            in_tray(GLUE_STICK),
            on_platform(TRAY, "display_table"),
        ),
        timeout_s=5400.0,
        tags=frozenset({"mujoco", "manipulation", "tray", "long_horizon"}),
    ),
]
