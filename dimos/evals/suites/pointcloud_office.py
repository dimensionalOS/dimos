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

"""Point-cloud comprehension over the go2_office_pc recording.

The agent sees the final ``global_map`` frame (the whole accumulated floor
plan, via ``PointCloud2.agent_encode``) and the ``odom`` path. Ground truth
was hand-labelled from the recording. Counts grade with off-by-one partial
credit; the two categorical cases are exact.

    dimos evals run dimos.evals.suites.pointcloud_office --agent dimos.evals.agents.question_answer
"""

from __future__ import annotations

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import choice, exact, first_number, within, yes_no
from dimos.evals.types import EvalCase, Suite

DATASET = "go2_office_pc"


def _env() -> Dataset:
    """Final accumulated map frame plus the odom path."""
    return Dataset(
        DATASET,
        select=(
            lambda s: s.streams.global_map,
            lambda s: s.streams.odom,
        ),
    )


SUITE: Suite = [
    EvalCase(
        id="pc_rooms_entered",
        inputs="How many distinct rooms did you walk into? Answer with just the number.",
        environment=_env(),
        grade=lambda o: within(1.0)(2.0, first_number(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "count"}),
    ),
    EvalCase(
        id="pc_rooms_passed",
        inputs=(
            "How many distinct rooms did you walk past or into in total? "
            "Answer with just the number."
        ),
        environment=_env(),
        grade=lambda o: within(1.0)(3.0, first_number(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "count"}),
    ),
    EvalCase(
        id="pc_open_doorways",
        inputs=(
            "How many open doorways did you see? The building has only standard "
            "residential-width doors. Answer with just the number."
        ),
        environment=_env(),
        grade=lambda o: within(1.0)(4.0, first_number(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "count"}),
    ),
    EvalCase(
        id="pc_biggest_room_occupied",
        inputs=(
            "In the largest room, do objects occupy more than 50% of its 2D floor "
            "area? Answer yes or no."
        ),
        environment=_env(),
        grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "yesno"}),
    ),
    EvalCase(
        id="pc_first_vs_second_room_size",
        inputs=(
            "Was the first room you walked through bigger or smaller than the "
            "second room you walked through? Answer with one word: bigger or smaller."
        ),
        environment=_env(),
        grade=lambda o: exact("bigger", choice(["bigger", "smaller"])(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "compare"}),
    ),
]
