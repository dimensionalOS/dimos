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

"""Beanbag count from point cloud alone (unitree go2).

Pointcloud-only: the agent sees the final ``global_map`` frame via
``PointCloud2.agent_encode`` and nothing else. Tests whether the cloud alone
carries enough to count circular beanbags on the ground.

    dimos evals run dimos.evals.suites.beanbag_count --agent dimos.evals.agents.question_answer
"""

from __future__ import annotations

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import first_number, within
from dimos.evals.types import EvalCase, Suite

DATASET = "go2_beanbag"


SUITE: Suite = [
    EvalCase(
        id="pc_beanbag_count",
        inputs="How many circular beanbags do you see on the ground? Answer with just the number.",
        environment=Dataset(
            DATASET,
            # final accumulated map frame; point cloud only, no camera
            select=(lambda s: s.streams.global_map.at(s.streams.global_map.last().ts, 1.0),),
        ),
        grade=lambda o: within(1.0)(3.0, first_number(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "count", "beanbag"}),
    ),
]
