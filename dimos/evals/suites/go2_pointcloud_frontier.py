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

"""Is an unmapped spot reachable over mapped floor, or walled off?

One frame of the go2_china_office replay. The label comes from a flood fill
over the full-resolution cloud: the spot at (1.54, 2.30) is closed off.
"""

from __future__ import annotations

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import choice, exact
from dimos.evals.types import EvalCase, Suite

SUITE: Suite = [
    EvalCase(
        id="go2_china_office_frontier_t93_1.54_2.3",
        inputs="You are the robot; your current pose is the odom observation shown (world "
        "frame: +x is east, +y is north, coordinates in meters). The lidar returned nothing "
        "from the area around the world point (1.54, 2.30), so it is unmapped. Reasoning "
        "only from the floor and obstacles the lidar did map, could you drive there across "
        "mapped floor to explore it, or is it closed off from you behind walls or other "
        "obstacles? Answer with one word: reachable if an open path over mapped floor leads "
        "to it, or walled if it is blocked off. Use only the mapped point cloud.",
        environment=Dataset(
            "go2_china_office",
            select=(
                lambda s: s.streams.lidar.range_time(92.84, 92.94),
                lambda s: s.streams.odom.range_time(92.5, 93.1),
            ),
        ),
        grade=lambda o: exact("walled", choice(["reachable", "walled"])(o.trajectory.final_answer)),
        timeout_s=300.0,
        tags=frozenset({"pointcloud", "mcq"}),
    ),
]
