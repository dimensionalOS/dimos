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

"""Hand-written smoke suite over the go2 replays.

Ground truth verified against the recordings (contact sheets + odom math):
go2_short is 62s of the sim Go2 in the NJ ranch scan (image times from the first
image) — kitchen 4-18s, dining chairs 24-27s, living-room sofa 34-43s, the open
front door 49-52s; path 22.7m, displacement 6.7m.
"""

from __future__ import annotations

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import choice, exact, first_number, within, yes_no
from dimos.evals.types import EvalCase, Suite

_FURNITURE = ["chairs", "sofas", "beds", "desks"]
_ROOMS = ["kitchen", "bedroom", "bathroom", "garage"]
_NOT_SEEN = ["a couch", "store shelves", "a swimming pool", "an office chair"]

SUITE: Suite = [
    EvalCase(
        id="short_kitchen",
        inputs=f"Which room do these images show? Answer with one of: {', '.join(_ROOMS)}.",
        environment=Dataset(
            "go2_short", select=(lambda s: s.streams.color_image.range_time(4, 18),)
        ),
        grade=lambda o: exact("kitchen", choice(_ROOMS)(o.trajectory.final_answer)),
        tags=frozenset({"image", "mcq"}),
    ),
    EvalCase(
        id="short_dining_furniture",
        inputs="Which furniture is most numerous in these observations? "
        f"Answer with one of: {', '.join(_FURNITURE)}.",
        environment=Dataset(
            "go2_short", select=(lambda s: s.streams.color_image.range_time(24, 27.5),)
        ),
        grade=lambda o: exact("chairs", choice(_FURNITURE)(o.trajectory.final_answer)),
        tags=frozenset({"image", "mcq"}),
    ),
    EvalCase(
        id="short_sofa_visible",
        inputs="Is a sofa visible in any of these images? Answer yes or no.",
        environment=Dataset(
            "go2_short", select=(lambda s: s.streams.color_image.range_time(34, 43),)
        ),
        grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
        tags=frozenset({"image", "presence"}),
    ),
    EvalCase(
        id="short_displacement",
        inputs="How far in a straight line is your final position from your first "
        "shown position, in meters? Answer with just the number.",
        environment=Dataset("go2_short", select=(lambda s: s.streams.odom,)),
        grade=lambda o: within(2.7)(6.7, first_number(o.trajectory.final_answer)),
        tags=frozenset({"odom", "numeric"}),
    ),
    EvalCase(
        id="short_lidar_points",
        inputs="How many points does the shown pointcloud contain? Answer with just the number.",
        environment=Dataset("go2_short", select=(lambda s: s.streams.lidar.limit(1),)),
        grade=lambda o: within(5000.0)(13524.0, first_number(o.trajectory.final_answer)),
        tags=frozenset({"pointcloud", "numeric"}),
    ),
    EvalCase(
        id="hk_not_seen",
        inputs="Which of these did you NOT see anywhere in the observations? "
        f"Answer with one of: {', '.join(_NOT_SEEN)}.",
        environment=Dataset("go2_hongkong_office", select=(lambda s: s.streams.color_image,)),
        grade=lambda o: exact("a swimming pool", choice(_NOT_SEEN)(o.trajectory.final_answer)),
        tags=frozenset({"image", "mcq"}),
    ),
]
