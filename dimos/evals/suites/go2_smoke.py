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

"""Hand-written passive smoke suite over the go2 replays.

Ground truth verified against the recordings (contact sheets + odom math):
go2_short is 60s — chairs room, store shelves, robot kiosk, glass booths, a
person at a table at the end; path 37.9m, displacement 1.7m.
"""

from __future__ import annotations

from dimos.evals.scorers import choice, exact, first_number, within, yes_no
from dimos.evals.types import PassiveEval, Suite

SUITE: Suite = [
    PassiveEval(
        id="short_person_visible",
        inputs="Is a person visible in any of these images?",
        expected="yes",
        parse=yes_no,
        score=exact,
        context=(lambda s: s.streams.color_image.range_time(40, 61),),
        dataset="go2_short",
        tags=frozenset({"image", "presence"}),
    ),
    PassiveEval(
        id="short_start_furniture",
        inputs="At the start of these observations, which furniture is most numerous? "
        "Answer with one of: chairs, sofas, beds, desks.",
        expected="chairs",
        parse=choice,
        score=exact,
        context=(lambda s: s.streams.color_image.range_time(0, 8),),
        dataset="go2_short",
        tags=frozenset({"image", "mcq"}),
    ),
    PassiveEval(
        id="short_displacement",
        inputs="How far in a straight line is your final position from your first "
        "shown position, in meters?",
        expected=1.7,
        parse=first_number,
        score=within(1.5),
        context=(lambda s: s.streams.odom,),
        dataset="go2_short",
        tags=frozenset({"odom", "numeric"}),
    ),
    PassiveEval(
        id="short_lidar_points",
        inputs="How many points does the shown pointcloud contain?",
        expected=20834.0,
        parse=first_number,
        score=within(5000.0),
        context=(lambda s: s.streams.lidar.limit(1),),
        dataset="go2_short",
        tags=frozenset({"pointcloud", "numeric"}),
    ),
    PassiveEval(
        id="hk_not_seen",
        inputs="Which of these did you NOT see anywhere in the observations? "
        "Answer with one of: a couch, store shelves, a swimming pool, an office chair.",
        expected="a swimming pool",
        parse=choice,
        score=exact,
        context=(lambda s: s.streams.color_image,),
        dataset="go2_hongkong_office",
        tags=frozenset({"image", "mcq"}),
    ),
]
