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

"""Documentation examples — the smallest useful evals, one concept each.

Run them::

    dimos evals run dimos.evals.suites.examples

Or from pytest / a notebook::

    from dimos.evals.runner import EvalRunner
    from dimos.evals.suites.examples import SUITE

    results = EvalRunner().run(SUITE)
"""

from __future__ import annotations

from dimos.evals.scorers import exact, first_number, within, yes_no
from dimos.evals.types import PassiveEval, Suite

# One lidar frame from the unitree go2 replay. The context selector returns the
# real mem2 Stream — `.limit(1)` keeps exactly the first PointCloud2. The str()
# fallback encoding exposes `num_points`, so the answer is verifiable.
single_lidar_frame = PassiveEval(
    id="example_single_lidar_frame",
    inputs="How many points does the shown pointcloud contain?",
    expected=20834.0,
    parse=first_number,
    score=within(5000.0),
    context=(lambda s: s.streams.lidar.limit(1),),
    dataset="go2_short",
    tags=frozenset({"example", "encoding", "pointcloud"}),
)

# A range of 10 image frames from the same replay: seconds 58..61 of the
# recording, capped to 10 observations. Image.agent_encode() turns each into an
# image content block; a person sits at a table in this stretch.
# Note: `.limit(n)` keeps the *first* n observations of the window — for a
# spread across a long window, give the runner the whole range and let its
# context budget downsample evenly instead.
ten_image_range = PassiveEval(
    id="example_ten_image_range",
    inputs="Is a person visible in any of these images?",
    expected="yes",
    parse=yes_no,
    score=exact,
    context=(lambda s: s.streams.color_image.range_time(58, 61).limit(10),),
    dataset="go2_short",
    tags=frozenset({"example", "encoding", "image"}),
)

SUITE: Suite = [single_lidar_frame, ten_image_range]
