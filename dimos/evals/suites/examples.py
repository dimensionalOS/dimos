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

    dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.question_answer

Or from pytest / a notebook::

    from dimos.evals.agents.question_answer import QuestionAnswer
    from dimos.evals.runner import EvalRunner
    from dimos.evals.suites.examples import SUITE

    results = EvalRunner().run(SUITE, QuestionAnswer())
"""

from __future__ import annotations

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import exact, first_number, within, yes_no
from dimos.evals.types import EvalCase, Suite

# One lidar frame from the unitree go2 replay. The environment says what the
# recording holds: `.limit(1)` keeps exactly the first PointCloud2. How it
# reaches the model is the agent (`qa` encodes it into one prompt).
single_lidar_frame = EvalCase(
    id="example_single_lidar_frame",
    inputs="How many points does the shown pointcloud contain?",
    environment=Dataset("go2_short", select=(lambda s: s.streams.lidar.limit(1),)),
    grade=lambda o: within(5000.0)(20834.0, first_number(o.trajectory.final_answer)),
    tags=frozenset({"example", "encoding", "pointcloud"}),
)

# A range of 10 image frames from the same replay: seconds 58..61 of the
# recording, capped to 10 observations. Image.agent_encode() turns each into an
# image content block; a person sits at a table in this stretch.
# Note: `.limit(n)` keeps the *first* n observations of the window — for a
# spread across a long window, select the whole range and let the agent
# downsample evenly instead.
ten_image_range = EvalCase(
    id="example_ten_image_range",
    inputs="Is a person visible in any of these images?",
    environment=Dataset(
        "go2_short", select=(lambda s: s.streams.color_image.range_time(58, 61).limit(10),)
    ),
    grade=lambda o: exact("yes", yes_no(o.trajectory.final_answer)),
    tags=frozenset({"example", "encoding", "image"}),
)

SUITE: Suite = [single_lidar_frame, ten_image_range]
