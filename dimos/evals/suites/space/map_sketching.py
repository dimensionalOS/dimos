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

"""SPACE MapSketching — which of four sketched maps preserves the environment's true layout?

Both media presentations ride in one suite, told apart by tags::

    dimos evals run dimos.evals.suites.space.map_sketching --tags ego --limit 20 --system-prompt ""

120 questions per presentation. The questions carry their own instructions (persona, choices, JSON answer
format), so run with an empty system prompt; --blind withholds the
walkthrough and is the paper-faithful ablation. Scoring is the benchmark's own
(see _bench); chance on these four-way items is 25%.
"""

from __future__ import annotations

from dimos.evals.suites.space._bench import qa_suite
from dimos.evals.types import Suite

SUITE: Suite = qa_suite("MapSketching", "map_sketching")
