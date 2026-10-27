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

"""DimosBlocks: five lettered blocks lie at random angles on the robosuite table; arrange them to
spell DIMOS. Same table, robot, cameras and solver settings as ``binbottle``."""

from dimos.evals.scorers import WORD, word_partial
from dimos.evals.suites.binbottle import environment
from dimos.evals.suites.robosuite import GUIDANCE
from dimos.evals.types import EvalCase, Suite

SUITE: Suite = [
    EvalCase(
        id="dimosblocks_spell_dimos",
        inputs="Arrange the letter blocks so they spell DIMOS. " + GUIDANCE,
        environment=environment("blocks", tuple(f"block_{c}_main" for c in WORD)),
        # Any placement on the table: 0.7 x pairs in reading order + 0.3 letters upright for one
        # reader (O at any angle).
        grade=word_partial,
        threshold=1.0,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "dimosblocks", "manipulation", "blocks"}),
    ),
]
