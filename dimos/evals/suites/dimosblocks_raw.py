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

"""The DimosBlocks case through the raw robot interface, for agents without dimOS.

dimos evals run dimos.evals.suites.dimosblocks_raw --agent dimos.evals.agents.pi \\
    --set no_dimos=true --set max_steps=150
"""

from dimos.evals.suites.binbottle_raw import raw
from dimos.evals.suites.dimosblocks_py import SUITE as DIMOS_SUITE
from dimos.evals.types import Suite

SUITE: Suite = [raw(case) for case in DIMOS_SUITE]
