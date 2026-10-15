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

"""The LIBERO-PRO suite with dimos's xArm7 in the Panda's place (``xarm-libero-sim``).

Same cases and tags as ``dimos.evals.suites.libero_pro``. The xArm starts from LIBERO's
sampled layouts (the recorded initial states are Panda states) and stands closer to the
workspace than the Panda, so scores are not comparable to the Panda suite.

    dimos evals run dimos.evals.suites.libero_pro_xarm --agent dimos.evals.agents.pi --tags libero_goal
"""

from dimos.evals.suites.libero_pro import cases
from dimos.evals.types import Suite

SUITE: Suite = cases("xarm-libero-sim")
