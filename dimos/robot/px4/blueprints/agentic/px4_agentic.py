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

"""``px4`` with the agent that flies by the skills.

Usage:
    dimos run px4-agentic
    dimos humancli                          # or: dimos agent-send "take off to 2 meters"
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.px4.blueprints.agentic._agentic_skills import _agentic_skills
from dimos.robot.px4.blueprints.basic.px4 import px4

px4_agentic = autoconnect(px4, _agentic_skills)
