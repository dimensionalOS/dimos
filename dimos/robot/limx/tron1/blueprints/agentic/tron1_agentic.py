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

"""TRON1 stack with agentic skills."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.limx.tron1.blueprints.agentic._agentic_skills import _tron1_agentic_skills
from dimos.robot.limx.tron1.blueprints.basic.tron1_basic import tron1_basic

tron1_agentic = autoconnect(
    tron1_basic,
    _tron1_agentic_skills,
)

