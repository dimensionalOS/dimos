#!/usr/bin/env python3
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

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.seat_guide import CameraSeatObservationProvider, SeatGuideSkillContainer
from dimos.agents.skills.unitree_speak_skill import UnitreeSpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer

_seat_guide_agentic = autoconnect(
    NavigationSkillContainer.blueprint(),
    CameraSeatObservationProvider.blueprint(),
    SeatGuideSkillContainer.blueprint(),
    UnitreeSkillContainer.blueprint(),
    WebInput.blueprint(),
    UnitreeSpeakSkill.blueprint(),
)

__all__ = ["_seat_guide_agentic"]
