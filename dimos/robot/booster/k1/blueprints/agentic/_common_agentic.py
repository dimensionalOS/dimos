# Copyright 2025-2026 Dimensional Inc.
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

"""Shared agentic skills for Booster K1 blueprints."""

from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import autoconnect
from dimos.robot.booster.k1.connection import K1Connection

_common_agentic = autoconnect(
    # TODO: re-enable once K1 odom/pointcloud streams are available
    # NavigationSkillContainer.blueprint(),
    PersonFollowSkillContainer.blueprint(camera_info=K1Connection.camera_info_static),
    WebInput.blueprint(),
    SpeakSkill.blueprint(),
)

__all__ = ["_common_agentic"]
