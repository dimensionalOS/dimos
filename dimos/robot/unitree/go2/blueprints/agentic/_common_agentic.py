#!/usr/bin/env python3
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

from dimos.agents.tools.navigation import NavigationToolContainer
from dimos.agents.tools.person_follow import PersonFollowToolContainer
from dimos.agents.tools.speak_tool import SpeakTool
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_tool_container import UnitreeToolContainer

_common_agentic = autoconnect(
    NavigationToolContainer.blueprint(),
    PersonFollowToolContainer.blueprint(camera_info=GO2Connection.camera_info_static),
    UnitreeToolContainer.blueprint(),
    WebInput.blueprint(),
    SpeakTool.blueprint(),
)

__all__ = ["_common_agentic"]
