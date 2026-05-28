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

from dimos.agents.skills.browser_notification_skill import BrowserNotificationSkill
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic

unitree_go2_companion_alert = autoconnect(
    unitree_go2_agentic,
    BrowserNotificationSkill.blueprint(),
)
# The companion alert demo is driven by MCP and the phone alert page; disabling
# voice/web input avoids optional local audio dependencies for this focused stack.
unitree_go2_companion_alert = unitree_go2_companion_alert.disabled_modules(WebInput, SpeakSkill)

__all__ = ["unitree_go2_companion_alert"]
