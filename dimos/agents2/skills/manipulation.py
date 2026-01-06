#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""Part 2 of the greeter tutorial: Standalone script for CLI usage.

Run this script, then in another terminal use:
    python -m dimos.agents2.cli.human.humancli
to interact with the agent.
"""

from dotenv import load_dotenv

from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.core.blueprints import autoconnect
from dimos.agents2.skills.manipulation_skill_container import GreeterForAgents, RobotCapabilities

load_dotenv()

# Compose the system
greetings_skill_blueprint = autoconnect(
    RobotCapabilities.blueprint(),
    GreeterForAgents.blueprint(),
    llm_agent(
        system_prompt="You are a friendly robot that can greet people. Use the greet skill when asked to say hello to someone."
    ),
    human_input(),
)