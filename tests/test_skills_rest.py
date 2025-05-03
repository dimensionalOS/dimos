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

import tests.test_header

from textwrap import dedent
from dimos.skills.skills import SkillLibrary

from dotenv import load_dotenv
from dimos.agents.claude_agent import ClaudeAgent
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.skills.rest.rest import GenericRestSkill
import reactivex as rx
import reactivex.operators as ops

# Load API key from environment
load_dotenv()

# Create a skill library and add the GenericRestSkill
skills = SkillLibrary()
skills.add(GenericRestSkill)

# Create a subject for agent responses
agent_response_subject = rx.subject.Subject()
agent_response_stream = agent_response_subject.pipe(ops.share())

# Create a text stream for agent responses in the web interface
text_streams = {
    "agent_responses": agent_response_stream,
}
web_interface = RobotWebInterface(port=5555, text_streams=text_streams)

# Create a ClaudeAgent instance
agent = ClaudeAgent(
    dev_name="test_agent",
    input_query_stream=web_interface.query_stream,
    skills=skills,
    system_query=dedent(
        """
        You are a virtual agent. When given a query, respond by using 
        the appropriate tool calls if needed to execute commands on the robot.
        
        IMPORTANT:
        Only return the response directly asked of the user. E.G. if the user asks for the time,
        only return the time. If the user asks for the weather, only return the weather.
        """),
    model_name="claude-3-7-sonnet-latest",
    thinking_budget_tokens=2000
)

# Subscribe to agent responses and send them to the subject
agent.get_response_observable().subscribe(
    lambda x: agent_response_subject.on_next(x)
)

# Start the web interface
web_interface.run()

# Run this query in the web interface:
# 
# Make a web request to nist to get the current time. 
# You should use http://worldclockapi.com/api/json/utc/now
#