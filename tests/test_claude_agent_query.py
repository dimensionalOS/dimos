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

from dotenv import load_dotenv
from dimos.agents.claude_agent import ClaudeAgent

# Load API key from environment
load_dotenv()

# Create a ClaudeAgent instance
agent = ClaudeAgent(
    dev_name="test_agent",
    query="What is the capital of France?"
)

# Use the stream_query method to get a response
response = agent.run_observable_query("What is the capital of France?").run()

print(f"Response from Claude Agent: {response}") 