#!/usr/bin/env python3

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