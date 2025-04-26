#
#
#

"""Tests for the agent module in the dimos package."""

import os
import pytest
from unittest import mock

import tests.test_header

from dimos.agents.agent import Agent, OpenAIAgent


def test_dotenv_loaded():
    """Test that environment variables are loaded correctly."""
    from dotenv import load_dotenv
    load_dotenv()
    
    openai_api_key = os.getenv("OPENAI_API_KEY")
    assert openai_api_key is not None, "OPENAI_API_KEY environment variable should be set"


@pytest.mark.skipif(os.getenv("OPENAI_API_KEY") is None, 
                   reason="OPENAI_API_KEY environment variable not set")
def test_openai_connection():
    """Test connection to OpenAI API."""
    from openai import OpenAI
    client = OpenAI()
    
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "What's in this image?"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg",
                        },
                    },
                ],
            }
        ],
        max_tokens=300,
    )
    
    assert response.choices[0].message.content is not None
    assert len(response.choices[0].message.content) > 0


def test_agent_initialization():
    """Test that an agent can be initialized correctly."""
    agent = Agent(dev_name="TestAgent")
    assert agent.dev_name == "TestAgent"
    assert hasattr(agent, "dev_name")
    assert hasattr(agent, "agent_type")
    assert hasattr(agent, "agent_memory")


@mock.patch('dimos.agents.agent.OpenAI')
def test_openai_agent_query(mock_openai):
    """Test that an OpenAI agent can process queries."""
    mock_client = mock.MagicMock()
    mock_openai.return_value = mock_client
    
    mock_response = mock.MagicMock()
    mock_response.choices = [mock.MagicMock()]
    mock_response.choices[0].message.content = "This is a test response"
    mock_client.chat.completions.create.return_value = mock_response
    
    agent = OpenAIAgent(dev_name="TestOpenAIAgent")
    
    mock_observable = mock.MagicMock()
    agent.stream_query = mock.MagicMock(return_value=mock_observable)
    
    mock_observer = mock.MagicMock()
    mock_observable.subscribe.return_value = mock.MagicMock()
    
    disposable = agent.stream_query("Test query").subscribe(mock_observer)
    
    mock_observer.on_next.assert_not_called()  # Not called yet
    
    agent._observable_query = mock.MagicMock()
    
    agent.stream_query.assert_called_once_with("Test query")


@mock.patch('dimos.agents.agent.OpenAI')
def test_openai_agent_with_skills(mock_openai, skill_library):
    """Test that an OpenAI agent can be initialized with skills."""
    mock_client = mock.MagicMock()
    mock_openai.return_value = mock_client
    
    agent = OpenAIAgent(
        dev_name="TestSkillAgent",
        skills=skill_library
    )
    
    assert agent.skills is not None
    assert agent.skills == skill_library
