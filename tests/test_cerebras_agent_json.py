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

import os
import sys
from typing import List, Optional
from pydantic import BaseModel, Field
from dotenv import load_dotenv
import re

# Add the project root to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.agents.cerebras_agent import CerebrasAgent

# Load environment variables
load_dotenv()

# Test Pydantic models with various validation constraints
class SimpleResponse(BaseModel):
    """A simple response model with basic fields."""
    message: str
    count: int = 0

class ListResponse(BaseModel):
    """A response model with list validation."""
    items: List[str] = Field(default_factory=list, min_items=1, max_items=5)

class NestedResponse(BaseModel):
    """A response model with nested structures."""
    name: str
    details: SimpleResponse
    tags: Optional[List[str]] = None

class SkillResponse(BaseModel):
    """A response model for a skill call with unsupported validation constraints."""
    items: List[str] = Field(default_factory=list, min_items=1, max_items=5)

class UnsupportedFieldsResponse(BaseModel):
    items: List[str] = Field(default_factory=list, min_items=1, max_items=5)
    wrgwrgrw: str = "should not be here"

def extract_json_from_response(response: str) -> str:
    # Remove markdown code block if present
    match = re.search(r"```(?:json)?\s*([\s\S]*?)```", response, re.IGNORECASE)
    if match:
        return match.group(1).strip()
    return response.strip()

def test_simple_json_response():
    """Test CerebrasAgent with a simple JSON response model."""
    print("\n=== Testing Simple JSON Response ===")
    
    # Create agent with simple response model
    agent = CerebrasAgent(
        dev_name="test_agent",
        query="Return a JSON object with a 'message' field containing 'Hello, World!' and a 'count' field set to 5. The response MUST be a valid JSON object.",
        response_model=SimpleResponse,
        system_query="You MUST respond with a valid JSON object matching this schema: {\"message\": string, \"count\": number}. Do not include any explanations or code examples."
    )
    
    # Run query
    response = agent.run_observable_query(
        "Return a JSON object with a 'message' field containing 'Hello, World!' and a 'count' field set to 5. The response MUST be a valid JSON object."
    ).run()
    print(f"Response: {response}")
    
    # Try to parse response as JSON
    try:
        import json
        json_str = extract_json_from_response(response)
        parsed = json.loads(json_str)
        print(f"Parsed JSON: {parsed}")
        assert "message" in parsed, "Response missing 'message' field"
        assert "count" in parsed, "Response missing 'count' field"
        print("✓ Simple JSON test passed")
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
        raise

def test_list_json_response():
    """Test CerebrasAgent with a list-based JSON response model."""
    print("\n=== Testing List JSON Response ===")
    
    agent = CerebrasAgent(
        dev_name="test_agent",
        query="Return a JSON object with an 'items' field containing a list of 3 colors. The field must be named 'items'.",
        response_model=ListResponse,
        system_query="You must respond with a JSON object matching this schema: {\"items\": [string, ...]}"
    )
    
    response = agent.run_observable_query(
        "Return a JSON object with an 'items' field containing a list of 3 colors. The field must be named 'items'."
    ).run()
    print(f"Response: {response}")
    
    try:
        import json
        json_str = extract_json_from_response(response)
        parsed = json.loads(json_str)
        print(f"Parsed JSON: {parsed}")
        assert "items" in parsed, "Response missing 'items' field"
        assert isinstance(parsed["items"], list), "Items field is not a list"
        print("✓ List JSON test passed")
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
        raise

def test_nested_json_response():
    """Test CerebrasAgent with a nested JSON response model."""
    print("\n=== Testing Nested JSON Response ===")
    
    # Create agent with nested response model
    agent = CerebrasAgent(
        dev_name="test_agent",
        query="Return a JSON object with a 'name' field set to 'John' and a 'details' field containing a nested object with 'message' set to 'Hello' and 'count' set to 3. The response MUST be a valid JSON object with this exact structure.",
        response_model=NestedResponse,
        system_query="You MUST respond with a valid JSON object matching this schema: {\"name\": string, \"details\": {\"message\": string, \"count\": number}}. The 'details' field MUST be a nested object."
    )
    
    # Run query
    response = agent.run_observable_query(
        "Return a JSON object with a 'name' field set to 'John' and a 'details' field containing a nested object with 'message' set to 'Hello' and 'count' set to 3. The response MUST be a valid JSON object with this exact structure."
    ).run()
    print(f"Response: {response}")
    
    # Try to parse response as JSON
    try:
        import json
        json_str = extract_json_from_response(response)
        parsed = json.loads(json_str)
        print(f"Parsed JSON: {parsed}")
        assert "name" in parsed, "Response missing 'name' field"
        assert "details" in parsed, "Response missing 'details' field"
        assert isinstance(parsed["details"], dict), "'details' field is not an object"
        assert "message" in parsed["details"], "Details missing 'message' field"
        assert "count" in parsed["details"], "Details missing 'count' field"
        print("✓ Nested JSON test passed")
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
        raise

def test_skill_json_response():
    """Test CerebrasAgent with a skill call that includes unsupported JSON schema fields."""
    print("\n=== Testing Skill JSON Response ===")
    
    agent = CerebrasAgent(
        dev_name="test_agent",
        query="Return a JSON object with an 'items' field containing a list of 3 colors. The field must be named 'items'.",
        response_model=SkillResponse,
        system_query="You must respond with a JSON object matching this schema: {\"items\": [string, ...]}"
    )
    
    response = agent.run_observable_query(
        "Return a JSON object with an 'items' field containing a list of 3 colors. The field must be named 'items'."
    ).run()
    print(f"Response: {response}")
    
    try:
        import json
        json_str = extract_json_from_response(response)
        parsed = json.loads(json_str)
        print(f"Parsed JSON: {parsed}")
        assert "items" in parsed, "Response missing 'items' field"
        assert isinstance(parsed["items"], list), "Items field is not a list"
        print("✓ Skill JSON test passed")
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
        raise

def test_unsupported_schema_fields():
    """Test that Cerebras API returns a 400 error for unsupported JSON schema fields."""
    print("\n=== Testing Unsupported Schema Fields ===")
    agent = CerebrasAgent(
        dev_name="test_agent",
        query="Return a JSON object with an 'items' field containing a list of 3 colors. The field must be named 'items'.",
        response_model=UnsupportedFieldsResponse,
        system_query="You must respond with a JSON object matching this schema: {\"items\": [string, ...]}"
    )
    try:
        response = agent.run_observable_query(
            "Return a JSON object with an 'items' field containing a list of 3 colors. The field must be named 'items'."
        ).run()
        print(f"Response: {response}")
        print("✗ ERROR: Expected a 400 error from Cerebras API, but got a response.")
    except Exception as e:
        print(f"Caught exception as expected: {e}")
        assert "Unsupported JSON schema fields" in str(e) or "wrong_api_format" in str(e), "Did not get expected unsupported fields error"
        print("✓ Correctly caught unsupported schema fields error.")

if __name__ == "__main__":
    # Run all tests
    test_simple_json_response()
    test_list_json_response()
    test_nested_json_response()
    test_skill_json_response()
    test_unsupported_schema_fields() 