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

import json
import re


def extract_json(response: str) -> dict | list:  # type: ignore[type-arg]
    """Extract JSON from potentially messy LLM response.

    Tries multiple strategies:
    1. Parse the entire response as JSON
    2. Find and parse JSON arrays in the response
    3. Find and parse JSON objects in the response

    Args:
        response: Raw text response that may contain JSON

    Returns:
        Parsed JSON object (dict or list)

    Raises:
        json.JSONDecodeError: If no valid JSON can be extracted
    """
    # First try to parse the whole response as JSON
    try:
        return json.loads(response)  # type: ignore[no-any-return]
    except json.JSONDecodeError:
        pass

    # If that fails, try to extract JSON from the messy response
    # Look for JSON arrays or objects in the text

    # Pattern to match JSON arrays (including nested arrays/objects)
    # This finds the outermost [...] structure
    array_pattern = r"\[(?:[^\[\]]*|\[(?:[^\[\]]*|\[[^\[\]]*\])*\])*\]"

    # Pattern to match JSON objects
    object_pattern = r"\{(?:[^{}]*|\{(?:[^{}]*|\{[^{}]*\})*\})*\}"

    # Try to find JSON arrays first (most common for detections)
    matches = re.findall(array_pattern, response, re.DOTALL)
    for match in matches:
        try:
            parsed = json.loads(match)
            # For detection arrays, we expect a list
            if isinstance(parsed, list):
                return parsed
        except json.JSONDecodeError:
            continue

    # Try JSON objects if no arrays found
    matches = re.findall(object_pattern, response, re.DOTALL)
    for match in matches:
        try:
            return json.loads(match)  # type: ignore[no-any-return]
        except json.JSONDecodeError:
            continue

    # If nothing worked, raise an error with the original response
    raise json.JSONDecodeError(
        f"Could not extract valid JSON from response: {response[:200]}...", response, 0
    )
