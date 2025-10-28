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

"""Tests for LLM utility functions."""

import json

import pytest

from dimos.utils.llm_utils import extract_json


def test_extract_json_clean_response() -> None:
    """Test extract_json with clean JSON response."""
    clean_json = '[["object", 1, 2, 3, 4]]'
    result = extract_json(clean_json)
    assert result == [["object", 1, 2, 3, 4]]


def test_extract_json_with_text_before_after() -> None:
    """Test extract_json with text before and after JSON."""
    messy = """Here's what I found:
    [
        ["person", 10, 20, 30, 40],
        ["car", 50, 60, 70, 80]
    ]
    Hope this helps!"""
    result = extract_json(messy)
    assert result == [["person", 10, 20, 30, 40], ["car", 50, 60, 70, 80]]


def test_extract_json_with_emojis() -> None:
    """Test extract_json with emojis and markdown code blocks."""
    messy = """Sure! 😊 Here are the detections:

    ```json
    [["human", 100, 200, 300, 400]]
    ```

    Let me know if you need anything else! 👍"""
    result = extract_json(messy)
    assert result == [["human", 100, 200, 300, 400]]


def test_extract_json_multiple_json_blocks() -> None:
    """Test extract_json when there are multiple JSON blocks."""
    messy = """First attempt (wrong format):
    {"error": "not what we want"}

    Correct format:
    [
        ["cat", 10, 10, 50, 50],
        ["dog", 60, 60, 100, 100]
    ]

    Another block: {"also": "not needed"}"""
    result = extract_json(messy)
    # Should return the first valid array
    assert result == [["cat", 10, 10, 50, 50], ["dog", 60, 60, 100, 100]]


def test_extract_json_object() -> None:
    """Test extract_json with JSON object instead of array."""
    response = 'The result is: {"status": "success", "count": 5}'
    result = extract_json(response)
    assert result == {"status": "success", "count": 5}


def test_extract_json_nested_structures() -> None:
    """Test extract_json with nested arrays and objects."""
    response = """Processing complete:
    [
        ["label1", 1, 2, 3, 4],
        {"nested": {"value": 10}},
        ["label2", 5, 6, 7, 8]
    ]"""
    result = extract_json(response)
    assert result[0] == ["label1", 1, 2, 3, 4]
    assert result[1] == {"nested": {"value": 10}}
    assert result[2] == ["label2", 5, 6, 7, 8]


def test_extract_json_invalid() -> None:
    """Test extract_json raises error when no valid JSON found."""
    response = "This response has no valid JSON at all!"
    with pytest.raises(json.JSONDecodeError) as exc_info:
        extract_json(response)
    assert "Could not extract valid JSON" in str(exc_info.value)


# Test with actual LLM response format
MOCK_LLM_RESPONSE = """
   Yes :)

   [
    ["humans", 76, 368, 219, 580],
    ["humans", 354, 372, 512, 525],
    ["humans", 409, 370, 615, 748],
    ["humans", 628, 350, 762, 528],
    ["humans", 785, 323, 960, 650]
   ]

   Hope this helps!😀😊 :)"""


def test_extract_json_with_real_llm_response() -> None:
    """Test extract_json with actual messy LLM response."""
    result = extract_json(MOCK_LLM_RESPONSE)
    assert isinstance(result, list)
    assert len(result) == 5
    assert result[0] == ["humans", 76, 368, 219, 580]
    assert result[-1] == ["humans", 785, 323, 960, 650]
