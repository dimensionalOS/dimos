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

"""
Temporal memory utilities for temporal memory. includes helper functions
and prompts that are used to build the prompt for the VLM.
"""

# Re-export everything from submodules
from .graph_utils import build_graph_context, extract_time_window
from .helpers import clamp_text, format_timestamp, is_scene_stale, next_entity_id_hint
from .parsers import parse_batch_distance_response, parse_window_response
from .prompts import (
    WINDOW_RESPONSE_SCHEMA,
    build_batch_distance_estimation_prompt,
    build_distance_estimation_prompt,
    build_query_prompt,
    build_summary_prompt,
    build_window_prompt,
    get_structured_output_format,
)
from .state import apply_summary_update, default_state, update_state_from_window

__all__ = [
    # Schema
    "WINDOW_RESPONSE_SCHEMA",
    # State management
    "apply_summary_update",
    # Prompts
    "build_batch_distance_estimation_prompt",
    "build_distance_estimation_prompt",
    # Graph utils
    "build_graph_context",
    "build_query_prompt",
    "build_summary_prompt",
    "build_window_prompt",
    # Helpers
    "clamp_text",
    "default_state",
    "extract_time_window",
    "format_timestamp",
    "get_structured_output_format",
    "is_scene_stale",
    "next_entity_id_hint",
    # Parsers
    "parse_batch_distance_response",
    "parse_window_response",
    "update_state_from_window",
]
