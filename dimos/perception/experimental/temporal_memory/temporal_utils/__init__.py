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

"""Temporal memory utilities."""

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

__all__ = [
    "WINDOW_RESPONSE_SCHEMA",
    "build_batch_distance_estimation_prompt",
    "build_distance_estimation_prompt",
    "build_graph_context",
    "build_query_prompt",
    "build_summary_prompt",
    "build_window_prompt",
    "clamp_text",
    "extract_time_window",
    "format_timestamp",
    "get_structured_output_format",
    "is_scene_stale",
    "next_entity_id_hint",
    "parse_batch_distance_response",
    "parse_window_response",
]
