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

"""Helper utility functions for temporal memory."""

from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from ..temporal_memory import Frame


def next_entity_id_hint(roster: Any) -> str:
    """Generate next entity ID based on existing roster (e.g., E1, E2, E3...)."""
    if not isinstance(roster, list):
        return "E1"
    max_n = 0
    for e in roster:
        if not isinstance(e, dict):
            continue
        eid = e.get("id")
        if isinstance(eid, str) and eid.startswith("E"):
            tail = eid[1:]
            if tail.isdigit():
                max_n = max(max_n, int(tail))
    return f"E{max_n + 1}"


def clamp_text(text: str, max_chars: int) -> str:
    """Clamp text to maximum characters."""
    if len(text) <= max_chars:
        return text
    return text[:max_chars] + "..."


def format_timestamp(seconds: float) -> str:
    """Format seconds as MM:SS.mmm timestamp string."""
    m = int(seconds // 60)
    s = seconds - 60 * m
    return f"{m:02d}:{s:06.3f}"


def is_scene_stale(frames: list["Frame"], stale_threshold: float = 5.0) -> bool:
    """Check if scene hasn't changed meaningfully between first and last frame.

    Args:
        frames: List of frames to check
        stale_threshold: Threshold for mean pixel difference (default: 5.0)

    Returns:
        True if scene is stale (hasn't changed enough), False otherwise
    """
    if len(frames) < 2:
        return False
    first_img = frames[0].image
    last_img = frames[-1].image
    if first_img is None or last_img is None:
        return False
    if not hasattr(first_img, "data") or not hasattr(last_img, "data"):
        return False
    diff = np.abs(first_img.data.astype(float) - last_img.data.astype(float))
    return bool(diff.mean() < stale_threshold)
