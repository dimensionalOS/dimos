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

"""Shared utilities for introspection renderers."""

import hashlib
import re

# Colors for type nodes and edges (bright, distinct, good on dark backgrounds)
TYPE_COLORS = [
    "#FF6B6B",  # coral red
    "#4ECDC4",  # teal
    "#FFE66D",  # yellow
    "#95E1D3",  # mint
    "#F38181",  # salmon
    "#AA96DA",  # lavender
    "#81C784",  # green
    "#64B5F6",  # light blue
    "#FFB74D",  # orange
    "#BA68C8",  # purple
    "#4DD0E1",  # cyan
    "#AED581",  # lime
    "#FF8A65",  # deep orange
    "#7986CB",  # indigo
    "#F06292",  # pink
    "#A1887F",  # brown
    "#90A4AE",  # blue grey
    "#DCE775",  # lime yellow
    "#4DB6AC",  # teal green
    "#9575CD",  # deep purple
    "#E57373",  # light red
    "#81D4FA",  # sky blue
    "#C5E1A5",  # light green
    "#FFCC80",  # light orange
    "#B39DDB",  # light purple
    "#80DEEA",  # light cyan
    "#FFAB91",  # peach
    "#CE93D8",  # light violet
    "#80CBC4",  # light teal
    "#FFF59D",  # light yellow
]

# Colors for group borders (bright, distinct, good on dark backgrounds)
GROUP_COLORS = [
    "#5C9FF0",  # blue
    "#FFB74D",  # orange
    "#81C784",  # green
    "#BA68C8",  # purple
    "#4ECDC4",  # teal
    "#FF6B6B",  # coral
    "#FFE66D",  # yellow
    "#7986CB",  # indigo
    "#F06292",  # pink
    "#4DB6AC",  # teal green
    "#9575CD",  # deep purple
    "#AED581",  # lime
    "#64B5F6",  # light blue
    "#FF8A65",  # deep orange
    "#AA96DA",  # lavender
]

# Colors for RPCs/Skills
RPC_COLOR = "#7986CB"  # indigo
SKILL_COLOR = "#4ECDC4"  # teal


def color_for_string(colors: list[str], s: str) -> str:
    """Get a consistent color for a string based on its hash."""
    h = int(hashlib.md5(s.encode()).hexdigest(), 16)
    return colors[h % len(colors)]


def sanitize_id(s: str) -> str:
    """Sanitize a string to be a valid graphviz node ID."""
    return re.sub(r"[^a-zA-Z0-9_]", "_", s)
