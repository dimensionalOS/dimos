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

"""Parse DimOS theme from tcss file."""

from __future__ import annotations

from pathlib import Path
import re


def parse_tcss_colors(tcss_path: str | Path) -> dict[str, str]:
    """Parse color variables from a tcss file.

    Args:
        tcss_path: Path to the tcss file

    Returns:
        Dictionary mapping variable names to color values
    """
    tcss_path = Path(tcss_path)
    content = tcss_path.read_text()

    # Match $variable: value; patterns
    pattern = r"\$([a-zA-Z0-9_-]+)\s*:\s*(#[0-9a-fA-F]{6}|#[0-9a-fA-F]{3});"
    matches = re.findall(pattern, content)

    return {name: value for name, value in matches}


# Load DimOS theme colors
_THEME_PATH = Path(__file__).parent / "dimos.tcss"
COLORS = parse_tcss_colors(_THEME_PATH)

# Export CSS path for Textual apps
CSS_PATH = str(_THEME_PATH)


# Convenience accessors for common colors
def get(name: str, default: str = "#ffffff") -> str:
    """Get a color by variable name."""
    return COLORS.get(name, default)


# Base color palette
BLACK = COLORS.get("black", "#0b0f0f")
RED = COLORS.get("red", "#ff0000")
GREEN = COLORS.get("green", "#00eeee")
YELLOW = COLORS.get("yellow", "#ffcc00")
BLUE = COLORS.get("blue", "#5c9ff0")
PURPLE = COLORS.get("purple", "#00eeee")
CYAN = COLORS.get("cyan", "#00eeee")
WHITE = COLORS.get("white", "#b5e4f4")

# Bright colors
BRIGHT_BLACK = COLORS.get("bright-black", "#404040")
BRIGHT_RED = COLORS.get("bright-red", "#ff0000")
BRIGHT_GREEN = COLORS.get("bright-green", "#00eeee")
BRIGHT_YELLOW = COLORS.get("bright-yellow", "#f2ea8c")
BRIGHT_BLUE = COLORS.get("bright-blue", "#8cbdf2")
BRIGHT_PURPLE = COLORS.get("bright-purple", "#00eeee")
BRIGHT_CYAN = COLORS.get("bright-cyan", "#00eeee")
BRIGHT_WHITE = COLORS.get("bright-white", "#ffffff")

# Core theme colors
BACKGROUND = COLORS.get("background", "#0b0f0f")
FOREGROUND = COLORS.get("foreground", "#b5e4f4")
CURSOR = COLORS.get("cursor", "#00eeee")

# Semantic aliases
BG = COLORS.get("bg", "#0b0f0f")
BORDER = COLORS.get("border", "#00eeee")
ACCENT = COLORS.get("accent", "#b5e4f4")
DIM = COLORS.get("dim", "#404040")
TIMESTAMP = COLORS.get("timestamp", "#ffffff")

# Message type colors
SYSTEM = COLORS.get("system", "#ff0000")
AGENT = COLORS.get("agent", "#88ff88")
TOOL = COLORS.get("tool", "#00eeee")
TOOL_RESULT = COLORS.get("tool-result", "#ffff00")
HUMAN = COLORS.get("human", "#ffffff")

# Status colors
SUCCESS = COLORS.get("success", "#00eeee")
ERROR = COLORS.get("error", "#ff0000")
WARNING = COLORS.get("warning", "#ffcc00")
INFO = COLORS.get("info", "#00eeee")

ascii_logo = """
   ▇▇▇▇▇▇╗ ▇▇╗▇▇▇╗   ▇▇▇╗▇▇▇▇▇▇▇╗▇▇▇╗   ▇▇╗▇▇▇▇▇▇▇╗▇▇╗ ▇▇▇▇▇▇╗ ▇▇▇╗   ▇▇╗ ▇▇▇▇▇╗ ▇▇╗
   ▇▇╔══▇▇╗▇▇║▇▇▇▇╗ ▇▇▇▇║▇▇╔════╝▇▇▇▇╗  ▇▇║▇▇╔════╝▇▇║▇▇╔═══▇▇╗▇▇▇▇╗  ▇▇║▇▇╔══▇▇╗▇▇║
   ▇▇║  ▇▇║▇▇║▇▇╔▇▇▇▇╔▇▇║▇▇▇▇▇╗  ▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇╗▇▇║▇▇║   ▇▇║▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇║▇▇║
   ▇▇║  ▇▇║▇▇║▇▇║╚▇▇╔╝▇▇║▇▇╔══╝  ▇▇║╚▇▇╗▇▇║╚════▇▇║▇▇║▇▇║   ▇▇║▇▇║╚▇▇╗▇▇║▇▇╔══▇▇║▇▇║
   ▇▇▇▇▇▇╔╝▇▇║▇▇║ ╚═╝ ▇▇║▇▇▇▇▇▇▇╗▇▇║ ╚▇▇▇▇║▇▇▇▇▇▇▇║▇▇║╚▇▇▇▇▇▇╔╝▇▇║ ╚▇▇▇▇║▇▇║  ▇▇║▇▇▇▇▇▇▇╗
   ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝
"""
