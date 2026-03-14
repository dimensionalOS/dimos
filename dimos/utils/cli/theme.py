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

"""DimOS theme system.

Provides named themes for the DIO TUI. Each theme defines:
  - Textual Theme fields (primary, background, etc.) for built-in widgets
  - Custom CSS variables (prefixed ``dio-``) for DimOS-specific styling
  - Python-level constants (ACCENT, DIM, AGENT, …) for Rich markup

Usage in CSS::

    background: $dio-bg;
    border: solid $dio-dim;
    color: $dio-text;

Usage in Python (Rich markup)::

    f"[{theme.AGENT}]agent response[/{theme.AGENT}]"
"""

from __future__ import annotations

from pathlib import Path
import re


def parse_tcss_colors(tcss_path: str | Path) -> dict[str, str]:
    """Parse color variables from a tcss file."""
    tcss_path = Path(tcss_path)
    content = tcss_path.read_text()
    pattern = r"\$([a-zA-Z0-9_-]+)\s*:\s*(#[0-9a-fA-F]{6}|#[0-9a-fA-F]{3});"
    matches = re.findall(pattern, content)
    return {name: value for name, value in matches}


# Load DimOS theme colors (used by standalone apps via CSS_PATH)
_THEME_PATH = Path(__file__).parent / "dimos.tcss"
COLORS = parse_tcss_colors(_THEME_PATH)

# Export CSS path for standalone Textual apps (not DIO)
CSS_PATH = str(_THEME_PATH)


# Convenience accessor
def get(name: str, default: str = "#ffffff") -> str:
    """Get a color by variable name."""
    return COLORS.get(name, default)


# ---------------------------------------------------------------------------
# Theme definitions
# ---------------------------------------------------------------------------

# Each entry maps custom CSS variable names (without ``$``) to hex values.
# These are injected into Textual's CSS variable system via Theme.variables.
# The keys here become ``$dio-bg``, ``$dio-dim``, etc. in CSS.
#
# Variable naming:
#   Core:      dio-bg, dio-fg, dio-text, dio-dim, dio-accent, dio-accent2
#   Palette:   dio-red, dio-orange, dio-yellow, dio-green, dio-blue, dio-purple, dio-cyan, dio-white, dio-grey
#   Chat:      dio-agent, dio-tool, dio-tool-result, dio-human, dio-timestamp
#   Buttons:   dio-btn-danger, dio-btn-danger-bg, dio-btn-warn, dio-btn-warn-bg,
#              dio-btn-muted, dio-btn-muted-bg, dio-btn-kill, dio-btn-kill-bg
#   Chrome:    dio-panel-bg, dio-hint-bg
#   Tabs:      dio-tab1, dio-tab1-bg, dio-tab2, dio-tab2-bg, dio-tab3, dio-tab3-bg
#   LCMSpy:    dio-bw-low, dio-bw-high
#   Dtop:      dio-label, dio-stale, dio-pid
#   Debug:     dio-debug-key, dio-debug-action, dio-debug-focus

_THEME_VARIABLES: dict[str, dict[str, str]] = {
    "dark-one": {
        # Core
        "dio-bg": "#0b0f0f",
        "dio-fg": "#b5e4f4",
        "dio-text": "#b5e4f4",
        "dio-dim": "#404040",
        "dio-accent": "#00eeee",
        "dio-accent2": "#ff8800",
        # Named palette
        "dio-red": "#ff0000",
        "dio-orange": "#ff8800",
        "dio-yellow": "#ffcc00",
        "dio-green": "#00ee88",
        "dio-blue": "#5c9ff0",
        "dio-purple": "#c07ff0",
        "dio-cyan": "#00eeee",
        "dio-white": "#ffffff",
        "dio-grey": "#777777",
        # Chat message colors
        "dio-agent": "#88ff88",
        "dio-tool": "#00eeee",
        "dio-tool-result": "#ffff00",
        "dio-human": "#ffffff",
        "dio-timestamp": "#ffffff",
        # Buttons
        "dio-btn-danger": "#cc4444",
        "dio-btn-danger-bg": "#882222",
        "dio-btn-warn": "#ccaa00",
        "dio-btn-warn-bg": "#886600",
        "dio-btn-muted": "#8899aa",
        "dio-btn-muted-bg": "#445566",
        "dio-btn-kill": "#ff4444",
        "dio-btn-kill-bg": "#882222",
        # UI chrome
        "dio-panel-bg": "#1a2a2a",
        "dio-hint-bg": "#1a2020",
        # Tabs
        "dio-tab1": "#00eeee",
        "dio-tab1-bg": "#1a2a2a",
        "dio-tab2": "#5c9ff0",
        "dio-tab2-bg": "#1a1a2a",
        "dio-tab3": "#c07ff0",
        "dio-tab3-bg": "#2a1a2a",
        # LCMSpy bandwidth gradient
        "dio-bw-low": "#00eeee",
        "dio-bw-high": "#ffcc00",
        # Dtop
        "dio-label": "#cccccc",
        "dio-stale": "#606060",
        "dio-pid": "#777777",
        # Debug
        "dio-debug-key": "#b5e4f4",
        "dio-debug-action": "#ffcc00",
        "dio-debug-focus": "#5c9ff0",
    },
    "dark-two": {
        # Core
        "dio-bg": "#0a0e1a",
        "dio-fg": "#a0b8d0",
        "dio-text": "#a0b8d0",
        "dio-dim": "#303850",
        "dio-accent": "#4488cc",
        "dio-accent2": "#dd8833",
        # Named palette
        "dio-red": "#cc4444",
        "dio-orange": "#dd8833",
        "dio-yellow": "#ccaa44",
        "dio-green": "#44aa88",
        "dio-blue": "#5588dd",
        "dio-purple": "#8866cc",
        "dio-cyan": "#4488cc",
        "dio-white": "#d0d8e0",
        "dio-grey": "#667788",
        # Chat message colors
        "dio-agent": "#66cc88",
        "dio-tool": "#4488cc",
        "dio-tool-result": "#ccaa44",
        "dio-human": "#d0d8e0",
        "dio-timestamp": "#8899bb",
        # Buttons
        "dio-btn-danger": "#bb5555",
        "dio-btn-danger-bg": "#662233",
        "dio-btn-warn": "#bbaa44",
        "dio-btn-warn-bg": "#665522",
        "dio-btn-muted": "#7788aa",
        "dio-btn-muted-bg": "#334455",
        "dio-btn-kill": "#dd5555",
        "dio-btn-kill-bg": "#662233",
        # UI chrome
        "dio-panel-bg": "#151c2e",
        "dio-hint-bg": "#101828",
        # Tabs
        "dio-tab1": "#4488cc",
        "dio-tab1-bg": "#151c2e",
        "dio-tab2": "#5588dd",
        "dio-tab2-bg": "#14183a",
        "dio-tab3": "#8866cc",
        "dio-tab3-bg": "#1c1430",
        # LCMSpy bandwidth gradient
        "dio-bw-low": "#4488cc",
        "dio-bw-high": "#ccaa44",
        # Dtop
        "dio-label": "#aabbcc",
        "dio-stale": "#404860",
        "dio-pid": "#667788",
        # Debug
        "dio-debug-key": "#a0b8d0",
        "dio-debug-action": "#ccaa44",
        "dio-debug-focus": "#5588dd",
    },
    "light": {
        # Core
        "dio-bg": "#f0f2f5",
        "dio-fg": "#1a1a2e",
        "dio-text": "#1a1a2e",
        "dio-dim": "#999999",
        "dio-accent": "#0077aa",
        "dio-accent2": "#cc6600",
        # Named palette
        "dio-red": "#cc2222",
        "dio-orange": "#cc6600",
        "dio-yellow": "#aa8800",
        "dio-green": "#228844",
        "dio-blue": "#2266cc",
        "dio-purple": "#7744aa",
        "dio-cyan": "#0077aa",
        "dio-white": "#1a1a2e",
        "dio-grey": "#666666",
        # Chat message colors
        "dio-agent": "#227744",
        "dio-tool": "#0077aa",
        "dio-tool-result": "#886600",
        "dio-human": "#1a1a2e",
        "dio-timestamp": "#555555",
        # Buttons
        "dio-btn-danger": "#cc2222",
        "dio-btn-danger-bg": "#ffdddd",
        "dio-btn-warn": "#aa7700",
        "dio-btn-warn-bg": "#fff3dd",
        "dio-btn-muted": "#555555",
        "dio-btn-muted-bg": "#dddddd",
        "dio-btn-kill": "#ee2222",
        "dio-btn-kill-bg": "#ffdddd",
        # UI chrome
        "dio-panel-bg": "#e4e8ee",
        "dio-hint-bg": "#dde0e6",
        # Tabs
        "dio-tab1": "#0077aa",
        "dio-tab1-bg": "#ddeef8",
        "dio-tab2": "#2266cc",
        "dio-tab2-bg": "#dde0f8",
        "dio-tab3": "#7744aa",
        "dio-tab3-bg": "#eeddf8",
        # LCMSpy bandwidth gradient
        "dio-bw-low": "#0077aa",
        "dio-bw-high": "#aa8800",
        # Dtop
        "dio-label": "#333333",
        "dio-stale": "#aaaaaa",
        "dio-pid": "#666666",
        # Debug
        "dio-debug-key": "#1a1a2e",
        "dio-debug-action": "#aa8800",
        "dio-debug-focus": "#2266cc",
    },
    "midnight": {
        # Core — deep navy with cool steel accents
        "dio-bg": "#0a0e1a",
        "dio-fg": "#a8bcd0",
        "dio-text": "#a8bcd0",
        "dio-dim": "#2e3a50",
        "dio-accent": "#5599dd",
        "dio-accent2": "#cc8844",
        # Named palette
        "dio-red": "#cc5555",
        "dio-orange": "#cc8844",
        "dio-yellow": "#ccaa55",
        "dio-green": "#55aa88",
        "dio-blue": "#5599dd",
        "dio-purple": "#9977cc",
        "dio-cyan": "#55aacc",
        "dio-white": "#d0d8e8",
        "dio-grey": "#667788",
        # Chat message colors
        "dio-agent": "#66cc88",
        "dio-tool": "#55aacc",
        "dio-tool-result": "#ccaa55",
        "dio-human": "#d0d8e8",
        "dio-timestamp": "#8899bb",
        # Buttons
        "dio-btn-danger": "#cc5555",
        "dio-btn-danger-bg": "#552233",
        "dio-btn-warn": "#bbaa55",
        "dio-btn-warn-bg": "#554422",
        "dio-btn-muted": "#7788aa",
        "dio-btn-muted-bg": "#334455",
        "dio-btn-kill": "#dd5555",
        "dio-btn-kill-bg": "#552233",
        # UI chrome
        "dio-panel-bg": "#121830",
        "dio-hint-bg": "#0e1428",
        # Tabs
        "dio-tab1": "#55aacc",
        "dio-tab1-bg": "#121830",
        "dio-tab2": "#5599dd",
        "dio-tab2-bg": "#14183a",
        "dio-tab3": "#9977cc",
        "dio-tab3-bg": "#1c1430",
        # LCMSpy bandwidth gradient
        "dio-bw-low": "#55aacc",
        "dio-bw-high": "#ccaa55",
        # Dtop
        "dio-label": "#aabbcc",
        "dio-stale": "#3a4860",
        "dio-pid": "#667788",
        # Debug
        "dio-debug-key": "#a8bcd0",
        "dio-debug-action": "#ccaa55",
        "dio-debug-focus": "#5599dd",
    },
    "ember": {
        # Core — warm dark with orange/red accent
        "dio-bg": "#120c0a",
        "dio-fg": "#e0c8b0",
        "dio-text": "#e0c8b0",
        "dio-dim": "#4a3028",
        "dio-accent": "#ee8844",
        "dio-accent2": "#cc6644",
        # Named palette
        "dio-red": "#dd4433",
        "dio-orange": "#ee8844",
        "dio-yellow": "#ddaa33",
        "dio-green": "#88aa44",
        "dio-blue": "#cc8844",
        "dio-purple": "#cc6688",
        "dio-cyan": "#ccaa66",
        "dio-white": "#e8d8c8",
        "dio-grey": "#887766",
        # Chat message colors
        "dio-agent": "#aacc66",
        "dio-tool": "#ee8844",
        "dio-tool-result": "#ddaa33",
        "dio-human": "#e8d8c8",
        "dio-timestamp": "#aa9080",
        # Buttons
        "dio-btn-danger": "#dd4433",
        "dio-btn-danger-bg": "#661a14",
        "dio-btn-warn": "#ccaa33",
        "dio-btn-warn-bg": "#665518",
        "dio-btn-muted": "#998877",
        "dio-btn-muted-bg": "#443830",
        "dio-btn-kill": "#ee4433",
        "dio-btn-kill-bg": "#661a14",
        # UI chrome
        "dio-panel-bg": "#2a1810",
        "dio-hint-bg": "#1a1210",
        # Tabs
        "dio-tab1": "#ee8844",
        "dio-tab1-bg": "#2a1810",
        "dio-tab2": "#cc8844",
        "dio-tab2-bg": "#2a2010",
        "dio-tab3": "#cc6688",
        "dio-tab3-bg": "#2a1420",
        # LCMSpy bandwidth gradient
        "dio-bw-low": "#ccaa66",
        "dio-bw-high": "#dd4433",
        # Dtop
        "dio-label": "#ccbbaa",
        "dio-stale": "#5a4838",
        "dio-pid": "#887766",
        # Debug
        "dio-debug-key": "#e0c8b0",
        "dio-debug-action": "#ddaa33",
        "dio-debug-focus": "#cc8844",
    },
    "forest": {
        # Core — deep green with natural earth tones
        "dio-bg": "#0a100c",
        "dio-fg": "#b0d0b8",
        "dio-text": "#b0d0b8",
        "dio-dim": "#2a3a2e",
        "dio-accent": "#44cc88",
        "dio-accent2": "#88cc44",
        # Named palette
        "dio-red": "#cc5544",
        "dio-orange": "#cc8844",
        "dio-yellow": "#aacc44",
        "dio-green": "#44cc88",
        "dio-blue": "#44aa99",
        "dio-purple": "#88aa66",
        "dio-cyan": "#55ccaa",
        "dio-white": "#d0e0d0",
        "dio-grey": "#668866",
        # Chat message colors
        "dio-agent": "#66dd88",
        "dio-tool": "#55ccaa",
        "dio-tool-result": "#aacc44",
        "dio-human": "#d0e0d0",
        "dio-timestamp": "#80aa88",
        # Buttons
        "dio-btn-danger": "#cc5544",
        "dio-btn-danger-bg": "#552218",
        "dio-btn-warn": "#aaaa44",
        "dio-btn-warn-bg": "#555522",
        "dio-btn-muted": "#778877",
        "dio-btn-muted-bg": "#334433",
        "dio-btn-kill": "#dd5544",
        "dio-btn-kill-bg": "#552218",
        # UI chrome
        "dio-panel-bg": "#142a1a",
        "dio-hint-bg": "#101a14",
        # Tabs
        "dio-tab1": "#44cc88",
        "dio-tab1-bg": "#142a1a",
        "dio-tab2": "#44aa99",
        "dio-tab2-bg": "#142a26",
        "dio-tab3": "#88aa66",
        "dio-tab3-bg": "#1e2a14",
        # LCMSpy bandwidth gradient
        "dio-bw-low": "#55ccaa",
        "dio-bw-high": "#aacc44",
        # Dtop
        "dio-label": "#aaccaa",
        "dio-stale": "#3a4a3e",
        "dio-pid": "#668866",
        # Debug
        "dio-debug-key": "#b0d0b8",
        "dio-debug-action": "#aacc44",
        "dio-debug-focus": "#44aa99",
    },
}

# Textual Theme constructor args for each theme
_THEME_BASES: dict[str, dict[str, object]] = {
    "dark-one": {
        "primary": "#00eeee",
        "secondary": "#5c9ff0",
        "warning": "#ffcc00",
        "error": "#ff0000",
        "success": "#88ff88",
        "accent": "#00eeee",
        "foreground": "#b5e4f4",
        "background": "#0b0f0f",
        "surface": "#0b0f0f",
        "panel": "#1a2a2a",
        "dark": True,
    },
    "dark-two": {
        "primary": "#4488cc",
        "secondary": "#5588dd",
        "warning": "#ccaa44",
        "error": "#cc4444",
        "success": "#44aa88",
        "accent": "#4488cc",
        "foreground": "#a0b8d0",
        "background": "#0a0e1a",
        "surface": "#0a0e1a",
        "panel": "#151c2e",
        "dark": True,
    },
    "light": {
        "primary": "#0077aa",
        "secondary": "#2266cc",
        "warning": "#aa8800",
        "error": "#cc2222",
        "success": "#228844",
        "accent": "#0077aa",
        "foreground": "#1a1a2e",
        "background": "#f0f2f5",
        "surface": "#f0f2f5",
        "panel": "#e4e8ee",
        "dark": False,
    },
    "midnight": {
        "primary": "#5599dd",
        "secondary": "#55aacc",
        "warning": "#ccaa55",
        "error": "#cc5555",
        "success": "#55aa88",
        "accent": "#5599dd",
        "foreground": "#a8bcd0",
        "background": "#0a0e1a",
        "surface": "#0a0e1a",
        "panel": "#121830",
        "dark": True,
    },
    "ember": {
        "primary": "#ee8844",
        "secondary": "#cc8844",
        "warning": "#ddaa33",
        "error": "#dd4433",
        "success": "#88aa44",
        "accent": "#ee8844",
        "foreground": "#e0c8b0",
        "background": "#120c0a",
        "surface": "#120c0a",
        "panel": "#2a1810",
        "dark": True,
    },
    "forest": {
        "primary": "#44cc88",
        "secondary": "#44aa99",
        "warning": "#aacc44",
        "error": "#cc5544",
        "success": "#44cc88",
        "accent": "#44cc88",
        "foreground": "#b0d0b8",
        "background": "#0a100c",
        "surface": "#0a100c",
        "panel": "#142a1a",
        "dark": True,
    },
}

THEME_NAMES: list[str] = list(_THEME_VARIABLES)
DEFAULT_THEME = "dark-one"


def get_textual_themes() -> list[object]:
    """Return a list of Textual ``Theme`` objects for all DimOS themes."""
    from textual.theme import Theme as TextualTheme

    themes = []
    for name in THEME_NAMES:
        base = _THEME_BASES[name]
        variables = _THEME_VARIABLES[name]
        themes.append(
            TextualTheme(
                name=f"dimos-{name}",
                variables=variables,
                **base,  # type: ignore[arg-type]
            )
        )
    return themes


def _vars_for(name: str) -> dict[str, str]:
    """Get the CSS variable dict for a theme by short name."""
    return _THEME_VARIABLES.get(name, _THEME_VARIABLES[DEFAULT_THEME])


# ---------------------------------------------------------------------------
# Active theme tracking + Python-level constants
# ---------------------------------------------------------------------------

active_theme: str = DEFAULT_THEME


def set_theme(name: str) -> None:
    """Switch the active theme and update all module-level color constants.

    This updates the Python constants used in Rich markup (e.g. ``theme.AGENT``).
    For Textual CSS variables, also call ``app.theme = f"dimos-{name}"``.
    """
    global active_theme
    if name not in _THEME_VARIABLES:
        return
    active_theme = name
    v = _THEME_VARIABLES[name]
    _apply_vars(v)


def _apply_vars(v: dict[str, str]) -> None:
    """Update module-level constants from a CSS-variable dict."""
    import dimos.utils.cli.theme as _self

    # Core
    _self.BACKGROUND = v["dio-bg"]
    _self.BG = v["dio-bg"]
    _self.FOREGROUND = v["dio-fg"]
    _self.ACCENT = v["dio-accent"]
    _self.DIM = v["dio-dim"]
    _self.CYAN = v["dio-accent"]
    _self.BORDER = v["dio-accent"]

    # Named palette
    _self.RED = v["dio-red"]
    _self.ORANGE = v["dio-orange"]
    _self.YELLOW = v["dio-yellow"]
    _self.GREEN = v["dio-green"]
    _self.BLUE = v["dio-blue"]
    _self.PURPLE = v["dio-purple"]
    _self.WHITE = v["dio-white"]
    _self.GREY = v["dio-grey"]

    # Chat
    _self.AGENT = v["dio-agent"]
    _self.TOOL = v["dio-tool"]
    _self.TOOL_RESULT = v["dio-tool-result"]
    _self.HUMAN = v["dio-human"]
    _self.TIMESTAMP = v["dio-timestamp"]

    # Semantic aliases
    _self.SYSTEM = v["dio-red"]
    _self.SUCCESS = v["dio-green"]
    _self.ERROR = v["dio-red"]
    _self.WARNING = v["dio-yellow"]
    _self.INFO = v["dio-accent"]

    # Legacy compat
    _self.BLACK = v["dio-bg"]
    _self.BRIGHT_BLACK = v["dio-dim"]
    _self.BRIGHT_WHITE = v["dio-white"]
    _self.CURSOR = v["dio-accent"]
    _self.BRIGHT_RED = v["dio-red"]
    _self.BRIGHT_GREEN = v["dio-green"]
    _self.BRIGHT_YELLOW = v["dio-yellow"]
    _self.BRIGHT_BLUE = v["dio-blue"]
    _self.BRIGHT_PURPLE = v["dio-purple"]
    _self.BRIGHT_CYAN = v["dio-accent"]

    # Button colors (available as Python constants for inline Rich markup)
    _self.BTN_DANGER = v["dio-btn-danger"]
    _self.BTN_DANGER_BG = v["dio-btn-danger-bg"]
    _self.BTN_WARN = v["dio-btn-warn"]
    _self.BTN_WARN_BG = v["dio-btn-warn-bg"]
    _self.BTN_MUTED = v["dio-btn-muted"]
    _self.BTN_MUTED_BG = v["dio-btn-muted-bg"]
    _self.BTN_KILL = v["dio-btn-kill"]
    _self.BTN_KILL_BG = v["dio-btn-kill-bg"]

    # Dtop
    _self.LABEL_COLOR = v["dio-label"]
    _self.STALE_COLOR = v["dio-stale"]
    _self.PID_COLOR = v["dio-pid"]

    # Debug
    _self.DEBUG_KEY = v["dio-debug-key"]
    _self.DEBUG_ACTION = v["dio-debug-action"]
    _self.DEBUG_FOCUS = v["dio-debug-focus"]


# ---------------------------------------------------------------------------
# Initial module-level constants (from dimos.tcss defaults)
# ---------------------------------------------------------------------------

# Base color palette
BLACK = COLORS.get("black", "#0b0f0f")
RED = COLORS.get("red", "#ff0000")
ORANGE = "#ff8800"
GREEN = COLORS.get("green", "#00eeee")
YELLOW = COLORS.get("yellow", "#ffcc00")
BLUE = COLORS.get("blue", "#5c9ff0")
PURPLE = COLORS.get("purple", "#00eeee")
CYAN = COLORS.get("cyan", "#00eeee")
WHITE = COLORS.get("white", "#b5e4f4")
GREY = "#777777"

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

# Button colors
BTN_DANGER = "#cc4444"
BTN_DANGER_BG = "#882222"
BTN_WARN = "#ccaa00"
BTN_WARN_BG = "#886600"
BTN_MUTED = "#8899aa"
BTN_MUTED_BG = "#445566"
BTN_KILL = "#ff4444"
BTN_KILL_BG = "#882222"

# Dtop colors
LABEL_COLOR = "#cccccc"
STALE_COLOR = "#606060"
PID_COLOR = "#777777"

# Debug colors
DEBUG_KEY = "#b5e4f4"
DEBUG_ACTION = "#ffcc00"
DEBUG_FOCUS = "#5c9ff0"

ascii_logo = """
   ▇▇▇▇▇▇╗ ▇▇╗▇▇▇╗   ▇▇▇╗▇▇▇▇▇▇▇╗▇▇▇╗   ▇▇╗▇▇▇▇▇▇▇╗▇▇╗ ▇▇▇▇▇▇╗ ▇▇▇╗   ▇▇╗ ▇▇▇▇▇╗ ▇▇╗
   ▇▇╔══▇▇╗▇▇║▇▇▇▇╗ ▇▇▇▇║▇▇╔════╝▇▇▇▇╗  ▇▇║▇▇╔════╝▇▇║▇▇╔═══▇▇╗▇▇▇▇╗  ▇▇║▇▇╔══▇▇╗▇▇║
   ▇▇║  ▇▇║▇▇║▇▇╔▇▇▇▇╔▇▇║▇▇▇▇▇╗  ▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇╗▇▇║▇▇║   ▇▇║▇▇╔▇▇╗ ▇▇║▇▇▇▇▇▇▇║▇▇║
   ▇▇║  ▇▇║▇▇║▇▇║╚▇▇╔╝▇▇║▇▇╔══╝  ▇▇║╚▇▇╗▇▇║╚════▇▇║▇▇║▇▇║   ▇▇║▇▇║╚▇▇╗▇▇║▇▇╔══▇▇║▇▇║
   ▇▇▇▇▇▇╔╝▇▇║▇▇║ ╚═╝ ▇▇║▇▇▇▇▇▇▇╗▇▇║ ╚▇▇▇▇║▇▇▇▇▇▇▇║▇▇║╚▇▇▇▇▇▇╔╝▇▇║ ╚▇▇▇▇║▇▇║  ▇▇║▇▇▇▇▇▇▇╗
   ╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝
"""
