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

"""DimOS terminal design system: colour tokens, and the components that use them.

Colours are parsed from ``dimos.tcss`` so the CLI and the Textual apps share one
source of truth. The components below are the terminal equivalent of a small
component library — a doorway mark, a bordered card that degrades by width, and
an in-place redraw helper — so surfaces do not each hand-roll their own boxes.

Everything here is stdlib-only and pure: components return lists of ready-to-print
rows and never write to a stream, so they stay cheap for the modules that import
this file only for its palette.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import re
import shutil
import sys
import textwrap
import time


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


# ---------------------------------------------------------------------------
# Components
#
# The terminal half of the design system. Pure functions returning rows of
# text: nothing here writes to a stream, so a module that imports this file for
# its colours pays nothing for the drawing code.
# ---------------------------------------------------------------------------

RGB = tuple[int, int, int]
SPINNER = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"
_ESC = re.compile(r"\x1b\[[0-9;]*m")


def rgb(name: str, default: str = "#ffffff") -> RGB:
    """A theme colour as an RGB triple, for truecolor escapes."""
    h = COLORS.get(name, default).lstrip("#")
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


# The portal gradient, in order. Sampled from the brand doorway asset.
PORTAL: list[RGB] = [rgb(f"portal-{i}", "#ffffff") for i in range(11)]
MUTED: RGB = (0x5A, 0x6A, 0x6A)
MINT: RGB = (0xB9, 0xF5, 0xBB)
SALMON: RGB = (0xF3, 0xAA, 0x7B)

GREY_RAMP: list[RGB] = [
    (0x36, 0x3F, 0x3F),
    (0x6E, 0x7E, 0x7E),
    (0x7E, 0x8E, 0x8E),
    (0x5A, 0x66, 0x66),
]
RED_RAMP: list[RGB] = [
    (0x5E, 0x1E, 0x1E),
    (0xC8, 0x3A, 0x30),
    (0xFF, 0x6A, 0x5F),
    (0x8A, 0x2A, 0x24),
]


def enabled() -> bool:
    """Components draw only for a human at a terminal, and never under NO_COLOR."""
    return sys.stdout.isatty() and not os.environ.get("NO_COLOR")


def term_width() -> int:
    return shutil.get_terminal_size((100, 24)).columns


def visible_len(s: str) -> int:
    """Length ignoring ANSI escapes — what the terminal actually spends."""
    return len(_ESC.sub("", s))


def paint(text: str, colour: RGB) -> str:
    """Wrap text in a truecolor escape, or return it bare off a terminal."""
    if not enabled():
        return text
    r, g, b = colour
    return f"\033[38;2;{r};{g};{b}m{text}\033[0m"


def lerp(a: RGB, b: RGB, t: float) -> RGB:
    return (
        round(a[0] + (b[0] - a[0]) * t),
        round(a[1] + (b[1] - a[1]) * t),
        round(a[2] + (b[2] - a[2]) * t),
    )


def ramp(stops: list[RGB], t: float) -> RGB:
    """Sample a list of colour stops at 0..1."""
    p = t * (len(stops) - 1)
    i = min(int(p), len(stops) - 2)
    return lerp(stops[i], stops[i + 1], p - i)


def dim(colour: RGB, k: float) -> RGB:
    return (
        min(255, round(colour[0] * k)),
        min(255, round(colour[1] * k)),
        min(255, round(colour[2] * k)),
    )


def wrap(rows: list[str], cols: int) -> list[str]:
    """Wrap prose to width, never breaking a long token.

    A URL or a one-time code split across lines is unusable, so those are left
    whole even when they overflow.
    """
    out: list[str] = []
    for r in rows:
        if visible_len(r) <= cols:
            out.append(r)
            continue
        plain = _ESC.sub("", r)
        out.extend(
            textwrap.wrap(plain, cols, break_long_words=False, break_on_hyphens=False) or [plain]
        )
    return out


def collapse(rows: list[str]) -> list[str]:
    """Turn two-column ``label   value`` rows into two rows.

    Used when the text column is too narrow to hold the widest line: dropping
    the label keeps the value whole, which matters when the value is a URL.
    """
    out: list[str] = []
    for r in rows:
        plain = _ESC.sub("", r).strip()
        if not plain:
            continue
        head, _, tail = plain.partition("  ")
        if tail.strip():
            out.append(paint(head, MUTED))
            out.append(tail.strip())
        else:
            out.append(r)
    return out


def sigil(
    state: str = "ok", cols: int = 30, rows: int = 15, slab: int = 6, top: int = 1, bot: int = 10
) -> list[str]:
    """The doorway mark: a lit slab with shade spill, a ground line, a reflection.

    ``state`` selects the ramp — ``ok`` lights it with the portal gradient,
    ``bad`` with red, anything else with grey. The picture lives in the
    characters rather than in background colours, so it reads with no colour at
    all and survives a text-animation pass unflattened.
    """
    stops = PORTAL if state == "ok" else (RED_RAMP if state == "bad" else GREY_RAMP)
    edge = rgb("agent") if state == "ok" else (RED_RAMP[2] if state == "bad" else GREY_RAMP[2])
    grid = [[" "] * cols for _ in range(rows)]
    tint: list[list[RGB | None]] = [[None] * cols for _ in range(rows)]

    x0 = (cols - (slab + 6)) // 2
    for y in range(top, bot + 1):
        c = ramp(stops, (y - top) / max(1, bot - top))
        cells = ["░", "░", "▒"] + ["█"] * slab + ["▒", "░", "░"]
        tints = (
            [dim(c, 0.34), dim(c, 0.34), dim(c, 0.62)]
            + [c] * slab
            + [dim(c, 0.62), dim(c, 0.34), dim(c, 0.34)]
        )
        for i, (ch, cc) in enumerate(zip(cells, tints, strict=True)):
            grid[y][x0 + i], tint[y][x0 + i] = ch, cc

    gy = bot + 1
    if gy < rows:
        gc = dim(edge, 0.55)
        for x in range(1, cols - 1):
            grid[gy][x], tint[gy][x] = "-", gc

    rw, base = slab + 2, ramp(stops, 1.0)
    rx = (cols - rw) // 2
    for j, (ch, k) in enumerate((("▓", 0.50), ("▒", 0.32), ("░", 0.18))):
        y = gy + 1 + j
        if y >= rows:
            break
        c = dim(base, k)
        for x in range(rx, rx + rw):
            on_edge = x in (rx, rx + rw - 1)
            grid[y][x] = "░" if on_edge else ch
            tint[y][x] = dim(c, 0.6) if on_edge else c

    return [
        "".join(
            paint(grid[y][x], t) if (t and grid[y][x] != " ") else grid[y][x]
            for x, t in enumerate(tint[y])
        )
        for y in range(rows)
    ]


def card(body: list[str], state: str = "ok", cols: int | None = None, cap: int = 84) -> list[str]:
    """A bordered card: sigil on the left, ``body`` on the right, degrading by width.

    The art is decoration and the words are the job, so as the terminal narrows
    this drops the sigil, then the border, then the two-column labels. The
    thresholds come from the widest line ``body`` needs rather than from fixed
    numbers, which is what keeps a long value — a URL, a one-time code — intact
    all the way down.

    ``cap`` stops a short message stretching across a wide terminal; callers
    with real content raise it.
    """
    if not enabled():
        # No terminal: the words alone. A box and a doorway in a CI log or a
        # journald entry are noise around the only part that matters.
        return [_ESC.sub("", r) for r in body if r.strip()]

    cols = min(cols or term_width(), cap)
    need = max((visible_len(r) for r in body), default=0)
    fr = rgb("agent") if state == "ok" else (RED_RAMP[2] if state == "bad" else GREY_RAMP[2])

    if cols < 44:
        return wrap(collapse(body), cols)

    # borders(2) + left pad(2) + art + gap(3) + text
    for art_cols, kw in (
        (22, dict(cols=22, rows=13, slab=5, top=1, bot=8)),
        (14, dict(cols=14, rows=9, slab=3, top=1, bot=5)),
    ):
        if cols >= 2 + 2 + art_cols + 3 + need:
            art = sigil(state, **kw)  # type: ignore[arg-type]
            off = max(0, (len(art) - len(body)) // 2)
            blank = paint("│", fr) + " " * (cols - 2) + paint("│", fr)
            out = [paint("╭" + "─" * (cols - 2) + "╮", fr), blank]
            # Grows for whichever column is taller: a body longer than the
            # doorway must not be cut off at the art's last row.
            for i in range(max(len(art), off + len(body))):
                right = body[i - off] if 0 <= i - off < len(body) else ""
                left = art[i] if i < len(art) else " " * visible_len(art[0])
                row = "  " + left + "   " + right
                out.append(
                    paint("│", fr)
                    + row
                    + " " * max(0, cols - 2 - visible_len(row))
                    + paint("│", fr)
                )
            out.append(blank)
            out.append(paint("╰" + "─" * (cols - 2) + "╯", fr))
            return out

    lines = wrap(body if cols - 4 >= need else collapse(body), cols - 4)
    out = [paint("╭" + "─" * (cols - 2) + "╮", fr)]
    for r in lines:
        row = "  " + r
        out.append(
            paint("│", fr) + row + " " * max(0, cols - 2 - visible_len(row)) + paint("│", fr)
        )
    out.append(paint("╰" + "─" * (cols - 2) + "╯", fr))
    return out


class Live:
    """Redraws one frame in place, for a surface that polls.

    Off a terminal it prints once and then stays silent, so a log gets the
    content exactly once instead of a flipbook of spinner frames.
    """

    def __init__(self) -> None:
        self._rows = 0
        self._static = not enabled()
        self._done = False

    def __enter__(self) -> Live:
        if not self._static:
            sys.stdout.write("\033[?25l")
        return self

    def __exit__(self, *exc: object) -> None:
        if not self._static:
            sys.stdout.write("\033[?25h")
            sys.stdout.flush()

    def update(self, rows: list[str]) -> None:
        if self._static:
            if not self._done:
                self._done = True
                print("\n".join(_ESC.sub("", r) for r in rows))
            return
        if self._rows:
            sys.stdout.write(f"\033[{self._rows}F")
        sys.stdout.write("\n".join("\033[K" + r for r in rows) + "\n")
        sys.stdout.flush()
        self._rows = len(rows)

    def clear(self) -> None:
        if not self._static and self._rows:
            sys.stdout.write(f"\033[{self._rows}F\033[J")
            sys.stdout.flush()
            self._rows = 0

    def spinner(self) -> Iterator[str]:
        i = 0
        while True:
            yield "" if self._static else SPINNER[i % len(SPINNER)]
            i += 1

    def pause(self, seconds: float, redraw: Callable[[], list[str]] | None = None) -> None:
        """Sleep the full interval, animating meanwhile.

        The caller's polling cadence is unchanged; only the frame rate is ours.
        """
        if self._static or redraw is None:
            time.sleep(seconds)
            return
        end = time.time() + seconds
        while time.time() < end:
            self.update(redraw())
            time.sleep(min(0.1, max(0.0, end - time.time())))


def show(rows: list[str], err: bool = False) -> None:
    """Print rows to stdout or stderr, stripped of colour when that stream is
    not a terminal. The destination decides, not stdout."""
    stream = sys.stderr if err else sys.stdout
    live = stream.isatty() and not os.environ.get("NO_COLOR")
    print("\n".join(rows if live else [_ESC.sub("", r) for r in rows]), file=stream)
