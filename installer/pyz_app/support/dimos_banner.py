#!/usr/bin/env python3
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

"""
RenderLogo: animated terminal logo w/ wave colors + glitch chars + scrollable log area.

Python 3.10+
No external deps.

Notes:
- Uses ANSI escape sequences (needs a real terminal).
- 256-color capable terminals recommended.
"""

from __future__ import annotations

import atexit
from collections.abc import Iterable
from dataclasses import dataclass
import colorsys
import json
import math
import random
import re
import shutil
import sys
import threading
import time
from typing import Any

DEFAULT_BANNER = [
    "██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗           ██████╗ ███████╗",
    "██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║          ██╔═══██╗██╔════╝",
    "██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║          ██║   ██║███████╗",
    "██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║          ██║   ██║╚════██║",
    "██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗     ╚██████╔╝███████║",
    "╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝      ╚═════╝ ╚══════╝",
    "",
    "                                         D I M E N S I O N A L   O S                                         ",
]

ASCII_BANNER_80 = [
    "█████╗ █╗██╗   ██╗████╗██╗   █╗████╗█╗ ████╗ ██╗   █╗ ███╗ ██╗       ████╗ ████╗",
    "█╔══██╗█║███╗ ███║█╔══╝███╗  █║█╔══╝█║██╔═██╗███╗  █║█╔═██╗██║      ██╔═██╗█╔══╝",
    "█║  ██║█║█╔████╔█║███  █╔██╗ █║████╗█║██║ ██║█╔██╗ █║█████║██║      ██║ ██║████╗",
    "█║  ██║█║█║╚██╔╝█║█╔═  █║╚██╗█║╚══█║█║██║ ██║█║╚██╗█║█╔═██║██║      ██║ ██║╚══█║",
    "█████╔╝█║█║ ╚═╝ █║████╗█║ ╚███║████║█║╚████╔╝█║ ╚███║█║ ██║█████╗   ╚████╔╝████║",
    "╚════╝ ╚╝═╝     ╚╝╚═══╝╚╝  ╚══╝╚═══╝╚╝ ╚═══╝ ╚╝  ╚══╝╚╝ ╚═╝╚════╝    ╚═══╝ ╚═══╝",
    "",
    "                          D I M E N S I O N A L   O S                           ",
]


MINI_BANNER = [
    "                                 ",
    "   D I M E N S I O N A L   O S   ",
    "                                 ",
]


def _ansi_fg256(n: int) -> str:
    return f"\x1b[38;5;{n}m"


def _build_palette() -> list[tuple[int, int, int]]:
    # 0-15: standard ANSI colors
    ansi16 = [
        (0, 0, 0),
        (128, 0, 0),
        (0, 128, 0),
        (128, 128, 0),
        (0, 0, 128),
        (128, 0, 128),
        (0, 128, 128),
        (192, 192, 192),
        (128, 128, 128),
        (255, 0, 0),
        (0, 255, 0),
        (255, 255, 0),
        (0, 0, 255),
        (255, 0, 255),
        (0, 255, 255),
        (255, 255, 255),
    ]
    palette: list[tuple[int, int, int]] = ansi16[:]

    # 16-231: 6x6x6 color cube
    levels = [0, 95, 135, 175, 215, 255]
    for r in levels:
        for g in levels:
            for b in levels:
                palette.append((r, g, b))

    # 232-255: grayscale ramp
    for i in range(24):
        v = 8 + i * 10
        palette.append((v, v, v))

    return palette


_PALETTE_RGB = _build_palette()


def _rgb_to_idx(rgb: tuple[int, int, int]) -> int:
    # Find nearest palette entry (Euclidean)
    r, g, b = rgb
    best_idx = 0
    best_dist = float("inf")
    for idx, (pr, pg, pb) in enumerate(_PALETTE_RGB):
        dr = pr - r
        dg = pg - g
        db = pb - b
        dist = dr * dr + dg * dg + db * db
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def _ansi_fg256_hue_shift(color_idx: int, shift_pct: float) -> str:
    """Return ANSI fg code with hue shifted by shift_pct (0-100)."""
    color_idx = max(0, min(255, int(color_idx)))
    shift_pct = max(0.0, min(100.0, float(shift_pct)))
    r, g, b = _PALETTE_RGB[color_idx]
    h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    h = (h + (shift_pct / 100.0)) % 1.0
    nr, ng, nb = colorsys.hsv_to_rgb(h, s, v)
    new_idx = _rgb_to_idx((int(nr * 255), int(ng * 255), int(nb * 255)))
    return new_idx


ANSI_RESET = "\x1b[0m"
ANSI_DIM = "\x1b[2m"
ANSI_CURSOR_HIDE = "\x1b[?25l"
ANSI_CURSOR_SHOW = "\x1b[?25h"
ANSI_HOME = "\x1b[H"
ANSI_ERASE_DOWN = "\x1b[J"


@dataclass
class _Glitch:
    orig: str
    ch: str
    ttl: int


class RenderLogo:
    GLITCH_CHARS = "▓▒░█#@$%&*+=-_:;!?/~\\|()[]{}<>^"

    def __init__(
        self,
        *,
        banner: list[str] = DEFAULT_BANNER,
        glitchyness: float = 10,
        stickyness: int = 14,
        fps: int = 30,
        wave_strength: int = 10,
        wave_speed: float = 0.1,
        wave_freq: float = 0.08,
        glitch_mutate_chance: float = 0.08,
        scrollable: bool = True,
        max_stored_lines: int = 50_000,
        separator_char: str = "─",
        wrap_long_words: bool = True,
        is_centered: bool = False,
    ) -> None:
        self._enabled = sys.stdout.isatty()
        self.banner = banner
        self.glitchyness = glitchyness
        self.stickyness = stickyness
        self.fps = max(1, int(fps))
        self.wave_strength = wave_strength
        self.wave_speed = wave_speed
        self.wave_freq = wave_freq
        self.glitch_mutate_chance = glitch_mutate_chance

        self.scrollable = scrollable
        self.max_stored_lines = max_stored_lines
        self.separator_char = separator_char
        self.wrap_long_words = wrap_long_words
        self.is_centered = is_centered

        self.frame_s = max(0.001, 1.0 / self.fps)

        # precompute mutable positions (non-space)
        self._mutable: list[tuple[int, int]] = []
        for y, line in enumerate(self.banner):
            for x, ch in enumerate(line):
                if ch not in (" ", "\t"):
                    self._mutable.append((y, x))

        self._glitches: dict[str, _Glitch] = {}  # "y,x" -> _Glitch
        self._log_lines: list[str] = []

        self._t = 0
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None

        # Ensure we restore the terminal even if user forgets.
        atexit.register(self.stop)

        if self._enabled:
            self._thread = threading.Thread(target=self._run, name="RenderLogo", daemon=True)
            self._thread.start()

    # -----------------------
    # Public API
    # -----------------------

    def log(self, *args: Any) -> None:
        if not self._enabled:
            msg = " ".join(self._stringify(a) for a in args)
            # match log-update fallback: just print a line without animation churn
            print(msg)
            return

        msg = " ".join(self._stringify(a) for a in args)

        cols, _rows = self._term_size()
        cols = min(110, cols)

        max_len = max((len(l) for l in self.banner), default=0)
        left_pad = max(0, (cols - max_len) // 2)
        content_width = max(10, cols - left_pad)

        for raw_line in msg.splitlines() or [""]:
            for wl in self._wrap_line(raw_line, content_width):
                self._log_lines.append(wl)

        if len(self._log_lines) > self.max_stored_lines:
            del self._log_lines[: len(self._log_lines) - self.max_stored_lines]

    def get_log_lines(self) -> list[str]:
        return list(self._log_lines)

    def stop(self) -> None:
        if self._stop_evt.is_set():
            return
        self._stop_evt.set()
        # Best-effort join without risking hangs in weird envs.
        if self._thread:
            try:
                self._thread.join(timeout=0.2)
            except Exception:
                pass

        # Clear animation artifacts and restore cursor.
        if self._enabled:
            try:
                sys.stdout.write(ANSI_CURSOR_SHOW + ANSI_RESET + "\n")
                sys.stdout.flush()
            except Exception:
                pass

    # -----------------------
    # Loop + rendering
    # -----------------------

    def _run(self) -> None:
        # Hide cursor while running.
        sys.stdout.write(ANSI_CURSOR_HIDE)
        sys.stdout.flush()

        while not self._stop_evt.is_set():
            start = time.monotonic()
            self._spawn_glitches()
            self._tick_glitches()

            frame = self.render(self._t)
            self._t += 1

            sys.stdout.write(frame)
            sys.stdout.flush()

            elapsed = time.monotonic() - start
            sleep_for = self.frame_s - elapsed
            if sleep_for > 0:
                self._stop_evt.wait(sleep_for)

    def render(self, t: int) -> str:
        cols, rows = self._term_size()

        # Pick the largest banner that fits.
        banners = [
            DEFAULT_BANNER,
            ASCII_BANNER_80,
            MINI_BANNER,
        ]
        chosen = MINI_BANNER
        for candidate in banners:
            width = max((len(l) for l in candidate), default=0)
            if width <= cols:
                chosen = candidate
                break
        self.banner = chosen

        max_len = max((len(l) for l in self.banner), default=0)
        if self.is_centered:
            left_pad = max(0, (cols - max_len) // 2)
        else:
            left_pad = 0
        content_width = max(1, cols - left_pad)

        # Compute how many rows we can use for logs and how much slack is left.
        static_height = len(self.banner) + 2  # logo block + blank + separator
        max_log_rows = max(1, rows - static_height)
        start_idx = max(0, len(self._log_lines) - max_log_rows)
        visible = self._log_lines[start_idx:]

        # Keep banner near top; no extra top padding, bottom fills rest.
        top_pad = 0
        bottom_pad = max(0, rows - (static_height + len(visible)))

        out_lines: list[str] = []
        # mimic log-update behavior: move to home and erase down, then draw
        out_lines.append(ANSI_HOME + ANSI_ERASE_DOWN)

        # Optional top padding to avoid hugging the top of the terminal.
        for _ in range(top_pad):
            out_lines.append("")

        # Logo
        for y, line in enumerate(self.banner):
            out = " " * left_pad
            for x, orig_ch in enumerate(line):
                key = f"{y},{x}"
                g = self._glitches.get(key)
                ch = g.ch if g else orig_ch
                color = self._color_for(x, y, t, is_glitched=bool(g))
                out += _ansi_fg256(color) + ch + ANSI_RESET
            out_lines.append(out)

        # Separator + blank
        out_lines.append("")
        sep = self.separator_char * min(max_len, content_width)
        out_lines.append((" " * left_pad) + ANSI_DIM + sep + ANSI_RESET)

        if self.scrollable:
            for l in visible:
                # stored lines are wrapped; final hard-cut if terminal shrunk
                if len(l) > content_width:
                    trimmed = l[: max(0, content_width - 1)] + "…"
                else:
                    trimmed = l
                out_lines.append((" " * left_pad) + trimmed)

            # pad to keep stable frame height (bottom padding after logs)
            pad_needed = max(0, rows - len(out_lines) - bottom_pad)
            for _ in range(bottom_pad + pad_needed):
                out_lines.append(" " * left_pad)

        return "\n".join(out_lines)

    # -----------------------
    # Glitches + colors
    # -----------------------

    def _spawn_glitches(self) -> None:
        # requested probability gate (same semantics as JS)
        if 0 < self.glitchyness < 1:
            if random.random() > self.glitchyness:
                return

        count = int(self.glitchyness) if self.glitchyness >= 1 else 1
        if not self._mutable:
            return

        for _ in range(count):
            y, x = random.choice(self._mutable)
            key = f"{y},{x}"
            orig = self.banner[y][x]

            existing = self._glitches.get(key)
            if existing:
                existing.ttl = max(existing.ttl, self.stickyness)
                continue

            ch = orig
            for _tries in range(6):
                ch = random.choice(self.GLITCH_CHARS)
                if ch != orig:
                    break

            self._glitches[key] = _Glitch(orig=orig, ch=ch, ttl=self.stickyness)

    def _tick_glitches(self) -> None:
        dead: list[str] = []
        for key, g in self._glitches.items():
            g.ttl -= 1
            if g.ttl <= 0:
                dead.append(key)
            elif random.random() < self.glitch_mutate_chance:
                g.ch = random.choice(self.GLITCH_CHARS)

        for key in dead:
            self._glitches.pop(key, None)

    def _color_for(self, x: int, y: int, t: int, *, is_glitched: bool) -> int:
        row_phase = ((y * 1103515245 + 12345) % 1000) / 1000.0

        blue_base = 28
        blue_span = max(1, self.wave_strength)
        # Keep wave mostly high to stay in blue; range ~0.15–1.0
        # w = math.sin(t * self.wave_speed + x * self.wave_freq + row_phase * math.tau) * 0.25 + 0.85
        w = math.sin(t * self.wave_speed + x * self.wave_freq + row_phase * math.tau) * 0.25 + 0.75
        c = blue_base + round(w * blue_span)

        if is_glitched:
            return 51
        normal_output = max(16, min(231, int(c)))
        return _ansi_fg256_hue_shift(normal_output, 5)

    # -----------------------
    # Wrapping + utils
    # -----------------------

    def _wrap_line(self, line: str, width: int) -> list[str]:
        if width <= 1:
            return [line]

        # preserve leading indentation
        m = re.match(r"^\s*", line)
        indent = m.group(0) if m else ""
        content = line[len(indent) :]

        if not content:
            return [indent]

        def hard_chunk(s: str, w: int) -> list[str]:
            return [s[i : i + w] for i in range(0, len(s), w)]

        has_spaces = bool(re.search(r"\s", content))
        max_content = max(1, width - len(indent))

        # If no spaces and too long, treat as long-word case
        if (not has_spaces) and (len(content) > max_content):
            chunks = hard_chunk(content, max_content)
            return [indent + c for c in chunks]

        words = [w for w in re.split(r"\s+", content) if w]
        lines: list[str] = []
        cur = indent
        cur_len = len(indent)

        def push_cur() -> None:
            nonlocal cur, cur_len
            lines.append(cur.rstrip())
            cur = indent
            cur_len = len(indent)

        for word in words:
            if len(word) > max_content:
                if cur_len > len(indent):
                    push_cur()
                chunks = hard_chunk(word, max_content)
                for c in chunks:
                    lines.append(indent + c)
                continue

            sep = " " if cur_len > len(indent) else ""
            add_len = len(sep) + len(word)

            if cur_len + add_len <= width:
                cur += sep + word
                cur_len += add_len
            else:
                push_cur()
                cur += word
                cur_len += len(word)

        if cur_len > len(indent):
            lines.append(cur.rstrip())

        return lines or [indent]

    @staticmethod
    def _term_size() -> tuple[int, int]:
        ts = shutil.get_terminal_size(fallback=(80, 24))
        return ts.columns, ts.lines

    @staticmethod
    def _stringify(a: Any) -> str:
        if isinstance(a, str):
            return a
        try:
            return json.dumps(a)
        except Exception:
            return str(a)


# -----------------------
# Example usage
# -----------------------
if __name__ == "__main__":
    logo = RenderLogo(glitchyness=10, stickyness=14, fps=30, scrollable=True)

    try:
        for i in range(1, 51):
            logo.log(
                f"[{i:02d}] hello from RenderLogo — a moderately long line to demonstrate wrapping behavior."
            )
            time.sleep(0.05)

        logo.log("Press Ctrl+C to stop.")
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        logo.stop()
