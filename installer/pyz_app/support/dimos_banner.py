#!/usr/bin/env python3
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
import json
import math
import random
import re
import shutil
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable

from rich.console import Console, Group
from rich.live import Live
from rich.text import Text


DEFAULT_BANNER = [
    "██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗           ██████╗ ███████╗",
    "██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║          ██╔═══██╗██╔════╝",
    "██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║          ██║   ██║███████╗",
    "██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║          ██║   ██║╚════██║",
    "██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗     ╚██████╔╝███████║",
    "╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝      ╚═════╝ ╚══════╝",
    "",
    "                                       D I M E N S I O N A L   O S                                ",
]


def _ansi_fg256(n: int) -> str:
    return f"\x1b[38;5;{n}m"


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
    ) -> None:
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
        self._thread = threading.Thread(target=self._run, name="RenderLogo", daemon=True)

        # Ensure we restore the terminal even if user forgets.
        atexit.register(self.stop)

        self._console = Console()
        self._live = Live(
            console=self._console,
            refresh_per_second=self.fps,
            screen=True,  # alternate screen to avoid scrollback spam
            auto_refresh=False,
        )
        self._live.start()

        self._thread.start()

    # -----------------------
    # Public API
    # -----------------------

    def log(self, *args: Any) -> None:
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
        try:
            self._thread.join(timeout=0.2)
        except Exception:
            pass

        # Restore terminal
        try:
            self._live.stop()
        except Exception:
            pass
        try:
            self._console.show_cursor(True)
        except Exception:
            pass

    # -----------------------
    # Loop + rendering
    # -----------------------

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            start = time.monotonic()
            self._spawn_glitches()
            self._tick_glitches()

            frame = self.render(self._t)
            self._t += 1

            try:
                self._live.update(frame, refresh=True)
            except Exception:
                # If Live cannot render for some reason, fall back to no-op to avoid crash.
                pass

            elapsed = time.monotonic() - start
            sleep_for = self.frame_s - elapsed
            if sleep_for > 0:
                self._stop_evt.wait(sleep_for)

    def render(self, t: int):
        cols, rows = self._term_size()

        max_len = max((len(l) for l in self.banner), default=0)
        left_pad = max(0, (cols - max_len) // 2)
        content_width = max(1, cols - left_pad)

        # Compute how many rows we can use for logs and how much slack is left.
        static_height = len(self.banner) + 2  # logo block + blank + separator
        max_log_rows = max(1, rows - static_height)
        start_idx = max(0, len(self._log_lines) - max_log_rows)
        visible = self._log_lines[start_idx:]

        # If logs don't fill the available space, use some of the slack to push the banner down.
        unused_rows = max(0, rows - static_height - len(visible))
        top_pad = unused_rows // 2  # simple centering of the whole block
        bottom_pad = unused_rows - top_pad

        renderables = []

        # Top padding
        for _ in range(top_pad):
            renderables.append(Text(""))

        # Logo
        for y, line in enumerate(self.banner):
            row = Text(" " * left_pad)
            for x, orig_ch in enumerate(line):
                key = f"{y},{x}"
                g = self._glitches.get(key)
                ch = g.ch if g else orig_ch
                color = self._color_for(x, y, t, is_glitched=bool(g))
                row.append(ch, style=f"color({color})")
            renderables.append(row)

        # Separator + blank
        renderables.append(Text(""))
        sep = self.separator_char * min(max_len, content_width)
        renderables.append(Text(" " * left_pad + sep, style="dim"))

        if self.scrollable:
            for l in visible:
                # stored lines are wrapped; final hard-cut if terminal shrunk
                if len(l) > content_width:
                    trimmed = l[: max(0, content_width - 1)] + "…"
                else:
                    trimmed = l
                renderables.append(Text(" " * left_pad + trimmed))

        # Bottom padding to fill remaining space (and avoid scrolling).
        while len(renderables) < rows - bottom_pad:
            renderables.append(Text(" " * left_pad))
        for _ in range(bottom_pad):
            renderables.append(Text(" " * left_pad))

        # Trim to exactly the terminal height to prevent scrollback growth.
        renderables = renderables[: rows]

        return Group(*renderables)

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
        # Similar "rowPhase" trick to JS; Python doesn't have >>>, so keep it simple+stable.
        row_phase = ((y * 1103515245 + 12345) % 1000) / 1000.0

        base = 38
        w = math.sin(t * self.wave_speed + x * self.wave_freq + row_phase * math.tau) * 0.5 + 0.5
        c = base + round(w * self.wave_strength)

        if is_glitched:
            return 51
        return max(16, min(231, int(c)))

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
            logo.log(f"[{i:02d}] hello from RenderLogo — a moderately long line to demonstrate wrapping behavior.")
            time.sleep(0.05)

        logo.log("Press Ctrl+C to stop.")
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        logo.stop()
