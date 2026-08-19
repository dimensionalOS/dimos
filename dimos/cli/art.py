# Copyright 2026 Dimensional Inc.
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

"""Terminal art for `dimos login` / `logout` / `whoami`.

Pure rendering: every function returns a list of ready-to-print rows and touches
no network, no filesystem and no credentials. `cloud.py` owns all of that.

The picture is a lit doorway — a slab with shade spill, a ground line and its
reflection — coloured by where you are in the flow: grey while nothing has
happened, red when something is refused, and the portal gradient once you are
through. The gradient stops are sampled from the brand asset rather than
invented.

Everything is carried by the *characters*, not by background colours, which is
what lets `beams` animate the wordmark without flattening it.

Nothing here draws when stdout is not a terminal: the same information comes
back as plain lines, so CI logs and journald stay readable.
"""

from collections.abc import Callable, Iterator, Sequence
import importlib.metadata
import os
import re
import shutil
import sys
import textwrap
import time

from terminaltexteffects.effects.effect_beams import Beams

from dimos.cli import theme

RGB = tuple[int, int, int]

# Sampled down the centre of the portal in the brand asset: pale yellow at the
# lintel, through cream and mint, to salmon at the threshold.
PORTAL: list[RGB] = [
    (0xF4, 0xF1, 0x9F),
    (0xFC, 0xEE, 0xB0),
    (0xFB, 0xE5, 0xC5),
    (0xEB, 0xE6, 0xD6),
    (0xCF, 0xEB, 0xCD),
    (0xBC, 0xF6, 0xC1),
    (0xB9, 0xF5, 0xBB),
    (0xDA, 0xE8, 0xBC),
    (0xFA, 0xE1, 0xBD),
    (0xF3, 0xB0, 0x97),
    (0xF3, 0xAA, 0x7B),
]
GREY: list[RGB] = [(0x36, 0x3F, 0x3F), (0x6E, 0x7E, 0x7E), (0x7E, 0x8E, 0x8E), (0x5A, 0x66, 0x66)]
RED: list[RGB] = [(0x5E, 0x1E, 0x1E), (0xC8, 0x3A, 0x30), (0xFF, 0x6A, 0x5F), (0x8A, 0x2A, 0x24)]

# dimos.tcss tokens
CYAN: RGB = (0x00, 0xEE, 0xEE)
PURPLE: RGB = (0xB0, 0x6B, 0xFF)
GREEN: RGB = (0x88, 0xFF, 0x88)
MINT: RGB = (0xB9, 0xF5, 0xBB)
TEXT: RGB = (0xB5, 0xE4, 0xF4)
DIM: RGB = (0x5A, 0x6A, 0x6A)
SALMON: RGB = (0xF3, 0xAA, 0x7B)

SPINNER = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"
_ESC = re.compile(r"\x1b\[[0-9;]*m")

# The panel says what DimOS is, not what you can type: `dimos --help` is the
# exhaustive command list and does that job better.
ABOUT = (
    "DimOS is the open-source operating system for physical space. Our platform lets "
    "developers build physical AI applications with natural-language "
    "control, across any hardware form factor: humanoids, quadrupeds, drones, and robotic arms."
)
COMMUNITY = "discord.gg/dimos"
CAPABILITIES: list[tuple[str, str]] = [
    ("Navigation", "SLAM, obstacle avoidance, route planning, exploration"),
    ("Perception", "detection, tracking, spatial memory"),
    ("Manipulation", "grasp synthesis, pick and place, constraints"),
]


def enabled() -> bool:
    """Art draws only for a human at a terminal, and never under NO_COLOR."""
    return sys.stdout.isatty() and not os.environ.get("NO_COLOR")


def width() -> int:
    return shutil.get_terminal_size((100, 24)).columns


def _vis(s: str) -> int:
    return len(_ESC.sub("", s))


def _paint(text: str, rgb: RGB) -> str:
    if not enabled():
        return text
    r, g, b = rgb
    return f"\033[38;2;{r};{g};{b}m{text}\033[0m"


def _lerp(a: RGB, b: RGB, t: float) -> RGB:
    return (
        round(a[0] + (b[0] - a[0]) * t),
        round(a[1] + (b[1] - a[1]) * t),
        round(a[2] + (b[2] - a[2]) * t),
    )


def _ramp(stops: Sequence[RGB], t: float) -> RGB:
    p = t * (len(stops) - 1)
    i = min(int(p), len(stops) - 2)
    return _lerp(stops[i], stops[i + 1], p - i)


def _dim(rgb: RGB, k: float) -> RGB:
    return (min(255, round(rgb[0] * k)), min(255, round(rgb[1] * k)), min(255, round(rgb[2] * k)))


def _stops(state: str) -> list[RGB]:
    return PORTAL if state == "ok" else (RED if state == "bad" else GREY)


def _border(state: str) -> RGB:
    return GREEN if state == "ok" else (RED[2] if state == "bad" else GREY[2])


def _wrap(rows: list[str], cols: int) -> list[str]:
    """Wrap prose, but never break a URL or a device code across lines."""
    out: list[str] = []
    for r in rows:
        if _vis(r) <= cols:
            out.append(r)
            continue
        plain = _ESC.sub("", r)
        out.extend(
            textwrap.wrap(plain, cols, break_long_words=False, break_on_hyphens=False) or [plain]
        )
    return out


def _collapse(rows: list[str]) -> list[str]:
    """Two-column rows become two rows, so a narrow terminal never truncates
    the value half — a clipped URL or code makes the command unusable."""
    out: list[str] = []
    for r in rows:
        plain = _ESC.sub("", r).strip()
        if not plain:
            continue
        head, _, tail = plain.partition("  ")
        if tail.strip():
            out.append(_paint(head, DIM))
            out.append(tail.strip())
        else:
            out.append(r)
    return out


def sigil(
    state: str = "ok", cols: int = 30, rows: int = 15, slab: int = 6, top: int = 1, bot: int = 10
) -> list[str]:
    """The doorway: a lit slab with shade spill, a ground line and a reflection."""
    stops = _stops(state)
    grid = [[" "] * cols for _ in range(rows)]
    tint: list[list[RGB | None]] = [[None] * cols for _ in range(rows)]

    x0 = (cols - (slab + 6)) // 2
    for y in range(top, bot + 1):
        c = _ramp(stops, (y - top) / max(1, bot - top))
        cells = ["░", "░", "▒"] + ["█"] * slab + ["▒", "░", "░"]
        tints = (
            [_dim(c, 0.34), _dim(c, 0.34), _dim(c, 0.62)]
            + [c] * slab
            + [_dim(c, 0.62), _dim(c, 0.34), _dim(c, 0.34)]
        )
        for i, (ch, cc) in enumerate(zip(cells, tints, strict=True)):
            grid[y][x0 + i], tint[y][x0 + i] = ch, cc

    gy = bot + 1
    if gy < rows:
        gc = _dim(_border(state), 0.55)
        for x in range(1, cols - 1):
            grid[gy][x], tint[gy][x] = "-", gc

    rw, base = slab + 2, _ramp(stops, 1.0)
    rx = (cols - rw) // 2
    for j, (ch, k) in enumerate((("▓", 0.50), ("▒", 0.32), ("░", 0.18))):
        y = gy + 1 + j
        if y >= rows:
            break
        c = _dim(base, k)
        for x in range(rx, rx + rw):
            edge = x in (rx, rx + rw - 1)
            grid[y][x] = "░" if edge else ch
            tint[y][x] = _dim(c, 0.6) if edge else c

    return [
        "".join(
            _paint(grid[y][x], t) if (t and grid[y][x] != " ") else grid[y][x]
            for x, t in enumerate(tint[y])
        )
        for y in range(rows)
    ]


def card(body: list[str], state: str = "ok", cols: int | None = None, cap: int = 84) -> list[str]:
    """Door left, text right, gutter all round — degrading by what fits.

    The art is decoration; the URL and the device code are the job. So the sigil
    goes first, then the border, then the layout. Thresholds are derived from the
    widest line the body needs, not guessed.
    """
    if not enabled():
        # No terminal: the words, nothing else. A box and a doorway in a CI log
        # or a journald entry are noise around the only part that matters.
        return [_ESC.sub("", r) for r in body if r.strip()]

    # Short messages should not stretch across a wide terminal; a card with real
    # content should. Callers raise the cap when they have something to say.
    cols = min(cols or width(), cap)
    need = max((_vis(r) for r in body), default=0)
    fr = _border(state)

    if cols < 44:
        return _wrap(_collapse(body), cols)

    for aw, kw in (
        (22, dict(cols=22, rows=13, slab=5, top=1, bot=8)),
        (14, dict(cols=14, rows=9, slab=3, top=1, bot=5)),
    ):
        if cols >= 2 + 2 + aw + 3 + need:
            art = sigil(state, **kw)  # type: ignore[arg-type]
            off = max(0, (len(art) - len(body)) // 2)
            blank = _paint("│", fr) + " " * (cols - 2) + _paint("│", fr)
            out = [_paint("╭" + "─" * (cols - 2) + "╮", fr), blank]
            # The card grows for whichever column is taller. A body longer than
            # the doorway used to be silently cut off at the art's last row.
            for i in range(max(len(art), off + len(body))):
                right = body[i - off] if 0 <= i - off < len(body) else ""
                row = "  " + (art[i] if i < len(art) else " " * _vis(art[0])) + "   " + right
                out.append(
                    _paint("│", fr) + row + " " * max(0, cols - 2 - _vis(row)) + _paint("│", fr)
                )
            out.append(blank)
            out.append(_paint("╰" + "─" * (cols - 2) + "╯", fr))
            return out

    lines = _wrap(body if cols - 4 >= need else _collapse(body), cols - 4)
    out = [_paint("╭" + "─" * (cols - 2) + "╮", fr)]
    for r in lines:
        row = "  " + r
        out.append(_paint("│", fr) + row + " " * max(0, cols - 2 - _vis(row)) + _paint("│", fr))
    out.append(_paint("╰" + "─" * (cols - 2) + "╯", fr))
    return out


def login_rows(uri: str, code: str, spin: str = "", clock: str = "") -> list[str]:
    """The one card the whole wait lives in: URL, code, and a ticking last line."""
    body = [
        _paint("Sign in to Dimensional", GREY[2]),
        _paint(f"Open      {uri}", DIM),
        "",
        _paint(f"Code      {code}", TEXT),
    ]
    if spin:
        body.append(
            _paint(f"{spin} waiting for approval", CYAN) + _paint(f"   expires in {clock}", DIM)
        )
    return card(body, "idle")


def signed_in_rows(email: str, key_id: str, where: str, version: str = "") -> list[str]:
    """One card: who you are at the top, then what DimOS is.

    Deliberately not two stacked cards — the account line and the introduction
    are one moment, and two borders read as two unrelated things.
    """
    cols = min(width(), 110)
    # Never let the wrap width go non-positive: card() re-wraps for narrow
    # terminals anyway, this only needs to be sane enough to build the rows.
    body = max(24, cols - 2 - (22 + 3) - 4)

    rows = [_paint("Signed in", GREEN), _paint(email, TEXT), ""]
    rows.append(_paint(f"key      {key_id}… · {where}", DIM))
    if version:
        rows.append(_paint(f"version  DimOS v{version}", DIM))
    rows += ["", _paint("About DimOS", TEXT)]
    rows += [_paint(ln, DIM) for ln in textwrap.wrap(ABOUT, body)]
    rows += ["", _paint("Join our community: ", DIM) + _paint(COMMUNITY, CYAN), ""]
    for name, blurb in CAPABILITIES:
        rows.append(_paint("▸ ", SALMON) + _paint(name, TEXT))
        rows += [_paint("   " + ln, DIM) for ln in textwrap.wrap(blurb, body - 3)]
        rows.append("")
    return card(rows[:-1], "ok", cols, cap=110)


def refused_rows(title: str, *details: str, hint: str = "dimos login") -> list[str]:
    body = [_paint(title, RED[2])]
    body += [_paint(d, DIM) for d in details]
    return card([*body, "", _paint(hint, CYAN)], "bad")


def logged_out_rows(revoke_url: str) -> list[str]:
    return card(
        [
            _paint("Logged out", GREY[2]),
            _paint("Key removed from this machine.", DIM),
            _paint("It stays valid until revoked.", DIM),
            "",
            _paint(revoke_url, CYAN),
        ],
        "idle",
    )


def whoami_rows(email: str, scopes: str) -> list[str]:
    """One line, deliberately. `whoami` is a lookup people script against, not a
    moment — a card with art here would be ceremony for an answer to a question."""
    return [_paint(email, TEXT) + _paint(f" (scopes: {scopes})", DIM)]


def panel_rows(version: str) -> list[str]:
    """What `dimos login` opens onto: the sigil beside what DimOS is."""
    cols = min(width(), 110)
    fr = GREEN
    title = f" DimOS v{version} "

    if cols < 64:
        return [_paint(f"DimOS v{version}", TEXT), _paint(COMMUNITY, CYAN)]

    art = sigil("ok") if cols >= 100 else []
    aw = max((_vis(r) for r in art), default=0)
    body = cols - 2 - (aw + 3 if art else 1) - 2

    right = [_paint(ln, DIM) for ln in textwrap.wrap(ABOUT, body)]
    right += ["", _paint("Join our community: ", DIM) + _paint(COMMUNITY, CYAN), ""]
    for name, blurb in CAPABILITIES:
        right.append(_paint("▸ ", SALMON) + _paint(name, TEXT))
        right += [_paint("   " + ln, DIM) for ln in textwrap.wrap(blurb, body - 3)]
        right.append("")

    out = [_paint("╭─" + title + "─" * max(0, cols - 3 - len(title)) + "╮", fr)]
    for i in range(max(len(art), len(right))):
        left = (art[i] if i < len(art) else " " * aw) if art else ""
        row = (" " + left + "  " if art else " ") + (right[i] if i < len(right) else "")
        out.append(_paint("│", fr) + row + " " * max(0, cols - 2 - _vis(row)) + _paint("│", fr))
    out.append(_paint("╰" + "─" * (cols - 2) + "╯", fr))
    return out


def wordmark_rows() -> list[str]:
    lines = theme.ascii_logo.strip("\n").split("\n")
    return [_paint(ln, _ramp(PORTAL, i / max(1, len(lines) - 1))) for i, ln in enumerate(lines)]


class Live:
    """Redraws one frame in place. Outside a terminal it prints once and stops,
    so a log gets the code exactly once instead of a flipbook of spinner frames."""

    def __init__(self) -> None:
        self._rows = 0
        self._static = not enabled()
        self._done = False

    def __enter__(self) -> "Live":
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
        """Sleep the full interval, animating meanwhile. The caller's polling
        cadence is unchanged; only the frame rate is ours."""
        if self._static or redraw is None:
            time.sleep(seconds)
            return
        end = time.time() + seconds
        while time.time() < end:
            self.update(redraw())
            time.sleep(min(0.1, max(0.0, end - time.time())))


def reveal(rows: list[str]) -> None:
    """Sweep the wordmark in, then leave it. Falls back to a plain print if the
    effects library is unavailable or we are not on a terminal."""
    if not enabled():
        return
    effect = Beams("\n".join(rows))
    # Without this the effect crops to the width it detects and the wordmark
    # loses its last letter. The art carries its own dimensions.
    effect.terminal_config.ignore_terminal_dimensions = True
    effect.terminal_config.frame_rate = 100_000
    frames = list(effect)
    started, n = time.time(), 60
    for k in range(n):
        sys.stdout.write(
            "\033[H" + frames[min(len(frames) - 1, int(len(frames) * k / (n - 1)))] + "\n"
        )
        sys.stdout.flush()
        slack = started + (k + 1) / 30 - time.time()
        if slack > 0:
            time.sleep(slack)


def show(rows: list[str], err: bool = False) -> None:
    """Print rows, stripped of colour when this is not a terminal."""
    out = "\n".join(rows if enabled() else [_ESC.sub("", r) for r in rows])
    print(out, file=sys.stderr if err else sys.stdout)


def version() -> str:
    """The installed version, for the panel title. Never fatal."""
    try:
        return importlib.metadata.version("dimos")
    except importlib.metadata.PackageNotFoundError:
        return "dev"
