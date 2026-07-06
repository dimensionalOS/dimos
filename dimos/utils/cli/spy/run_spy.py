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

"""`dimos spy` TUI: live table of all topics across all pubsub transports.

Textual app (DataTable, 0.5s refresh, theme colors, 'q' to quit). One row per
(transport, topic) from TransportSpy.snapshot():

    Transport | Topic | Type | Freq (Hz) | Bandwidth | Total | Age

- Topic/Type come from split_type_suffix(); rows sort by total traffic.
- Age is seconds since TopicStats.last_seen (liveness: dims/greys out stale rows).
- `--transport lcm --transport zenoh` (repeatable) filters sources; default all.
- LCM system config warning: call lcmservice.autoconf(check_only=True) before
  entering raw TUI mode.
"""

from __future__ import annotations

import time

from rich.text import Text
from textual.app import App, ComposeResult
from textual.color import Color
from textual.widgets import DataTable

from dimos.protocol.pubsub.spy import (
    SOURCE_FACTORIES,
    SpyKey,
    TopicStats,
    TransportSpy,
    default_sources,
    split_type_suffix,
)
from dimos.utils.cli import theme
from dimos.utils.human import human_bytes

# Rows older than this (seconds since last message) render dimmed as "stale".
STALE_AGE = 3.0
# Window for freq / bandwidth readouts.
STAT_WINDOW = 5.0


def gradient(max_value: float, value: float) -> str:
    """Gradient from cyan (low) to yellow (high) using DimOS theme colors."""
    ratio = min(value / max_value, 1.0) if max_value else 0.0
    cyan = Color.parse(theme.CYAN)
    yellow = Color.parse(theme.YELLOW)
    return cyan.blend(yellow, ratio).hex


def topic_text(base: str) -> Text:
    """Format a base topic name with DimOS theme colors."""
    if base[:4] == "/rpc":
        return Text(base[:4], style=theme.BLUE) + Text(base[4:], style=theme.BRIGHT_WHITE)
    return Text(base, style=theme.BRIGHT_WHITE)


def _parse_transports(argv: list[str]) -> list[str] | None:
    """Parse repeated --transport flags; None means all default sources.

    `dimos spy` accepts only `--transport <name>` (repeatable). Any other token
    — a stray positional or unknown flag — is rejected with a clear error rather
    than silently ignored.
    """
    transports: list[str] = []
    extra: list[str] = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--transport":
            i += 1
            if i < len(argv):
                transports.append(argv[i])
            else:
                raise SystemExit("Error: --transport requires a transport name")
        elif arg.startswith("--transport="):
            transports.append(arg.split("=", 1)[1])
        else:
            extra.append(arg)
        i += 1
    if extra:
        raise SystemExit(
            f"Error: unexpected argument(s) {', '.join(extra)} — "
            "`dimos spy` accepts only --transport <name> (repeatable)."
        )
    unknown = [t for t in transports if t not in SOURCE_FACTORIES]
    if unknown:
        valid = ", ".join(SOURCE_FACTORIES)
        raise SystemExit(
            f"Error: unknown transport(s) {', '.join(unknown)} — valid choices: {valid}"
        )
    return transports or None


class SpyApp(App):  # type: ignore[type-arg]
    """A real-time dashboard for all-transport pubsub traffic using Textual."""

    CSS_PATH = "../dimos.tcss"

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    DataTable {{
        height: 2fr;
        width: 1fr;
        border: solid {theme.BORDER};
        background: {theme.BG};
        scrollbar-size: 0 0;
    }}
    DataTable > .datatable--header {{
        color: {theme.ACCENT};
        background: transparent;
    }}
    """

    refresh_interval: float = 0.5  # seconds

    BINDINGS = [
        ("q", "quit"),
        ("ctrl+c", "quit"),
    ]

    def __init__(self, transports: list[str] | None = None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        if transports is None:
            # Default: every available transport; unavailable backends are
            # skipped with a warning (see default_sources).
            sources = default_sources()
        else:
            unknown = [n for n in transports if n not in SOURCE_FACTORIES]
            if unknown:
                raise ValueError(
                    f"unknown transport(s) {', '.join(unknown)} — valid choices: "
                    f"{', '.join(SOURCE_FACTORIES)}"
                )
            # Construct only the requested sources: a filtered-out transport is
            # never imported or instantiated, and an unavailable one that was
            # explicitly requested stays a hard error.
            sources = [SOURCE_FACTORIES[name]() for name in transports]
        # Warn about missing system config before entering TUI raw mode (LCM only).
        if any(s.name == "lcm" for s in sources):
            from dimos.protocol.service.lcmservice import autoconf

            autoconf(check_only=True)

        self.spy = TransportSpy(sources=sources)
        self.spy.start()
        self.table: DataTable | None = None  # type: ignore[type-arg]

    def compose(self) -> ComposeResult:
        self.table = DataTable(zebra_stripes=False, cursor_type=None)  # type: ignore[arg-type]
        self.table.add_column("Transport")
        self.table.add_column("Topic")
        self.table.add_column("Type")
        self.table.add_column("Freq (Hz)")
        self.table.add_column("Bandwidth")
        self.table.add_column("Total")
        self.table.add_column("Age")
        yield self.table

    def on_mount(self) -> None:
        self.set_interval(self.refresh_interval, self.refresh_table)

    async def on_unmount(self) -> None:
        self.spy.stop()

    def refresh_table(self) -> None:
        now = time.time()
        snap = self.spy.snapshot()
        rows: list[tuple[SpyKey, TopicStats]] = sorted(
            snap.items(), key=lambda kv: kv[1].total_bytes, reverse=True
        )
        self.table.clear(columns=False)  # type: ignore[union-attr]

        for key, stats in rows:
            base, msg_type = split_type_suffix(key.topic)
            freq = stats.freq(STAT_WINDOW, now)
            bps = stats.bytes_per_sec(STAT_WINDOW, now)
            age = now - stats.last_seen if stats.last_seen is not None else None
            stale = age is not None and age > STALE_AGE

            if stale:
                # Liveness: dim the whole row for topics gone quiet.
                self.table.add_row(  # type: ignore[union-attr]
                    Text(key.transport, style=theme.DIM),
                    Text(base, style=theme.DIM),
                    Text(msg_type or "", style=theme.DIM),
                    Text(f"{freq:.1f}", style=theme.DIM),
                    Text(f"{human_bytes(bps)}/s", style=theme.DIM),
                    Text(human_bytes(stats.total_bytes), style=theme.DIM),
                    Text(f"{age:.0f}s", style=theme.DIM),
                )
                continue

            age_str = f"{age:.1f}s" if age is not None else "-"
            self.table.add_row(  # type: ignore[union-attr]
                Text(key.transport, style=theme.BLUE),
                topic_text(base),
                Text(msg_type or "", style=theme.BLUE),
                Text(f"{freq:.1f}", style=gradient(10, freq)),
                Text(f"{human_bytes(bps)}/s", style=gradient(1024 * 3, bps)),
                Text(human_bytes(stats.total_bytes)),
                Text(age_str, style=theme.ACCENT),
            )


def main() -> None:
    """Entry point for `dimos spy` (argv: --transport filters)."""
    import sys

    SpyApp(transports=_parse_transports(sys.argv[1:])).run()


def _lcm_only_argv(args: list[str]) -> list[str]:
    """Build the `spy` argv for the LCM-only entry points (`lcmspy` / `dimos lcmspy`).

    Rejects an explicit --transport override (these entry points can't choose
    transports); all other args pass through to `spy`.
    """
    if any(a == "--transport" or a.startswith("--transport=") for a in args):
        raise SystemExit(
            "Error: lcmspy is LCM-only; use `dimos spy --transport ...` to choose transports."
        )
    return ["spy", "--transport", "lcm", *args]


def lcm_main() -> None:
    """`lcmspy` console-script shim: the spy over the LCM source only."""
    import sys

    sys.argv = _lcm_only_argv(sys.argv[1:])
    main()


if __name__ == "__main__":
    main()
