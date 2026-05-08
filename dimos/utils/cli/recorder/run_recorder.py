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

"""recorder — Terminal VLC for dimos recordings.

Record from live LCM traffic, play back recordings, trim, seek.
Run ``rerun-bridge`` in another terminal to visualize playback.

Usage::

    recorder                        # interactive — record from LCM
    recorder play my_recording.db   # play an existing recording
    recorder --help
"""

from __future__ import annotations

from collections import deque
import sys
import time
from typing import Any

from rich.text import Text
from textual.app import App, ComposeResult
from textual.color import Color
from textual.containers import Horizontal
from textual.widgets import DataTable, Footer, Header, Static

from dimos.record.record_replay import RecordReplay
from dimos.utils.cli import theme

# Braille sparkline constants (same as dtop)
_BRAILLE_BASE = 0x2800
_LDOTS = (0x00, 0x40, 0x44, 0x46, 0x47)
_RDOTS = (0x00, 0x80, 0xA0, 0xB0, 0xB8)
_SPARK_WIDTH = 16

from dimos.record.record_replay import topic_to_stream_name


def _heat(ratio: float) -> str:
    """Map 0..1 to cyan -> yellow -> red."""
    cyan = Color.parse(theme.CYAN)
    yellow = Color.parse(theme.YELLOW)
    red = Color.parse(theme.RED)
    if ratio <= 0.5:
        return cyan.blend(yellow, ratio * 2).hex
    return yellow.blend(red, (ratio - 0.5) * 2).hex


def _spark(history: deque[float], max_val: float, width: int = _SPARK_WIDTH) -> Text:
    """Braille sparkline from a deque of values."""
    n = width * 2
    vals = list(history)
    if len(vals) < n:
        vals = [0.0] * (n - len(vals)) + vals
    else:
        vals = vals[-n:]
    result = Text()
    if max_val <= 0:
        max_val = 1.0
    for i in range(0, n, 2):
        lv = min(vals[i] / max_val, 1.0)
        rv = min(vals[i + 1] / max_val, 1.0)
        li = min(int(lv * 4 + 0.5), 4)
        ri = min(int(rv * 4 + 0.5), 4)
        ch = chr(_BRAILLE_BASE | _LDOTS[li] | _RDOTS[ri])
        result.append(ch, style=_heat(max(lv, rv)))
    return result


def _fmt_time(seconds: float) -> str:
    """Format seconds as MM:SS.s"""
    if seconds < 0:
        seconds = 0
    m = int(seconds) // 60
    s = seconds - m * 60
    return f"{m:02d}:{s:05.2f}"


def _progress_bar(position: float, duration: float, width: int = 40) -> Text:
    """Render a progress bar with position indicator."""
    if duration <= 0:
        return Text("░" * width, style=theme.DIM)
    ratio = min(position / duration, 1.0)
    filled = int(ratio * width)
    result = Text()
    result.append("█" * filled, style=theme.CYAN)
    if filled < width:
        result.append("▓", style=theme.BRIGHT_CYAN)
        result.append("░" * (width - filled - 1), style=theme.DIM)
    return result


def _short_type(channel: str) -> str:
    """Extract the short type name from a channel string."""
    if "#" not in channel:
        return ""
    return channel.rsplit("#", 1)[-1].rsplit(".", 1)[-1]


class RecorderApp(App[None]):
    """Terminal VLC for dimos recordings.

    Shows all live LCM topics (like lcmspy).  Select topics then press
    ``r`` to record, ``space`` to play back, arrow keys to seek, etc.
    """

    CSS_PATH = "../dimos.tcss"

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    #streams {{
        height: 1fr;
        border: solid {theme.BORDER};
        background: {theme.BG};
        scrollbar-size: 0 0;
    }}
    #streams > .datatable--header {{
        color: {theme.ACCENT};
        background: transparent;
    }}
    #streams > .datatable--cursor {{
        background: {theme.BRIGHT_BLACK};
    }}
    #timeline {{
        height: 5;
        padding: 1 2;
        background: {theme.BG};
        border-top: solid {theme.DIM};
    }}
    #controls {{
        height: 3;
        padding: 0 2;
        background: {theme.BG};
        border-top: solid {theme.DIM};
    }}
    #status-left {{
        width: 1fr;
    }}
    #status-right {{
        width: auto;
    }}
    """

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("space", "toggle_select", "Toggle"),
        ("a", "select_all", "All"),
        ("n", "select_none", "None"),
        ("r", "toggle_record", "Rec"),
        ("p", "toggle_play", "Play"),
        ("s", "stop_all", "Stop"),
        ("left", "seek_back", "-5s"),
        ("right", "seek_fwd", "+5s"),
        ("shift+left", "seek_back_big", "-30s"),
        ("shift+right", "seek_fwd_big", "+30s"),
        ("[", "mark_trim_start", "In"),
        ("]", "mark_trim_end", "Out"),
        ("t", "do_trim", "Trim"),
        ("d", "do_delete", "Del"),
    ]

    def __init__(
        self,
        db_path: str | None = None,
        autoplay: bool = False,
    ) -> None:
        super().__init__()
        from dimos.protocol.service.lcmservice import autoconf

        autoconf(check_only=True)

        if db_path is None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._db_path = f"recording_{ts}.db"
        else:
            self._db_path = db_path
        self._autoplay = autoplay
        self._recorder: RecordReplay | None = None
        self._lcm: Any = None
        self._spy: Any = None

        # Per-stream sparkline history keyed by stream_name
        self._freq_history: dict[str, deque[float]] = {}
        # Set of selected stream names (for recording)
        self._selected: set[str] = set()

        self._trim_in: float | None = None
        self._trim_out: float | None = None

        self._table: DataTable[Any] | None = None

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        self._table = DataTable(zebra_stripes=False, cursor_type="row")
        self._table.id = "streams"
        self._table.add_column("REC", key="sel", width=5)
        self._table.add_column("Topic", key="topic")
        self._table.add_column("Type", key="type")
        self._table.add_column("Freq", key="freq")
        self._table.add_column("Bandwidth", key="bw")
        self._table.add_column("Recorded", key="rec")
        self._table.add_column("Activity", key="activity")
        yield self._table
        yield Static(id="timeline")
        with Horizontal(id="controls"):
            yield Static(id="status-left")
            yield Static(id="status-right")
        yield Footer()

    def on_mount(self) -> None:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM
        from dimos.utils.cli.lcmspy.lcmspy import GraphLCMSpy

        self._lcm = LCM()

        # Live topic discovery via LCM spy (same as lcmspy tool)
        self._spy = GraphLCMSpy(graph_log_window=0.5)
        self._spy.start()

        self._recorder = RecordReplay(self._db_path)
        self.title = f"recorder — {self._recorder.path}"
        self.set_interval(0.5, self._refresh)

        if self._autoplay and self._db_path:
            self._start_playback()

    async def on_unmount(self) -> None:
        if self._recorder:
            await self._recorder.close()
        if self._spy:
            self._spy.stop()
        if self._lcm and hasattr(self._lcm, "stop"):
            self._lcm.stop()

    def action_toggle_select(self) -> None:
        """Toggle selection on the row under the cursor."""
        if self._table is None or self._table.row_count == 0:
            return
        row_key, _ = self._table.coordinate_to_cell_key(self._table.cursor_coordinate)
        name = str(row_key.value)
        if name in self._selected:
            self._selected.discard(name)
        else:
            self._selected.add(name)

    def action_select_all(self) -> None:
        """Select all visible topics."""
        if self._spy is None:
            return
        with self._spy._topic_lock:
            channels = list(self._spy.topic.keys())
        for ch in channels:
            self._selected.add(topic_to_stream_name(ch))
        # Also include any already-recorded streams
        if self._recorder:
            self._selected.update(self._recorder.store.list_streams())

    def action_select_none(self) -> None:
        self._selected.clear()

    def action_toggle_play(self) -> None:
        if self._recorder is None:
            return
        if self._recorder.is_recording:
            return
        if self._recorder.is_playing:
            if self._recorder.is_paused:
                self._recorder.resume()
            else:
                self._recorder.pause()
        else:
            self._start_playback()

    def action_toggle_record(self) -> None:
        if self._recorder is None:
            return
        if self._recorder.is_playing:
            return
        if self._recorder.is_recording:
            self._recorder.stop_recording()
        else:
            topics = self._selected_topics() if self._selected else self._all_topics()
            self._recorder.start_recording([self._lcm], topics=topics)

    async def action_stop_all(self) -> None:
        if self._recorder is None:
            return
        self._recorder.stop_recording()
        await self._recorder.stop_playback()

    async def action_seek_back(self) -> None:
        await self._seek_relative(-5.0)

    async def action_seek_fwd(self) -> None:
        await self._seek_relative(5.0)

    async def action_seek_back_big(self) -> None:
        await self._seek_relative(-30.0)

    async def action_seek_fwd_big(self) -> None:
        await self._seek_relative(30.0)

    def action_mark_trim_start(self) -> None:
        if self._recorder:
            self._trim_in = self._recorder.position

    def action_mark_trim_end(self) -> None:
        if self._recorder:
            self._trim_out = self._recorder.position

    async def action_do_trim(self) -> None:
        if self._recorder and self._trim_in is not None and self._trim_out is not None:
            await self._recorder.stop_playback()
            lo, hi = sorted((self._trim_in, self._trim_out))
            self._recorder.trim(lo, hi)
            self._trim_in = self._trim_out = None

    async def action_do_delete(self) -> None:
        if self._recorder and self._trim_in is not None and self._trim_out is not None:
            await self._recorder.stop_playback()
            lo, hi = sorted((self._trim_in, self._trim_out))
            self._recorder.delete_range(lo, hi)
            self._trim_in = self._trim_out = None

    def _all_topics(self) -> list[Any]:
        """All discovered topics from the spy."""
        from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic

        if not self._spy:
            return []
        return [LCMTopic.from_channel_str(ch) for ch in list(self._spy.topic)]

    def _selected_topics(self) -> list[Any]:
        """Map selected stream names back to LCM Topics via the spy."""
        from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic
        from dimos.record.record_replay import topic_to_stream_name

        if not self._spy:
            return []
        return [
            LCMTopic.from_channel_str(ch)
            for ch in list(self._spy.topic)
            if topic_to_stream_name(ch) in self._selected
        ]

    def _start_playback(self) -> None:
        if self._recorder is None or self._lcm is None:
            return
        if hasattr(self._lcm, "start"):
            self._lcm.start()
        self._recorder.play(speed=1.0)

    async def _seek_relative(self, delta: float) -> None:
        if self._recorder:
            await self._recorder.seek(self._recorder.position + delta)

    def _refresh(self) -> None:
        if self._table is None:
            return
        assert self._recorder is not None
        spy = self._spy

        # Build unified row list: live topics + recorded-only streams
        # Each row: (stream_name, channel, spy_topic_or_None)
        rows: dict[str, tuple[str, Any]] = {}  # stream_name -> (channel, spy_topic)

        if spy is not None:
            with spy._topic_lock:
                live_topics: dict[str, Any] = dict(spy.topic)  # channel -> GraphTopic
            for channel, spy_topic in live_topics.items():
                sname = topic_to_stream_name(channel)
                rows[sname] = (channel, spy_topic)

        # Add streams that exist in the recording but are not live
        recorded_streams = set(self._recorder.store.list_streams())
        for sname in recorded_streams:
            if sname not in rows:
                rows[sname] = (sname, None)

        # Render table
        # Remember cursor position so we can restore it
        cursor_row = self._table.cursor_coordinate.row if self._table.row_count > 0 else 0
        self._table.clear(columns=False)

        sorted_names = sorted(rows.keys())
        for sname in sorted_names:
            channel, spy_topic = rows[sname]

            # Selection marker
            is_sel = sname in self._selected
            if is_sel:
                sel = Text(" [●] ", style=f"bold {theme.BRIGHT_GREEN}")
            else:
                sel = Text(" [ ] ", style=theme.DIM)

            # Topic name — green when actively recording, bright when selected
            if self._recorder.is_recording and sname in (self._recorder.store.list_streams()):
                topic_style = f"bold {theme.BRIGHT_GREEN}"
            elif is_sel:
                topic_style = theme.BRIGHT_WHITE
            else:
                topic_style = theme.FOREGROUND

            # Type
            type_str = _short_type(channel) if "#" in channel else ""

            # Live freq / bandwidth from spy
            if spy_topic is not None:
                freq = spy_topic.freq(5.0)
                freq_text = Text(f"{freq:.1f} Hz", style=_heat(min(freq / 30.0, 1.0)))
                kbps = spy_topic.kbps(5.0)
                bw_text = Text(spy_topic.kbps_hr(5.0), style=_heat(min(kbps / 3072, 1.0)))
            else:
                freq_text = Text("—", style=theme.DIM)
                bw_text = Text("—", style=theme.DIM)

            # Recorded count
            if sname in recorded_streams:
                count = self._recorder.store.stream(sname).count()
                rec_text = Text(str(count), style=theme.YELLOW)
            else:
                rec_text = Text("", style=theme.DIM)

            # Sparkline from spy freq history
            if sname not in self._freq_history:
                self._freq_history[sname] = deque(maxlen=_SPARK_WIDTH * 2)
            if spy_topic is not None:
                self._freq_history[sname].append(spy_topic.freq(0.5))
            else:
                self._freq_history[sname].append(0.0)
            max_f = max(self._freq_history[sname]) if self._freq_history[sname] else 1.0
            activity = _spark(self._freq_history[sname], max_f)

            self._table.add_row(
                sel,
                Text(sname, style=topic_style),
                Text(type_str, style=theme.BLUE),
                freq_text,
                bw_text,
                rec_text,
                activity,
                key=sname,
            )

        # Restore cursor
        if self._table.row_count > 0:
            row = min(cursor_row, self._table.row_count - 1)
            self._table.move_cursor(row=row)

        # Timeline
        duration = self._recorder.duration
        position = self._recorder.position

        timeline = Text()
        timeline.append("  ")
        timeline.append(_fmt_time(position), style=theme.BRIGHT_WHITE)
        timeline.append(" ", style=theme.DIM)
        timeline.append_text(_progress_bar(position, duration, width=50))
        timeline.append(" ", style=theme.DIM)
        timeline.append(_fmt_time(duration), style=theme.FOREGROUND)

        if self._trim_in is not None or self._trim_out is not None:
            timeline.append("\n  ")
            timeline.append("[", style=theme.YELLOW)
            timeline.append(
                _fmt_time(self._trim_in) if self._trim_in is not None else "--:--",
                style=theme.YELLOW if self._trim_in is not None else theme.DIM,
            )
            timeline.append(" .. ", style=theme.DIM)
            timeline.append(
                _fmt_time(self._trim_out) if self._trim_out is not None else "--:--",
                style=theme.YELLOW if self._trim_out is not None else theme.DIM,
            )
            timeline.append("]", style=theme.YELLOW)

        self.query_one("#timeline", Static).update(timeline)

        # Status bar
        status = Text()
        if self._recorder.is_recording:
            status.append(" ● REC ", style=f"bold on {theme.RED}")
        elif self._recorder.is_paused:
            status.append(" ❚❚ PAUSED ", style=theme.YELLOW)
        elif self._recorder.is_playing:
            status.append(" ▶ PLAYING ", style=theme.BRIGHT_GREEN)
        else:
            status.append(" ■ STOPPED ", style=theme.DIM)

        n_live = len([r for r in rows.values() if r[1] is not None])
        n_sel = len(self._selected)
        status.append(f"  {n_live} live", style=theme.FOREGROUND)
        if n_sel:
            status.append(f"  {n_sel} selected", style=theme.BRIGHT_GREEN)
        if recorded_streams:
            status.append(f"  {len(recorded_streams)} recorded", style=theme.YELLOW)

        # Contextual hint
        if not (self._recorder.is_recording or self._recorder.is_playing):
            if n_live > 0 and n_sel == 0:
                status.append("  SPACE select, A all, R rec", style=theme.DIM)
            elif n_sel > 0:
                status.append("  R to record selected", style=theme.DIM)

        self.query_one("#status-left", Static).update(status)

        rhs = Text()
        rhs.append(f"{self._recorder.path} ", style=theme.DIM)
        self.query_one("#status-right", Static).update(rhs)


def main() -> None:
    db_path: str | None = None
    autoplay = False

    args = sys.argv[1:]
    if args and args[0] == "play" and len(args) > 1:
        db_path = args[1]
        autoplay = True
    elif args and not args[0].startswith("-"):
        db_path = args[0]

    RecorderApp(db_path=db_path, autoplay=autoplay).run()


if __name__ == "__main__":
    main()
