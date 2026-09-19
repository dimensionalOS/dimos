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

"""Did the recording actually capture the rates it was supposed to?

A stereo calibration is fitted from pairs of frames, and a recorder that fell
behind -- the head cloud lost a tenth of its frames the last time the full nav
stack shared the Orin with it -- leaves gaps that look, in the fit, like a
scene that moved. The recording does not say it dropped anything; the only
evidence is the stamps. This reads them and reports, per stream, how many
messages there are, when they start and stop, the mean rate over the whole
file, and the min/median/max rate over sliding windows -- the windows being
what exposes a ten-second stall that a whole-file mean smooths over.

It reads through :mod:`dimos.memory.raw_replay` rather than the observation
store, so it works on the robot's SQLite 3.37 and on a recording still being
written.

Usage::

    python -m dimos.robot.galaxea.r1pro.recording_rates <db> \\
        [--streams a,b] [--window-s 10] [--require head_left_color=28]

``--require`` compares the whole-recording mean against the number given and
exits 1 when any is not met, so a capture script can refuse a bad recording
before anyone fits against it.
"""

from __future__ import annotations

import argparse
import bisect
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
import statistics
import sys

from dimos.memory.raw_replay import RawRecording

DEFAULT_WINDOW_S = 10.0


@dataclass(frozen=True)
class StreamRate:
    """What one stream's stamps say about its rate."""

    name: str
    count: int
    first_ts: float | None
    last_ts: float | None
    # (count - 1) / (last - first): the mean inter-arrival rate, which is the
    # honest "Hz" for a list of stamps. None when there is nothing to divide.
    mean_hz: float | None
    window_s: float
    window_min_hz: float | None
    window_median_hz: float | None
    window_max_hz: float | None

    @property
    def duration_s(self) -> float | None:
        if self.first_ts is None or self.last_ts is None:
            return None
        return self.last_ts - self.first_ts


def window_hz(stamps: Sequence[float], window_s: float) -> list[float]:
    """Message rate inside each window of *window_s*, slid by half a window.

    Windows are only counted where a whole one fits between the first and last
    stamp, so the ends of a recording do not read as a stall. A recording
    shorter than one window gets a single figure: its overall rate. Stamps
    must be sorted, which :class:`RawStream` guarantees.
    """
    if len(stamps) < 2 or window_s <= 0:
        return []
    first, last = stamps[0], stamps[-1]
    span = last - first
    if span <= 0:
        return []
    if span <= window_s:
        return [(len(stamps) - 1) / span]
    rates: list[float] = []
    step = window_s / 2
    start = first
    # A hair of slack so the last window is not lost to float rounding.
    while start + window_s <= last + 1e-9:
        lo = bisect.bisect_left(stamps, start)
        hi = bisect.bisect_left(stamps, start + window_s)
        rates.append((hi - lo) / window_s)
        start += step
    return rates


def rate_of(name: str, stamps: Sequence[float], window_s: float) -> StreamRate:
    count = len(stamps)
    first = stamps[0] if stamps else None
    last = stamps[-1] if stamps else None
    mean = None
    if first is not None and last is not None and last > first:
        mean = (count - 1) / (last - first)
    windows = window_hz(stamps, window_s)
    return StreamRate(
        name=name,
        count=count,
        first_ts=first,
        last_ts=last,
        mean_hz=mean,
        window_s=window_s,
        window_min_hz=min(windows) if windows else None,
        window_median_hz=statistics.median(windows) if windows else None,
        window_max_hz=max(windows) if windows else None,
    )


def stream_rates(
    db_path: str | Path,
    streams: Iterable[str] | None = None,
    window_s: float = DEFAULT_WINDOW_S,
) -> dict[str, StreamRate]:
    """Rates for the named streams, or every stream when none are named.

    Raises ``KeyError`` (from the reader) for a stream the recording lacks,
    naming both.
    """
    with RawRecording(db_path) as recording:
        names = list(streams) if streams is not None else recording.list_streams()
        return {name: rate_of(name, recording.stream(name).stamps, window_s) for name in names}


def check_requirements(rates: dict[str, StreamRate], requirements: dict[str, float]) -> list[str]:
    """One line per unmet requirement; empty when all are met."""
    failures = []
    for name, required_hz in requirements.items():
        rate = rates.get(name)
        if rate is None:
            failures.append(f"{name}: not in the recording (required {required_hz:g} Hz)")
        elif rate.mean_hz is None:
            failures.append(
                f"{name}: {rate.count} message(s), no rate to measure (required {required_hz:g} Hz)"
            )
        elif rate.mean_hz < required_hz:
            failures.append(
                f"{name}: {rate.mean_hz:.2f} Hz mean over the recording, "
                f"{required_hz:g} required (window min {_hz(rate.window_min_hz)})"
            )
    return failures


def _hz(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f}"


def _ts(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def format_table(rates: dict[str, StreamRate], window_s: float) -> str:
    header = (
        f"{'stream':<24} {'count':>8} {'first_ts':>16} {'last_ts':>16} {'dur_s':>9} "
        f"{'mean_hz':>8} {f'min{window_s:g}s':>8} {f'med{window_s:g}s':>8} "
        f"{f'max{window_s:g}s':>8}"
    )
    lines = [header]
    for rate in rates.values():
        duration = rate.duration_s
        lines.append(
            f"{rate.name:<24} {rate.count:>8} {_ts(rate.first_ts):>16} {_ts(rate.last_ts):>16} "
            f"{'n/a' if duration is None else f'{duration:.1f}':>9} "
            f"{_hz(rate.mean_hz):>8} {_hz(rate.window_min_hz):>8} "
            f"{_hz(rate.window_median_hz):>8} {_hz(rate.window_max_hz):>8}"
        )
    return "\n".join(lines)


def parse_requirement(text: str) -> tuple[str, float]:
    name, sep, hz = text.partition("=")
    if not sep or not name:
        raise argparse.ArgumentTypeError(f"expected <stream>=<hz>, got {text!r}")
    try:
        return name, float(hz)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r}: {hz!r} is not a number of Hz") from None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m dimos.robot.galaxea.r1pro.recording_rates",
        description="Per-stream message rates of a dimos sqlite recording.",
    )
    parser.add_argument("db", type=Path, help="the recording (.db)")
    parser.add_argument(
        "--streams",
        type=lambda s: [name for name in s.split(",") if name],
        default=None,
        help="comma-separated stream names; default: every stream",
    )
    parser.add_argument(
        "--window-s",
        type=float,
        default=DEFAULT_WINDOW_S,
        help=f"length of the sliding windows in seconds (default {DEFAULT_WINDOW_S:g})",
    )
    parser.add_argument(
        "--require",
        type=parse_requirement,
        action="append",
        default=[],
        metavar="STREAM=HZ",
        help="fail unless the stream's mean rate is at least HZ; repeatable",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        rates = stream_rates(args.db, args.streams, args.window_s)
    except (FileNotFoundError, KeyError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(format_table(rates, args.window_s))
    failures = check_requirements(rates, dict(args.require))
    if failures:
        print("rate requirements not met:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
