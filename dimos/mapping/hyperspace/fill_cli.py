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

"""``dimos map fill-depth`` -- fill a recording's depth holes once, in place.

Stereo returns nothing off glass, a dark shelf or a shiny floor, and a box placed off
that depth lands metres past the thing. The `depth2depth` module does this live; this
is the same pass after the fact, for the recordings that were made without it.

Only the frames an index embedded, because those are the only frames a box is ever
placed off -- three minutes on grocery rather than twenty.

    dimos map fill-depth ~/datasets/lite_recorder/grocery.db
"""

from __future__ import annotations

from pathlib import Path
import time

import typer

from dimos.mapping.hyperspace.cli import (
    WAL_CAP_BYTES,
    fold_the_wal,
    hold_the_wal,
    open_store,
    pick_device,
    pick_stream,
    wal_bytes,
)
from dimos.mapping.hyperspace.ingest import fill_depth, filled_stream_for
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def main(
    recording_path: Path = typer.Argument(..., help="the .db recording to fill, in place"),
    model: str = typer.Option(
        "default", "--model", help="the metric depth checkpoint ('default' is the packaged one)"
    ),
    device: str = typer.Option("auto", "--device"),
    everywhere: bool = typer.Option(
        False,
        "--everywhere",
        help="fill every colour frame, not only the ones an index embedded",
    ),
    every: int = typer.Option(1, "--every", help="with --everywhere, keep one colour frame in N"),
) -> None:
    recording_path = recording_path.expanduser()
    recording = open_store(recording_path)
    if recording_path.suffix == ".db":
        hold_the_wal(recording)

    name = filled_stream_for("")
    if name in recording.list_streams():
        typer.echo(f"{name} is already there; rewriting it")

    started = time.monotonic()
    last = [started]

    def progress(written: int, ts: float) -> None:
        now = time.monotonic()
        if now - last[0] < 5.0:
            return
        last[0] = now
        rate = written / (now - started)
        typer.echo(f"  {written} frames, {rate:.1f}/s, at t={ts:.1f}")
        if recording_path.suffix == ".db" and wal_bytes(recording) > WAL_CAP_BYTES:
            fold_the_wal(recording)

    written = fill_depth(
        recording,
        model=model,
        device=pick_device(device),
        color_stream=pick_stream(recording, None, "color", "image"),
        depth_stream=pick_stream(recording, None, "depth", "image"),
        every=every,
        everywhere=everywhere,
        on_frame=progress,
    )
    took = time.monotonic() - started
    typer.echo(
        f"{written} frames filled into {name} in {took:.0f}s ({written / max(took, 1e-9):.1f}/s)"
    )
    if recording_path.suffix == ".db":
        fold_the_wal(recording)
    recording.stop()


if __name__ == "__main__":
    typer.run(main)
