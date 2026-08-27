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

from __future__ import annotations

import os

import typer

from dimos.cli.commands.lifecycle import DEFAULT_CONFIG_PATH, run


def replay(
    ctx: typer.Context,
    path: str = typer.Argument(..., help="Recording: memory.db path or dataset name"),
    topics: str = typer.Option("*", "--topics", help="Comma-separated globs on stream names"),
    speed: float = typer.Option(1.0, "--speed", help="Playback rate multiplier"),
    loop: bool = typer.Option(False, "--loop", help="Restart from the beginning when done"),
) -> None:
    """Replay a recording on the bus: `dimos run replay` with one Out per recorded stream.

    Extra arguments go to `dimos run` (e.g. another blueprint to feed, `--disable`).
    """
    # The replay blueprint builds its ports from these at import, in this process
    # and in every worker.
    os.environ["REPLAY_DB"] = path
    os.environ["REPLAY_TOPICS"] = topics
    run(
        ctx,
        robot_types=["replay", *ctx.args, "--replay.speed", str(speed), "--replay.loop", str(loop)],
        daemon=False,
        disable=[],
        config_path=DEFAULT_CONFIG_PATH,
        local_relay=None,
        relay_url=None,
        show_help=False,
    )
