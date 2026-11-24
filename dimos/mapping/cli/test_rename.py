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

from pathlib import Path

import pytest
import typer
from typer.testing import CliRunner

from dimos.mapping.cli.rename import main
from dimos.memory.store.sqlite import SqliteStore


@pytest.mark.parametrize(
    ("seek", "duration", "expected"),
    [(0.5, 0.1, []), (1.0, 0.5, [101.0, 101.25])],
)
def test_rename_uses_recording_window_before_dropping_streams(
    tmp_path: Path, seek: float, duration: float, expected: list[float]
) -> None:
    source = tmp_path / "source.db"
    destination = tmp_path / "destination.db"
    with SqliteStore(path=str(source)) as store:
        store.stream("reference", str).append("start", ts=100.0)
        for ts in (100.25, 101.0, 101.25, 101.5, 102.0):
            store.stream("lidar", str).append(f"scan-{ts}", ts=ts)
        for ts in (100.75, 101.0, 101.25, 101.5, 102.0):
            store.stream("aux", str).append(f"aux-{ts}", ts=ts)
    app = typer.Typer()
    app.command()(main)

    result = CliRunner().invoke(
        app,
        [
            str(source),
            "--out",
            str(destination),
            "--rename",
            "lidar=renamed",
            "--drop",
            "reference",
            "--seek",
            str(seek),
            "--duration",
            str(duration),
        ],
    )

    assert result.exit_code == 0, result.output
    with SqliteStore(path=str(destination), must_exist=True) as store:
        assert store.list_streams() == ["aux", "renamed"]
        assert [obs.ts for obs in store.stream("renamed").order_by("ts")] == expected
        assert [obs.ts for obs in store.stream("aux").order_by("ts")] == expected
