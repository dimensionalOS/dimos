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

from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.store.sqlite import SqliteStore


def test_track_cli_dumps_jsonl(tmp_path: Path, mocker) -> None:  # type: ignore[no-untyped-def]
    db = tmp_path / "rec.db"
    out = tmp_path / "out"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", str).append("frame", ts=50.0)

    class Tracker:
        def update(self, image: object) -> list[object]:
            return []

        def stop(self) -> None:
            return None

    built = mocker.patch(
        "dimos.perception.identity.build_identity_tracker",
        return_value=Tracker(),
    )
    result = CliRunner().invoke(mem_app, ["track", str(db), "--out", str(out), "--no-reid"])
    assert result.exit_code == 0, result.output
    assert "wrote 1 track seconds" in result.output
    built.assert_called_once()
    assert built.call_args.kwargs["reid"] is False
    assert built.call_args.kwargs["embedder"] == "siglip"
    row = (out / "tracks.jsonl").read_text(encoding="utf-8").splitlines()[0]
    assert '"tracks":[]' in row
