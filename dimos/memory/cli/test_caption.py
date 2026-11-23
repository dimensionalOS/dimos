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

import json
from pathlib import Path

from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.store.sqlite import SqliteStore


def test_caption_ingests_jsonl_onto_the_store(tmp_path: Path) -> None:
    db = tmp_path / "rec.db"
    jsonl = tmp_path / "captions.jsonl"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", str).append("frame", ts=1000.0)
    jsonl.write_text(
        json.dumps({"t": 0.0, "parsed": {"setting": "garage", "objects_and_text": ["red car"]}})
        + "\n",
        encoding="utf-8",
    )

    result = CliRunner().invoke(mem_app, ["caption", str(db), "--jsonl", str(jsonl)])

    assert result.exit_code == 0, result.output
    assert "wrote 1 captions" in result.output
    with SqliteStore(path=str(db), must_exist=True) as store:
        texts = [obs.data for obs in store.streams.color_image_caption]
    assert "red car" in texts[0]


def test_dump_captions_writes_daily_jsonl(tmp_path: Path) -> None:
    db = tmp_path / "rec.db"
    out = tmp_path / "out"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", str).append("frame", ts=1000.0)
        store.stream("color_image_caption", str).append(
            json.dumps({"setting": "garage", "objects_and_text": ["red car"]}), ts=1000.0
        )

    result = CliRunner().invoke(mem_app, ["dump-captions", str(db), "--out", str(out)])

    assert result.exit_code == 0, result.output
    assert "wrote 1 captions in 1 day file(s)" in result.output
    line = json.loads((out / "captions.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert line["t"] == 0.0
    assert line["objects_and_text"] == ["red car"]
