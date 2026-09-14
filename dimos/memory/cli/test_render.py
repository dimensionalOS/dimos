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

import json
from pathlib import Path
import sqlite3

import pytest

from dimos.memory.cli.render import render_store
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def test_render_skips_stream_with_unresolvable_payload_type(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    store.stream("odom", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stop()

    # A stream recorded with a type whose module no longer exists. Sorts before "odom".
    conn = sqlite3.connect(path)
    cfg = json.loads(conn.execute("SELECT config FROM _streams WHERE name = 'odom'").fetchone()[0])
    cfg["payload_module"] = "dimos.gone.Missing.Missing"
    conn.execute("INSERT INTO _streams (name, config) VALUES ('april_tag', ?)", (json.dumps(cfg),))
    conn.commit()
    conn.close()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    out = render_store(store, out=str(tmp_path / "out.rrd"), no_gui=True)
    store.stop()

    assert "skip april_tag: payload type unavailable" in capsys.readouterr().out
    assert Path(out).stat().st_size > 0
