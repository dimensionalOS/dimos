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

from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
import numpy as np
import pytest

from dimos.memory.store.mcap import McapStore
from dimos.memory.store.mcap_append import McapAppender
from dimos.models.embedding.base import Embedding


def _recording(path: Path, frames: int = 20) -> None:
    with open(path, "wb") as f:
        writer = Writer(f)
        writer.start()
        channel = writer.register_channel("/camera/raw", "raw", 0)
        for i in range(frames):
            writer.add_message(channel, log_time=i * 10**8, publish_time=i * 10**8, data=bytes([i]))
        writer.finish()


def _topics(path: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    with open(path, "rb") as f:
        for _, channel, _ in make_reader(f, validate_crcs=True).iter_messages():
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
    return counts


def _unit(i: int, width: int = 8) -> Embedding:
    vector = np.zeros(width, np.float32)
    vector[i % width] = 1.0
    return Embedding(vector)


def test_a_derived_stream_lands_in_the_mcap_and_is_searched_from_ram(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    _recording(path)
    store = McapStore(path=str(path))
    index = store.stream("index", dict)
    for i in range(12):
        index.append({"frame": i}, ts=i * 0.1, embedding=_unit(i))
    store.stop()

    assert _topics(path) == {"/camera/raw": 20, "/index": 12}
    assert sorted(p.name for p in tmp_path.iterdir()) == ["rec.mcap"], "no file beside it"

    again = McapStore(path=str(path))
    rows = again.stream("index", dict)
    assert rows.count() == 12
    assert [obs.data["frame"] for obs in rows][:3] == [0, 1, 2]
    best = rows.search(_unit(3), k=2).to_list()
    assert {obs.data["frame"] for obs in best} == {3, 11}  # the same unit vector twice
    assert best[0].similarity == pytest.approx(1.0)
    again.stop()


def test_a_second_session_appends_after_the_first(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    _recording(path)
    store = McapStore(path=str(path))
    store.stream("index", dict).append({"n": 0}, ts=0.0)
    store.stop()

    store = McapStore(path=str(path))
    store.stream("index", dict).append({"n": 1}, ts=1.0)
    store.stop()
    assert _topics(path) == {"/camera/raw": 20, "/index": 2}


def test_only_one_process_writes_and_readers_do_not_block_it(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    _recording(path)
    seed = McapStore(path=str(path))
    seed.stream("index", dict).append({"n": 0}, ts=0.0)
    seed.stop()

    reader = McapStore(path=str(path))
    assert reader.stream("index", dict).count() == 1  # reading takes no write lock

    writer = McapStore(path=str(path))
    writer.stream("index", dict).append({"n": 1}, ts=1.0)  # so this one can write
    with pytest.raises(RuntimeError, match="another process"):
        McapAppender(path)
    writer.stop()
    reader.stop()
    assert _topics(path)["/index"] == 2
