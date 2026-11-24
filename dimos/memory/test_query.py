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

"""The agent memory query ops report relative seconds and dump daily caption JSONL."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.memory.query import (
    MAX_INSPECT_FRAMES,
    _normalize_inspect_times,
    _save_frame,
    captions_from_jsonl,
    dump_caption_jsonl,
    embed_color_stream,
    handle_request,
    inspect,
    write_caption_stream,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.models.embedding.base import Embedding
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

ORIGIN = 1_000.0


def _emb(vec: list[float]) -> Embedding:
    v = np.array(vec, dtype=np.float32)
    v /= np.linalg.norm(v) + 1e-10
    return Embedding(vector=v)


def _person_embed(_text: str) -> Embedding:
    return _emb([1.0, 0.0, 0.0])


def _caption(setting: str, *objects: str) -> str:
    return json.dumps(
        {"setting": setting, "people_count": 0, "people": [], "objects_and_text": list(objects)},
        ensure_ascii=False,
    )


@pytest.fixture
def recording(tmp_path: Path) -> Path:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        color = store.stream("color_image", str)
        captions = store.stream("color_image_caption", str)
        samples = (
            (0.0, "lot", "person-a"),
            (5.0, "lot", "white sedan"),
            (10.0, "lot", "person-b"),
            (15.0, "mall", "REGAL sign"),
        )
        for offset, setting, obj in samples:
            ts = ORIGIN + offset
            color.append(obj, ts=ts)
            captions.append(_caption(setting, obj), ts=ts)
    return path


def test_inspect_opens_the_nearest_color_frame(recording: Path) -> None:
    with SqliteStore(path=str(recording), must_exist=True) as store:
        result = inspect(store, 5.0)

    assert result["hits"][0]["t"] == 5.0
    assert "caption" not in result["hits"][0]


def test_inspect_without_a_nearby_frame_says_so(recording: Path) -> None:
    with (
        SqliteStore(path=str(recording), must_exist=True) as store,
        pytest.raises(LookupError, match="no color_image near"),
    ):
        inspect(store, 80.0, tolerance=1.0)


def test_unknown_op_is_rejected(recording: Path) -> None:
    with pytest.raises(ValueError, match="unknown op"):
        handle_request({"op": "search", "path": str(recording), "query": "person"})


def test_dump_caption_jsonl_writes_one_file_for_a_single_utc_day(
    recording: Path, tmp_path: Path
) -> None:
    dest = tmp_path / "case"
    dest.mkdir()
    with SqliteStore(path=str(recording), must_exist=True) as store:
        files = dump_caption_jsonl(store, dest)

    day = "1970-01-01"
    assert files == [{"date": day, "t0": 0.0, "t1": 15.0, "n": 4, "path": f"captions/{day}.jsonl"}]
    root = dest / "captions.jsonl"
    shard = dest / "captions" / f"{day}.jsonl"
    assert root.read_bytes() == shard.read_bytes()
    rows = [json.loads(line) for line in root.read_text(encoding="utf-8").splitlines()]
    assert rows[1]["t"] == 5.0
    assert rows[1]["objects_and_text"] == ["white sedan"]
    manifest = [
        json.loads(line) for line in (dest / "captions" / "manifest.jsonl").read_text().splitlines()
    ]
    assert manifest == files


def test_dump_caption_jsonl_splits_at_utc_midnight(tmp_path: Path) -> None:
    origin = datetime(2026, 1, 3, 23, 59, 50, tzinfo=timezone.utc).timestamp()
    db = tmp_path / "rec.db"
    with SqliteStore(path=str(db)) as store:
        color = store.stream("color_image", str)
        captions = store.stream("color_image_caption", str)
        color.append("a", ts=origin)
        captions.append(_caption("lot", "before"), ts=origin)
        color.append("b", ts=origin + 20.0)
        captions.append(_caption("lot", "after"), ts=origin + 20.0)
        files = dump_caption_jsonl(store, tmp_path)

    assert [row["date"] for row in files] == ["2026-01-03", "2026-01-04"]
    assert files[0]["n"] == 1 and files[1]["n"] == 1
    assert not (tmp_path / "captions.jsonl").exists()
    first = json.loads((tmp_path / files[0]["path"]).read_text().splitlines()[0])
    second = json.loads((tmp_path / files[1]["path"]).read_text().splitlines()[0])
    assert first["t"] == 0.0
    assert second["t"] == 20.0
    assert first["objects_and_text"] == ["before"]
    assert second["objects_and_text"] == ["after"]


def test_dump_without_captions_says_to_caption(tmp_path: Path) -> None:
    path = tmp_path / "plain.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("color_image", str).append("frame", ts=ORIGIN)
        with pytest.raises(LookupError, match="dimos mem caption"):
            dump_caption_jsonl(store, tmp_path)


class _FixedEmbedder:
    device = "cpu"

    def embed(self, *images: object) -> Embedding | list[Embedding]:
        vecs = [_emb([1.0, 0.0, 0.0]) for _ in images]
        return vecs if len(vecs) > 1 else vecs[0]


def test_embed_color_stream_writes_vectors(tmp_path: Path) -> None:
    path = tmp_path / "plain.db"
    with SqliteStore(path=str(path)) as store:
        color = store.stream("color_image", str)
        color.append("a", ts=ORIGIN)
        color.append("b", ts=ORIGIN + 1.0)
        written = embed_color_stream(store, _FixedEmbedder(), hz=10.0)
        assert written == 2
        hits = store.streams.color_image_embedded.search(_person_embed("person"), k=2).to_list()
        assert {round(float(obs.ts) - ORIGIN, 3) for obs in hits} == {0.0, 1.0}


def test_embed_color_stream_refuses_a_populated_dest(recording: Path) -> None:
    path = recording.parent / "embedded.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("color_image", str).append("a", ts=ORIGIN)
        embed_color_stream(store, _FixedEmbedder())
        with pytest.raises(ValueError, match="already has observations"):
            embed_color_stream(store, _FixedEmbedder())


def test_write_caption_stream_and_jsonl(tmp_path: Path) -> None:
    db = tmp_path / "rec.db"
    jsonl = tmp_path / "captions.jsonl"
    jsonl.write_text(
        json.dumps({"t": 0.0, "parsed": {"setting": "garage", "objects_and_text": ["red car"]}})
        + "\n"
        + json.dumps({"t": 1.5, "caption": "white sedan at curb"})
        + "\n",
        encoding="utf-8",
    )
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", str).append("frame", ts=ORIGIN)
        n = write_caption_stream(store, captions_from_jsonl(jsonl))
        assert n == 2
        texts = [obs.data for obs in store.streams.color_image_caption]
    assert "red car" in texts[0]
    assert texts[1] == "white sedan at curb"


def test_write_caption_stream_refuses_a_populated_dest(recording: Path) -> None:
    with (
        SqliteStore(path=str(recording), must_exist=True) as store,
        pytest.raises(ValueError, match="already has observations"),
    ):
        write_caption_stream(store, [(0.0, "x")])


def test_save_frame_writes_jpeg_next_to_the_case(tmp_path: Path) -> None:
    obs = Observation(
        id=7,
        ts=ORIGIN + 2.5,
        _data=Image.from_numpy(np.full((4, 4, 3), 40, dtype=np.uint8), format=ImageFormat.RGB),
    )
    path = _save_frame(obs, tmp_path / "memory-frames", ORIGIN)
    assert path is not None
    saved = Path(path)
    assert saved.is_file()
    assert saved.parent.name == "memory-frames"
    assert "t2.500_id7" in saved.name


def test_inspect_attaches_the_color_frame(tmp_path: Path, mocker: MockerFixture) -> None:
    db = tmp_path / "rec.db"
    out = tmp_path / "memory-frames"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", str).append("empty", ts=ORIGIN)
        store.stream("color_image", str).append("car", ts=ORIGIN + 2.0)
        store.stream("color_image_caption", str).append(_caption("lot"), ts=ORIGIN)
        store.stream("color_image_caption", str).append(
            _caption("lot", "white car"), ts=ORIGIN + 2.0
        )
    frame = Observation(
        id=9,
        ts=ORIGIN + 2.0,
        _data=Image.from_numpy(np.full((4, 4, 3), 80, dtype=np.uint8), format=ImageFormat.RGB),
    )
    mocker.patch("dimos.memory.query._color_at", return_value=frame)
    result = handle_request({"op": "inspect", "path": str(db), "t": 2.0, "out_dir": str(out)})
    path = result["hits"][0]["path"]
    assert path is not None
    assert Path(path).is_file()
    assert result["hits"][0]["t"] == 2.0
    assert "caption" not in result["hits"][0]
    assert "montage" not in result


def test_normalize_inspect_times_rejects_too_many() -> None:
    assert _normalize_inspect_times(1.5) == [1.5]
    assert _normalize_inspect_times([1.0, 2.0]) == [1.0, 2.0]
    with pytest.raises(ValueError, match=str(MAX_INSPECT_FRAMES)):
        _normalize_inspect_times([0.0, 1.0, 2.0, 3.0, 4.0])


def test_inspect_multi_t_writes_a_labeled_montage(tmp_path: Path, mocker: MockerFixture) -> None:
    db = tmp_path / "rec.db"
    out = tmp_path / "memory-frames"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", str).append("a", ts=ORIGIN)
        store.stream("color_image", str).append("b", ts=ORIGIN + 2.0)

    frames = {
        0.0: Observation(
            id=1,
            ts=ORIGIN,
            _data=Image.from_numpy(np.full((8, 12, 3), 30, dtype=np.uint8), format=ImageFormat.RGB),
        ),
        2.0: Observation(
            id=2,
            ts=ORIGIN + 2.0,
            _data=Image.from_numpy(
                np.full((8, 12, 3), 200, dtype=np.uint8), format=ImageFormat.RGB
            ),
        ),
    }

    def _at(_store: object, t: float, *, tolerance: float = 1.0) -> Observation:
        _ = tolerance
        return frames[float(t)]

    mocker.patch("dimos.memory.query._color_at", side_effect=_at)
    result = handle_request(
        {"op": "inspect", "path": str(db), "t": [0.0, 2.0], "out_dir": str(out)}
    )
    assert len(result["hits"]) == 2
    montage = result["montage"]
    assert montage is not None
    assert Path(montage).is_file()
    assert "montage_" in Path(montage).name
