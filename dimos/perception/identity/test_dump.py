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

import numpy as np

from dimos.memory.store.memory import MemoryStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.identity.dump import dump_track_jsonl, sidecar_tracks_jsonl, track_row


def _image(ts: float) -> Image:
    return Image.from_numpy(np.zeros((16, 16, 3), dtype=np.uint8), ts=ts)


def _box(image: Image, track_id: int, name: str = "person") -> Detection2DBBox:
    return Detection2DBBox(
        bbox=(1.0, 2.0, 3.4, 8.88),
        track_id=track_id,
        class_id=0,
        confidence=0.9,
        name=name,
        ts=image.ts,
        image=image,
    )


def test_sidecar_tracks_jsonl_sits_next_to_the_recording(tmp_path: Path) -> None:
    rec = tmp_path / "handpick" / "foo.db"
    assert sidecar_tracks_jsonl(rec) == tmp_path / "handpick" / "tracks" / "foo" / "tracks.jsonl"


def test_track_row_is_compact() -> None:
    image = _image(1.0)
    row = track_row(12.04, [_box(image, 7), _box(image, -1, "car")])
    assert row == {
        "t": 12.04,
        "tracks": [{"id": 7, "name": "person", "xyxy": [1.0, 2.0, 3.4, 8.9]}],
    }


def test_dump_follows_caption_timestamps(tmp_path: Path) -> None:
    origin = 1000.0
    calls: list[float] = []

    def update(image: Image) -> list[Detection2DBBox]:
        calls.append(image.ts)
        return [_box(image, 7)]

    with MemoryStore() as store:
        color = store.stream("color_image", Image)
        color.append(_image(origin), ts=origin)
        color.append(_image(origin + 0.4), ts=origin + 0.4)
        color.append(_image(origin + 1.1), ts=origin + 1.1)
        captions = store.stream("color_image_caption", str)
        captions.append("{}", ts=origin)
        captions.append("{}", ts=origin + 1.0)
        files = dump_track_jsonl(store, tmp_path, update)

    assert calls == [origin, origin + 1.1]
    assert files[0]["n"] == 2
    lines = (tmp_path / "tracks.jsonl").read_text(encoding="utf-8").splitlines()
    rows = [json.loads(line) for line in lines]
    assert [row["t"] for row in rows] == [0.0, 1.0]
    assert rows[1]["tracks"][0]["id"] == 7


def test_dump_without_captions_updates_once_per_hz(tmp_path: Path) -> None:
    origin = 1000.0
    calls: list[float] = []

    def update(image: Image) -> list[Detection2DBBox]:
        calls.append(image.ts)
        return [_box(image, 1)]

    with MemoryStore() as store:
        color = store.stream("color_image", Image)
        color.append(_image(origin), ts=origin)
        color.append(_image(origin + 0.4), ts=origin + 0.4)
        color.append(_image(origin + 1.0), ts=origin + 1.0)
        files = dump_track_jsonl(store, tmp_path, update, hz=1.0)

    assert calls == [origin, origin + 1.0]
    assert files[0]["n"] == 2
    rows = [
        json.loads(line)
        for line in (tmp_path / "tracks.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [row["t"] for row in rows] == [0.0, 1.0]


def test_dump_until_stops_before_later_captions(tmp_path: Path) -> None:
    origin = 1000.0
    calls: list[float] = []

    def update(image: Image) -> list[Detection2DBBox]:
        calls.append(image.ts)
        return [_box(image, 1)]

    with MemoryStore() as store:
        color = store.stream("color_image", Image)
        color.append(_image(origin), ts=origin)
        color.append(_image(origin + 1.5), ts=origin + 1.5)
        captions = store.stream("color_image_caption", str)
        captions.append("{}", ts=origin)
        captions.append("{}", ts=origin + 1.0)
        captions.append("{}", ts=origin + 2.0)
        files = dump_track_jsonl(store, tmp_path, update, until=1.0)

    assert calls == [origin, origin + 1.5]
    assert files[0]["n"] == 2
    rows = [
        json.loads(line)
        for line in (tmp_path / "tracks.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [row["t"] for row in rows] == [0.0, 1.0]
