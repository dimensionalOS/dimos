# Copyright 2025-2026 Dimensional Inc.
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

"""The CDR decoders and the derived-streams wrapper, against a real ROS 2 mcap.

The recording is one ``~/Commands/db_to_mcap`` wrote from a mem2 db that sits
beside it, so every decoded message can be compared with the db's own copy.
Skipped where that dataset is not on the machine.
"""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import struct

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.teleop.memory_world.recording import (
    EMBEDDING_PAYLOAD_MODULE,
    RecordingWithDerivedStreams,
    StoredEmbeddings,
    decode_camera_info,
    decode_image,
    decode_multiarray,
    decode_tf_message,
    derived_db_path,
    embedding_stream_name,
    grid_side,
    open_recording,
    open_ros2_mcap,
    stream_name_of,
)

DATASET = Path("~/datasets/d455/sf_office1_2").expanduser()
needs_dataset = pytest.mark.skipif(
    not (DATASET / "main.mcap").is_file() or not (DATASET / "main.db").is_file(),
    reason="sf_office1_2 recording not on this machine",
)


class _Cdr:
    """A little CDR writer: every primitive is padded to its own size from the body start."""

    def __init__(self) -> None:
        self.body = bytearray()

    def prim(self, code: str, *values: float) -> _Cdr:
        size = struct.calcsize(code)
        self.body += b"\0" * (-len(self.body) % size)
        self.body += struct.pack("<" + code * len(values), *values)
        return self

    def string(self, text: str) -> _Cdr:
        raw = text.encode() + b"\0"
        self.prim("I", len(raw))
        self.body += raw
        return self

    def raw(self, data: bytes) -> _Cdr:
        self.prim("I", len(data))
        self.body += data
        return self

    def bytes(self) -> bytes:
        return b"\x00\x01\x00\x00" + bytes(self.body)


def encode_multiarray(vectors: np.ndarray) -> bytes:
    """``std_msgs/msg/Float32MultiArray`` the way siglipify lays it out: [patch, dim] or [dim]."""
    count, dims = vectors.shape
    writer = _Cdr()
    if count == 1:
        writer.prim("I", 1).string("dim").prim("I", dims, dims)
    else:
        writer.prim("I", 2).string("patch").prim("I", count, count * dims)
        writer.string("dim").prim("I", dims, dims)
    flat = vectors.astype(np.float32).ravel().tolist()
    writer.prim("I", 0)  # data_offset
    return writer.prim("I", len(flat)).prim("f", *flat).bytes()


def seed_embedding_stream(
    db_path: str,
    name: str,
    model: str,
    rows: list[tuple[float, int | None, np.ndarray]],
    text_aligned: bool | None = True,
) -> None:
    """Write (ts, source_id, vectors) rows the way siglipify's mem2 writer does.

    ``text_aligned=None`` leaves the mark out, as siglipify did before it applied
    the head per patch."""
    conn = sqlite3.connect(db_path)
    try:
        conn.executescript(
            f'''CREATE TABLE IF NOT EXISTS "{name}" (
                   id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, value NUMERIC,
                   pose_x REAL, pose_y REAL, pose_z REAL,
                   pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
                   tags BLOB DEFAULT (jsonb('{{}}')));
               CREATE TABLE IF NOT EXISTS "{name}_blob" (id INTEGER PRIMARY KEY, data BLOB NOT NULL);
               CREATE TABLE IF NOT EXISTS _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL);'''
        )
        conn.execute(
            "INSERT OR REPLACE INTO _streams (name, config) VALUES (?, ?)",
            (
                name,
                json.dumps(
                    {
                        "payload_module": EMBEDDING_PAYLOAD_MODULE,
                        "codec_id": "cdr",
                        "model": model,
                        **({} if text_aligned is None else {"text_aligned": text_aligned}),
                    }
                ),
            ),
        )
        for ts, source_id, vectors in rows:
            tags = {"model": model}
            if source_id is not None:
                tags["source_id"] = source_id
            cursor = conn.execute(
                f'INSERT INTO "{name}" (ts, tags) VALUES (?, jsonb(?))', (ts, json.dumps(tags))
            )
            conn.execute(
                f'INSERT INTO "{name}_blob" (id, data) VALUES (?, ?)',
                (cursor.lastrowid, encode_multiarray(vectors)),
            )
        conn.commit()
    finally:
        conn.close()


def test_stream_names_match_the_db_convention() -> None:
    assert stream_name_of("/realsense_color_image") == "realsense_color_image"
    assert stream_name_of("rt/utlidar/cloud") == "utlidar_cloud"


def test_decode_image_by_hand() -> None:
    pixels = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    body = (
        _Cdr()
        .prim("i", 12)
        .prim("I", 500)  # header stamp
        .string("cam")
        .prim("I", 2, 3)  # height, width
        .string("rgb8")
        .prim("B", 0)  # is_bigendian
        .prim("I", 9)  # step
        .raw(pixels.tobytes())
        .bytes()
    )
    image = decode_image(body)
    assert image.data.shape == (2, 3, 3)
    assert np.array_equal(image.data, pixels)
    assert image.frame_id == "cam"
    assert image.ts == pytest.approx(12.0000005)


def test_decode_tf_message_by_hand() -> None:
    writer = _Cdr().prim("I", 2)
    for parent, child in (("world", "base"), ("base", "cam")):
        writer.prim("i", 1).prim("I", 0).string(parent).string(child)
        writer.prim("d", 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    message = decode_tf_message(writer.bytes())
    assert [(t.frame_id, t.child_frame_id) for t in message.transforms] == [
        ("world", "base"),
        ("base", "cam"),
    ]
    assert message.transforms[1].translation.z == 3.0
    assert message.transforms[1].rotation.w == 1.0


def test_decode_multiarray_by_hand() -> None:
    grid = np.arange(6, dtype=np.float32).reshape(2, 3)
    assert decode_multiarray(encode_multiarray(grid)).sizes == (2, 3)
    np.testing.assert_array_equal(decode_multiarray(encode_multiarray(grid)).vectors(), grid)
    pooled = decode_multiarray(encode_multiarray(np.ones((1, 4), np.float32)))
    assert pooled.sizes == (4,)
    assert pooled.vectors().shape == (1, 4)


def test_embedding_streams_are_named_the_way_siglipify_names_them() -> None:
    name = embedding_stream_name("color_image", "google/siglip2-giant-opt-patch16-384")
    assert name == "color_image_siglip2_giant_opt_p16_384"


def test_grid_side_needs_a_square() -> None:
    assert grid_side(576) == 24
    assert grid_side(1) == 1
    with pytest.raises(ValueError, match="square"):
        grid_side(10)


def test_stored_embeddings_read_siglipify_rows_from_a_db(tmp_path: Path) -> None:
    """The stream's payload type is a ROS message dimos cannot import, so the store
    cannot open it, but its rows are still readable."""
    path = str(tmp_path / "rec.db")
    store = SqliteStore(path=path)
    store.start()
    try:
        store.stream("color_image", int).append(1, ts=1.0)
        seed_embedding_stream(
            path,
            "color_image_siglip2_giant_opt_p16_384",
            "google/siglip2-giant-opt-patch16-384",
            [(1.5, 3, np.ones((4, 2), np.float32)), (1.0, None, np.zeros((4, 2), np.float32))],
        )
        rows = StoredEmbeddings(store, "color_image_siglip2_giant_opt_p16_384")
        assert "color_image_siglip2_giant_opt_p16_384" in store.list_streams()
        assert rows.count() == 2
        loaded = list(rows)
        assert [(r.ts, r.source_id, r.model) for r in loaded] == [
            (1.0, None, "google/siglip2-giant-opt-patch16-384"),
            (1.5, 3, "google/siglip2-giant-opt-patch16-384"),
        ]
        assert loaded[1].vectors.shape == (4, 2) and loaded[1].vectors.dtype == np.float32
        assert rows.text_aligned() is True
    finally:
        store.stop()


@needs_dataset
def test_mcap_decodes_like_the_db() -> None:
    mcap = open_ros2_mcap(DATASET / "main.mcap")
    db = SqliteStore(path=str(DATASET / "main.db"), must_exist=True)
    try:
        for name in ("pointlio_lidar", "realsense_color_image", "realsense_depth_image", "tf"):
            a, b = mcap.streams[name].first(), db.streams[name].first()
            assert a.ts == pytest.approx(b.ts, abs=1e-6)
            if name == "pointlio_lidar":
                assert np.allclose(a.data.points_f32(), b.data.points_f32())
            elif name == "tf":
                for ours, theirs in zip(a.data.transforms, b.data.transforms, strict=True):
                    assert (ours.frame_id, ours.child_frame_id) == (
                        theirs.frame_id,
                        theirs.child_frame_id,
                    )
                    assert ours.translation.x == pytest.approx(theirs.translation.x)
            else:
                assert np.array_equal(a.data.data, b.data.data)
                assert a.data.format == b.data.format
        info = mcap.streams["realsense_color_image_camera_info"].first().data
        assert decode_camera_info is not None and info.width == 848 and info.K[0] > 0
        # a time window reads only the chunks it needs, so this must be fast
        t = db.streams["realsense_color_image"].first().ts + 100.0
        assert 1 <= len(list(mcap.streams["realsense_color_image"].at(t, tolerance=0.1))) <= 8
    finally:
        db.stop()


@needs_dataset
def test_derived_streams_live_beside_the_mcap(tmp_path: Path) -> None:
    assert derived_db_path("/x/y/main.mcap") == Path("/x/y/main.derived.db")
    store = RecordingWithDerivedStreams(
        open_ros2_mcap(DATASET / "main.mcap"), SqliteStore(path=str(tmp_path / "derived.db"))
    )
    try:
        assert "tf" in store.list_streams() and "voxel_diff" not in store.list_streams()
        diffs = store.stream("voxel_diff", PointCloud2)
        diffs.append(PointCloud2.from_numpy(np.zeros((1, 3), np.float32), "world", 1.0), ts=1.0)
        assert "voxel_diff" in store.list_streams()
        assert store.streams["voxel_diff"].count() == 1
        with pytest.raises(ValueError):
            store.delete_stream("tf")
        store.delete_stream("voxel_diff")
        assert "voxel_diff" not in store.list_streams()
    finally:
        store.stop()


def test_open_recording_rejects_missing_db(tmp_path: Path) -> None:
    with pytest.raises(Exception):
        open_recording(tmp_path / "missing.db")
