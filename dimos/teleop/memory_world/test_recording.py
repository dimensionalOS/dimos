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
from unittest import mock

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
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
    pick_lidar,
    stream_name_of,
)
from dimos.teleop.memory_world.tf_tree import TfTree

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


def test_decode_image_honours_row_padding_and_big_endian_depth() -> None:
    depth = np.array([[1, 2, 3], [4, 5, 60000]], dtype=">u2")  # big-endian 16UC1
    rows = b"".join(row.tobytes() + b"\xee\xee" for row in depth)  # step 8: two pad bytes per row
    body = (
        _Cdr()
        .prim("i", 0)
        .prim("I", 0)
        .string("depth")
        .prim("I", 2, 3)
        .string("16UC1")
        .prim("B", 1)  # is_bigendian
        .prim("I", 8)  # step
        .raw(rows)
        .bytes()
    )
    image = decode_image(body)
    assert image.data.dtype == np.uint16 and image.data.shape == (2, 3)
    assert image.data.tolist() == [[1, 2, 3], [4, 5, 60000]]


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


def test_pick_lidar_takes_the_stream_whose_poses_match_tf(tmp_path: Path) -> None:
    """Names say nothing: the robot's own "lidar" lives in another world, the
    SLAM's scans sit where tf says the sensor is, and a frame tf cannot place
    is out however good its name."""
    store = SqliteStore(path=str(tmp_path / "rec.db"))
    store.start()
    tree = TfTree()
    identity = (0.0, 0.0, 0.0, 1.0)
    try:
        streams = {
            "lidar": ("world", lambda t: (t + 12.0, 0.0, 0.0)),  # the robot's SLAM, 12 m off
            "slam_lidar": ("sensor", lambda t: (t, 0.0, 0.5)),  # 0.5 m from the body
            "orphan": ("nowhere", lambda t: (t, 0.0, 0.0)),  # frame not in tf
        }
        for ts in range(0, 12):
            t = float(ts)
            tree.add("world", "base", t, (t, 0.0, 0.0), identity)
            tree.add("base", "sensor", t, (0.0, 0.0, 0.0), identity)
            for name, (frame, position) in streams.items():
                cloud = PointCloud2.from_numpy(np.zeros((1, 3), np.float32), frame, t)
                store.stream(name, PointCloud2).append(
                    cloud, ts=t, pose=PoseStamped(position=Vector3(*position(t)))
                )
        assert pick_lidar(store, ["lidar", "orphan", "slam_lidar"], tree, "world") == "slam_lidar"
        assert pick_lidar(store, ["orphan"], tree, "world") is None
    finally:
        store.stop()


# ---- adding embeddings with siglipify -----------------------------------------


def test_detect_streams_pairs_camera_info_with_the_chosen_image(tmp_path: Path) -> None:
    import cv2

    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
    from dimos.teleop.memory_world.recording import detect_streams

    ok, encoded = cv2.imencode(".webp", np.zeros((4, 4, 3), dtype=np.uint8))
    assert ok
    store = SqliteStore(path=str(tmp_path / "rig.db"))
    store.start()
    for side, info in (
        ("left", "left_camera_info"),
        ("right", "right_camera_info"),
        ("top", "top_image_camera_info"),
    ):
        store.stream(f"{side}_image", CompressedImage).append(
            CompressedImage(data=encoded.tobytes(), format="webp", frame_id=side, ts=1.0), ts=1.0
        )
        store.stream(info, CameraInfo).append(
            CameraInfo(width=4, height=4, frame_id=side, ts=1.0), ts=1.0
        )
    store.stop()
    recording = open_recording(tmp_path / "rig.db")
    recording.start()
    try:
        chosen = detect_streams(recording, image="right_image")  # the <prefix>_camera_info form
        assert (chosen["image"], chosen["camera_info"]) == ("right_image", "right_camera_info")
        top = detect_streams(recording, image="top_image")  # the <image>_camera_info form
        assert (top["image"], top["camera_info"]) == ("top_image", "top_image_camera_info")
        detected = detect_streams(recording)  # whichever it picks, the pair agrees
        assert detect_streams(recording, image="no_such_image")["image"] == detected["image"]
        assert detected["camera_info"].startswith(detected["image"].removesuffix("_image"))
    finally:
        recording.stop()


def test_ingest_refuses_a_named_stream_the_recording_lacks(tmp_path: Path) -> None:
    import pytest

    from dimos.teleop.memory_world.hyperspace_ingest import ingest_recording

    store = SqliteStore(path=str(tmp_path / "bare.db"))
    store.start()
    store.stop()
    with pytest.raises(SystemExit, match="image"):
        ingest_recording(tmp_path / "bare.db", model_name="m", streams={"image": "nope"})


def test_siglipify_config_names_the_stream_and_model() -> None:
    from dimos.teleop.memory_world.embed import siglipify_command, siglipify_config

    text = siglipify_config("google/siglip2-giant-opt-patch16-384", "color_image", 5)
    assert 'model = "google/siglip2-giant-opt-patch16-384"' in text
    assert 'embedding = "patches"' in text and "stride = 5" in text
    assert 'streams = ["color_image"]' in text
    command = siglipify_command("github:jeff-hykin/siglipify", "/data/rec.mcap")
    assert command[:3] == ["nix", "run", "github:jeff-hykin/siglipify"]
    assert command[-3:] == ["run", "/data/rec.mcap", "--config"]  # the job appends the config path


def test_embedding_job_reports_progress_then_adopts() -> None:
    """Progress bars redraw with carriage returns; each redraw is a progress line."""
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    seen: list[str] = []
    job = EmbeddingJob(on_finished=lambda j: finished.set())
    script = "printf 'loading\\r10/20\\r20/20\\nappended 20\\n'; test -f \"$1\""
    assert job.start(
        ["bash", "-c", script, "--"], "model = 'x'\n", adopt=lambda: seen.append("adopted")
    )
    assert not job.start(["true"], "", adopt=lambda: None)  # one at a time
    assert finished.wait(10)
    assert seen == ["adopted"]
    assert job.status() == {"embedding": "done", "progress": "embeddings added"}


def test_embedding_job_failure_keeps_the_last_line() -> None:
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    job = EmbeddingJob(on_finished=lambda j: finished.set())
    job.start(["bash", "-c", "echo 'no such stream'; exit 3", "--"], "", adopt=lambda: None)
    assert finished.wait(10)
    status = job.status()
    assert status["embedding"] == "failed" and "no such stream" in status["progress"]


def test_tf_root_is_the_frame_with_no_parent() -> None:
    from dimos.teleop.memory_world.recording import tf_root
    from dimos.teleop.memory_world.tf_tree import TfTree

    tree = TfTree()
    for parent, child in (("odom", "base_link"), ("base_link", "camera"), ("base_link", "lidar")):
        tree.add(parent, child, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert tf_root(tree) == "odom"
    tree.add("map2", "other", 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert tf_root(tree) is None, "two roots: nothing to pick"
    assert tf_root(TfTree()) is None


def test_open_recording_reads_compressed_images_as_images(tmp_path: Path) -> None:
    """The stitched Pi recordings store colour as webp CompressedImage."""
    import cv2

    from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.teleop.memory_world.recording import detect_streams

    pixels = np.zeros((8, 12, 3), dtype=np.uint8)
    pixels[:, :, 1] = 200
    ok, encoded = cv2.imencode(".webp", pixels)
    assert ok
    store = SqliteStore(path=str(tmp_path / "stitch.db"))
    store.start()
    store.stream("color_image", CompressedImage).append(
        CompressedImage(data=encoded.tobytes(), format="webp", frame_id="d455_color", ts=5.0),
        ts=5.0,
    )
    store.stop()

    recording = open_recording(tmp_path / "stitch.db")
    recording.start()
    try:
        image = recording.streams["color_image"].first().data
        assert isinstance(image, Image)
        assert image.data.shape == (8, 12, 3) and image.frame_id == "d455_color"
        assert detect_streams(recording)["image"] == "color_image"
    finally:
        recording.stop()


def test_build_tf_tree_holds_static_transforms_and_uses_their_stamps(tmp_path: Path) -> None:
    """tf_static is published once; a camera hung off base_link by it is placed for all time,
    and a transform carrying its own stamp is filed under that stamp."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree

    def moving(x: float, ts: float) -> TFMessage:
        return TFMessage(
            Transform(
                translation=Vector3(x, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="odom",
                child_frame_id="base_link",
                ts=ts,
            )
        )

    store = SqliteStore(path=str(tmp_path / "ros.db"))
    store.start()
    tf = store.stream("tf", TFMessage)
    tf.append(moving(0.0, 10.0), ts=10.0)
    tf.append(moving(10.0, 20.0), ts=20.5)  # batched: stored half a second late
    store.stream("tf_static", TFMessage).append(
        TFMessage(
            Transform(
                translation=Vector3(0.0, 0.0, 1.5),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base_link",
                child_frame_id="camera",
            )
        ),
        ts=9.0,
    )
    try:
        tree = build_tf_tree(store, "tf")
        camera = tree.lookup("odom", "camera", 15.0)
        assert camera is not None
        assert np.allclose(camera[:3, 3], [5.0, 0.0, 1.5])  # halfway by the transforms' own stamps
        assert tree.span("odom", "camera") == (10.0, 20.0)  # the static edge does not bound it
        assert tree.lookup("odom", "camera", 500.0) is None  # but the moving one still does
    finally:
        store.stop()


def test_the_raw_scans_win_over_an_icp_stitch(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A stitched recording carries both, and the stitch is the one we do not want.

    Its loop closure moves the clouds out from under Hyperspace's keyframe poses, so
    search lands where the map no longer agrees. The recording is taken as it is: the
    tree is the tf stream plus its statics, with nothing substituted into it.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, detect_streams

    store = SqliteStore(path=str(tmp_path / "stitched.db"), must_exist=False)
    store.start()
    try:
        cloud = PointCloud2.from_numpy(np.zeros((1, 3), np.float32), "livox_frame", 1.0)
        for name in ("livox_lidar", "livox_lidar_corrected"):
            store.stream(name, PointCloud2).append(cloud, ts=1.0)
        store.stream("tf", TFMessage).append(
            TFMessage(
                Transform(
                    translation=Vector3(5.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="odom",
                    child_frame_id="base_link",
                    ts=1.0,
                )
            ),
            ts=1.0,
        )
        # A loop-closed odometry that moved every pose a metre in y. It must be ignored.
        store.stream("pointlio_odometry_corrected", Odometry).append(
            Odometry(
                frame_id="odom",
                child_frame_id="corrected_odom",
                pose=Pose(Vector3(5.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                ts=1.0,
            ),
            ts=1.0,
        )

        assert detect_streams(store)["lidar"] == "livox_lidar"  # not the stitched copy
        base = build_tf_tree(store, "tf").lookup("odom", "base_link", 1.0, 0.1)
        assert [round(float(v), 3) for v in base[:3, 3]] == [5.0, 0.0, 0.0]  # the recording's own
    finally:
        store.stop()


def _tf_store(tmp_path):  # type: ignore[no-untyped-def]
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=str(tmp_path / "rec.db"), must_exist=False)
    store.start()
    return store


def _edge(parent: str, child: str, x: float, ts: float = 1.0):  # type: ignore[no-untyped-def]
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def test_a_folded_static_edge_still_holds_at_the_end_of_the_recording(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A static edge carries the one stamp it was latched at, and says it holds for all time.

    Copied into the moving stream unchanged, it stops being a statement for all time and
    becomes a series of identical stamps -- which holds at that instant and nowhere else.
    A camera mount that expires five seconds into a ten-minute recording places nothing.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        for step in (1.0, 2.0, 3.0):
            tf.append(TFMessage(_edge("odom", "base", step, step)), ts=step)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 0.5, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 3.0)[0, 3] == 3.5

        assert fold_static_tf(store, "tf", "tf_static") == 1
        tree = build_tf_tree(store, "tf")
        assert tree.lookup("odom", "cam", 3.0)[0, 3] == 3.5  # still placed at the far end
        assert tree.lookup("odom", "cam", 1.0)[0, 3] == 1.5
    finally:
        store.stop()


def test_folding_compares_every_sample_the_moving_stream_carries(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A rig that republishes its mounts gets a stale one in there often enough that
    checking the first sample proves nothing about the rest.

    Leaving such an edge alone and deleting the static stream that disagreed with it is the
    one outcome that loses the right answer outright.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0)  # agrees
        tf.append(TFMessage(_edge("base", "cam", 9.0, 2.0)), ts=2.0)  # and then does not
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )

        assert fold_static_tf(store, "tf", "tf_static") == 1
        assert build_tf_tree(store, "tf").lookup("base", "cam", 2.0)[0, 3] == 2.0
    finally:
        store.stop()


def test_a_static_edge_is_folded_even_where_the_moving_stream_agrees(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """An edge is static because it holds for all time. A moving stream that happens to
    carry the same value in the samples it carries does not say that anywhere.

    A rig that republishes a mount for the first half of a recording and stops leaves the
    second half with no mount at all, and comparing values would call that agreement and
    delete the static stream that was the only thing making it hold throughout.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(
            TFMessage(_edge("base", "cam", 2.0, 1.0), _edge("odom", "base", 1.0, 1.0)), ts=1.0
        )
        tf.append(TFMessage(_edge("odom", "base", 2.0, 2.0)), ts=2.0)  # the mount stops
        tf.append(TFMessage(_edge("odom", "base", 3.0, 3.0)), ts=3.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )

        assert fold_static_tf(store, "tf", "tf_static") == 1
        assert "tf_static" not in store.list_streams()
        tree = build_tf_tree(store, "tf")
        assert tree.lookup("odom", "cam", 3.0)[0, 3] == 5.0  # placed where it had stopped
        assert tree.lookup("odom", "cam", 2.0)[0, 3] == 4.0  # and in the middle
        # Once, not once per sample: a single-sample series holds from its stamp forward
        # for ever, so one copy says what a static says and a long tf is not multiplied
        # by its mounts.
        mounts = [
            t
            for obs in store.streams["tf"]
            for t in obs.data.transforms
            if (str(t.frame_id), str(t.child_frame_id)) == ("base", "cam")
        ]
        assert len(mounts) == 1
    finally:
        store.stop()


def test_folding_into_an_empty_tf_refuses_rather_than_destroying_the_statics(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """There is nowhere to fold into, and going ahead writes nothing while deleting the
    static stream on the way out -- every mount in the recording, gone, reported as done."""
    import pytest

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage)  # declared and empty
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        with pytest.raises(SystemExit):
            fold_static_tf(store, "tf", "tf_static")
        assert "tf_static" in store.list_streams()
    finally:
        store.stop()


def test_a_tf_truncated_by_a_dead_rebuild_is_refused_rather_than_read(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A rebuild killed outright leaves the real stream short and the whole copy beside it.

    Nothing else would notice: a truncated tf still loads, still answers, and places the
    second half of the recording nowhere. It is the one tree everything reads, so the
    reader everything goes through is where this has to stop.
    """
    import pytest

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, detect_streams

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)  # what survived
        staged = store.stream("tf__rebuilt", TFMessage)
        for step in (1.0, 2.0, 3.0):
            staged.append(TFMessage(_edge("odom", "base", step, step)), ts=step)

        with pytest.raises(SystemExit) as refusal:
            build_tf_tree(store, "tf")
        assert "1 samples" in str(refusal.value) and "3" in str(refusal.value)
        # And the copy is never mistaken for the recording's own tf.
        assert detect_streams(store)["tf"] == "tf"
    finally:
        store.stop()


def test_a_static_tf_cut_short_by_a_dead_rebuild_is_refused_too(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The moving tf is not the only stream a rebuild can die in the middle of.

    A truncated static tf is worse than a truncated moving one, because it is silent in
    both directions: the tree loads with a mount missing, and a fold would then delete the
    only complete copy there is.
    """
    import pytest

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage).append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)
        store.stream("tf_static", TFMessage).append(TFMessage(_edge("base", "imu", 4.0)), ts=1.0)
        staged = store.stream("tf_static__rebuilt", TFMessage)
        staged.append(TFMessage(_edge("base", "imu", 4.0), _edge("base", "gps", 2.0)), ts=1.0)

        with pytest.raises(SystemExit):
            build_tf_tree(store, "tf")
        with pytest.raises(SystemExit):
            fold_static_tf(store, "tf", "tf_static")
        assert "tf_static__rebuilt" in store.list_streams()  # the complete copy stays
    finally:
        store.stop()


def test_a_folded_mount_covers_the_whole_span_the_tf_can_be_asked_about(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A tf message carries its own stamp, and it is not the stamp it was recorded at.

    TfTree reads the message's. Stamping the folded copies by the recording stamp alone
    leaves the edge holding over a window slightly inside the recording, and the frames at
    either end -- the first and last thing the camera saw -- are placed nowhere.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("odom", "base", 1.0, 10.0)), ts=11.0)  # said at 10, kept at 11
        tf.append(TFMessage(_edge("odom", "base", 2.0, 20.0)), ts=21.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 10.0)[0, 3] == 3.0

        fold_static_tf(store, "tf", "tf_static")
        tree = build_tf_tree(store, "tf")
        assert tree.lookup("odom", "cam", 10.0)[0, 3] == 3.0  # the first frame, still placed
        assert tree.lookup("odom", "cam", 20.0)[0, 3] == 4.0  # and the last
    finally:
        store.stop()


def test_a_rebuild_that_died_before_writing_back_is_caught_at_detection(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """The worst moment to die is between dropping the old stream and writing the new one.

    The staged copy is skipped by detection, so nothing would be picked for the role at
    all: the module would start with no tf, place everything by image stamps, and complain
    about a stream named "". The whole recording is there, under one wrong name.
    """
    import pytest

    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import detect_streams

    store = _tf_store(tmp_path)
    try:
        store.stream("tf__rebuilt", TFMessage).append(
            TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0
        )
        with pytest.raises(SystemExit) as refusal:
            detect_streams(store)
        assert "tf__rebuilt" in str(refusal.value)
    finally:
        store.stop()


def test_an_empty_stream_never_takes_a_role_from_a_real_one(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """An ingest killed outright can leave a stream called `tf` behind with nothing in it.

    It holds the right payload and the plainest name there is, so it outranks the
    recording's own `robot_tf` from then on and the world has no transforms at all. An
    empty stream cannot fill a role, so it is not a candidate for one.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import detect_streams

    store = _tf_store(tmp_path)
    try:
        store.stream("robot_tf", TFMessage).append(
            TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0
        )
        store.stream("tf", TFMessage)  # what a killed ingest leaves
        assert detect_streams(store)["tf"] == "robot_tf"
    finally:
        store.stop()


def test_a_static_folded_into_a_one_sample_tf_still_holds_for_all_time(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A recording whose odometry produced one tf message still has pictures all through it.

    _Edge.at holds a single-sample series for all time, which is exactly what a static edge
    means. Two samples at the same instant do not: they expire a tolerance either side. So
    where the two ends of the span coincide, one copy goes in and not two.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage).append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 100.0)[0, 3] == 3.0
    finally:
        store.stop()


def test_an_unstamped_transform_does_not_move_the_folded_edge_past_the_first_frame(  # type: ignore[no-untyped-def]
    tmp_path,
) -> None:
    """TfTree reads a transform's own stamp and falls back to the observation's when it is 0.

    Measuring the span any other way makes the earliest stamp look like 0, the folded copy
    is written there, and the reader turns that back into the recording stamp of the first
    row -- later than the first frame the camera took, which is then placed nowhere.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        # One transform unstamped (the reader will call it 10, the row's own stamp) and one
        # stamped at 5. The earliest moment the tree can be asked about is 5, not 0.
        tf.append(
            TFMessage(_edge("odom", "base", 1.0, 0.0), _edge("base", "wheel", 1.0, 5.0)),
            ts=10.0,
        )
        tf.append(TFMessage(_edge("odom", "base", 2.0, 20.0)), ts=20.0)
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("base", "cam", 5.0)[0, 3] == 2.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("base", "cam", 5.0)[0, 3] == 2.0
    finally:
        store.stop()


def test_a_static_declared_twice_folds_the_first_value(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A latched static tf republished with a different value: the tree keeps the FIRST.

    Folding has to keep the same one, or the recording means something different after it
    was folded than it did before -- and an mcap, which is folded nowhere because it cannot
    be written to, would then disagree with the .db beside it.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        for step in (1.0, 2.0):
            tf.append(TFMessage(_edge("odom", "base", step, step)), ts=step)
        latched = store.stream("tf_static", TFMessage)
        latched.append(TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0)
        latched.append(TFMessage(_edge("base", "cam", 9.0, 2.0)), ts=2.0)  # a later, different one
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 2.0)[0, 3] == 4.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 2.0)[0, 3] == 4.0
    finally:
        store.stop()


def test_a_recording_the_module_refuses_is_not_left_open_and_half_named(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """Naming the streams can refuse the recording, and the refusal has to reach the user.

    Publishing the store before naming it turned that into one traceback and a module that
    went on serving with every role still empty -- logging "no '' stream; falling back to
    the poses stamped on images", which is word for word the failure the refusal exists to
    prevent.
    """
    import threading
    from types import SimpleNamespace

    import pytest

    from dimos.teleop.memory_world.module import MemoryWorldModule

    stopped = []

    class Module(MemoryWorldModule):
        def __init__(self) -> None:
            self._store = None
            self._store_lock = threading.RLock()
            self.config = SimpleNamespace(store_path=str(tmp_path / "walk.db"))

        def _name_streams(self, store) -> None:  # type: ignore[no-untyped-def, override]
            raise SystemExit("a rebuild died in this recording")

    module = Module()
    opened = SimpleNamespace(stop=lambda: stopped.append(True))
    with mock.patch("dimos.teleop.memory_world.module.open_recording", lambda _p: opened):
        for _ in range(2):  # and again: a refusal is not a thing you get past by retrying
            with pytest.raises(SystemExit):
                module._ensure_store()
    assert module._store is None
    assert stopped == [True, True]


def test_a_folded_mount_outlives_the_tf_the_way_a_static_did(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A static edge holds for all time, and folding must not quietly put an end on it.

    An odometry that published once still places at any later moment, because a
    single-sample series holds forward for ever. A mount folded in as two samples would
    stop holding after the last of them, so the camera would go missing from a recording
    whose robot never moved again -- while the odometry beside it carried on.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        tf = store.stream("tf", TFMessage)
        tf.append(TFMessage(_edge("odom", "base", 1.0, 1.0)), ts=1.0)  # published once
        tf.append(TFMessage(_edge("base", "wheel", 1.0, 2.0)), ts=2.0)  # something else, later
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 100.0)[0, 3] == 3.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("odom", "cam", 100.0)[0, 3] == 3.0
    finally:
        store.stop()
