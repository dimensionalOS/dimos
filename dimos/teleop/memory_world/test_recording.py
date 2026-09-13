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
import time
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
    """Progress bars redraw with carriage returns; each redraw is a progress line.

    The fixture below is built entirely around the ``\\r`` splitting, but this used to
    assert only that the job adopted and ended on its constant done_message -- so
    deleting the `.replace("\\r", "\\n")` from embed.py left the whole suite green.
    What that removes is user-visible: siglipify and the Hyperspace ingest both redraw a
    bar with carriage returns and no newline, so without the split the viewer's
    "Preparing search" line never moves off "starting siglipify" for the whole run.
    So the progress is WATCHED here, not just the outcome.
    """
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    seen: list[str] = []
    progress: list[str] = []
    job = EmbeddingJob(on_finished=lambda j: finished.set())
    # A bar redrawing in place: no newline until the very end.
    # Paced, because the point is that each redraw becomes its own progress line while
    # the job runs -- printf'ing it all at once finishes before anything can observe it.
    script = (
        "printf 'loading\\r'; sleep 0.08;"
        " printf '10/20\\r'; sleep 0.08;"
        " printf '20/20\\r'; sleep 0.08;"
        " printf '\\nappended 20\\n'; test -f \"$1\""
    )

    watching = threading.Event()

    def watch() -> None:
        while not watching.is_set():
            line = job.status()["progress"]
            if not progress or progress[-1] != line:
                progress.append(line)
            time.sleep(0.002)

    watcher = threading.Thread(target=watch, daemon=True)
    watcher.start()
    assert job.start(
        ["bash", "-c", script, "--"], "model = 'x'\n", adopt=lambda: seen.append("adopted")
    )
    assert not job.start(["true"], "", adopt=lambda: None)  # one at a time
    assert finished.wait(10)
    watching.set()
    watcher.join(2)

    assert seen == ["adopted"]
    assert job.status() == {"embedding": "done", "progress": "embeddings added"}
    # Each redraw has to have been its own progress line. Without the carriage-return
    # split the job reports "starting siglipify" until the first newline arrives.
    assert "10/20" in progress and "20/20" in progress, progress


def test_embedding_job_failure_keeps_the_last_line() -> None:
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    job = EmbeddingJob(on_finished=lambda j: finished.set())
    job.start(["bash", "-c", "echo 'no such stream'; exit 3", "--"], "", adopt=lambda: None)
    assert finished.wait(10)
    status = job.status()
    assert status["embedding"] == "failed" and "no such stream" in status["progress"]


def test_a_finished_job_cleaning_up_does_not_mark_its_successor_failed() -> None:
    """The guard in the sibling below must only ever speak for its OWN run.

    `_run` publishes "done" before it reaches its finally block, so a second job can
    start in that window -- and the finally then found a state reading "running" and
    flipped it to "failed". The live job was reported as failed, and a third job was let
    in beside it, which is the one thing `start()` exists to prevent.

    Checking the process handle instead is not enough, and that is the point: a new run
    sets `_process` only once its subprocess has spawned, so in this window the handle
    still says the OLD run owns it while the new one already reads "running".

    Made deterministic rather than raced: B is started from inside the very `_set("done")`
    call that opens the window, so A always walks into its cleanup with B live.
    """
    from dimos.teleop.memory_world.embed import EmbeddingJob

    job = EmbeddingJob()
    started_b: list[bool] = []
    real_set = job._set

    def set_and_hand_over(state: str, progress: str) -> None:
        real_set(state, progress)
        if state == "done" and not started_b:
            # The window: A has published "done" and has not reached its finally.
            started_b.append(job.start(["true"], "", adopt=lambda: time.sleep(0.5)))

    job._set = set_and_hand_over  # type: ignore[method-assign]
    assert job.start(["true"], "", adopt=lambda: None)

    for _ in range(500):  # A's thread runs to completion, cleanup included
        if started_b:
            break
        time.sleep(0.01)
    assert started_b == [True], "B never got the window this test is about"
    time.sleep(0.2)  # A's finally has every chance to run

    assert job.status()["embedding"] == "running", (
        f"the finished job reported its live successor as {job.status()['embedding']!r}"
    )
    assert not job.start(["true"], "", adopt=lambda: None), "a third job was let in"


def test_an_embedding_job_that_dies_without_an_exception_does_not_wedge_the_next_one() -> None:
    """`start()` refuses to run while the state reads "running", so a state stuck there
    is not one dead job -- it is every future job, for the life of the process, with the
    viewer's readiness poll waiting on all of them.

    `except Exception` is what sets "failed", and SystemExit and KeyboardInterrupt are
    not Exceptions. `threading` swallows a SystemExit out of a thread without printing
    anything, so the job goes quiet and the state never moves. `adopt` running inside the
    try is the reachable version of this; the handler raising on its own is another.
    """
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    job = EmbeddingJob(on_finished=lambda j: finished.set())

    def adopt_and_exit() -> None:
        raise SystemExit(1)

    assert job.start(["true"], "", adopt=adopt_and_exit)
    assert finished.wait(10)

    status = job.status()
    assert status["embedding"] == "failed", f"stuck at {status['embedding']!r}"

    # The damage this actually does: nothing can be started afterwards.
    finished.clear()
    assert job.start(["true"], "", adopt=lambda: None), "a later job can never run"
    assert finished.wait(10)
    assert job.status()["embedding"] == "done"


def test_embedding_job_failure_keeps_a_last_line_that_had_no_newline() -> None:
    """A process that dies mid-line never terminates it, and that line is the reason.

    The sibling above uses `echo`, which appends a newline, so the reason arrives as a
    complete line and the loop sees it. `printf` without one leaves it in the buffer,
    where flushing it to the log is not enough: the failure the viewer is shown is built
    from `last`, so the reason has to reach THAT, not just the log.
    """
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    job = EmbeddingJob(on_finished=lambda j: finished.set())
    job.start(["bash", "-c", "printf 'CUDA out of memory'; exit 1", "--"], "", adopt=lambda: None)
    assert finished.wait(10)
    status = job.status()
    assert status["embedding"] == "failed"
    assert "CUDA out of memory" in status["progress"], status


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

        detected = detect_streams(store)
        assert detected["lidar"] == "livox_lidar"  # not the stitched copy
        # The pick alone does not pin this: `<x>_corrected` is always the longer name, so
        # rank()'s len(name) tiebreak decided it and cutting the disqualifier down to
        # ("costmap",) left the whole suite green. The candidate list is what
        # `name_streams` hands to pick_lidar, and it is where the stitch has to be absent.
        assert "livox_lidar_corrected" not in detected["lidar_candidates"]
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


def test_naming_the_streams_can_read_the_store_it_is_naming(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """_name_streams reads the tf tree to pick the lidar, and that calls _ensure_store.

    Publishing the store only after naming it therefore opened the recording again for
    every such read, and again, until RecursionError -- on the ordinary first start of a
    module, which is every start.
    """
    import threading
    from types import SimpleNamespace

    from dimos.teleop.memory_world.module import MemoryWorldModule

    opens = []

    class Module(MemoryWorldModule):
        def __init__(self) -> None:
            self._store = None
            self._store_lock = threading.RLock()
            self.config = SimpleNamespace(store_path=str(tmp_path / "walk.db"))

        def _name_streams(self, store) -> None:  # type: ignore[no-untyped-def, override]
            # What _tf_tree does, which is what naming the lidar needs.
            assert self._ensure_store() is store

    module = Module()
    with mock.patch(
        "dimos.teleop.memory_world.module.open_recording",
        lambda path: opens.append(path) or SimpleNamespace(stop=lambda: None),
    ):
        store = module._ensure_store()
    assert len(opens) == 1
    assert module._store is store


def test_a_recording_the_module_refuses_is_not_left_open_and_half_named(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """Naming the streams can refuse the recording, and the refusal has to reach the user.

    Leaving the store published after a refusal means every later _ensure_store hands back
    a store with every role still empty, and the module goes on logging "no '' stream;
    falling back to the poses stamped on images" -- word for word the failure the refusal
    exists to prevent.
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


def test_a_static_latched_before_the_robot_moved_still_holds_at_that_moment(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """A rig latches its mounts and then starts moving, so pictures exist in the gap.

    Stamping the folded copy at the earliest MOVING sample puts the mount after those, and
    a photograph whose whole path to the camera is static -- which used to resolve at any
    time at all -- is placed nowhere.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree, fold_static_tf

    store = _tf_store(tmp_path)
    try:
        store.stream("tf", TFMessage).append(
            TFMessage(_edge("base", "wheel", 1.0, 10.0)), ts=10.0
        )  # nothing moves until 10
        store.stream("tf_static", TFMessage).append(
            TFMessage(_edge("world", "base", 1.0, 1.0), _edge("base", "cam", 2.0, 1.0)), ts=1.0
        )
        assert build_tf_tree(store, "tf").lookup("world", "cam", 1.0)[0, 3] == 3.0

        fold_static_tf(store, "tf", "tf_static")
        assert build_tf_tree(store, "tf").lookup("world", "cam", 1.0)[0, 3] == 3.0
        assert build_tf_tree(store, "tf").lookup("world", "cam", 100.0)[0, 3] == 3.0
    finally:
        store.stop()


def test_an_empty_depth_camera_info_does_not_outrank_a_populated_one(tmp_path: Path) -> None:
    """The third place in this package to learn that a name is not its contents.

    `detect_streams` skips empty streams because a killed ingest leaves the NAME behind.
    `precomputed_stream_name` skips them because an empty embeddings stream is adopted as
    an index that can never be built. This one picked `<depth>_camera_info` over
    `<depth-without-_image>_camera_info` on name alone -- and the Hyperspace ingest then
    died at "stream ... is empty" AFTER deleting the index it was replacing.
    """
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.teleop.memory_world.recording import depth_info_stream_for

    store = SqliteStore(path=str(tmp_path / "r.db"))
    store.start()
    try:
        # The higher-ranked name exists and is EMPTY; the lower-ranked one has intrinsics.
        store.stream("depth_image_camera_info", CameraInfo)
        populated = store.stream("depth_camera_info", CameraInfo)
        populated.append(CameraInfo(width=8, height=8, frame_id="depth", ts=1.0), ts=1.0)

        chosen = depth_info_stream_for(store, "depth_image", "color_camera_info")
        assert chosen == "depth_camera_info", "an empty stream is not intrinsics"

        # And with neither usable, it still falls back to the colour camera's.
        bare = SqliteStore(path=str(tmp_path / "bare.db"))
        bare.start()
        try:
            bare.stream("depth_image_camera_info", CameraInfo)  # empty
            assert (
                depth_info_stream_for(bare, "depth_image", "color_camera_info")
                == "color_camera_info"
            )
        finally:
            bare.stop()
    finally:
        store.stop()


def test_the_ingest_preflight_refuses_a_half_done_rebuild_before_it_deletes(tmp_path: Path) -> None:
    """An empty stream is not the only way the ingest refuses AFTER deleting the index.

    `_ingest` calls `build_tf_tree`, which refuses a staged rebuild -- and a killed
    calibration leaves exactly that, `tf` and `tf__rebuilt` both non-empty. The preflight
    guard added a round earlier checked five stream NAMES for emptiness and missed this,
    so the index was deleted and the run then died anyway. Rebuilding a Hyperspace index
    is a long GPU run, so that is real loss, and it is the ninth time in this loop a fix
    reached one of its cases and not a sibling.

    This is the guard as the ingest calls it, on the artifact the calibration leaves.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import (
        STAGED_SUFFIX,
        refuse_if_a_rebuild_is_half_done,
    )

    store = SqliteStore(path=str(tmp_path / "r.db"))
    store.start()
    try:
        for name in ("tf", "tf" + STAGED_SUFFIX):
            stream = store.stream(name, TFMessage)
            stream.append(TFMessage(_edge("odom", "base", 1.0, 0.0)), ts=0.0)

        with pytest.raises(SystemExit) as refused:
            refuse_if_a_rebuild_is_half_done(store, "tf")
        assert "tf" in str(refused.value)

        # And with no staged copy it says nothing, so the ordinary ingest is not blocked.
        plain = SqliteStore(path=str(tmp_path / "plain.db"))
        plain.start()
        try:
            plain.stream("tf", TFMessage).append(TFMessage(_edge("odom", "base", 1.0, 0.0)), ts=0.0)
            refuse_if_a_rebuild_is_half_done(plain, "tf")
        finally:
            plain.stop()
    finally:
        store.stop()


def test_a_ros_named_depth_topic_still_finds_its_own_camera_info(tmp_path: Path) -> None:
    """`camera_depth_image_rect_raw` does not END with `_image`, so stripping a suffix
    did nothing and the depth camera's own intrinsics were never looked for.

    What was used instead was the COLOUR camera's K, as the DEPTH camera's. The patch
    correction then mapped colour to colour and indexed a 1280-wide raster into an
    848-wide one: every patch past uv 0.66 falls outside the depth image and is dropped,
    and the ones that survive sample about half a frame too far right and down.
    """
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.teleop.memory_world.recording import depth_info_stream_for

    store = SqliteStore(path=str(tmp_path / "ros.db"))
    store.start()
    try:
        own = store.stream("camera_depth_camera_info", CameraInfo)
        own.append(CameraInfo(width=848, height=480, frame_id="depth", ts=1.0), ts=1.0)
        colour = store.stream("camera_color_camera_info", CameraInfo)
        colour.append(CameraInfo(width=1280, height=720, frame_id="colour", ts=1.0), ts=1.0)

        chosen = depth_info_stream_for(
            store, "camera_depth_image_rect_raw", "camera_color_camera_info"
        )
        assert chosen == "camera_depth_camera_info", (
            "the depth camera's own intrinsics, not the colour camera's"
        )
    finally:
        store.stop()


def test_detect_streams_pairs_an_image_with_its_own_camera_info(tmp_path: Path) -> None:
    """The same suffix bug as the depth test above, on the COLOUR side this time.

    `camera_color_image_raw` does not END with `_image` either, so `removesuffix` was a
    no-op, both candidates in the pairing loop were the identical string, neither existed,
    and the generic hint ranking chose a CameraInfo on its own -- on a recording with two
    cameras, the other one's. That K reaches `_camera_hfov`, `sensor_intrinsics` and
    `patch_world_position`, which is to say it reaches where the answers are placed.
    """
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.teleop.memory_world.recording import detect_streams

    store = SqliteStore(path=str(tmp_path / "two_cameras.db"))
    store.start()
    try:
        for name, width in (("camera_color_camera_info", 1280), ("cam2_color_camera_info", 640)):
            store.stream(name, CameraInfo).append(
                CameraInfo(width=width, height=480, frame_id=name, ts=1.0), ts=1.0
            )
        # `Image`, not `CompressedImage`: detect_streams only honours the caller's
        # chosen image when it is in the `Image` list, and picks its own otherwise.
        for name in ("camera_color_image_raw", "cam2_color_image_raw"):
            store.stream(name, Image).append(
                Image(data=np.zeros((1, 1, 3), np.uint8), frame_id=name, ts=1.0),
                ts=1.0,
            )

        found = detect_streams(store, image="camera_color_image_raw")
        assert found["camera_info"] == "camera_color_camera_info", (
            f"paired with another camera's intrinsics: {found['camera_info']}"
        )
        other = detect_streams(store, image="cam2_color_image_raw")
        assert other["camera_info"] == "cam2_color_camera_info", (
            f"paired with another camera's intrinsics: {other['camera_info']}"
        )
    finally:
        store.stop()


def test_the_roi_survives_the_mcap_decode(monkeypatch: pytest.MonkeyPatch) -> None:
    """A camera_info read from an mcap must say the same thing as one read from a db.

    `_CameraInfoWire` decodes the roi correctly and `decode_camera_info` then has to copy
    it across by hand, because `CameraInfo.__init__` takes no roi and zeroes the five
    fields. It did not, so every camera_info out of an mcap reported "no roi" -- the exact
    field `sensor_intrinsics` reads to tell a CROP from a resize. A cropped rig had its
    patches placed 0.84 m out laterally with the sign of x flipped, while the SAME
    recording read from a .db was right, because `lcm_decode` copies them over.

    The wire decode was never the broken half, so this stubs it: what is under test is
    which of the decoded fields `decode_camera_info` carries into the message it returns.
    Hand-rolling CDR bytes here would test the cdr reader instead, and would be one more
    probe aimed slightly beside the question.
    """
    from types import SimpleNamespace

    from dimos.teleop.memory_world import recording as rec
    from dimos.teleop.memory_world.visual_search import sensor_intrinsics

    wire = SimpleNamespace(
        header=SimpleNamespace(frame_id="camera_optical", stamp=SimpleNamespace(sec=1, nanosec=0)),
        height=960,
        width=1280,
        distortion_model="plumb_bob",
        d=np.zeros(5),
        k=np.array([900.0, 0.0, 640.0, 0.0, 900.0, 480.0, 0.0, 0.0, 1.0]),
        r=np.eye(3).ravel(),
        p=np.zeros(12),
        binning_x=0,
        binning_y=0,
        roi=SimpleNamespace(x_offset=100, y_offset=50, height=480, width=640, do_rectify=0),
    )
    monkeypatch.setattr(rec.cdr, "decode", lambda buf, kind: (wire, 0))
    monkeypatch.setattr(rec.ros, "_ts", lambda header: 1.0)

    decoded = rec.decode_camera_info(b"")
    assert (decoded.roi_x_offset, decoded.roi_y_offset) == (100, 50)
    assert (decoded.roi_width, decoded.roi_height) == (640, 480)
    # And the consequence the roi is read for: a crop, not a 1280x960 frame to resize.
    assert sensor_intrinsics(decoded) == ((900.0, 900.0, 540.0, 430.0), (640, 480))


def test_terminate_stops_the_work_and_not_just_the_launcher() -> None:
    """The real command is `nix run <flake> -- run <recording> ...`, and `nix run`
    commonly FORKS into the built program rather than exec'ing it.

    So the work siglipify does -- writing into the recording -- happens in a grandchild,
    and signalling the immediate child alone never reached it. `MemoryWorldModule.stop()`
    calls `terminate()` and does not join the thread, so `memworld --stop` returned while
    the embedding subprocess was still writing to the db the next server would open.

    The job also went on reporting "running" throughout, because the read loop blocks on
    a pipe the grandchild still holds open. Measured before the fix: a stop at 0.5 s had
    no effect at all until the grandchild finished on its own six seconds later.
    """
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    finished = threading.Event()
    job = EmbeddingJob(on_finished=lambda j: finished.set())

    # A launcher that forks and waits, exactly like `nix run`: the sleep is the work.
    assert job.start(["sh", "-c", "( sleep 30; echo never ) & wait $!"], "", adopt=lambda: None)
    for _ in range(500):  # let the grandchild actually exist before stopping it
        if job.status()["embedding"] == "running":
            break
        time.sleep(0.01)
    time.sleep(0.3)

    started = time.monotonic()
    job.terminate()
    assert finished.wait(10), "terminate() did not stop the job"
    took = time.monotonic() - started

    assert took < 5.0, f"the stop waited {took:.1f}s for work it was supposed to end"
    assert job.status()["embedding"] != "running", "the job still reports itself running"


def test_a_finished_job_does_not_kill_its_successor() -> None:
    """The group id has to belong to the run that is cleaning up, not to whichever run
    the instance field currently names.

    `_run` publishes "done" before it reaches its cleanup, so a second job can start in
    that window -- and the cleanup read `self._pgid`, which by then was B's. A's finally
    then signalled B's process group: B died with -15 and never ran its adoption
    callback, so the embeddings it had just written were never picked up.

    The same ownership mistake `_run_id` was added for, one field along. `process` was
    already held as a local for exactly this reason.
    """
    import threading

    from dimos.teleop.memory_world.embed import EmbeddingJob

    b_running = threading.Event()
    b_adopted = threading.Event()
    outcomes: list[str] = []

    job = EmbeddingJob(on_finished=lambda j: outcomes.append(j.state))
    real_set = job._set

    def hand_over(state: str, progress: str) -> None:
        real_set(state, progress)
        if state == "done" and not outcomes:
            # A has published "done" and has not reached its cleanup: B's window. A is
            # HELD here until B's subprocess actually exists, so A always walks into its
            # cleanup with B's group live -- otherwise the field still holds A's own id
            # and the bug cannot show.
            a_pgid = job._pgid
            job._set = real_set  # type: ignore[method-assign]
            job.start(["sh", "-c", "sleep 1"], "", adopt=lambda: b_adopted.set())
            for _ in range(500):
                if job._pgid is not None and job._pgid != a_pgid:
                    b_running.set()
                    return
                time.sleep(0.01)
            raise AssertionError("B never spawned")

    job._set = hand_over  # type: ignore[method-assign]
    assert job.start(["true"], "", adopt=lambda: None)

    # B runs `sleep 1` and then adopts. If A's cleanup signalled B's group, B dies with
    # -15 instead and its adoption never happens.
    assert b_running.wait(10), "B never spawned"
    assert b_adopted.wait(10), (
        f"the successor was killed by the finished job's cleanup; outcomes: {outcomes}"
    )
    assert job.status()["embedding"] != "failed", job.status()


def test_a_frame_published_onto_itself_does_not_hide_the_root() -> None:
    """`tf_root` is `parents - children`, and a self-edge puts its frame in BOTH sets.

    One such edge -- a duplicate or misconfigured broadcaster republishing a frame onto
    itself, which is a real ROS shape -- subtracts the true root out of the set, so
    `tf_root` returns None for a tree that has exactly one. `name_streams` then leaves
    `world_frame` at its default, the tree does not have that frame, and every lookup
    afterwards returns None: a recording whose tf is otherwise perfectly usable places
    nothing at all.
    """
    from dimos.teleop.memory_world.recording import tf_root
    from dimos.teleop.memory_world.tf_tree import TfTree

    clean = TfTree()
    clean.add("map", "odom", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    clean.add("odom", "base_link", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert tf_root(clean) == "map"

    with_self_edge = TfTree()
    with_self_edge.add("map", "map", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with_self_edge.add("map", "odom", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with_self_edge.add("odom", "base_link", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert tf_root(with_self_edge) == "map", "a frame published onto itself hid the root"

    # Genuinely several roots is still None, which is what the None means.
    forest = TfTree()
    forest.add("a", "b", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    forest.add("c", "d", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert tf_root(forest) is None


def test_a_realsense_with_aligned_depth_has_a_depth_stream(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """`realsense2_camera align_depth.enable:=true` names its depth stream
    `camera_aligned_depth_to_color_image_raw` -- which contains "color".

    `pick("depth", ..., depth_like=True)` has already restricted the candidates to
    depth-named streams, so the colour disqualifiers on that role could only SUBTRACT a
    legitimate one. A rig with aligned depth was detected as having no depth at all, and
    the ingest refused with "has no depth stream; cannot ingest" a recording that plainly
    has one. Aligned depth is the setting this package's whole depth path wants.
    """
    import numpy as np

    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.teleop.memory_world.recording import detect_streams

    store = SqliteStore(path=str(tmp_path / "realsense.db"), must_exist=False)
    store.start()
    try:
        store.stream("camera_color_image_raw", Image).append(
            Image(np.zeros((8, 8, 3), np.uint8)), ts=1.0
        )
        store.stream("camera_aligned_depth_to_color_image_raw", Image).append(
            Image(np.zeros((8, 8), np.uint16)), ts=1.0
        )
        found = detect_streams(store)
        assert found["image"] == "camera_color_image_raw"
        assert found["depth"] == "camera_aligned_depth_to_color_image_raw"
    finally:
        store.stop()

    # And an infrared stream is still not depth, which is what the remaining word is for.
    store = SqliteStore(path=str(tmp_path / "infra.db"), must_exist=False)
    store.start()
    try:
        store.stream("camera_color_image_raw", Image).append(
            Image(np.zeros((8, 8, 3), np.uint8)), ts=1.0
        )
        store.stream("camera_infra1_image_rect_raw", Image).append(
            Image(np.zeros((8, 8), np.uint16)), ts=1.0
        )
        assert detect_streams(store)["depth"] is None
    finally:
        store.stop()


def test_a_colourised_depth_image_is_not_the_depth_stream(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """Named like depth is not the same as being depth.

    A `colorizer` node publishes depth as RGB for a person to look at, under a name like
    `depth_color` -- which is shorter than `camera_aligned_depth_to_color_image_raw`, and
    length is how the ranking breaks a tie between two equally well named candidates. The
    RGB one won, and `patch_world_position` raises "too many values to unpack" on it,
    because a metre is stored in ONE channel.

    The count breaks the tie rather than disqualifying: a stream says how many channels it
    has only as clearly as its codec lets it, and a db whose images went through the
    default jpeg codec hands back three channels of RGB whatever went in. Depth in a real
    recording is stored losslessly -- the grocery recording's `depth_image` is
    `lz4+lcm`, and reads back (720, 1280) uint16 DEPTH16 -- which is what this fixture
    does.
    """
    import numpy as np

    from dimos.memory.codecs.lcm import LcmCodec
    from dimos.memory.codecs.lz4 import Lz4Codec
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    from dimos.teleop.memory_world.recording import detect_streams

    store = SqliteStore(path=str(tmp_path / "colorized.db"), must_exist=False)
    store.start()
    try:
        store.stream("camera_color_image_raw", Image).append(
            Image(np.zeros((8, 8, 3), np.uint8)), ts=1.0
        )
        store.stream(
            "camera_aligned_depth_to_color_image_raw", Image, codec=Lz4Codec(LcmCodec(Image))
        ).append(Image(np.full((8, 8), 1000, np.uint16), format=ImageFormat.DEPTH16), ts=1.0)
        store.stream("depth_color", Image, codec=Lz4Codec(LcmCodec(Image))).append(
            Image(np.zeros((8, 8, 3), np.uint8), format=ImageFormat.RGB), ts=1.0
        )

        found = detect_streams(store)
        assert found["depth"] == "camera_aligned_depth_to_color_image_raw", (
            f"a three-channel image was picked as depth: {found['depth']}"
        )
    finally:
        store.stop()


def test_a_depth_stream_that_will_not_decode_is_the_last_resort(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """ "Cannot tell" is not "no objection".

    The channel count breaks the depth tie, and a candidate whose sample will not decode
    has no count -- so it was treated as unobjectionable and won on its shorter name. The
    module cannot read a metre out of it by any route: a `depth` stream with a truncated
    blob, which is what a killed writer leaves, beat a `camera_depth_image` beside it that
    reads as real DEPTH16.
    """
    import sqlite3

    import numpy as np

    from dimos.memory.codecs.lcm import LcmCodec
    from dimos.memory.codecs.lz4 import Lz4Codec
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    from dimos.teleop.memory_world.recording import detect_streams

    path = str(tmp_path / "truncated.db")
    store = SqliteStore(path=path, must_exist=False)
    store.start()
    try:
        store.stream("camera_color_image_raw", Image).append(
            Image(np.zeros((8, 8, 3), np.uint8)), ts=1.0
        )
        for name in ("depth", "camera_depth_image"):
            store.stream(name, Image, codec=Lz4Codec(LcmCodec(Image))).append(
                Image(np.full((8, 8), 1000, np.uint16), format=ImageFormat.DEPTH16), ts=1.0
            )
    finally:
        store.stop()

    connection = sqlite3.connect(path)
    blobs = [
        row[0]
        for row in connection.execute(
            "select name from sqlite_master where type='table' and name like 'depth%blob%'"
        )
    ]
    assert blobs, "the fixture stored no blob to truncate"
    for table in blobs:
        connection.execute(f"update {table} set data = ?", (b"broken lz4",))
    connection.commit()
    connection.close()

    # ONE bad frame is not a bad stream. A writer killed mid-frame leaves a blob that will
    # not decode and the rest intact, and judging the stream on its first sample alone put
    # the colourised image ahead of nineteen good frames of real depth.
    deep = str(tmp_path / "one_bad_frame.db")
    store = SqliteStore(path=deep, must_exist=False)
    store.start()
    try:
        store.stream("camera_color_image_raw", Image).append(
            Image(np.zeros((8, 8, 3), np.uint8)), ts=0.0
        )
        for i in range(20):
            store.stream("depth_image", Image, codec=Lz4Codec(LcmCodec(Image))).append(
                Image(np.full((8, 8), 1000, np.uint16), format=ImageFormat.DEPTH16), ts=float(i)
            )
            store.stream("depth_color", Image, codec=Lz4Codec(LcmCodec(Image))).append(
                Image(np.zeros((8, 8, 3), np.uint8), format=ImageFormat.RGB), ts=float(i)
            )
    finally:
        store.stop()

    connection = sqlite3.connect(deep)
    table = next(
        row[0]
        for row in connection.execute(
            "select name from sqlite_master where type='table' and name like 'depth_image%blob%'"
        )
    )
    rowid = next(connection.execute(f"select rowid from {table} order by rowid limit 1"))[0]
    connection.execute(f"update {table} set data = ? where rowid = ?", (b"broken lz4", rowid))
    connection.commit()
    connection.close()

    store = SqliteStore(path=deep, must_exist=True)
    store.start()
    try:
        assert detect_streams(store)["depth"] == "depth_image", (
            "one undecodable frame threw away nineteen good ones"
        )
    finally:
        store.stop()

    store = SqliteStore(path=path, must_exist=True)
    store.start()
    try:
        assert detect_streams(store)["depth"] == "camera_depth_image", (
            "a stream whose sample will not decode was picked as depth"
        )
    finally:
        store.stop()


def test_an_empty_original_beside_a_staged_copy_is_a_dead_rebuild(tmp_path) -> None:  # type: ignore[no-untyped-def]
    """`rebuild_stream` drops the original and then writes it back, so a rebuild dying
    INSIDE that window leaves the name present and holding nothing.

    The guard asked only whether the name existed, so it let that through -- and the
    empty original is then dropped from the ranking as having no payload type, leaving
    the staged copy the only candidate of its type. The module read a half-written
    rebuild as the recording's own tf. An empty original counts as not there, and a
    staged copy is never a candidate for the recording's own stream in the first place.
    """
    import pytest

    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import STAGED_SUFFIX, detect_streams

    store = SqliteStore(path=str(tmp_path / "dead.db"), must_exist=False)
    store.start()
    try:
        staged = store.stream(f"tf{STAGED_SUFFIX}", TFMessage)
        for step in range(3):
            staged.append(TFMessage(_edge("odom", "base", float(step))), ts=float(step))
        store.stream("tf", TFMessage)  # present, and holding nothing

        with pytest.raises(SystemExit, match="a rebuild died"):
            detect_streams(store)
    finally:
        store.stop()

    # A live original beside a staged copy is fine, and the original is what is picked.
    store = SqliteStore(path=str(tmp_path / "live.db"), must_exist=False)
    store.start()
    try:
        for name in ("tf", f"tf{STAGED_SUFFIX}"):
            stream = store.stream(name, TFMessage)
            stream.append(TFMessage(_edge("odom", "base", 1.0)), ts=1.0)
        assert detect_streams(store)["tf"] == "tf"
    finally:
        store.stop()
