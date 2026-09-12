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
import sqlite3
from unittest import mock

import numpy as np
import pytest

from dimos.teleop.memory_world.hyperspace_search import (
    KEYFRAME_STREAM,
    PATCH_STREAM,
    Cluster,
    assign_points,
    cluster_voxels,
    memory_db_for,
    memory_db_ready,
)


def _blob(origin: tuple[int, int, int], size: int) -> list[tuple[int, int, int]]:
    ox, oy, oz = origin
    return [(ox + i, oy + j, oz + k) for i in range(size) for j in range(size) for k in range(size)]


def test_only_one_ingest_at_a_time_holds_the_claim(tmp_path: Path) -> None:
    """The claim is an OS lock, so a run killed outright (the module stops an ingest
    with SIGTERM) releases it, and a leftover lock file never blocks the next run."""
    import os

    from dimos.teleop.memory_world.hyperspace_ingest import _claim

    lock = tmp_path / "rec.hyperspace.db.building.lock"
    recording = tmp_path / "rec.db"
    held = _claim(lock, recording)
    assert lock.read_text().strip() == str(os.getpid())  # ours, and it says who
    with pytest.raises(SystemExit, match="another ingest"):
        _claim(lock, recording)  # a second run is turned away while we hold it

    os.close(held)  # what a dying process gets for free from the kernel
    assert lock.exists()  # the file stays: a fresh inode would not be locked
    os.close(_claim(lock, recording))


def test_ingest_command_names_only_the_streams_the_module_chose() -> None:
    from dimos.teleop.memory_world.hyperspace_ingest import ingest_command

    command = ingest_command(
        "/tmp/rec.mcap",
        model_name="m",
        device="cpu",
        hz=2.0,
        streams={"image": "left_image", "depth": None, "camera_info": "", "tf": "tf"},
    )
    # A role the module left unset is a role the ingest detects for itself; naming it
    # empty on the command line would be an ingest of a stream that does not exist.
    assert command[-2:] == ["--image=left_image", "--tf=tf"]
    assert "--hz" in command and command[command.index("--hz") + 1] == "2.0"
    detected = ingest_command("/tmp/rec.mcap", model_name="m", device="cpu", hz=2.0)
    assert not [arg for arg in detected if arg.startswith("--image")]  # else: detect


def test_two_blobs_become_two_clusters_best_first() -> None:
    weak = _blob((0, 0, 0), 2)  # 8 voxels
    strong = _blob((40, 40, 0), 3)  # 27 voxels, far away
    indices = np.asarray(weak + strong)
    scores = np.asarray([0.4] * len(weak) + [0.9] * len(strong))

    clusters, labels = cluster_voxels(indices, scores, voxel_size=0.1)

    assert [c.n_voxels for c in clusters] == [27, 8]
    assert clusters[0].index == 0 and clusters[1].index == 1
    assert np.allclose(clusters[0].centre, ((40 + 1.5) * 0.1, (40 + 1.5) * 0.1, 1.5 * 0.1))
    assert list(labels[: len(weak)]) == [1] * len(weak)
    assert list(labels[len(weak) :]) == [0] * len(strong)
    assert clusters[0].peak == 0.9 and clusters[0].score > clusters[1].score


def test_a_one_voxel_gap_does_not_split_a_cluster() -> None:
    left = _blob((0, 0, 0), 2)
    right = _blob((3, 0, 0), 2)  # x = 3,4; the gap is x = 2
    indices = np.asarray(left + right)
    clusters, labels = cluster_voxels(indices, np.ones(len(indices)), voxel_size=0.1)
    assert len(clusters) == 1
    assert set(labels.tolist()) == {0}


def test_tiny_and_faint_blobs_are_dropped() -> None:
    big = _blob((0, 0, 0), 3)
    speck = [(20, 20, 20), (20, 20, 21)]  # below MIN_CLUSTER_VOXELS
    faint = _blob((60, 0, 0), 2)  # enough voxels, negligible score
    indices = np.asarray(big + speck + faint)
    scores = np.asarray([1.0] * len(big) + [1.0] * len(speck) + [0.01] * len(faint))
    clusters, labels = cluster_voxels(indices, scores, voxel_size=0.1)
    assert len(clusters) == 1
    assert list(labels[len(big) :]) == [-1] * (len(speck) + len(faint))


def test_empty_input() -> None:
    clusters, labels = cluster_voxels(np.zeros((0, 3), int), np.zeros(0), 0.1)
    assert clusters == [] and labels.shape == (0,)


def test_assign_points_uses_radius_plus_slack() -> None:
    clusters = [
        Cluster(index=0, centre=(0.0, 0.0, 0.0), radius=0.5, score=1, peak=1, n_voxels=9),
        Cluster(index=1, centre=(10.0, 0.0, 0.0), radius=0.5, score=1, peak=1, n_voxels=9),
    ]
    points = np.asarray([[0.3, 0, 0], [9.0, 0, 0], [5.0, 0, 0], [0, 1.2, 0]])
    assert assign_points(points, clusters, slack_m=0.75).tolist() == [0, 1, -1, 0]


def test_memory_db_ready_needs_both_streams(tmp_path) -> None:
    recording = tmp_path / "walk.mcap"
    recording.write_bytes(b"")
    assert memory_db_for(recording) == tmp_path / "walk.hyperspace.db"
    assert not memory_db_ready(recording)

    db = sqlite3.connect(memory_db_for(recording))
    db.execute("CREATE TABLE _streams (name TEXT)")
    db.execute("INSERT INTO _streams VALUES ('hyperspace_keyframes')")
    db.execute("CREATE TABLE hyperspace_keyframes (id INTEGER)")
    db.commit()
    assert not memory_db_ready(recording), "patches missing"

    db.execute("INSERT INTO _streams VALUES ('hyperspace_patches')")
    db.commit()
    assert not memory_db_ready(recording), "no keyframes yet"

    db.execute("INSERT INTO hyperspace_keyframes VALUES (1)")
    db.commit()
    db.close()
    assert memory_db_ready(recording)


def test_keyframes_in_the_recording_are_not_ready_until_the_ingest_says_so(tmp_path) -> None:
    """An mcap's companion is moved into place whole, so its existence says the ingest
    finished. Keyframes written into the recording itself appear one at a time.

    Without a marker a viewer opening during an ingest -- or after one was killed outright,
    which no ``finally`` can clean up after -- would search the handful of pictures that
    happened to be written by then and present them as the whole recording.
    """
    recording = tmp_path / "walk.db"
    assert memory_db_for(recording) == recording

    db = sqlite3.connect(recording)
    db.execute("CREATE TABLE _streams (name TEXT)")
    db.execute("INSERT INTO _streams VALUES ('hyperspace_keyframes'), ('hyperspace_patches')")
    db.execute("CREATE TABLE hyperspace_keyframes (id INTEGER)")
    db.execute("INSERT INTO hyperspace_keyframes VALUES (1)")
    db.commit()
    assert not memory_db_ready(recording), "an ingest that is still running, or was killed"

    db.execute("INSERT INTO _streams VALUES ('hyperspace_complete')")
    db.commit()
    db.close()
    assert memory_db_ready(recording)


def _tiny_recording(path, tf_name: str = "tf", with_static: bool = False):  # type: ignore[no-untyped-def]
    """A recording with just enough of every role for ingest_recording to accept it."""
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    def edge(parent: str, child: str, x: float) -> Transform:
        return Transform(
            translation=Vector3(x, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent,
            child_frame_id=child,
            ts=1.0,
        )

    store = SqliteStore(path=str(path), must_exist=False)
    store.start()
    store.stream("color_image", Image).append(
        Image(data=np.zeros((2, 2, 3), dtype=np.uint8), frame_id="cam"), ts=1.0
    )
    store.stream("depth_image", Image).append(
        Image(data=np.zeros((2, 2), dtype=np.uint16), frame_id="cam"), ts=1.0
    )
    store.stream("camera_info", CameraInfo).append(CameraInfo(frame_id="cam"), ts=1.0)
    store.stream(tf_name, TFMessage).append(TFMessage(edge("odom", "base", 1.0)), ts=1.0)
    if with_static:
        store.stream("tf_static", TFMessage).append(TFMessage(edge("base", "cam", 2.0)), ts=1.0)
    return store


def _stub_hyperspace(monkeypatch, ingest):  # type: ignore[no-untyped-def]
    """Stand in for the parts of dimos.mapping.hyperspace an ingest would load.

    The real ones pull torch and a vision tower; what is under test here is which db the
    keyframes are written into and what survives a failure, not the embedding.
    """
    import sys
    from types import SimpleNamespace

    patches = SimpleNamespace(KeyframeGateConfig=lambda **kw: None)
    monkeypatch.setitem(
        sys.modules, "dimos.mapping.hyperspace", SimpleNamespace(patches=patches, __path__=[])
    )
    monkeypatch.setitem(sys.modules, "dimos.mapping.hyperspace.patches", patches)
    monkeypatch.setitem(
        sys.modules, "dimos.mapping.hyperspace.cli", SimpleNamespace(pick_device=lambda d: "cpu")
    )
    monkeypatch.setitem(
        sys.modules,
        "dimos.mapping.hyperspace.embedder",
        SimpleNamespace(
            SigLIP2Patches=lambda **kw: SimpleNamespace(start=lambda: None, stop=lambda: None)
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "dimos.mapping.hyperspace.ingest",
        SimpleNamespace(IngestConfig=lambda **kw: None),
    )
    monkeypatch.setattr("dimos.teleop.memory_world.hyperspace_ingest._ingest", ingest)


def test_the_keyframes_are_written_into_the_recording_and_no_companion_appears(  # type: ignore[no-untyped-def]
    tmp_path, monkeypatch
) -> None:
    """One db. A .db recording gets its own keyframes, its own patches and its own marker,
    and nothing is created beside it -- which is the whole point of the change.

    It also gets one tf tree: a mount declared only in tf_static is invisible to Hyperspace,
    which reads the moving tf alone, so the statics are folded in before anything is
    embedded against them.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.std_msgs.String import String
    from dimos.teleop.memory_world.hyperspace_ingest import ingest_recording

    recording = tmp_path / "walk.db"
    _tiny_recording(recording, with_static=True).stop()

    def fake_ingest(store, memory, model, **kw):  # type: ignore[no-untyped-def]
        assert memory is store, "the recording IS the memory db"
        # The real ingest reads every stream it is named. The fold deleted tf_static, so
        # naming it here is a KeyError -- after the old index has already been dropped.
        for role, name in kw["streams"].items():
            if name:
                assert name in store.list_streams(), f"{role} names {name!r}, which is gone"
        memory.stream(KEYFRAME_STREAM, String).append(String("a keyframe"), ts=1.0)
        memory.stream(PATCH_STREAM, String).append(String("a patch"), ts=1.0)
        return {"images": 1, "kept": 1}

    _stub_hyperspace(monkeypatch, fake_ingest)
    ingest_recording(recording, model_name="stub")

    assert not (tmp_path / "walk.hyperspace.db").exists()
    assert memory_db_ready(recording)
    store = SqliteStore(path=str(recording), must_exist=True)
    store.start()
    try:
        names = set(store.list_streams())
        assert {KEYFRAME_STREAM, PATCH_STREAM, "hyperspace_complete"} <= names
        assert "tf_static" not in names  # folded into tf, so there is one tree
        edges = {
            (str(t.frame_id), str(t.child_frame_id))
            for obs in store.streams["tf"]
            for t in obs.data.transforms
        }
        assert edges == {("odom", "base"), ("base", "cam")}
    finally:
        store.stop()


def test_an_ingest_that_fails_before_embedding_keeps_the_index_that_is_there(  # type: ignore[no-untyped-def]
    tmp_path, monkeypatch
) -> None:
    """A bad model name, an unreadable device, a typo'd stream: none of them may cost the
    search index the recording already has.

    The keyframes live in the recording now, so there is no staging db to throw away -- the
    old ones are dropped in place. They are therefore dropped as late as possible, after
    everything that can fail without writing an embedding already has.
    """
    from dimos.msgs.std_msgs.String import String
    from dimos.teleop.memory_world.hyperspace_ingest import ingest_recording

    recording = tmp_path / "walk.db"
    store = _tiny_recording(recording, with_static=True)
    store.stream(KEYFRAME_STREAM, String).append(String("from the last run"), ts=1.0)
    store.stream(PATCH_STREAM, String).append(String("from the last run"), ts=1.0)
    store.stream("hyperspace_complete", String).append(String("finished"), ts=1.0)
    store.stop()
    assert memory_db_ready(recording)

    def never_called(*args, **kwargs):  # type: ignore[no-untyped-def]
        raise AssertionError("the ingest should not have started")

    _stub_hyperspace(monkeypatch, never_called)
    import sys
    from types import SimpleNamespace

    def dying_model(**kw):  # type: ignore[no-untyped-def]
        raise RuntimeError("no such model")

    monkeypatch.setitem(
        sys.modules,
        "dimos.mapping.hyperspace.embedder",
        SimpleNamespace(SigLIP2Patches=dying_model),
    )
    with pytest.raises(RuntimeError):
        ingest_recording(recording, model_name="does-not-exist")

    assert memory_db_ready(recording), "the index that was there survived"

    # And the recording's tf is untouched: folding rewrites it for good, so it waits
    # behind everything that can fail without having written an embedding.
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=str(recording), must_exist=True)
    store.start()
    try:
        assert "tf_static" in store.list_streams()
    finally:
        store.stop()


def test_the_empty_tf_hyperspace_opens_is_swept_up(tmp_path, monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Hyperspace's ingestor opens a stream called "tf" in the memory db, which is now the
    recording itself.

    On a recording whose tf is called something else, that empty stream would outrank the
    real one in detect_streams from then on and the world would have no transforms at all --
    a recording broken by indexing it.
    """
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.std_msgs.String import String
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.hyperspace_ingest import ingest_recording

    recording = tmp_path / "walk.db"
    _tiny_recording(recording, tf_name="robot_tf").stop()

    def fake_ingest(store, memory, model, **kw):  # type: ignore[no-untyped-def]
        memory.stream("tf", TFMessage)  # what PatchIngestor does on construction
        memory.stream(KEYFRAME_STREAM, String).append(String("a keyframe"), ts=1.0)
        memory.stream(PATCH_STREAM, String).append(String("a patch"), ts=1.0)
        return {"images": 1, "kept": 1}

    _stub_hyperspace(monkeypatch, fake_ingest)
    ingest_recording(recording, model_name="stub")

    store = SqliteStore(path=str(recording), must_exist=True)
    store.start()
    try:
        assert "tf" not in store.list_streams()
        assert "robot_tf" in store.list_streams()
    finally:
        store.stop()


def test_an_index_finished_after_startup_is_picked_up_by_the_status_poll() -> None:
    """An ingest run from a terminal finishes into the very file this process has open.

    Nothing tells the module, and the viewer hides its Prepare button the moment the index
    is there, so the page waits on "loading Hyperspace" for good. One db makes this the
    ordinary case rather than a corner: there is no companion file appearing to notice.
    """
    import threading
    from types import SimpleNamespace

    from dimos.teleop.memory_world.hyperspace_answers import HyperspaceAnswers

    loaded = threading.Event()
    ready = False

    class Module(HyperspaceAnswers):
        def __init__(self) -> None:
            self._hyperspace = None
            self._hyperspace_error = None
            self._adopting = threading.Lock()
            self._prepare_job = SimpleNamespace(
                status=lambda: {"embedding": "idle", "progress": 0.0}
            )
            self.config = SimpleNamespace(store_path="/nowhere/walk.db")

        def _load_hyperspace(self, reload: bool = False) -> bool:  # type: ignore[override]
            loaded.set()
            return True

    module = Module()
    with mock.patch(
        "dimos.teleop.memory_world.hyperspace_answers.memory_db_ready", lambda _p: ready
    ):
        module._adopt_an_index_that_appeared()
        assert not loaded.wait(0.2), "nothing to adopt yet"
        ready = True
        module._adopt_an_index_that_appeared()
        assert loaded.wait(2.0), "the index that appeared was never loaded"
