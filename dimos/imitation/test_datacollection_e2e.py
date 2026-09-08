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

"""End-to-end coverage from live collection through both dataset formats."""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np
import pyarrow.parquet as pq
import pytest

from dimos.core.stream import Stream, Transport
from dimos.imitation.collection.episode_monitor import (
    EpisodeEvent,
    EpisodeStatus,
    RecordingState,
)
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.imitation.dataprep.build import inspect_dataset, run_dataprep
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    EpisodeExtractor,
    OutputConfig,
    StreamField,
    SyncConfig,
    extract_episodes,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.testing.waiting import wait_until

pytestmark = [
    pytest.mark.skipif_macos,
    pytest.mark.skipif_aarch64,
    pytest.mark.skipif_no_turbojpeg,
]


class _DirectTransport(Transport[Any]):
    """Synchronous in-process transport used to exercise real port subscriptions."""

    def __init__(self) -> None:
        self._subscribers: list[Callable[[Any], Any]] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._subscribers.clear()

    def broadcast(self, selfstream: Stream[Any] | None, value: Any) -> None:
        for callback in tuple(self._subscribers):
            callback(value)

    def subscribe(
        self,
        callback: Callable[[Any], Any],
        selfstream: Stream[Any] | None = None,
    ) -> Callable[[], None]:
        self._subscribers.append(callback)

        def unsubscribe() -> None:
            self._subscribers.remove(callback)

        return unsubscribe


def _status(
    ts: float,
    event: EpisodeEvent,
    state: RecordingState,
    saved: int,
    discarded: int,
    task: str,
) -> EpisodeStatus:
    return EpisodeStatus(
        ts=ts,
        last_event=event,
        state=state,
        episodes_saved=saved,
        episodes_discarded=discarded,
        task_label=task,
    )


def _dataprep_config(db_path: Path, output: OutputConfig) -> DataPrepConfig:
    return DataPrepConfig(
        source=str(db_path),
        episodes=EpisodeExtractor(status_stream="status"),
        observation={
            "camera": StreamField(stream="color_image"),
            "state": StreamField(stream="coordinator_joint_state", field="position"),
        },
        action={
            "action": StreamField(stream="coordinator_joint_state", field="position"),
        },
        sync=SyncConfig(anchor="camera", rate_hz=1.0, tolerance_ms=1.0, action_shift=1),
        output=output,
    )


def _record_session(db_path: Path) -> None:
    recorder = CollectionRecorder(
        db_path=db_path,
        record_tf=False,
        poseless_streams=["color_image", "coordinator_joint_state", "status"],
    )
    transports = {
        "color_image": _DirectTransport(),
        "coordinator_joint_state": _DirectTransport(),
        "status": _DirectTransport(),
    }
    for name, transport in transports.items():
        getattr(recorder, name).transport = transport
    counts = {name: 0 for name in transports}

    def publish(name: str, message: Any) -> None:
        counts[name] += 1
        transports[name].publish(message)
        wait_until(
            lambda: recorder.store.stream(name).count() == counts[name],
            timeout=5.0,
            interval=0.005,
            message=f"{name} message {counts[name]} was not recorded",
        )

    episodes = [
        (100.0, "pick", True, 0.0),
        (104.0, "discard-me", False, 10.0),
        (108.0, "place", True, 20.0),
    ]
    try:
        recorder.start()
        saved = 0
        discarded = 0
        for start_ts, task, success, base in episodes:
            publish(
                "status",
                _status(start_ts, "start", "recording", saved, discarded, task),
            )
            for frame in range(3):
                ts = start_ts + frame
                pixel = int(base + frame) * 4
                publish(
                    "color_image",
                    Image(
                        data=np.full((16, 16, 3), pixel, dtype=np.uint8),
                        format=ImageFormat.RGB,
                        frame_id="camera",
                        ts=ts,
                    ),
                )
                publish(
                    "coordinator_joint_state",
                    JointState(
                        ts=ts,
                        frame_id="arm",
                        name=["joint_0", "joint_1"],
                        position=[base + frame, base + 100.0 + frame],
                        velocity=[0.0, 0.0],
                        effort=[0.0, 0.0],
                    ),
                )
            if success:
                saved += 1
                event: EpisodeEvent = "save"
            else:
                discarded += 1
                event = "discard"
            publish(
                "status",
                _status(start_ts + 2.0, event, "idle", saved, discarded, task),
            )
        publish("status", _status(112.0, "start", "recording", 2, 1, "interrupted"))
    finally:
        recorder.stop()


def _read_video(path: Path) -> list[np.ndarray[Any, Any]]:
    capture = cv2.VideoCapture(str(path))
    frames: list[np.ndarray[Any, Any]] = []
    try:
        while True:
            ok, bgr = capture.read()
            if not ok:
                return frames
            frames.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    finally:
        capture.release()


EXPECTED_STATE = np.asarray(
    [[0.0, 100.0], [1.0, 101.0], [20.0, 120.0], [21.0, 121.0]],
    dtype=np.float32,
)
EXPECTED_ACTION = np.asarray(
    [[1.0, 101.0], [2.0, 102.0], [21.0, 121.0], [22.0, 122.0]],
    dtype=np.float32,
)


@pytest.fixture(scope="module")
def recorded_session(
    tmp_path_factory: pytest.TempPathFactory,
) -> tuple[Path, dict[float, np.ndarray[Any, Any]]]:
    db_path = tmp_path_factory.mktemp("recorded-session") / "recording.db"
    _record_session(db_path)
    with SqliteStore(path=str(db_path), must_exist=True) as store:
        assert store.stream("color_image").count() == 9
        assert store.stream("coordinator_joint_state").count() == 9
        assert store.stream("status").count() == 7
        episodes = extract_episodes(store, EpisodeExtractor(status_stream="status"))
        assert [
            (episode.start_ts, episode.end_ts, episode.success, episode.task_label)
            for episode in episodes
        ] == [
            (100.0, 102.0, True, "pick"),
            (104.0, 106.0, False, "discard-me"),
            (108.0, 110.0, True, "place"),
        ]
        recorded_images: dict[float, np.ndarray[Any, Any]] = {
            observation.ts: observation.data.data
            for observation in store.stream("color_image", Image).to_list()
        }

    recording_info = inspect_dataset(db_path)
    assert recording_info["episodes"] == 3
    assert recording_info["saved_episodes"] == 2
    assert recording_info["discarded_episodes"] == 1
    assert recording_info["incomplete_episodes"] == [
        {"start_ts": 112.0, "task_label": "interrupted"}
    ]
    return db_path, recorded_images


def test_collection_to_hdf5_roundtrip(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    db_path, recorded_images = recorded_session

    hdf5_path = run_dataprep(
        _dataprep_config(
            db_path,
            OutputConfig(
                format="hdf5",
                path=tmp_path / "dataset.hdf5",
                metadata={"robot": "synthetic"},
            ),
        )
    )
    hdf5_info = inspect_dataset(hdf5_path)
    assert (hdf5_info["episodes"], hdf5_info["frames"], hdf5_info["fps"]) == (2, 4, 1.0)
    assert hdf5_info["episode_lengths"] == {
        "min": 2,
        "max": 2,
        "mean": 2.0,
        "uniform": True,
    }

    with h5py.File(hdf5_path, "r") as h5:
        first = h5["episodes/episode_000000"]
        second = h5["episodes/episode_000001"]
        assert [first.attrs["start_ts"], second.attrs["start_ts"]] == [100.0, 108.0]
        np.testing.assert_array_equal(first["timestamp"][:], [0.0, 1.0])
        np.testing.assert_array_equal(second["timestamp"][:], [0.0, 1.0])
        np.testing.assert_array_equal(
            np.concatenate([first["observation/state"][:], second["observation/state"][:]]),
            EXPECTED_STATE,
        )
        np.testing.assert_array_equal(
            np.concatenate([first["action/action"][:], second["action/action"][:]]),
            EXPECTED_ACTION,
        )
        np.testing.assert_array_equal(
            first["observation/camera"][:],
            np.stack([recorded_images[100.0], recorded_images[101.0]]),
        )
        np.testing.assert_array_equal(
            second["observation/camera"][:],
            np.stack([recorded_images[108.0], recorded_images[109.0]]),
        )

    hdf5_meta = json.loads((tmp_path / "dataset.dimos_meta.json").read_text())
    assert [
        (episode["start_ts"], episode["end_ts"], episode["task_label"])
        for episode in hdf5_meta["episodes"]
    ] == [(100.0, 102.0, "pick"), (108.0, 110.0, "place")]


def test_collection_to_lerobot_roundtrip(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    db_path, recorded_images = recorded_session
    try:
        lerobot_path = run_dataprep(
            _dataprep_config(
                db_path,
                OutputConfig(
                    format="lerobot",
                    path=tmp_path / "lerobot",
                    metadata={"robot": "synthetic"},
                ),
            )
        )
    except RuntimeError as exc:
        if "VideoWriter" in str(exc):
            pytest.skip(f"no mp4v encoder available in this environment: {exc}")
        raise

    lerobot_info = inspect_dataset(lerobot_path)
    assert (lerobot_info["episodes"], lerobot_info["frames"], lerobot_info["fps"]) == (2, 4, 1.0)
    data = pq.read_table(lerobot_path / "data/chunk-000/file-000.parquet")
    assert data.column("timestamp").to_pylist() == pytest.approx([0.0, 1.0, 0.0, 1.0])
    assert data.column("episode_index").to_pylist() == [0, 0, 1, 1]
    assert data.column("frame_index").to_pylist() == [0, 1, 0, 1]
    np.testing.assert_array_equal(
        np.asarray(data.column("observation.state").to_pylist()), EXPECTED_STATE
    )
    np.testing.assert_array_equal(np.asarray(data.column("action").to_pylist()), EXPECTED_ACTION)

    episode_rows = pq.read_table(
        lerobot_path / "meta/episodes/chunk-000/file-000.parquet"
    ).to_pylist()
    assert [row["length"] for row in episode_rows] == [2, 2]
    assert [(row["dataset_from_index"], row["dataset_to_index"]) for row in episode_rows] == [
        (0, 2),
        (2, 4),
    ]
    assert [row["tasks"] for row in episode_rows] == [["pick"], ["place"]]

    video = _read_video(lerobot_path / "videos/observation.images.camera/chunk-000/file-000.mp4")
    assert len(video) == 4
    expected_means = [recorded_images[ts].mean() for ts in (100.0, 101.0, 108.0, 109.0)]
    np.testing.assert_allclose([frame.mean() for frame in video], expected_means, atol=5.0)
