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

"""End-to-end coverage from live collection through host-side DataPrep."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
from typing import Any, cast
import uuid

import h5py
import numpy as np
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.collection.recorder import collection_recorder
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.build import inspect_dataset, run_dataprep
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    EpisodeExtractor,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
    SyncConfig,
    extract_episodes,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.imitation_msgs.EpisodeStatus import (
    EpisodeEvent,
    EpisodeStatus,
    RecordingState,
)
from dimos.msgs.protocol import DimosMsg
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic
from dimos.utils.testing.waiting import wait_until

pytestmark = [
    pytest.mark.native_e2e,
    pytest.mark.skipif_macos,
    pytest.mark.skipif_aarch64,
    pytest.mark.skipif_no_turbojpeg,
]


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
            "camera": FeatureSpec(
                stream="color_image",
                field="data",
                dtype="video",
                shape=(16, 16, 3),
                names=["height", "width", "channels"],
            ),
            "state": FeatureSpec(
                stream="coordinator_joint_state",
                field="position",
                dtype="float32",
                shape=(2,),
                names=["joint_0", "joint_1"],
            ),
        },
        action={
            "action": FeatureSpec(
                stream="coordinator_joint_state",
                field="position",
                dtype="float32",
                shape=(2,),
                names=["joint_0", "joint_1"],
            ),
        },
        sync=SyncConfig(anchor="camera", rate_hz=1.0, tolerance_ms=1.0),
        quality=QualityConfig(max_camera_gap_ms=1100.0),
        output=output,
    )


def _record_session(db_path: Path, executable: Path) -> dict[str, int]:
    config = _dataprep_config(db_path, OutputConfig(path=db_path.parent / "unused"))
    profile = CollectionProfile(
        name="synthetic",
        robot_type="synthetic",
        observations={
            name: CollectionFeature(
                **feature.model_dump(),
                message_type=Image if name == "camera" else JointState,
            )
            for name, feature in config.observation.items()
        },
        actions={
            name: CollectionFeature(**feature.model_dump(), message_type=JointState)
            for name, feature in config.action.items()
        },
        sync=config.sync,
        quality=config.quality,
    )
    atom = collection_recorder(
        profile=profile, recording=db_path.parent, format="sqlite"
    ).active_blueprints[0]
    recorder = atom.module(**atom.kwargs, executable=str(executable))
    topic_prefix = f"dimos/test/collection-export/{uuid.uuid4().hex}"
    payload_types = {
        "color_image": Image,
        "coordinator_joint_state": JointState,
        "status": EpisodeStatus,
    }
    transports = {
        name: ZenohTransport(
            Topic(f"{topic_prefix}/{name}", cast("type[DimosMsg]", kind), qos=QOS_NEVER_DROP)
        )
        for name, kind in payload_types.items()
    }
    for name, transport in transports.items():
        getattr(recorder, name).transport = transport
    counts = {name: 0 for name in transports}

    def stream_count(name: str) -> int:
        with SqliteStore(path=str(db_path), must_exist=True) as store:
            return store.stream(name).count()

    def publish(name: str, message: Any) -> None:
        counts[name] += 1
        transports[name].broadcast(None, message)
        wait_until(
            lambda: stream_count(name) == counts[name],
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
        ready = _status(1.0, "init", "idle", 0, 0, "")
        wait_until(
            lambda: stream_count("status") > 0
            or (transports["status"].broadcast(None, ready), False)[1],
            timeout=10.0,
            interval=0.1,
            message="native collection status subscription did not become ready",
        )
        counts["status"] = stream_count("status")
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
        for transport in transports.values():
            transport.stop()
    return counts


EXPECTED_STATE = np.asarray(
    [
        [0.0, 100.0],
        [1.0, 101.0],
        [2.0, 102.0],
        [20.0, 120.0],
        [21.0, 121.0],
        [22.0, 122.0],
    ],
    dtype=np.float32,
)
EXPECTED_ACTION = EXPECTED_STATE.copy()


@pytest.fixture(scope="module")
def recorded_session(
    tmp_path_factory: pytest.TempPathFactory,
) -> tuple[Path, dict[float, np.ndarray[Any, Any]]]:
    subprocess.run(
        ["cargo", "build", "--locked", "-p", "dimos-memory-recorder"],
        cwd=DIMOS_PROJECT_ROOT,
        check=True,
    )
    executable = DIMOS_PROJECT_ROOT / "target" / "debug" / "dimos-memory-recorder"
    db_path = tmp_path_factory.mktemp("recorded-session") / "session" / "recording.db"
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(global_config, "transport", "zenoh")
        counts = _record_session(db_path, executable)
    with SqliteStore(path=str(db_path), must_exist=True) as store:
        assert store.stream("color_image").count() == 9
        assert store.stream("coordinator_joint_state").count() == 9
        assert store.stream("status").count() == counts["status"]
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
        RecordingSchema.read(db_path.parent).dataprep_config(
            db_path.parent,
            OutputConfig(
                format="hdf5",
                path=tmp_path / "dataset.hdf5",
                metadata={"robot": "synthetic"},
            ),
        )
    )
    hdf5_info = inspect_dataset(hdf5_path)
    assert (hdf5_info["episodes"], hdf5_info["frames"], hdf5_info["fps"]) == (2, 6, 1.0)
    assert hdf5_info["episode_lengths"] == {
        "min": 3,
        "max": 3,
        "mean": 3.0,
        "uniform": True,
    }

    with h5py.File(hdf5_path, "r") as h5:
        first = h5["episodes/episode_000000"]
        second = h5["episodes/episode_000001"]
        assert [first.attrs["start_ts"], second.attrs["start_ts"]] == [100.0, 108.0]
        np.testing.assert_array_equal(first["timestamp"][:], [0.0, 1.0, 2.0])
        np.testing.assert_array_equal(second["timestamp"][:], [0.0, 1.0, 2.0])
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
            np.stack([recorded_images[100.0], recorded_images[101.0], recorded_images[102.0]]),
        )
        np.testing.assert_array_equal(
            second["observation/camera"][:],
            np.stack([recorded_images[108.0], recorded_images[109.0], recorded_images[110.0]]),
        )

    hdf5_meta = json.loads((tmp_path / "dataset.dimos_meta.json").read_text())
    assert [
        (episode["start_ts"], episode["end_ts"], episode["task_label"])
        for episode in hdf5_meta["episodes"]
    ] == [(100.0, 102.0, "pick"), (108.0, 110.0, "place")]
