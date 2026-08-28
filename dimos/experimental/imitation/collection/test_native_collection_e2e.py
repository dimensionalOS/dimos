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

"""Native OpenYAM collection interoperability over the production transport."""

from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Any, cast
import uuid

import numpy as np
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.experimental.imitation.collection.recorder import NativeCollectionRecorder
from dimos.experimental.memory.rust_recorder import RustSqliteStoreConfig
from dimos.imitation.dataprep.core import EpisodeExtractor, extract_episodes
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.protocol import DimosMsg
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic
from dimos.utils.testing.waiting import wait_until

pytestmark = [
    pytest.mark.self_hosted_large,
    pytest.mark.skipif_macos,
    pytest.mark.skipif_aarch64,
    pytest.mark.skipif_no_turbojpeg,
]

_RUST_WORKSPACE = DIMOS_PROJECT_ROOT / "native" / "rust"
_EXECUTABLE = _RUST_WORKSPACE / "target" / "debug" / "dimos-memory-recorder"


@pytest.fixture(scope="module")
def native_recorder_executable() -> Path:
    subprocess.run(
        ["cargo", "build", "-p", "dimos-memory-recorder"],
        cwd=_RUST_WORKSPACE,
        check=True,
    )
    return _EXECUTABLE


def _stream_count(path: Path, name: str) -> int:
    with SqliteStore(path=str(path), must_exist=True) as store:
        return store.stream(name).count()


def test_native_collection_records_typed_zenoh_streams(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    native_recorder_executable: Path,
) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")
    artifact = tmp_path / "native-openyam.db"
    recorder = NativeCollectionRecorder(
        executable=str(native_recorder_executable),
        store=RustSqliteStoreConfig(path=str(artifact)),
    )
    topic_prefix = f"dimos/test/native-collection/{uuid.uuid4().hex}"
    payload_types: dict[str, type[Any]] = {
        "color_image": Image,
        "coordinator_joint_state": JointState,
        "status": EpisodeStatus,
        "tf": TFMessage,
    }
    publishers: dict[str, ZenohTransport[Any]] = {}
    for name, payload_type in payload_types.items():
        transport = ZenohTransport(
            Topic(
                f"{topic_prefix}/{name}",
                cast("type[DimosMsg]", payload_type),
                qos=QOS_NEVER_DROP,
            )
        )
        getattr(recorder, name).transport = transport
        publishers[name] = transport

    def publish(name: str, message: Any) -> None:
        publishers[name].broadcast(None, message)

    try:
        recorder.start()
        ready = EpisodeStatus(
            ts=1.0,
            state="idle",
            last_event="init",
            episodes_saved=0,
            episodes_discarded=0,
        )
        wait_until(
            lambda: (publish("status", ready), _stream_count(artifact, "status") > 0)[1],
            timeout=10.0,
            interval=0.1,
            message="native recorder did not receive the Zenoh status stream",
        )

        publish(
            "status",
            EpisodeStatus(
                ts=10.0,
                state="recording",
                last_event="start",
                episodes_saved=0,
                episodes_discarded=0,
                task_label="pick",
            ),
        )
        publish(
            "color_image",
            Image(
                ts=10.0,
                frame_id="wrist_camera_link",
                format=ImageFormat.RGB,
                data=np.full((16, 16, 3), 127, dtype=np.uint8),
            ),
        )
        publish(
            "coordinator_joint_state",
            JointState(
                ts=10.0,
                frame_id="coordinator",
                name=["shoulder", "wrist"],
                position=[0.25, -0.5],
                velocity=[0.0, 0.0],
                effort=[0.0, 0.0],
            ),
        )
        publish(
            "status",
            EpisodeStatus(
                ts=11.0,
                state="idle",
                last_event="save",
                episodes_saved=1,
                episodes_discarded=0,
                task_label="pick",
            ),
        )
        wait_until(
            lambda: all(
                _stream_count(artifact, name) >= count
                for name, count in {
                    "color_image": 1,
                    "coordinator_joint_state": 1,
                    "status": 3,
                }.items()
            ),
            timeout=10.0,
            interval=0.05,
            message="native recorder did not persist every collection stream",
        )
    finally:
        recorder.stop()
        for publisher in publishers.values():
            publisher.stop()

    with SqliteStore(path=str(artifact), must_exist=True) as store:
        image_observation = store.stream("color_image", Image).last()
        image = image_observation.data
        image_ts = image_observation.ts
        joint_observation = store.stream("coordinator_joint_state", JointState).last()
        joints = joint_observation.data
        joint_ts = joint_observation.ts
        statuses = [
            (observation.ts, observation.data.last_event)
            for observation in store.stream("status", EpisodeStatus).to_list()
        ]
        episodes = extract_episodes(store, EpisodeExtractor(status_stream="status"))

    assert image_ts == 10.0
    assert image.frame_id == "wrist_camera_link"
    assert image.data.shape == (16, 16, 3)
    assert joint_ts == 10.0
    assert joints.position == [0.25, -0.5]
    assert statuses[-2:] == [
        (10.0, "start"),
        (11.0, "save"),
    ]
    assert [(episode.start_ts, episode.end_ts, episode.task_label) for episode in episodes] == [
        (10.0, 11.0, "pick")
    ]
