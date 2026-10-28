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

"""Collect synthetic profile streams through the real recorder and export for training."""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import socket
from typing import Any
import uuid

from dimos_lerobot.dataprep import write
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import dataset_to_policy_features
import numpy as np
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import CollectionRecorder, collect
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.schema import FeatureSpec, OutputConfig
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool
from dimos.utils.testing.waiting import wait_until


@pytest.fixture(scope="module")
def recorder_executable() -> Path:
    default = (
        DIMOS_PROJECT_ROOT / "dimos/experimental/memory/rust/target/debug/dimos-memory-recorder"
    )
    executable = Path(os.environ.get("DIMOS_MEMORY_RECORDER_TEST_EXECUTABLE", str(default)))
    if not executable.is_file():
        raise FileNotFoundError(
            "Build dimos/experimental/memory/rust/Cargo.toml with cargo build --locked, "
            "or set DIMOS_MEMORY_RECORDER_TEST_EXECUTABLE to the recorder binary"
        )
    return executable


@pytest.fixture
def mock_profile(camera_count: int) -> CollectionProfile:
    state = FeatureSpec(
        stream="measured",
        message_type=JointState,
        field="position",
        dtype="float32",
        shape=(2,),
        names=["right", "left"],
    )
    return CollectionProfile(
        observations={
            "joint_state": state,
            **{
                f"view_{i}": FeatureSpec(
                    stream=f"camera_{i}",
                    message_type=Image,
                    field="data",
                    dtype="video",
                    shape=(64, 64, 3),
                    names=["height", "width", "channels"],
                )
                for i in range(camera_count)
            },
        },
        actions={
            "target": state.model_copy(
                update={
                    "stream": "accepted",
                    "source_kind": "joint_position_updates",
                },
                deep=True,
            )
        },
        metadata={"name": "mock-profile", "robot_type": "synthetic"},
    )


@pytest.fixture
def recorded_profile(
    tmp_path: Path,
    mock_profile: CollectionProfile,
    recorder_executable: Path,
    monkeypatch: pytest.MonkeyPatch,
    camera_count: int,
) -> Path:
    monkeypatch.setattr(global_config, "transport", "zenoh")
    monkeypatch.setattr(global_config, "zenoh_multicast", False)
    monkeypatch.setattr(global_config, "zenoh_gossip", False)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{listener.getsockname()[1]}"
    directory = tmp_path / "recording"
    atom = collect(profile=mock_profile, recording=directory, format="sqlite").active_blueprints[0]
    types = {**mock_profile.input_types(), "status": String}
    remapping = {name: f"recorded_{name}" for name in types}
    prefix = f"dimos/test/mock-profile/{uuid.uuid4().hex}"
    artifact = directory / "recording.db"
    with ExitStack() as cleanup:
        pool = ZenohSessionPool()
        cleanup.callback(pool.close_all)
        publishers: dict[str, ZenohTransport[Any]] = {}
        for name, kind in types.items():
            publisher: ZenohTransport[Any] = ZenohTransport(
                Topic(f"{prefix}/{name}", kind, qos=QOS_NEVER_DROP),
                session_pool=pool,
                mode="client",
                connect=[endpoint],
                multicast=False,
                gossip=False,
                connect_timeout=5,
            )
            cleanup.callback(publisher.stop)
            publishers[name] = publisher
        assert atom.module is CollectionRecorder
        recorder = CollectionRecorder(
            **atom.kwargs,
            executable=str(recorder_executable),
            source_dir=None,
            build_command=None,
            stream_remapping=remapping,
            session=ZenohConfig(
                mode="peer",
                listen=[endpoint],
                connect=[],
                multicast=False,
                gossip=False,
                connect_timeout=0,
            ),
        )
        cleanup.callback(recorder.stop)
        for name, publisher in publishers.items():
            getattr(recorder, name).transport = publisher
        recorder.build()
        assert (directory / "schema.json").is_file()
        assert artifact.is_file()
        recorder.start()

        def count(name: str) -> int:
            with SqliteStore(path=str(artifact), must_exist=True) as store:
                if name == "status":
                    return sum(
                        EpisodeStatus.from_json(row.data.data).last_event != "init"
                        for row in store.stream(remapping[name], String).to_list()
                    )
                return store.stream(remapping[name], types[name]).count()

        def ready() -> bool:
            with SqliteStore(path=str(artifact), must_exist=True) as store:
                if store.stream(remapping["status"], String).count():
                    return True
            publishers["status"].broadcast(
                None,
                String(
                    EpisodeStatus(
                        ts=0,
                        state="idle",
                        last_event="init",
                        episodes_saved=0,
                        episodes_discarded=0,
                    ).to_json()
                ),
            )
            return False

        wait_until(
            ready, timeout=10, interval=0.02, message="native recorder did not receive warmup"
        )

        def emit(name: str, message: Any) -> None:
            expected = count(name) + 1
            publishers[name].broadcast(None, message)
            wait_until(
                lambda: count(name) == expected,
                timeout=5,
                interval=0.01,
                message=f"native recorder did not persist {name} message {expected}",
            )

        emit("accepted", JointState(ts=9.9, name=["left"], position=[1000]))
        emit("accepted", JointState(ts=9.9, name=["right"], position=[2000]))
        emit(
            "status",
            String(
                EpisodeStatus(
                    ts=10,
                    state="recording",
                    last_event="start",
                    task_label="mock pick",
                    episodes_saved=0,
                    episodes_discarded=0,
                ).to_json()
            ),
        )
        for frame in range(3):
            ts = 10 + frame / 30
            emit(
                "measured", JointState(ts=ts, name=["left", "right"], position=[frame, 100 + frame])
            )
            if frame == 1:
                emit("accepted", JointState(ts=ts, name=["left"], position=[1001]))
            elif frame == 2:
                emit("accepted", JointState(ts=ts, name=["right"], position=[2002]))
            for camera in range(camera_count):
                emit(
                    f"camera_{camera}",
                    Image(
                        ts=ts,
                        format=ImageFormat.RGB,
                        data=np.full((64, 64, 3), camera * 50 + frame * 10 + 20, dtype=np.uint8),
                    ),
                )
        emit(
            "status",
            String(
                EpisodeStatus(
                    ts=10.1,
                    state="idle",
                    last_event="save",
                    task_label="mock pick",
                    episodes_saved=1,
                    episodes_discarded=0,
                ).to_json()
            ),
        )
    return directory


@pytest.mark.parametrize("camera_count", [1, 2])
def test_mock_profile_collects_and_converts_to_training_dataset(
    tmp_path: Path,
    recorded_profile: Path,
    camera_count: int,
) -> None:
    moved = tmp_path / "moved-recording"
    recorded_profile.rename(moved)
    schema = RecordingSchema.read(moved)
    assert schema.observation["joint_state"].stream == "recorded_measured"
    assert schema.action["target"].stream == "recorded_accepted"
    assert schema.episodes.status_stream == "recorded_status"
    config = schema.dataprep_config(moved, OutputConfig(path=tmp_path / "dataset"))

    root = run_dataprep(config, writer=write)

    dataset = LeRobotDataset("local/mock-profile", root=root, video_backend="pyav")
    assert dataset_to_policy_features(dataset.meta.features) == {
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(2,)),
        "action": PolicyFeature(type=FeatureType.ACTION, shape=(2,)),
        **{
            f"observation.images.view_{i}": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 64, 64)
            )
            for i in range(camera_count)
        },
    }
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["observation.state"]), [[100, 0], [101, 1], [102, 2]]
    )
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["action"]), [[2000, 1000], [2000, 1001], [2002, 1001]]
    )
    assert [row["tasks"] for row in dataset.meta.episodes] == [["mock pick"]]
    assert dataset.meta.total_frames == 3
    for i in range(camera_count):
        pixels = dataset[0][f"observation.images.view_{i}"]
        assert tuple(pixels.shape) == (3, 64, 64)
        assert float(pixels.mean()) == pytest.approx((i * 50 + 20) / 255, abs=0.03)
    sidecar = json.loads((root / "dimos_meta.json").read_text())
    assert sidecar["feature_names"]["joint_state"] == "observation.state"
    assert sidecar["feature_names"]["target"] == "action"
