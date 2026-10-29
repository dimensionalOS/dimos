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

"""Convert Python-generated profile recordings into real training datasets."""

import json
from pathlib import Path
from typing import Any

from dimos_lerobot.dataprep import write
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import dataset_to_policy_features
import numpy as np
import pytest

from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.schema import FeatureSpec, OutputConfig, RecordingSchema
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String


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
def recorded_profile(tmp_path: Path, mock_profile: CollectionProfile, camera_count: int) -> Path:
    """Generate synthetic profile data with Python, without invoking a recorder process."""
    directory = tmp_path / "recording"
    directory.mkdir()
    schema = mock_profile.to_schema()
    schema.payload = "recording.db"
    (directory / "schema.json").write_text(schema.model_dump_json())
    types = {**mock_profile.input_types(), "status": String}
    with SqliteStore(path=str(directory / schema.payload)) as store:

        def emit(name: str, message: Any) -> None:
            value = String(message.to_json()) if isinstance(message, EpisodeStatus) else message
            store.stream(name, types[name], codec="json" if name == "status" else "lcm").append(
                value,
                ts=message.ts,
            )

        emit("accepted", JointState(ts=9.9, name=["left"], position=[1000]))
        emit("accepted", JointState(ts=9.9, name=["right"], position=[2000]))
        emit(
            "status",
            EpisodeStatus(
                ts=10,
                state="recording",
                last_event="start",
                task_label="mock pick",
                episodes_saved=0,
                episodes_discarded=0,
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
            EpisodeStatus(
                ts=10.1,
                state="idle",
                last_event="save",
                task_label="mock pick",
                episodes_saved=1,
                episodes_discarded=0,
            ),
        )
    return directory


@pytest.mark.parametrize("camera_count", [1, 2])
def test_mock_profile_recording_converts_to_training_dataset(
    tmp_path: Path,
    recorded_profile: Path,
    camera_count: int,
) -> None:
    moved = tmp_path / "moved-recording"
    recorded_profile.rename(moved)
    schema = RecordingSchema.read(moved)
    assert schema.observation["joint_state"].stream == "measured"
    assert schema.action["target"].stream == "accepted"
    assert schema.episodes.status_stream == "status"
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
