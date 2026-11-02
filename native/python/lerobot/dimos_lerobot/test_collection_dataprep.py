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

"""End-to-end coverage from recorded episodes through native LeRobot export."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import av
from dimos_lerobot.dataprep import inspect_dataset, write
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import dataset_to_policy_features
from mcap.writer import Writer
import numpy as np
import pytest
import pytest_mock

from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.schema import (
    DataPrepConfig,
    EpisodeExtractor,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
    RecordingSchema,
    SyncConfig,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String

EXPECTED_STATE = np.asarray(
    [[0, 100], [1, 101], [2, 102], [20, 120], [21, 121], [22, 122]], dtype=np.float32
)
EXPECTED_ACTION = EXPECTED_STATE.copy()


def _dataprep_config(path: Path, output: OutputConfig) -> DataPrepConfig:
    state = FeatureSpec(
        stream="state", field="position", dtype="float32", shape=(2,), names=["joint_0", "joint_1"]
    )
    return DataPrepConfig(
        recording_schema=RecordingSchema(
            episodes=EpisodeExtractor(status_stream="status"),
            observation={
                "camera": FeatureSpec(
                    stream="camera",
                    field="data",
                    dtype="video",
                    shape=(64, 64, 3),
                    names=["height", "width", "channels"],
                ),
                "state": state,
            },
            action={"action": state},
            sync=SyncConfig(anchor="camera", rate_hz=1, tolerance_ms=1),
            quality=QualityConfig(max_camera_gap_ms=1100),
        ),
        source=str(path),
        output=output,
    )


@pytest.fixture
def recorded_session(tmp_path: Path) -> tuple[Path, dict[float, np.ndarray[Any, Any]]]:
    path = tmp_path / "recording.mcap"
    images = {}
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start()
        channels = {}
        for name, kind in (
            ("camera", Image),
            ("state", JointState),
            ("target", JointState),
            ("status", String),
        ):
            channels[name] = writer.register_channel(
                topic=name,
                message_encoding="lcm",
                schema_id=0,
                metadata={
                    "dimos.payload_type": f"{kind.__module__}.{kind.__qualname__}",
                    "dimos.observation_time": "publish_time",
                },
            )

        def add(name: str, ts: float, message: Any) -> None:
            writer.add_message(
                channel_id=channels[name],
                log_time=round(ts * 1e9),
                publish_time=round(ts * 1e9),
                data=message.lcm_encode(),
            )

        saved = discarded = 0
        for start, task, success, base in (
            (100.0, "pick", True, 0),
            (104.0, "discard-me", False, 10),
            (108.0, "place", True, 20),
        ):
            add(
                "status",
                start,
                String(
                    EpisodeStatus(
                        ts=start,
                        state="recording",
                        last_event="start",
                        episodes_saved=saved,
                        episodes_discarded=discarded,
                        task_label=task,
                    ).to_json()
                ),
            )
            for frame in range(3):
                ts = start + frame
                images[ts] = np.full((64, 64, 3), (base + frame) * 4, dtype=np.uint8)
                add(
                    "camera",
                    ts,
                    Image(data=images[ts], format=ImageFormat.RGB, ts=ts, frame_id="camera"),
                )
                add(
                    "state",
                    ts,
                    JointState(
                        ts=ts,
                        frame_id="arm",
                        name=["joint_0", "joint_1"],
                        position=[base + frame, base + 100.0 + frame],
                        velocity=[],
                        effort=[],
                    ),
                )
                add(
                    "target",
                    ts,
                    JointState(
                        ts=ts,
                        name=["joint_0", "joint_1"],
                        position=[base + frame + 1000.0, base + frame + 2000.0],
                    ),
                )
            saved += int(success)
            discarded += int(not success)
            add(
                "status",
                start + 2,
                String(
                    EpisodeStatus(
                        ts=start + 2,
                        state="idle",
                        last_event="save" if success else "discard",
                        episodes_saved=saved,
                        episodes_discarded=discarded,
                        task_label=task,
                    ).to_json()
                ),
            )
        add(
            "status",
            112,
            String(
                EpisodeStatus(
                    ts=112,
                    state="recording",
                    last_event="start",
                    episodes_saved=2,
                    episodes_discarded=1,
                    task_label="interrupted",
                ).to_json()
            ),
        )
        writer.finish()
    return path, images


def _read_video(path: Path) -> list[np.ndarray[Any, Any]]:
    with av.open(str(path)) as container:
        frames = [frame.to_ndarray(format="rgb24") for frame in container.decode(video=0)]
    return frames


def test_collection_to_lerobot_roundtrip(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    db_path, recorded_images = recorded_session
    lerobot_path = run_dataprep(
        _dataprep_config(
            db_path,
            OutputConfig(
                format="lerobot",
                path=tmp_path / "lerobot",
                metadata={"robot": "synthetic", "repo_id": "dimos/collection-test"},
            ),
        ),
        writer=write,
    )

    lerobot_info = inspect_dataset(lerobot_path)
    assert set(lerobot_info["observation"]) == {"observation.images.camera", "observation.state"}
    assert lerobot_info["observation"]["observation.state"] == {"shape": [2], "dtype": "float32"}
    assert (lerobot_info["episodes"], lerobot_info["frames"], lerobot_info["fps"]) == (2, 6, 1.0)
    dataset = LeRobotDataset("dimos/collection-test", root=lerobot_path)
    data = dataset.hf_dataset
    assert data["timestamp"] == pytest.approx([0.0, 1.0, 2.0, 0.0, 1.0, 2.0])
    np.testing.assert_array_equal(np.asarray(data["observation.state"]), EXPECTED_STATE)
    np.testing.assert_array_equal(np.asarray(data["action"]), EXPECTED_ACTION)

    episode_rows = list(dataset.meta.episodes)
    assert [row["tasks"] for row in episode_rows] == [["pick"], ["place"]]

    video = _read_video(lerobot_path / "videos/observation.images.camera/chunk-000/file-000.mp4")
    assert len(video) == 6
    expected_means = [
        recorded_images[ts].mean() for ts in (100.0, 101.0, 102.0, 108.0, 109.0, 110.0)
    ]
    np.testing.assert_allclose([frame.mean() for frame in video], expected_means, atol=5.0)


def test_explicit_range_exports_use_configured_task_label(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    recording, _ = recorded_session
    output = OutputConfig(
        format="lerobot",
        path=tmp_path / "range-dataset",
        metadata={"repo_id": "dimos/range-test", "default_task_label": "manual pick"},
    )
    config = _dataprep_config(recording, output)
    config.recording_schema.episodes = EpisodeExtractor(extractor="ranges", ranges=[(100, 102)])

    root = run_dataprep(config, writer=write)

    metadata = LeRobotDataset("dimos/range-test", root=root).meta
    assert metadata.total_frames == 3
    assert [row["tasks"] for row in metadata.episodes] == [["manual pick"]]
    assert (root / "dimos_meta.json").is_file()


@pytest.mark.parametrize("existing", [False, True])
def test_sidecar_failure_preserves_complete_dataset_and_allows_retry(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
    mocker: pytest_mock.MockerFixture,
    existing: bool,
) -> None:
    recording, _ = recorded_session
    output = OutputConfig(
        format="lerobot",
        path=tmp_path / "dataset",
        metadata={"repo_id": "local/sidecar-test"},
    )
    config = _dataprep_config(recording, output)
    previous = {}
    if existing:
        run_dataprep(config, writer=write)
        previous = {
            path.relative_to(output.path): path.read_bytes()
            for path in output.path.rglob("*")
            if path.is_file()
        }
    fail = mocker.patch(
        "dimos.imitation.dataprep.build._write_dimos_meta",
        side_effect=OSError(28, "No space left on device"),
    )

    with pytest.raises(OSError, match="No space left"):
        run_dataprep(config, writer=write)

    if existing:
        assert {
            path.relative_to(output.path): path.read_bytes()
            for path in output.path.rglob("*")
            if path.is_file()
        } == previous
    else:
        assert not output.path.exists()
    assert not list(tmp_path.glob(".dataset-*"))
    mocker.stop(fail)
    root = run_dataprep(config, writer=write)
    assert root == output.path
    assert (root / "dimos_meta.json").is_file()
    assert LeRobotDataset("local/sidecar-test", root=root).meta.total_frames == 6


@pytest.mark.parametrize("canonical", [False, True])
def test_mcap_conversion_produces_training_feature_keys(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
    canonical: bool,
) -> None:
    recording, _ = recorded_session
    config = _dataprep_config(
        recording,
        OutputConfig(
            format="lerobot",
            path=tmp_path / "training-dataset",
            metadata={"repo_id": "local/training-keys"},
        ),
    )
    image_key = "observation.images.images" if canonical else "images"
    state_key = "observation.state" if canonical else "joint_state"
    action_key = "action" if canonical else "target"
    config.recording_schema.observation = {
        state_key: config.recording_schema.observation["state"],
        image_key: config.recording_schema.observation["camera"],
    }
    config.recording_schema.action = {
        action_key: config.recording_schema.action["action"].model_copy(update={"stream": "target"})
    }
    config.recording_schema.sync.anchor = image_key

    root = run_dataprep(config, writer=write)

    dataset = LeRobotDataset("local/training-keys", root=root)
    info = json.loads((root / "meta/info.json").read_text())
    assert set(info["features"]) == {
        "observation.state",
        "observation.images.images",
        "action",
        "complementary_info.is_filled",
        "timestamp",
        "frame_index",
        "episode_index",
        "index",
        "task_index",
    }
    # This is the feature inference function used by LeRobot's policy factory during training.
    assert dataset_to_policy_features(dataset.meta.features) == {
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(2,)),
        "observation.images.images": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 64, 64)),
        "action": PolicyFeature(type=FeatureType.ACTION, shape=(2,)),
    }
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["observation.state"]), EXPECTED_STATE
    )
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["action"]),
        [[1000, 2000], [1001, 2001], [1002, 2002], [1020, 2020], [1021, 2021], [1022, 2022]],
    )
    assert len(_read_video(root / "videos/observation.images.images/chunk-000/file-000.mp4")) == 6
    sidecar = json.loads((root / "dimos_meta.json").read_text())
    assert sidecar["feature_names"] == {
        state_key: "observation.state",
        image_key: "observation.images.images",
        action_key: "action",
    }
    assert sidecar["observation"][state_key]["stream"] == "state"
    assert sidecar["observation"][image_key]["stream"] == "camera"
    summary = inspect_dataset(root)
    assert set(summary["observation"]) == {"observation.state", "observation.images.images"}
    assert set(summary["action"]) == {"action"}


def test_multiple_features_keep_separate_canonical_keys(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    recording, _ = recorded_session
    config = _dataprep_config(
        recording,
        OutputConfig(
            path=tmp_path / "multiple",
            metadata={"repo_id": "local/multiple"},
        ),
    )
    state = config.recording_schema.observation["state"]
    config.recording_schema.observation = {
        "left_state": state.model_copy(update={"shape": (1,), "names": ["joint_0"]}),
        "right_state": state.model_copy(update={"shape": (1,), "names": ["joint_1"]}),
        "camera": config.recording_schema.observation["camera"],
    }
    config.recording_schema.action = {
        "left_target": state.model_copy(
            update={"stream": "target", "shape": (1,), "names": ["joint_0"]}
        ),
        "right_target": state.model_copy(
            update={"stream": "target", "shape": (1,), "names": ["joint_1"]}
        ),
    }

    root = run_dataprep(config, writer=write)

    dataset = LeRobotDataset("local/multiple", root=root)
    assert dataset_to_policy_features(dataset.meta.features) == {
        "observation.left_state": PolicyFeature(type=FeatureType.STATE, shape=(1,)),
        "observation.right_state": PolicyFeature(type=FeatureType.STATE, shape=(1,)),
        "observation.images.camera": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 64, 64)),
        "action.left_target": PolicyFeature(type=FeatureType.ACTION, shape=(1,)),
        "action.right_target": PolicyFeature(type=FeatureType.ACTION, shape=(1,)),
    }
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["action.left_target"]), [1000, 1001, 1002, 1020, 1021, 1022]
    )


def test_renamed_feature_collision_preserves_existing_dataset(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    recording, _ = recorded_session
    config = _dataprep_config(
        recording,
        OutputConfig(
            path=tmp_path / "collision",
            metadata={"repo_id": "local/collision"},
        ),
    )
    root = run_dataprep(config, writer=write)
    previous = {
        path.relative_to(root): path.read_bytes() for path in root.rglob("*") if path.is_file()
    }
    config.recording_schema.observation["observation.images.camera"] = (
        config.recording_schema.observation["camera"]
    )

    with pytest.raises(ValueError, match="rename collision"):
        run_dataprep(config, writer=write)

    assert {
        path.relative_to(root): path.read_bytes() for path in root.rglob("*") if path.is_file()
    } == previous
