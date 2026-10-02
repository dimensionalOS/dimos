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

from pathlib import Path
from typing import Any

import av
from dimos_lerobot.dataprep import inspect_dataset, write
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from mcap.writer import Writer
import numpy as np
import pytest

from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    EpisodeExtractor,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
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
        source=str(path),
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
        for name, kind in (("camera", Image), ("state", JointState), ("status", String)):
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
    assert (lerobot_info["episodes"], lerobot_info["frames"], lerobot_info["fps"]) == (2, 6, 1.0)
    dataset = LeRobotDataset("dimos/collection-test", root=lerobot_path)
    data = dataset.hf_dataset
    assert data["timestamp"] == pytest.approx([0.0, 1.0, 2.0, 0.0, 1.0, 2.0])
    assert data["episode_index"] == [0, 0, 0, 1, 1, 1]
    assert data["frame_index"] == [0, 1, 2, 0, 1, 2]
    np.testing.assert_array_equal(np.asarray(data["state"]), EXPECTED_STATE)
    np.testing.assert_array_equal(np.asarray(data["action"]), EXPECTED_ACTION)

    episode_rows = list(dataset.meta.episodes)
    assert [row["length"] for row in episode_rows] == [3, 3]
    assert [(row["dataset_from_index"], row["dataset_to_index"]) for row in episode_rows] == [
        (0, 3),
        (3, 6),
    ]
    assert [row["tasks"] for row in episode_rows] == [["pick"], ["place"]]

    video = _read_video(lerobot_path / "videos/camera/chunk-000/file-000.mp4")
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
    config.episodes = EpisodeExtractor(extractor="ranges", ranges=[(100, 102)])

    root = run_dataprep(config, writer=write)

    metadata = LeRobotDataset("dimos/range-test", root=root).meta
    assert metadata.total_frames == 3
    assert [row["tasks"] for row in metadata.episodes] == [["manual pick"]]
    assert (root / "dimos_meta.json").is_file()
