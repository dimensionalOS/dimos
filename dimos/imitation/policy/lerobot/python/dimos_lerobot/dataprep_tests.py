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

from collections.abc import Iterator
import json
from pathlib import Path

from dimos_lerobot.dataprep import write
import numpy as np
import pytest

from dimos.imitation.dataprep.core import OutputConfig, Sample

JOINTS = [f"arm/joint{index}" for index in range(1, 7)] + ["arm/gripper"]


def samples() -> Iterator[Sample]:
    for episode, task in (("first", "pick"), ("second", "place")):
        for frame in range(3):
            value = float(frame + (10 if episode == "second" else 0))
            yield Sample(
                ts=value,
                episode_id=episode,
                observation={
                    "wrist": np.full((64, 64, 3), frame, dtype=np.uint8),
                    "joints": np.full(7, value, dtype=np.float32),
                },
                action={"next_joints": np.full(7, value + 1, dtype=np.float32)},
                task_label=task,
            )


def output(path: Path) -> OutputConfig:
    return OutputConfig(
        format="lerobot",
        path=path,
        metadata={
            "repo_id": "local/openyam-test",
            "fps": 15,
            "robot_type": "openyam",
            "joint_names": JOINTS,
        },
    )


def test_native_writer_creates_canonical_openyam_dataset(tmp_path: Path) -> None:
    root = write(samples(), output(tmp_path / "dataset"))

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 6
    assert info["fps"] == 15
    assert info["robot_type"] == "openyam"
    assert set(info["features"]) >= {
        "observation.images.wrist",
        "observation.state",
        "action",
    }
    assert info["features"]["observation.state"]["names"] == JOINTS


def test_native_writer_requires_repo_id(tmp_path: Path) -> None:
    config = output(tmp_path / "dataset").model_copy(update={"metadata": {"fps": 15}})

    with pytest.raises(ValueError, match="repo_id is required"):
        write(samples(), config)


def test_native_writer_rejects_fractional_fps(tmp_path: Path) -> None:
    config = output(tmp_path / "dataset")
    config.metadata["fps"] = 14.5

    with pytest.raises(ValueError, match="positive integer"):
        write(samples(), config)
