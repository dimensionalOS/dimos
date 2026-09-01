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

from dimos_lerobot import dataprep
from dimos_lerobot.dataprep import write
import numpy as np
import pytest
import pytest_mock

from dimos.imitation.dataprep.core import OutputConfig, Sample

JOINTS = [f"arm/joint{index}" for index in range(1, 7)] + ["arm/gripper"]


def test_module_entry_point_converts_one_config(mocker: pytest_mock.MockerFixture) -> None:
    convert = mocker.patch.object(dataprep, "convert")

    dataprep.main(["config.json"])

    convert.assert_called_once_with("config.json")


@pytest.mark.parametrize("args", [[], ["one.json", "two.json"]])
def test_module_entry_point_requires_one_config(args: list[str]) -> None:
    with pytest.raises(SystemExit, match="usage: python -m dimos_lerobot.dataprep CONFIG_JSON"):
        dataprep.main(args)


def samples() -> Iterator[Sample]:
    for episode, task in (("first", "pick"), ("second", "place")):
        for frame in range(3):
            value = float(frame + (10 if episode == "second" else 0))
            yield Sample(
                ts=value,
                episode_id=episode,
                observation={
                    "observation.images.wrist": np.full((64, 64, 3), frame, dtype=np.uint8),
                    "observation.state": np.full(7, value, dtype=np.float32),
                    "observation.effort": np.full(7, value * 0.1, dtype=np.float32),
                },
                action={"action": np.full(7, value + 1, dtype=np.float32)},
                task_label=task,
                complementary_info={"is_filled": np.asarray([False])},
            )


def output(path: Path) -> OutputConfig:
    return OutputConfig(
        format="lerobot",
        path=path,
        metadata={
            "repo_id": "local/openyam-test",
            "fps": 30,
            "robot_type": "openyam",
            "feature_schema": {
                "observation.images.wrist": {
                    "dtype": "video",
                    "shape": [64, 64, 3],
                    "names": ["height", "width", "channels"],
                },
                "observation.state": {"dtype": "float32", "shape": [7], "names": JOINTS},
                "observation.effort": {"dtype": "float32", "shape": [7], "names": JOINTS},
                "action": {"dtype": "float32", "shape": [7], "names": JOINTS},
                "complementary_info.is_filled": {
                    "dtype": "bool",
                    "shape": [1],
                    "names": ["is_filled"],
                },
            },
        },
    )


def test_native_writer_creates_canonical_openyam_dataset(tmp_path: Path) -> None:
    root = write(samples(), output(tmp_path / "dataset"))

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 6
    assert info["fps"] == 30
    assert info["robot_type"] == "openyam"
    assert set(info["features"]) >= {
        "observation.images.wrist",
        "observation.state",
        "action",
        "observation.effort",
        "complementary_info.is_filled",
    }
    assert info["features"]["observation.state"]["names"] == JOINTS


def test_native_writer_requires_repo_id(tmp_path: Path) -> None:
    config = output(tmp_path / "dataset").model_copy(update={"metadata": {"fps": 30}})

    with pytest.raises(ValueError, match="repo_id is required"):
        write(samples(), config)


def test_native_writer_rejects_fractional_fps(tmp_path: Path) -> None:
    config = output(tmp_path / "dataset")
    config.metadata["fps"] = 14.5

    with pytest.raises(ValueError, match="positive integer"):
        write(samples(), config)
