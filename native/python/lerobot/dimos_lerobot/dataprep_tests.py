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
from io import StringIO
import json
from pathlib import Path
import sys

from dimos_lerobot import dataprep
from dimos_lerobot.dataprep import write
import numpy as np
import pytest
import pytest_mock

from dimos.imitation.dataprep._lerobot_protocol import BuildRequest, BuildResult
from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig, Sample

JOINTS = [f"arm/joint{index}" for index in range(1, 7)] + ["arm/gripper"]


def test_module_entry_point_executes_typed_build_request(
    tmp_path: Path,
    mocker: pytest_mock.MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    config = DataPrepConfig(
        source="recording.mcap",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )
    request = BuildRequest(config=config)
    run = mocker.patch.object(dataprep, "run_dataprep", return_value=tmp_path / "dataset")
    monkeypatch.setattr(sys, "stdin", StringIO(request.model_dump_json()))

    dataprep.main([])

    result = BuildResult.model_validate_json(capsys.readouterr().out)
    assert result.path == tmp_path / "dataset"
    assert run.call_args.args[0].source == "recording.mcap"
    assert run.call_args.kwargs["writer"] is write


@pytest.mark.parametrize("args", [["config.json"], ["one", "two"]])
def test_module_entry_point_rejects_arguments(args: list[str]) -> None:
    with pytest.raises(SystemExit, match="usage: python -m dimos_lerobot.dataprep"):
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

    summary = dataprep.inspect_dataset(root)
    assert summary["episodes"] == 2
    assert summary["frames"] == 6
    assert summary["episode_lengths"] == {
        "min": 3,
        "max": 3,
        "mean": 3.0,
        "uniform": True,
    }
    assert summary["observation"]["observation.images.wrist"]["dtype"] == "video"


def test_native_writer_requires_repo_id(tmp_path: Path) -> None:
    config = output(tmp_path / "dataset").model_copy(update={"metadata": {"fps": 30}})

    with pytest.raises(ValueError, match="repo_id is required"):
        write(samples(), config)


def test_native_writer_rejects_fractional_fps(tmp_path: Path) -> None:
    config = output(tmp_path / "dataset")
    config.metadata["fps"] = 14.5

    with pytest.raises(ValueError, match="positive integer"):
        write(samples(), config)


def numeric_output(path: Path) -> OutputConfig:
    config = output(path)
    config.metadata["feature_schema"] = {
        "observation.state": {"dtype": "float32", "shape": [1], "names": ["joint"]},
        "action": {"dtype": "float32", "shape": [1], "names": ["joint"]},
    }
    return config


def numeric_samples(task_label: str | None = "pick") -> Iterator[Sample]:
    for episode in ("first", "second"):
        yield Sample(
            ts=0,
            episode_id=episode,
            observation={"observation.state": np.asarray([1], dtype=np.float32)},
            action={"action": np.asarray([2], dtype=np.float32)},
            task_label=task_label,
        )


@pytest.mark.parametrize("default_label", [None, "manual range"])
def test_unlabeled_samples_use_configured_default_task(
    tmp_path: Path, default_label: str | None
) -> None:
    config = numeric_output(tmp_path / "dataset")
    if default_label is not None:
        config.metadata["default_task_label"] = default_label

    root = write(numeric_samples(None), config)

    metadata = dataprep.LeRobotDatasetMetadata(repo_id="local/openyam-test", root=root)
    assert [row["tasks"] for row in metadata.episodes] == [[default_label or "task"]] * 2


@pytest.mark.parametrize("explicit_robot", [None, "new-robot"])
def test_robot_metadata_is_preserved_with_explicit_type_precedence(
    tmp_path: Path, explicit_robot: str | None
) -> None:
    config = numeric_output(tmp_path / "dataset")
    config.metadata.pop("robot_type")
    config.metadata["robot"] = "xarm7"
    if explicit_robot is not None:
        config.metadata["robot_type"] = explicit_robot

    root = write(numeric_samples(), config)

    assert dataprep.inspect_dataset(root)["robot"] == (explicit_robot or "xarm7")


def test_rebuild_replaces_dataset_only_after_success(tmp_path: Path) -> None:
    config = numeric_output(tmp_path / "dataset")
    root = write(numeric_samples(), config)

    write(iter([next(numeric_samples("replacement"))]), config)

    summary = dataprep.inspect_dataset(root)
    assert (summary["episodes"], summary["frames"]) == (1, 1)
    assert [
        row["tasks"]
        for row in dataprep.LeRobotDatasetMetadata(repo_id="local/openyam-test", root=root).episodes
    ] == [["replacement"]]
    assert sorted(path.name for path in tmp_path.iterdir()) == ["dataset"]


@pytest.mark.parametrize("existing", [False, True])
def test_later_episode_failure_preserves_previous_output_and_allows_retry(
    tmp_path: Path, existing: bool
) -> None:
    config = numeric_output(tmp_path / "dataset")
    if existing:
        write(numeric_samples("original"), config)
    valid = list(numeric_samples())
    invalid = valid[1].model_copy(update={"action": {"action": np.asarray([1, 2])}})

    with pytest.raises(ValueError, match="shape changed"):
        write(iter([valid[0], invalid]), config)

    if existing:
        metadata = dataprep.LeRobotDatasetMetadata(repo_id="local/openyam-test", root=config.path)
        assert [row["tasks"] for row in metadata.episodes] == [["original"]] * 2
    else:
        assert not config.path.exists()
    root = write(numeric_samples("retry"), config)
    assert dataprep.inspect_dataset(root)["frames"] == 2
    assert sorted(path.name for path in tmp_path.iterdir()) == ["dataset"]


def test_existing_unrelated_directory_is_preserved(tmp_path: Path) -> None:
    config = numeric_output(tmp_path / "dataset")
    config.path.mkdir()
    asset = config.path / "unrelated.txt"
    asset.write_text("keep me")

    with pytest.raises(FileExistsError, match="LeRobot dataset"):
        write(numeric_samples(), config)

    assert asset.read_text() == "keep me"


def test_native_dataset_retains_boolean_fill_flags(tmp_path: Path) -> None:
    config = numeric_output(tmp_path / "dataset")
    config.metadata["feature_schema"]["complementary_info.is_filled"] = {
        "dtype": "bool",
        "shape": [1],
        "names": ["is_filled"],
    }
    samples = (
        sample.model_copy(update={"complementary_info": {"is_filled": np.asarray([index == 1])}})
        for index, sample in enumerate(numeric_samples())
    )

    root = write(samples, config)

    dataset = dataprep.LeRobotDataset("local/openyam-test", root=root)
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["complementary_info.is_filled"]),
        [False, True],
    )
    assert dataset.meta.features["complementary_info.is_filled"]["dtype"] == "bool"
