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
from typing import Any

from dimos_lerobot import dataprep
from dimos_lerobot.dataprep import write
from lerobot.datasets import LeRobotDatasetMetadata
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
import pytest
import pytest_mock

from dimos.imitation.dataprep.formats.lerobot.protocol import (
    BuildRequest,
    BuildResult,
    InspectRequest,
    InspectResult,
)
from dimos.imitation.dataprep.schema import DataPrepConfig, OutputConfig, Sample


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


def numeric_output(path: Path) -> OutputConfig:
    return OutputConfig(
        format="lerobot",
        path=path,
        metadata={
            "repo_id": "local/openyam-test",
            "fps": 30,
            "feature_schema": {
                "observation.state": {"dtype": "float32", "shape": [1], "names": ["joint"]},
                "action": {"dtype": "float32", "shape": [1], "names": ["joint"]},
            },
        },
    )


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

    metadata = LeRobotDatasetMetadata(repo_id="local/openyam-test", root=root)
    assert [row["tasks"] for row in metadata.episodes] == [[default_label or "task"]] * 2


@pytest.mark.parametrize("explicit_robot", [None, "new-robot"])
def test_robot_metadata_is_preserved_with_explicit_type_precedence(
    tmp_path: Path, explicit_robot: str | None
) -> None:
    config = numeric_output(tmp_path / "dataset")
    config.metadata["robot"] = "xarm7"
    if explicit_robot is not None:
        config.metadata["robot_type"] = explicit_robot

    root = write(numeric_samples(), config)

    assert dataprep.inspect_dataset(root)["robot"] == (explicit_robot or "xarm7")


@pytest.mark.parametrize("existing", [False, True])
def test_later_episode_failure_preserves_previous_output_and_allows_retry(
    tmp_path: Path, existing: bool
) -> None:
    config = numeric_output(tmp_path / "dataset")
    if existing:
        write(numeric_samples("original"), config)
    valid = list(numeric_samples())
    invalid = valid[1].model_copy(update={"action": {"action": np.asarray([1, 2])}})

    with pytest.raises(ValueError, match="shape"):
        write(iter([valid[0], invalid]), config)

    if existing:
        metadata = LeRobotDatasetMetadata(repo_id="local/openyam-test", root=config.path)
        assert [row["tasks"] for row in metadata.episodes] == [["original"]] * 2
    else:
        assert not config.path.exists()
    root = write(iter([next(numeric_samples("retry"))]), config)
    summary = dataprep.inspect_dataset(root)
    assert (summary["episodes"], summary["frames"]) == (1, 1)
    metadata = LeRobotDatasetMetadata(repo_id="local/openyam-test", root=root)
    assert [row["tasks"] for row in metadata.episodes] == [["retry"]]
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

    dataset = LeRobotDataset("local/openyam-test", root=root)
    np.testing.assert_array_equal(
        np.asarray(dataset.hf_dataset["complementary_info.is_filled"]),
        [False, True],
    )
    assert dataset.meta.features["complementary_info.is_filled"]["dtype"] == "bool"


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [("repo_id", None, "repo_id is required"), ("fps", 14.5, "positive integer")],
)
def test_writer_rejects_invalid_export_metadata(
    tmp_path: Path,
    field: str,
    value: Any,
    message: str,
) -> None:
    config = numeric_output(tmp_path / "dataset")
    config.metadata[field] = value
    with pytest.raises(ValueError, match=message):
        write(numeric_samples(), config)


@pytest.mark.parametrize("sidecar", [False, True])
@pytest.mark.parametrize(
    "names", [("state", "joint_target"), ("action.sensor", "observation.target")]
)
def test_inspect_preserves_custom_feature_names_in_direct_and_json_results(
    tmp_path: Path,
    sidecar: bool,
    names: tuple[str, str],
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    config = numeric_output(tmp_path / "dataset")
    features = config.metadata["feature_schema"]
    observation_name, action_name = names
    features[observation_name] = features.pop("observation.state")
    features[action_name] = features.pop("action")
    features["complementary_info.is_filled"] = {
        "dtype": "bool",
        "shape": [1],
        "names": ["is_filled"],
    }
    samples = (
        sample.model_copy(
            update={
                "observation": {observation_name: sample.observation["observation.state"]},
                "action": {action_name: sample.action["action"]},
                "complementary_info": {"is_filled": np.asarray([False])},
            }
        )
        for sample in numeric_samples()
    )
    root = config.path
    dataset = LeRobotDataset.create(
        repo_id="local/legacy",
        fps=30,
        features=features,
        root=root,
        use_videos=True,
    )
    for sample in samples:
        dataset.add_frame(
            {
                **sample.observation,
                **sample.action,
                "complementary_info.is_filled": sample.complementary_info["is_filled"],
                "task": "pick",
            }
        )
    dataset.save_episode(parallel_encoding=False)
    dataset.finalize()  # type: ignore[no-untyped-call]
    if sidecar:
        (root / "dimos_meta.json").write_text(
            json.dumps(
                {
                    "observation": {observation_name: {}},
                    "action": {action_name: {}},
                }
            )
        )

    summary = dataprep.inspect_dataset(root)

    entry = {"shape": [1], "dtype": "float32"}
    if sidecar:
        assert summary["observation"] == {observation_name: entry}
        assert summary["action"] == {action_name: entry}
    elif observation_name == "state":
        assert summary["observation"] == {}
        assert summary["action"] == {}
        assert summary["features"]["state"] == entry
        assert summary["features"]["joint_target"] == entry
    else:
        assert summary["observation"] == {"observation.target": entry}
        assert summary["action"] == {"action.sensor": entry}
    assert summary["features"]["complementary_info.is_filled"] == {"shape": [1], "dtype": "bool"}
    capsys.readouterr()
    monkeypatch.setattr(sys, "stdin", StringIO(InspectRequest(path=root).model_dump_json()))
    dataprep.main([])
    assert InspectResult.model_validate_json(capsys.readouterr().out).info == summary
