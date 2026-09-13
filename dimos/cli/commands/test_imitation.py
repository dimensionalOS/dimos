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


import json

import pytest
from typer.testing import CliRunner

from dimos.cli.commands.imitation import imitation_app
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.imitation.collection.recording import RecordingSchema
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.robot.manipulators.openyam.collection import OPENYAM_TEACH_COLLECTION


def test_help_exposes_attached_controls_and_no_workflow_launcher():
    result = CliRunner().invoke(imitation_app, ["--help"])
    assert result.exit_code == 0
    for command in ("collect", "rollout", "prepare", "inspect", "visualize", "train"):
        assert command in result.output
    assert CliRunner().invoke(imitation_app, ["list"]).exit_code == 2
    assert CliRunner().invoke(imitation_app, ["run"]).exit_code == 2
    assert CliRunner().invoke(imitation_app, ["collect", "--module", "foo"]).exit_code == 2


@pytest.mark.parametrize(
    ("command", "app"),
    [
        ("collect", "CollectionApp"),
        ("rollout", "RolloutApp"),
    ],
)
def test_live_commands_only_connect_and_detach(command, app, mocker):
    driver = mocker.Mock()
    driver.find_module_by_spec.return_value.get_status.return_value = EpisodeStatus(
        ts=1.0, state="idle", episodes_saved=0, episodes_discarded=0
    )
    mocker.patch("dimos.cli.commands.imitation.Dimos.connect", return_value=driver)
    ui = mocker.patch(f"dimos.cli.commands.imitation.{app}")
    result = CliRunner().invoke(imitation_app, [command])
    assert result.exit_code == 0, result.output
    ui.return_value.run.assert_called_once_with()
    driver.run.assert_not_called()
    driver.stop.assert_called_once_with()


@pytest.mark.parametrize(
    "error",
    [
        LookupError("No deployed module matches EpisodeControlSpec"),
        ValueError("Multiple modules match EpisodeControlSpec"),
    ],
)
def test_discovery_errors_close_connection_and_explain_failure(error, mocker):
    driver = mocker.Mock()
    driver.find_module_by_spec.side_effect = error
    mocker.patch("dimos.cli.commands.imitation.Dimos.connect", return_value=driver)
    result = CliRunner().invoke(imitation_app, ["collect"])
    assert result.exit_code == 1
    assert str(error) in result.output
    driver.stop.assert_called_once_with()


@pytest.fixture
def recording(tmp_path):
    directory = tmp_path / "session"
    directory.mkdir()
    (directory / "schema.json").write_text(OPENYAM_TEACH_COLLECTION.to_schema().model_dump_json())
    (directory / "recording.mcap").touch()
    return directory


def test_prepare_uses_saved_schema_not_robot_lookup(recording, tmp_path, mocker):
    output = tmp_path / "dataset"
    prepare = mocker.patch("dimos.cli.commands.imitation.run_lerobot_dataprep", return_value=output)
    result = CliRunner().invoke(imitation_app, ["prepare", str(recording), "--output", str(output)])
    assert result.exit_code == 0, result.output
    config = prepare.call_args.args[0]
    assert config.source == str(recording / "recording.mcap")
    assert config.observation == RecordingSchema.read(recording).observation
    assert config.output.path == output


def test_prepare_rejects_existing_output(recording, tmp_path):
    result = CliRunner().invoke(
        imitation_app, ["prepare", str(recording), "--output", str(tmp_path)]
    )
    assert result.exit_code == 2
    assert "already exists" in result.output


@pytest.mark.parametrize("flags", [["--json"], ["--json", "--verbose"]])
def test_inspect_reads_recording_schema(recording, mocker, flags):
    inspect = mocker.patch(
        "dimos.cli.commands.imitation.inspect_recording",
        return_value={"episodes": 2, "rate": 30.000210029462686},
    )
    result = CliRunner().invoke(imitation_app, ["inspect", str(recording), *flags])
    assert result.exit_code == 0, result.output
    assert json.loads(result.output) == {"episodes": 2, "rate": 30.000210029462686}
    assert inspect.call_args.args == (recording / "recording.mcap",)


@pytest.mark.parametrize("flags", [[], ["--verbose"]])
def test_inspect_defaults_to_human_output(recording, mocker, flags):
    mocker.patch(
        "dimos.cli.commands.imitation.inspect_recording",
        return_value={
            "format": "recording",
            "path": str(recording / "recording.mcap"),
            "streams": {"wrist_image": 1953},
            "status_stream": None,
            "episodes": 0,
            "saved_episodes": 0,
            "discarded_episodes": 0,
            "incomplete_episodes": [],
        },
    )
    result = CliRunner().invoke(imitation_app, ["inspect", str(recording), *flags])
    assert result.exit_code == 0, result.output
    assert "No episodes" in result.output
    assert "1,953" in result.output
    assert "Not assessed" in result.output


@pytest.mark.parametrize("flags", [[], ["--json"], ["--verbose"]])
def test_inspect_preserves_failure_exit_code(recording, mocker, flags):
    mocker.patch(
        "dimos.cli.commands.imitation.inspect_recording", side_effect=ValueError("bad payload")
    )
    result = CliRunner().invoke(imitation_app, ["inspect", str(recording), *flags])
    assert result.exit_code == 1
    assert "Inspection failed: bad payload" in result.output


def test_train_forwards_arguments_and_exit_code(mocker):
    run = mocker.patch(
        "dimos.cli.commands.imitation.subprocess.run", return_value=mocker.Mock(returncode=17)
    )
    result = CliRunner().invoke(
        imitation_app, ["train", "--policy.type=act", "--dataset.repo_id=local/test"]
    )
    assert result.exit_code == 17
    assert run.call_args.args[0][-3:] == [
        "lerobot-train",
        "--policy.type=act",
        "--dataset.repo_id=local/test",
    ]


@pytest.fixture
def visualization(tmp_path, monkeypatch, mocker):
    dataset = tmp_path / "dataset with spaces"
    (dataset / "meta").mkdir(parents=True)
    (dataset / "meta" / "info.json").write_text("{}")
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("dimos.utils.cache._CACHE_LOCK_DIR", tmp_path / "locks")
    monkeypatch.setattr("dimos.utils.cache._CACHE_GATE_PATH", tmp_path / "gate.lock")
    run = mocker.patch(
        "dimos.cli.commands.imitation.subprocess.run", return_value=mocker.Mock(returncode=0)
    )
    return dataset, run


@pytest.mark.parametrize(("flags", "episode"), [([], "0"), (["--episode", "2"], "2")])
def test_visualize_launches_local_viewer(visualization, monkeypatch, flags, episode):
    dataset, run = visualization
    monkeypatch.setenv("HF_HUB_OFFLINE", "0")
    monkeypatch.setenv("VIRTUAL_ENV", "/host/venv")
    result = CliRunner().invoke(imitation_app, ["visualize", dataset.name, *flags])
    assert result.exit_code == 0, result.output
    project = DIMOS_PROJECT_ROOT / "dimos" / "imitation" / "policy" / "lerobot" / "python"
    assert run.call_args.args[0] == [
        "uv",
        "run",
        "--project",
        str(project),
        "--frozen",
        "lerobot-dataset-viz",
        "--root",
        str(dataset),
        "--repo-id",
        "local/dataset",
        "--episode-index",
        episode,
        "--num-workers",
        "0",
        "--mode",
        "local",
    ]
    assert run.call_args.kwargs["env"]["HF_HUB_OFFLINE"] == "1"
    assert "VIRTUAL_ENV" not in run.call_args.kwargs["env"]
    assert set(run.call_args.kwargs) == {"env", "check"}


@pytest.mark.parametrize(
    ("kind", "message"),
    [
        ("missing", "directory does not exist"),
        ("file", "directory does not exist"),
        ("empty", "Not a prepared LeRobot dataset"),
        ("recording", "imitation prepare"),
        ("negative", "--episode"),
    ],
)
def test_visualize_rejects_invalid_input(visualization, tmp_path, kind, message):
    dataset, run = visualization
    path = tmp_path / kind
    flags = []
    if kind == "file":
        path.touch()
    elif kind in {"empty", "recording"}:
        path.mkdir()
        if kind == "recording":
            (path / "schema.json").write_text("{}")
    elif kind == "negative":
        path = dataset
        flags = ["--episode", "-1"]
    result = CliRunner().invoke(imitation_app, ["visualize", str(path), *flags])
    assert result.exit_code == 2
    assert message in result.output
    run.assert_not_called()


def test_visualize_propagates_viewer_failure(visualization):
    dataset, run = visualization
    run.return_value.returncode = 17
    result = CliRunner().invoke(imitation_app, ["visualize", str(dataset)])
    assert result.exit_code == 17


def test_visualize_reports_missing_launcher(visualization):
    dataset, run = visualization
    run.side_effect = FileNotFoundError("uv")
    result = CliRunner().invoke(imitation_app, ["visualize", str(dataset)])
    assert result.exit_code == 1
    assert "uv" in result.output


def test_visualize_can_be_interrupted(visualization):
    dataset, run = visualization
    run.side_effect = KeyboardInterrupt
    result = CliRunner().invoke(imitation_app, ["visualize", str(dataset)])
    assert result.exit_code == 130
    assert list((dataset.parent / "locks").iterdir()) == []


def test_visualize_help_does_not_launch_viewer(visualization):
    _, run = visualization
    result = CliRunner().invoke(imitation_app, ["visualize", "--help"])
    assert result.exit_code == 0
    assert "--episode" in result.output
    run.assert_not_called()
