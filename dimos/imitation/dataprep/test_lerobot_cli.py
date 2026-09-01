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

from pathlib import Path
import subprocess

import pytest
import pytest_mock

from dimos.imitation.dataprep.cli import _load_config
from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig
from dimos.imitation.dataprep.lerobot import lerobot_project, run_lerobot_dataprep
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS


def test_openyam_profile_replaces_the_deleted_json_config(tmp_path: Path) -> None:
    config = _load_config(
        None,
        "openyam",
        tmp_path / "session.mcap",
        tmp_path / "dataset",
        None,
    )

    assert config.sync.rate_hz == 30.0
    assert config.observation["observation.state"].names == OPENYAM_JOINTS
    assert config.action["action"].names == OPENYAM_JOINTS


def test_profile_and_json_config_are_mutually_exclusive(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="mutually exclusive"):
        _load_config(tmp_path / "config.json", "openyam", None, None, None)


def test_conversion_runs_packaged_module_in_policy_project(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch.dict("os.environ", {"VIRTUAL_ENV": "/parent/.venv"})
    run = mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    assert run_lerobot_dataprep(config) == tmp_path / "dataset"

    command = run.call_args.args[0]
    project = Path(__file__).parents[1] / "policy" / "lerobot" / "python"
    assert lerobot_project() == project
    assert command[:3] == ["uv", "run", "--frozen"]
    assert command[-4:-1] == ["python", "-m", "dimos_lerobot.dataprep"]
    assert "--python" not in command
    assert run.call_args.kwargs["cwd"] == project
    assert "VIRTUAL_ENV" not in run.call_args.kwargs["env"]
    assert run.call_args.kwargs["capture_output"] is True
    assert run.call_args.kwargs["text"] is True


def test_conversion_reports_missing_uv(tmp_path: Path, mocker: pytest_mock.MockerFixture) -> None:
    mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        side_effect=FileNotFoundError("uv"),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    with pytest.raises(RuntimeError, match="uv is required"):
        run_lerobot_dataprep(config)


def test_conversion_reports_child_process_diagnostics(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        return_value=subprocess.CompletedProcess([], 9, stdout="partial output", stderr="bad config"),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    with pytest.raises(RuntimeError, match="status 9: partial output\nbad config"):
        run_lerobot_dataprep(config)
