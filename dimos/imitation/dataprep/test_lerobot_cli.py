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

from dimos.core.python_native_environment import PythonNativeProject
from dimos.imitation.dataprep.cli import _load_config
from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig
from dimos.imitation.dataprep.lerobot import run_lerobot_dataprep
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


def test_conversion_uses_packaged_locked_environment(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    run = mocker.patch.object(
        PythonNativeProject,
        "run",
        return_value=subprocess.CompletedProcess([], 0),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    assert run_lerobot_dataprep(config) == tmp_path / "dataset"

    command = run.call_args.args[0]
    assert command[:2] == ["uv", "run"]
    assert "--project" in command
    assert "--locked" in command
    assert command[command.index("--python") + 1] == "3.12"
    entrypoint = command.index("dimos_lerobot.dataprep:convert")
    assert command[entrypoint - 3 : entrypoint + 1] == [
        "python",
        "-m",
        "dimos.core.python_native_call",
        "dimos_lerobot.dataprep:convert",
    ]


def test_conversion_reports_missing_uv(tmp_path: Path, mocker: pytest_mock.MockerFixture) -> None:
    mocker.patch.object(
        PythonNativeProject,
        "run",
        side_effect=FileNotFoundError("uv"),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    with pytest.raises(RuntimeError, match="uv is required"):
        run_lerobot_dataprep(config)
