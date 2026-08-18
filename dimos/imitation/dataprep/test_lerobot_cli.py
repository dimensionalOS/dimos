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

from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig
from dimos.imitation.dataprep.lerobot import run_lerobot_dataprep


def test_conversion_uses_packaged_locked_environment(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
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
    assert command[:2] == ["uv", "run"]
    assert "--project" in command
    assert "--locked" in command
    assert command[command.index("--python") + 1] == "3.12"
    entrypoint = command.index("dimos_lerobot.dataprep")
    assert command[entrypoint - 2 : entrypoint + 1] == [
        "python",
        "-m",
        "dimos_lerobot.dataprep",
    ]
    assert run.call_args.kwargs["env"]["UV_PROJECT_ENVIRONMENT"]


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
