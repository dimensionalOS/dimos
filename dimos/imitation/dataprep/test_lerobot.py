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
from pathlib import Path
import subprocess

import pytest
import pytest_mock

from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig
from dimos.imitation.dataprep.lerobot import (
    inspect_lerobot_dataset,
    lerobot_project,
    run_lerobot_dataprep,
)


def test_conversion_runs_packaged_module_in_policy_project(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch.dict("os.environ", {"VIRTUAL_ENV": "/parent/.venv"})
    run = mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [],
            0,
            stdout=(
                f'[dataprep] wrote 1 episode\n{{"command":"build","path":"{tmp_path / "dataset"}"}}'
            ),
            stderr="",
        ),
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
    assert command[-3:] == ["python", "-m", "dimos_lerobot.dataprep"]
    assert "--python" not in command
    assert run.call_args.kwargs["cwd"] == project
    assert "VIRTUAL_ENV" not in run.call_args.kwargs["env"]
    assert run.call_args.kwargs["capture_output"] is True
    assert run.call_args.kwargs["text"] is True
    assert '"command":"build"' in run.call_args.kwargs["input"]
    assert json.loads(run.call_args.kwargs["input"])["config"]["source"] == str(
        Path("recording.db").resolve()
    )


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
        return_value=subprocess.CompletedProcess(
            [], 9, stdout="partial output", stderr="bad config"
        ),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    with pytest.raises(RuntimeError, match="status 9: bad config"):
        run_lerobot_dataprep(config)


def test_inspection_uses_the_same_isolated_entrypoint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: pytest_mock.MockerFixture
) -> None:
    monkeypatch.chdir(tmp_path)
    run = mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [], 0, stdout='{"command":"inspect","info":{"format":"lerobot"}}', stderr=""
        ),
    )

    assert inspect_lerobot_dataset(Path("dataset")) == {"format": "lerobot"}
    assert '"command":"inspect"' in run.call_args.kwargs["input"]
    assert json.loads(run.call_args.kwargs["input"])["path"] == str(tmp_path / "dataset")
