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
from dimos.imitation.dataprep.lerobot import (
    inspect_lerobot_dataset,
    lerobot_project,
    run_lerobot_dataprep,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS

OPENYAM_PROFILE = "dimos.robot.manipulators.openyam.learning:OPENYAM_LEARNING_PROFILE"
NOT_A_PROFILE = object()


class _InvalidProfile:
    def dataprep_config(self) -> str:
        return "not a config"


INVALID_PROFILE = _InvalidProfile()


def test_dynamic_profile_supplies_schema_and_cli_supplies_paths(tmp_path: Path) -> None:
    config = _load_config(
        OPENYAM_PROFILE,
        tmp_path / "session.mcap",
        tmp_path / "dataset",
        "fill",
    )

    assert config.sync.rate_hz == 30.0
    assert config.observation["observation.state"].names == OPENYAM_JOINTS
    assert config.action["action"].names == OPENYAM_JOINTS
    assert config.source == str(tmp_path / "session.mcap")
    assert config.output.path == tmp_path / "dataset"
    assert config.quality.mode == "fill"


@pytest.mark.parametrize("reference", ["openyam", ":profile", "module:"])
def test_dynamic_profile_requires_module_attribute_syntax(reference: str, tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="module:attribute"):
        _load_config(reference, tmp_path / "session.mcap", None, None)


@pytest.mark.parametrize(
    ("reference", "message"),
    [
        (
            "dimos.imitation.dataprep.test_lerobot_cli:NOT_A_PROFILE",
            "does not implement DataPrepProfile",
        ),
        (
            "dimos.imitation.dataprep.test_lerobot_cli:INVALID_PROFILE",
            "did not return DataPrepConfig",
        ),
        (
            "dimos.imitation.dataprep.test_lerobot_cli:MISSING",
            "profile attribute 'MISSING' not found",
        ),
    ],
)
def test_dynamic_profile_rejects_invalid_objects(
    reference: str, message: str, tmp_path: Path
) -> None:
    with pytest.raises((TypeError, ValueError), match=message):
        _load_config(reference, tmp_path / "session.mcap", None, None)


def test_conversion_runs_packaged_module_in_policy_project(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch.dict("os.environ", {"VIRTUAL_ENV": "/parent/.venv"})
    run = mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [], 0, stdout=f'{{"command":"build","path":"{tmp_path / "dataset"}"}}', stderr=""
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
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    run = mocker.patch(
        "dimos.imitation.dataprep.lerobot.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [], 0, stdout='{"command":"inspect","info":{"format":"lerobot"}}', stderr=""
        ),
    )

    assert inspect_lerobot_dataset(tmp_path / "dataset") == {"format": "lerobot"}
    assert '"command":"inspect"' in run.call_args.kwargs["input"]
