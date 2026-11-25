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
import sys

import pytest
import pytest_mock

from dimos.imitation.dataprep.formats.lerobot.adapter import (
    inspect_lerobot_dataset,
    run_lerobot_dataprep,
)
from dimos.imitation.dataprep.formats.lerobot.protocol import ErrorResult
from dimos.imitation.dataprep.schema import DataPrepConfig, OutputConfig
from dimos.utils.data import get_project_root


def test_conversion_runs_module_in_checkout_policy_project(
    tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch.dict(
        "os.environ",
        {
            "VIRTUAL_ENV": "/parent/.venv",
            "UV_PYTHON": "3.10",
            "UV_PROJECT_ENVIRONMENT": "/parent/env",
        },
    )
    run = mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter._run_with_progress",
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
    project = get_project_root() / "native/python/lerobot"
    assert command[:3] == ["uv", "run", "--frozen"]
    assert command[-3:] == ["python", "-m", "dimos_lerobot.dataprep"]
    assert run.call_args.kwargs["cwd"] == project
    assert "VIRTUAL_ENV" not in run.call_args.kwargs["env"]
    assert "UV_PYTHON" not in run.call_args.kwargs["env"]
    assert run.call_args.kwargs["env"]["UV_PROJECT_ENVIRONMENT"] != "/parent/env"
    assert json.loads(run.call_args.kwargs["input"])["config"]["source"] == str(
        Path("recording.db").resolve()
    )


def test_conversion_reports_missing_uv(tmp_path: Path, mocker: pytest_mock.MockerFixture) -> None:
    mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter.subprocess.run",
        side_effect=FileNotFoundError("uv"),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    with pytest.raises(RuntimeError, match="uv is required"):
        run_lerobot_dataprep(config)


@pytest.mark.parametrize(
    ("stdout", "stderr", "detail"),
    [
        ("partial output", "bad config", "bad config"),
        ("progress\nfailed on stdout\n", "", "failed on stdout"),
        ("", "", "no diagnostic output"),
        ("", "Downloading torch\nerror: download failed\n", "error: download failed"),
        (
            "",
            (
                "Creating virtual environment\n"
                "  \u00d7 No solution found when resolving dependencies:\n"
                "  ╰─▶ Because required-package was not found in the cache\n"
                "      and your project depends on required-package, we can\n"
                "      conclude that your project's requirements are unsatisfiable.\n"
                "\n"
                "      hint: Packages were unavailable because the network was disabled. When\n"
                "      the network is disabled, registry packages may only be read from the\n"
                "      cache.\n"
            ),
            (
                "\u00d7 No solution found when resolving dependencies: "
                "╰─▶ Because required-package was not found in the cache "
                "and your project depends on required-package, we can "
                "conclude that your project's requirements are unsatisfiable. "
                "hint: Packages were unavailable because the network was disabled. When "
                "the network is disabled, registry packages may only be read from the cache."
            ),
        ),
        (
            "",
            "Downloading torch\nerror: Failed to download torch\n  Caused by: Connection refused\n",
            "error: Failed to download torch Caused by: Connection refused",
        ),
        (
            "",
            (
                "Installed 157 packages\n"
                "Traceback (most recent call last):\n"
                '  File "dataprep.py", line 191, in main\n'
                "    request = REQUEST_ADAPTER.validate_json(sys.stdin.read())\n"
                "pydantic_core.ValidationError: 1 validation error for BuildRequest\n"
                "config.output.path\n"
                "  Field required [type=missing, input_value={}, input_type=dict]\n"
            ),
            (
                "pydantic_core.ValidationError: 1 validation error for BuildRequest "
                "config.output.path Field required [type=missing, input_value={}, input_type=dict]"
            ),
        ),
        (
            ErrorResult(error="No successful episodes.\nUse explicit ranges.").model_dump_json(),
            "Creating virtual environment\nDownloading torch\nInstalled 157 packages\n",
            "No successful episodes. Use explicit ranges.",
        ),
    ],
)
def test_conversion_reports_child_process_diagnostics(tmp_path, mocker, stdout, stderr, detail):
    mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter._run_with_progress",
        return_value=subprocess.CompletedProcess([], 9, stdout=stdout, stderr=stderr),
    )
    config = DataPrepConfig(
        source="recording.db",
        output=OutputConfig(format="lerobot", path=tmp_path / "dataset"),
    )

    with pytest.raises(RuntimeError) as caught:
        run_lerobot_dataprep(config)

    assert str(caught.value) == f"LeRobot dataprep exited with status 9: {detail}"


def test_inspection_uses_the_same_isolated_entrypoint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: pytest_mock.MockerFixture
) -> None:
    monkeypatch.chdir(tmp_path)
    run = mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter._run_with_progress",
        return_value=subprocess.CompletedProcess(
            [], 0, stdout='{"command":"inspect","info":{"format":"lerobot"}}', stderr=""
        ),
    )

    assert inspect_lerobot_dataset(Path("dataset")) == {"format": "lerobot"}
    assert '"command":"inspect"' in run.call_args.kwargs["input"]
    assert json.loads(run.call_args.kwargs["input"])["path"] == str(tmp_path / "dataset")


@pytest.mark.parametrize("cached", [False, True])
def test_inspect_reports_activity_and_streams_progress_before_child_exits(
    tmp_path, mocker, capsys, cached
):
    environment = tmp_path / "environment"
    environment.mkdir()
    if cached:
        (environment / "pyvenv.cfg").touch()
    mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter.isolated_python_environment",
        return_value={"UV_PROJECT_ENVIRONMENT": str(environment)},
    )
    acknowledgement = tmp_path / "progress-seen"
    script = """
import json
from pathlib import Path
import sys
import time
request = json.load(sys.stdin)
print("Downloading test package", file=sys.stderr, flush=True)
acknowledgement = Path(sys.argv[1])
deadline = time.monotonic() + 5
while not acknowledgement.exists() and time.monotonic() < deadline:
    time.sleep(0.01)
assert acknowledgement.exists(), "Progress was not forwarded while the child was running"
print(json.dumps({"command": "inspect", "info": {"path": request["path"]}}))
"""
    mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter.isolated_python_run_command",
        return_value=[sys.executable, "-c", script, str(acknowledgement)],
    )
    original_write = sys.stderr.write

    def acknowledge_progress(text):
        if "Downloading test package" in text:
            acknowledgement.touch()
        return original_write(text)

    mocker.patch.object(sys.stderr, "write", side_effect=acknowledge_progress)

    assert inspect_lerobot_dataset(tmp_path) == {"path": str(tmp_path)}

    output = capsys.readouterr()
    assert output.out == ""
    assert "Inspecting LeRobot dataset..." in output.err
    assert "Preparing LeRobot" not in output.err
    assert ("several GB" in output.err) is (not cached)
    assert "Downloading test package" in output.err


def test_streamed_installer_failure_retains_diagnostic_summary(tmp_path, mocker, capsys):
    mocker.patch(
        "dimos.imitation.dataprep.formats.lerobot.adapter.isolated_python_run_command",
        return_value=[
            sys.executable,
            "-c",
            (
                "import sys; sys.stdin.read(); "
                "sys.stderr.write('Downloading package\\nerror: download failed\\n'); sys.exit(9)"
            ),
        ],
    )

    with pytest.raises(RuntimeError) as caught:
        inspect_lerobot_dataset(tmp_path)

    assert str(caught.value) == "LeRobot dataprep exited with status 9: error: download failed"
    output = capsys.readouterr()
    assert "Downloading package" in output.err
    assert "error: download failed" in output.err
    assert output.out == ""
