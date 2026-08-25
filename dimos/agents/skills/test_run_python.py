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
from pathlib import Path
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.agents.skills.run_python import MAX_TIMEOUT_SECONDS, RunPythonSkill
from dimos.memory.module import Recorder
from dimos.memory.store.sqlite import SqliteStore


@pytest.fixture
def run_python_skill(tmp_path: Path, mocker: MockerFixture) -> Iterator[RunPythonSkill]:
    recording_path = tmp_path / "recording.db"
    with SqliteStore(path=str(recording_path)) as store:
        store.stream("numbers", int).append(7)

    skill = RunPythonSkill()
    recorder = mocker.Mock(spec=Recorder)
    recorder.recording_path.return_value = str(recording_path)
    skill._recorder = recorder
    with skill:
        yield skill


def test_run_python_reads_recorded_streams(run_python_skill: RunPythonSkill) -> None:
    result = run_python_skill.run_python(
        "print(store.list_streams())\nprint(store.streams.numbers.last().data)"
    )

    assert result == "['numbers']\n7\n"


def test_run_python_starts_a_fresh_process_for_each_call(
    run_python_skill: RunPythonSkill,
) -> None:
    first = run_python_skill.run_python(
        "import builtins\nbuiltins.run_python_probe = 42\nprint(builtins.run_python_probe)"
    )
    second = run_python_skill.run_python(
        "import builtins\nprint(hasattr(builtins, 'run_python_probe'))"
    )

    assert first == "42\n"
    assert second == "False\n"


def test_run_python_returns_stdout_and_traceback(run_python_skill: RunPythonSkill) -> None:
    result = run_python_skill.run_python("print('started')\nraise ValueError('boom')")

    assert result.startswith("started\nTraceback (most recent call last):")
    assert result.endswith("ValueError: boom\n")


def test_run_python_reports_when_code_prints_nothing(run_python_skill: RunPythonSkill) -> None:
    result = run_python_skill.run_python("answer = 6 * 7")

    assert result == "(no output - did you forget to print()?)"


def test_run_python_caps_large_output(run_python_skill: RunPythonSkill) -> None:
    run_python_skill.config.max_output_chars = 5

    result = run_python_skill.run_python("print('abcdefghij')")

    assert result == "abcde\n... [truncated, 11 chars total]"


@pytest.mark.parametrize("timeout", [0.0, MAX_TIMEOUT_SECONDS + 0.1])
def test_run_python_rejects_timeout_outside_bounds(
    run_python_skill: RunPythonSkill,
    timeout: float,
) -> None:
    with pytest.raises(ValueError, match="timeout must be greater than 0 and at most 100"):
        run_python_skill.run_python("print('hello')", timeout=timeout)


def test_run_python_reports_execution_timeout(
    run_python_skill: RunPythonSkill,
    mocker: MockerFixture,
) -> None:
    run = mocker.patch(
        "dimos.agents.skills.run_python.subprocess.run",
        side_effect=subprocess.TimeoutExpired(cmd="python", timeout=2.0),
    )

    result = run_python_skill.run_python("print('hello')", timeout=2.0)

    assert result == "(execution timed out after 2 seconds)"
    assert run.call_args.kwargs["timeout"] == 2.0
