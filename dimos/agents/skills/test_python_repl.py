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

import pytest
from pytest_mock import MockerFixture

from dimos.agents.skills.python_repl import PythonReplSkill, RecordingPathSpec
from dimos.memory.store.sqlite import SqliteStore


@pytest.fixture
def python_repl(tmp_path: Path, mocker: MockerFixture) -> Iterator[PythonReplSkill]:
    recording_path = tmp_path / "recording.db"
    with SqliteStore(path=str(recording_path)) as store:
        store.stream("numbers", int).append(7)

    skill = PythonReplSkill()
    recorder = mocker.Mock(spec=RecordingPathSpec)
    recorder.recording_path.return_value = str(recording_path)
    skill._recorder = recorder
    with skill:
        yield skill


def test_run_python_reads_recorded_streams(python_repl: PythonReplSkill) -> None:
    result = python_repl.run_python(
        "print(store.list_streams())\nprint(store.streams.numbers.last().data)"
    )

    assert result == "['numbers']\n7\n"


def test_run_python_starts_a_fresh_process_for_each_call(
    python_repl: PythonReplSkill,
) -> None:
    first = python_repl.run_python(
        "import builtins\nbuiltins.python_repl_probe = 42\nprint(builtins.python_repl_probe)"
    )
    second = python_repl.run_python(
        "import builtins\nprint(hasattr(builtins, 'python_repl_probe'))"
    )

    assert first == "42\n"
    assert second == "False\n"


def test_run_python_returns_stdout_and_traceback(python_repl: PythonReplSkill) -> None:
    result = python_repl.run_python("print('started')\nraise ValueError('boom')")

    assert result.startswith("started\nTraceback (most recent call last):")
    assert result.endswith("ValueError: boom\n")


def test_run_python_reports_when_code_prints_nothing(python_repl: PythonReplSkill) -> None:
    result = python_repl.run_python("answer = 6 * 7")

    assert result == "(no output — end your code with print())"


def test_run_python_caps_large_output(python_repl: PythonReplSkill) -> None:
    python_repl.config.max_output_chars = 5

    result = python_repl.run_python("print('abcdefghij')")

    assert result == "abcde\n... [truncated, 11 chars total]"
