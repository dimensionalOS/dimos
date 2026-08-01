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

import pytest

from dimos.benchmark.spatial.pi_baseline import cli
from dimos.benchmark.spatial.pi_baseline.cli import main


@pytest.mark.parametrize("command", ["review", "report"])
def test_phase3_experiment_commands_remain_disabled(
    command: str, capsys: pytest.CaptureFixture[str]
) -> None:
    arguments = ["experiment", command, "experiment-1"]
    if command == "review":
        arguments.extend(
            ["--private-root", "private", "--reviewer", "reviewer", "--decision", "approved"]
        )
    else:
        arguments.extend(["--private-root", "private", "--review-decision", "decision.json"])
    assert main(arguments) == 1
    assert capsys.readouterr().err == "pi-baseline: operation unavailable\n"


def test_phase2_commands_require_explicit_bindings() -> None:
    with pytest.raises(SystemExit) as error:
        main(["experiment", "run", "experiment-1"])
    assert error.value.code == 2


def test_run_paired_is_not_a_production_command() -> None:
    with pytest.raises(SystemExit):
        main(["run-paired", "experiment-1"])


def test_session_view_needs_only_an_attempt_directory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, bool]] = []
    monkeypatch.setattr(
        cli,
        "view_attempt",
        lambda path, *, open_browser: calls.append((str(path), open_browser)) or 0,
    )

    assert main(["session", "view", "attempt-1"]) == 0
    assert calls == [("attempt-1", True)]


def test_session_view_error_is_bounded_and_does_not_echo_path(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    secret_path = str(tmp_path) + "/secret-session-content"
    assert main(["session", "view", secret_path]) == 1
    error = capsys.readouterr().err
    assert error == "pi-baseline: session viewer unavailable (invalid_evidence)\n"
    assert secret_path not in error
