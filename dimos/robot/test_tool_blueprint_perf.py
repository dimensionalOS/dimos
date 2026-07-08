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

from __future__ import annotations

from pathlib import Path
import sys

from dimos.robot.tool_blueprint_perf import ToolArgs, build_command, main, run_blueprint_perf


def _make_args(tmp_path: Path, **kwargs: object) -> ToolArgs:
    return ToolArgs(
        blueprint=kwargs.get("blueprint", "unitree-go2"),  # type: ignore[arg-type]
        mode=kwargs.get("mode", "replay"),  # type: ignore[arg-type]
        viewer=kwargs.get("viewer", "none"),  # type: ignore[arg-type]
        duration_seconds=kwargs.get("duration_seconds", 0.2),  # type: ignore[arg-type]
        warmup_seconds=kwargs.get("warmup_seconds", 0.05),  # type: ignore[arg-type]
        output=kwargs.get("output", tmp_path / "result.json"),  # type: ignore[arg-type]
        command_timeout_seconds=kwargs.get("command_timeout_seconds", 0.2),  # type: ignore[arg-type]
        stdout_tail_lines=kwargs.get("stdout_tail_lines", 10),  # type: ignore[arg-type]
        stderr_tail_lines=kwargs.get("stderr_tail_lines", 10),  # type: ignore[arg-type]
    )


def test_build_command_replay() -> None:
    assert build_command(blueprint="unitree-go2", mode="replay", viewer="none") == [
        sys.executable,
        "-m",
        "dimos.robot.cli.dimos",
        "--replay",
        "--viewer=none",
        "run",
        "unitree-go2",
    ]


def test_build_command_simulation() -> None:
    assert build_command(blueprint="unitree-g1-basic-sim", mode="simulation", viewer="rerun") == [
        sys.executable,
        "-m",
        "dimos.robot.cli.dimos",
        "--simulation=true",
        "--viewer=rerun",
        "run",
        "unitree-g1-basic-sim",
    ]


def test_build_command_normal() -> None:
    assert build_command(blueprint="unitree-go2", mode="normal", viewer="none") == [
        sys.executable,
        "-m",
        "dimos.robot.cli.dimos",
        "--viewer=none",
        "run",
        "unitree-go2",
    ]


def test_json_schema_shape(tmp_path: Path, mocker) -> None:
    code = "import sys, time; print('ok'); print('warn', file=sys.stderr); time.sleep(0.2)"
    mocker.patch(
        "dimos.robot.tool_blueprint_perf.build_command",
        return_value=[sys.executable, "-c", code],
    )
    args = _make_args(
        tmp_path,
        blueprint="unitree-go2",
        mode="normal",
        duration_seconds=0.2,
        warmup_seconds=0.05,
    )

    result = run_blueprint_perf(args)

    assert set(result) == {"metadata", "run", "performance", "logs"}
    assert result["run"]["blueprint"] == "unitree-go2"
    assert result["run"]["mode"] == "normal"
    assert "wall_clock_seconds" in result["performance"]
    assert "stdout" in result["logs"]
    assert "stderr" in result["logs"]
    assert args.output.exists()


def test_log_tail_truncation(tmp_path: Path, mocker) -> None:
    code = (
        "import sys, time\n"
        "for i in range(6):\n"
        "    print(f'out-{i}')\n"
        "    print(f'err-{i}', file=sys.stderr)\n"
        "    sys.stdout.flush()\n"
        "    sys.stderr.flush()\n"
        "time.sleep(0.2)\n"
    )
    mocker.patch(
        "dimos.robot.tool_blueprint_perf.build_command",
        return_value=[sys.executable, "-c", code],
    )
    args = _make_args(tmp_path, stdout_tail_lines=3, stderr_tail_lines=2)

    result = run_blueprint_perf(args)

    assert result["logs"]["stdout"]["tail_lines"] == ["out-3", "out-4", "out-5"]
    assert result["logs"]["stdout"]["truncated"] is True
    assert result["logs"]["stderr"]["tail_lines"] == ["err-4", "err-5"]
    assert result["logs"]["stderr"]["truncated"] is True


def test_startup_success_heuristic_for_bounded_run(tmp_path: Path, mocker) -> None:
    code = "import time; print('started'); time.sleep(0.3)"
    mocker.patch(
        "dimos.robot.tool_blueprint_perf.build_command",
        return_value=[sys.executable, "-c", code],
    )
    args = _make_args(tmp_path, duration_seconds=0.15, warmup_seconds=0.05)

    result = run_blueprint_perf(args)

    assert result["run"]["startup_succeeded"] is True
    assert result["run"]["completed_bounded_run"] is True
    assert result["run"]["returncode"] is not None


def test_process_failure_before_warmup(tmp_path: Path, mocker) -> None:
    code = "import sys; print('boom', file=sys.stderr); sys.exit(7)"
    mocker.patch(
        "dimos.robot.tool_blueprint_perf.build_command",
        return_value=[sys.executable, "-c", code],
    )
    args = _make_args(tmp_path, duration_seconds=0.3, warmup_seconds=0.2)

    result = run_blueprint_perf(args)

    assert result["run"]["startup_succeeded"] is False
    assert result["run"]["completed_bounded_run"] is False
    assert result["run"]["exit_reason"] == "exited_before_warmup"
    assert result["run"]["returncode"] == 7
    assert "boom" in result["logs"]["stderr"]["tail_lines"]


def test_main_returns_nonzero_on_startup_failure(tmp_path: Path, mocker) -> None:
    mocker.patch(
        "dimos.robot.tool_blueprint_perf.run_blueprint_perf",
        return_value={
            "metadata": {},
            "run": {"startup_succeeded": False},
            "performance": {},
            "logs": {},
        },
    )

    rc = main(
        [
            "--blueprint",
            "unitree-go2",
            "--output",
            str(tmp_path / "result.json"),
        ]
    )

    assert rc == 1
