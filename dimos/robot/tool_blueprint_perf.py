#!/usr/bin/env python3

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

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Literal

import psutil

Mode = Literal["replay", "simulation", "normal"]

DEFAULT_BLUEPRINT = "unitree-go2"
DEFAULT_MODE: Mode = "replay"
DEFAULT_VIEWER = "none"
DEFAULT_DURATION_SECONDS = 10.0
DEFAULT_WARMUP_SECONDS = 3.0
DEFAULT_COMMAND_TIMEOUT_SECONDS = 10.0
DEFAULT_STDOUT_TAIL_LINES = 50
DEFAULT_STDERR_TAIL_LINES = 50
DEFAULT_OUTPUT = Path("blueprint_perf.json")


@dataclass(frozen=True)
class ToolArgs:
    blueprint: str
    mode: Mode
    simulation_backend: str
    viewer: str
    duration_seconds: float
    warmup_seconds: float
    output: Path
    command_timeout_seconds: float
    stdout_tail_lines: int
    stderr_tail_lines: int


class LineTail:
    """Keep only the most recent lines from a stream."""

    def __init__(self, max_lines: int) -> None:
        self._max_lines = max_lines
        self._lines: deque[str] = deque(maxlen=max_lines)
        self._total_lines = 0
    def add(self, line: str) -> None:
        self._total_lines += 1
        self._lines.append(line.rstrip("\n"))

    def snapshot(self) -> dict[str, Any]:
        return {
            "tail_lines": list(self._lines),
            "tail_line_count": len(self._lines),
            "total_line_count": self._total_lines,
            "truncated": self._total_lines > self._max_lines,
        }


def build_command(*, blueprint: str, mode: Mode, simulation_backend: str, viewer: str) -> list[str]:
    cmd = [sys.executable, "-m", "dimos.robot.cli.dimos"]
    if mode == "replay":
        cmd.extend(["--replay", f"--viewer={viewer}", "run", blueprint])
    elif mode == "simulation":
        cmd.extend([f"--simulation={simulation_backend}", f"--viewer={viewer}", "run", blueprint])
    else:
        cmd.extend([f"--viewer={viewer}", "run", blueprint])
    return cmd


def _nonnegative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise ValueError("must be >= 0")
    return parsed


def _positive_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise ValueError("must be finite and > 0")
    return parsed


def _nonnegative_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0:
        raise ValueError("must be finite and >= 0")
    return parsed


def parse_args(argv: Sequence[str] | None = None) -> ToolArgs:
    import argparse

    parser = argparse.ArgumentParser(
        description="Run a bounded DimOS blueprint CLI smoke/performance subprocess."
    )
    parser.add_argument("--blueprint", default=DEFAULT_BLUEPRINT, help="Blueprint to run.")
    parser.add_argument(
        "--mode",
        choices=("replay", "simulation", "normal"),
        default=DEFAULT_MODE,
        help="How to construct the DimOS command.",
    )
    parser.add_argument(
        "--simulation-backend",
        default="mujoco",
        help="Simulation backend name to pass when --mode=simulation.",
    )
    parser.add_argument("--viewer", default=DEFAULT_VIEWER, help="Viewer mode to pass to DimOS.")
    parser.add_argument(
        "--duration-seconds",
        type=_positive_finite_float,
        default=DEFAULT_DURATION_SECONDS,
        help="How long to let the command run before terminating it.",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=_nonnegative_finite_float,
        default=DEFAULT_WARMUP_SECONDS,
        help="How long the process must stay alive to count startup as successful.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Where to write the structured JSON result.",
    )
    parser.add_argument(
        "--command-timeout-seconds",
        type=_nonnegative_finite_float,
        default=DEFAULT_COMMAND_TIMEOUT_SECONDS,
        help="How long to wait after terminate() before escalating to kill().",
    )
    parser.add_argument(
        "--stdout-tail-lines",
        type=_nonnegative_int,
        default=DEFAULT_STDOUT_TAIL_LINES,
        help="How many stdout lines to keep in the JSON tail.",
    )
    parser.add_argument(
        "--stderr-tail-lines",
        type=_nonnegative_int,
        default=DEFAULT_STDERR_TAIL_LINES,
        help="How many stderr lines to keep in the JSON tail.",
    )

    try:
        ns = parser.parse_args(argv)
    except ValueError as exc:
        parser.error(str(exc))
    return ToolArgs(
        blueprint=ns.blueprint,
        mode=ns.mode,
        simulation_backend=ns.simulation_backend,
        viewer=ns.viewer,
        duration_seconds=ns.duration_seconds,
        warmup_seconds=ns.warmup_seconds,
        output=ns.output,
        command_timeout_seconds=ns.command_timeout_seconds,
        stdout_tail_lines=ns.stdout_tail_lines,
        stderr_tail_lines=ns.stderr_tail_lines,
    )


def _terminate_process(proc: subprocess.Popen[str], timeout_seconds: float) -> tuple[str, bool]:
    proc.terminate()
    try:
        proc.wait(timeout=timeout_seconds)
        return ("terminated", False)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        return ("killed_after_timeout", True)


def _safe_cpu_times(proc: psutil.Process) -> tuple[float | None, float | None]:
    try:
        cpu = proc.cpu_times()
    except (psutil.Error, OSError):
        return (None, None)
    return (cpu.user, cpu.system)


def _tail_file(stream: Any, max_lines: int) -> dict[str, Any]:
    tail = LineTail(max_lines)
    stream.seek(0)
    for line in stream:
        tail.add(line)
    return tail.snapshot()


def run_blueprint_perf(args: ToolArgs) -> dict[str, Any]:
    command = build_command(
        blueprint=args.blueprint,
        mode=args.mode,
        simulation_backend=args.simulation_backend,
        viewer=args.viewer,
    )
    started_at = datetime.now(UTC)
    monotonic_start = time.monotonic()
    with (
        tempfile.TemporaryFile(mode="w+t") as stdout_file,
        tempfile.TemporaryFile(mode="w+t") as stderr_file,
        subprocess.Popen(
            command,
            stdout=stdout_file,
            stderr=stderr_file,
            stdin=subprocess.DEVNULL,
            text=True,
        ) as proc,
    ):
        ps_proc = psutil.Process(proc.pid)
        cpu_user_start, cpu_system_start = _safe_cpu_times(ps_proc)
        last_cpu_user = cpu_user_start
        last_cpu_system = cpu_system_start
        peak_rss_bytes = 0
        survived_past_warmup = False
        warmup_reached_at: float | None = None
        exit_reason = "completed_bounded_duration"
        terminated = False
        escalated_to_kill = False

        try:
            while True:
                elapsed = time.monotonic() - monotonic_start
                returncode = proc.poll()

                if returncode is not None:
                    if elapsed < args.warmup_seconds:
                        exit_reason = "exited_before_warmup"
                    else:
                        survived_past_warmup = True
                        warmup_reached_at = args.warmup_seconds
                        exit_reason = "exited_after_warmup"
                    break

                if not survived_past_warmup and elapsed >= args.warmup_seconds:
                    survived_past_warmup = True
                    warmup_reached_at = elapsed

                try:
                    rss = ps_proc.memory_info().rss
                    peak_rss_bytes = max(peak_rss_bytes, rss)
                except (psutil.Error, OSError):
                    pass
                cpu_user_sample, cpu_system_sample = _safe_cpu_times(ps_proc)
                if cpu_user_sample is not None:
                    last_cpu_user = cpu_user_sample
                if cpu_system_sample is not None:
                    last_cpu_system = cpu_system_sample

                if elapsed >= args.duration_seconds:
                    cpu_user_sample, cpu_system_sample = _safe_cpu_times(ps_proc)
                    if cpu_user_sample is not None:
                        last_cpu_user = cpu_user_sample
                    if cpu_system_sample is not None:
                        last_cpu_system = cpu_system_sample
                    exit_reason, escalated_to_kill = _terminate_process(
                        proc, args.command_timeout_seconds
                    )
                    terminated = True
                    break

                time.sleep(0.05)
        finally:
            if proc.poll() is None:
                cpu_user_sample, cpu_system_sample = _safe_cpu_times(ps_proc)
                if cpu_user_sample is not None:
                    last_cpu_user = cpu_user_sample
                if cpu_system_sample is not None:
                    last_cpu_system = cpu_system_sample
                exit_reason, escalated_to_kill = _terminate_process(
                    proc, args.command_timeout_seconds
                )
                terminated = True
            else:
                cpu_user_sample, cpu_system_sample = _safe_cpu_times(ps_proc)
                if cpu_user_sample is not None:
                    last_cpu_user = cpu_user_sample
                if cpu_system_sample is not None:
                    last_cpu_system = cpu_system_sample
                proc.wait()

        ended_at = datetime.now(UTC)
        wall_clock_seconds = time.monotonic() - monotonic_start
        cpu_user_end, cpu_system_end = _safe_cpu_times(ps_proc)
        if cpu_user_end is None:
            cpu_user_end = last_cpu_user
        if cpu_system_end is None:
            cpu_system_end = last_cpu_system
        pid = proc.pid
        returncode = proc.returncode
        stdout_snapshot = _tail_file(stdout_file, args.stdout_tail_lines)
        stderr_snapshot = _tail_file(stderr_file, args.stderr_tail_lines)

    startup_succeeded = survived_past_warmup
    completed_bounded_run = survived_past_warmup and terminated

    cpu_user_seconds = None
    if cpu_user_start is not None and cpu_user_end is not None:
        cpu_user_seconds = max(0.0, cpu_user_end - cpu_user_start)

    cpu_system_seconds = None
    if cpu_system_start is not None and cpu_system_end is not None:
        cpu_system_seconds = max(0.0, cpu_system_end - cpu_system_start)

    result = {
        "metadata": {
            "tool": "dimos.robot.tool_blueprint_perf",
            "generated_at": ended_at.isoformat(),
            "python_executable": sys.executable,
        },
        "run": {
            "command": command,
            "blueprint": args.blueprint,
            "mode": args.mode,
            "simulation_backend": args.simulation_backend,
            "viewer": args.viewer,
            "pid": pid,
            "started_at": started_at.isoformat(),
            "ended_at": ended_at.isoformat(),
            "duration_seconds": args.duration_seconds,
            "warmup_seconds": args.warmup_seconds,
            "command_timeout_seconds": args.command_timeout_seconds,
            "returncode": returncode,
            "exit_reason": exit_reason,
            "startup_succeeded": startup_succeeded,
            "completed_bounded_run": completed_bounded_run,
            "survived_past_warmup": survived_past_warmup,
            "warmup_reached_at_seconds": warmup_reached_at,
            "escalated_to_kill": escalated_to_kill,
        },
        "performance": {
            "wall_clock_seconds": wall_clock_seconds,
            "peak_rss_bytes": peak_rss_bytes,
            "cpu_user_seconds": cpu_user_seconds,
            "cpu_system_seconds": cpu_system_seconds,
        },
        "logs": {
            "stdout": stdout_snapshot,
            "stderr": stderr_snapshot,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_blueprint_perf(args)
    print(json.dumps(result["run"], indent=2))
    return 0 if result["run"]["startup_succeeded"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
