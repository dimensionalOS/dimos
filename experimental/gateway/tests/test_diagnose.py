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

"""Launch steps and problems from structured records, and the dimos code that logs what they read."""

import ast
import builtins
import errno
import importlib
from pathlib import Path
import sqlite3
import traceback
from typing import Any

import pytest

from experimental.gateway.utils import diagnose
from experimental.gateway.utils.diagnose import problems, steps

ROOT = Path(__file__).parents[3]


def record(level: str = "info", event: str = "x", **extra: Any) -> dict[str, Any]:
    return {
        "timestamp": "t",
        "level": level,
        "logger": "l",
        "event": event,
        "extra": extra,
        "raw": "",
    }


def failure(error: BaseException) -> dict[str, Any]:
    """What dimos logs for an exception: structlog's format_exc_info puts the traceback in `exception`."""
    return record("error", "Error", exception="".join(traceback.format_exception(error)))


START = [
    record(event="Starting DimOS"),
    record(event="Building the blueprint"),
    record(event="Starting the modules"),
    record(event="Deployed module.", module="A"),
    record(event="Deployed module.", module="B"),
]


def states(found: list[dict[str, Any]]) -> list[str]:
    return [step["state"] for step in found]


def test_steps_follow_the_stages() -> None:
    starting = steps(START, "starting")
    assert [step["code"] for step in starting] == [
        "starting",
        "building",
        "starting_modules",
        "running",
    ]
    assert states(starting) == ["done", "done", "now", "todo"]
    assert starting[2]["data"] == {"deployed": 2, "total": None}
    assert states(steps(START, "running")) == ["done", "done", "done", "done"]
    assert steps(START, "stopped")[3]["code"] == "stopped"
    assert states(steps([], "starting")) == ["now", "todo", "todo", "todo"]
    assert states(steps([], "failed")) == ["failed", "todo", "todo", "todo"]
    assert states(steps(START[:2], "failed")) == ["done", "failed", "todo", "todo"]


def raised(error: BaseException, cause: BaseException | None = None) -> BaseException:
    try:
        try:
            if cause:
                raise cause
        except BaseException:
            raise error
        raise error
    except BaseException as caught:
        return caught


@pytest.mark.parametrize(
    "error, code",
    [
        (
            ModuleNotFoundError("No module named 'unitree_sdk2py'", name="unitree_sdk2py"),
            "missing_python_package",
        ),
        (OSError(errno.EADDRINUSE, "in use"), "port_in_use"),
        (OSError(errno.EHOSTUNREACH, "no route"), "host_unreachable"),
        (ConnectionRefusedError(errno.ECONNREFUSED, "refused"), "connection_refused"),
        (TimeoutError("timed out"), "timed_out"),
        (MemoryError(), "out_of_memory"),
        # behind a wrapper, as a worker's failure reaches the coordinator
        (
            raised(RuntimeError("Failed to deploy module"), cause=OSError(errno.EADDRINUSE, "x")),
            "port_in_use",
        ),
        (ValueError("something else"), "error"),
    ],
)
def test_exceptions_get_their_code(error: BaseException, code: str) -> None:
    assert problems([failure(error)])[0]["code"] == code


def test_a_missing_package_says_which() -> None:
    found = problems([failure(ModuleNotFoundError("No module named 'x'", name="x"))])[0]
    assert found["data"]["missing_module"] == "x" and found["message"] == "No module named 'x'"


def test_a_refusal_dimos_only_prints_is_the_output_last_line() -> None:
    """Bad arguments, an unknown blueprint, an unmet requirement: dimos prints them and logs nothing."""
    assert problems([]) == []
    output = (
        "$ dimos run nope\n14:56:51.285 [inf][imos/cli/commands/lifecycle.py] Starting DimOS\n"
        "Unknown blueprint or module: nope\nDid you mean one of these?\n  spot-replay\n"
    )
    assert diagnose.error_text([], output) == "Unknown blueprint or module: nope"


def test_a_sqlite_file_that_cant_open() -> None:
    try:
        sqlite3.connect("/nonexistent/dir/x.db")
    except sqlite3.OperationalError as error:
        if not getattr(error, "sqlite_errorname", None):
            pytest.skip("python < 3.11 has no sqlite_errorname")
        assert problems([failure(error)])[0]["code"] == "recording_unopenable"


def test_known_problems_win_else_the_last_three_errors() -> None:
    generic = [record("error", f"boom {i}") for i in range(5)]
    assert [p["message"] for p in problems(generic)] == ["boom 2", "boom 3", "boom 4"]
    known = failure(MemoryError())
    assert [p["code"] for p in problems([*generic, known, known])] == ["out_of_memory"]
    assert problems(START) == []


def test_exception_classes_it_names_exist() -> None:
    for name in diagnose.EXCEPTION_CODES:
        module, _, attribute = name.rpartition(".")
        if not module:
            assert hasattr(builtins, attribute), name
            continue
        try:
            kind = getattr(importlib.import_module(module), attribute)
        except ModuleNotFoundError:
            continue  # a third-party package that isn't installed here (torch, unitree_webrtc_connect)
        # how a traceback names a class
        assert f"{kind.__module__}.{kind.__qualname__}" == name, name


def logged_events() -> set[str]:
    """Every message passed to a logger call in dimos's own code."""
    found: set[str] = set()
    for file in ROOT.rglob("*.py"):
        if file.relative_to(ROOT).parts[0] == "experimental" or file.name.startswith("test_"):
            continue
        text = file.read_text()
        if not any(event in text for event in diagnose.STAGE_EVENTS):
            continue
        for node in ast.walk(ast.parse(text)):
            if (
                isinstance(node, ast.Call)
                and getattr(node.func, "attr", None) in ("info", "error", "warning")
                and node.args
                and isinstance(node.args[0], ast.Constant)
            ):
                found.add(node.args[0].value)
    return found


def test_dimos_logs_the_stage_messages_read_here() -> None:
    assert set(diagnose.STAGE_EVENTS) <= logged_events()


def test_error_text() -> None:
    assert diagnose.error_text([], "$ dimos run x\n\x1b[31mError: nope\x1b[0m\n\n") == "Error: nope"
    assert diagnose.error_text([], "$ dimos run x\n") == "dimos exited during startup"
    assert diagnose.error_text(problems([failure(MemoryError("big"))]), "") == "big"


def test_a_port_in_use_says_which_port_and_who_holds_it() -> None:
    import asyncio
    import os
    import socket

    holder = socket.socket()
    holder.bind(("127.0.0.1", 0))
    holder.listen()
    port = holder.getsockname()[1]

    async def bind() -> BaseException:
        try:
            await asyncio.start_server(lambda *_: None, "127.0.0.1", port)
        except OSError as error:
            return error
        raise AssertionError("bound a port that's taken")

    try:
        error = asyncio.run(bind())
        found = problems([failure(error)])[0]
    finally:
        holder.close()
    assert found["code"] == "port_in_use" and found["data"]["exception_code"] == "EADDRINUSE"
    assert found["data"]["port"] == port and found["data"]["holder_pid"] == os.getpid()
    assert "python" in (found["data"]["holder_command"] or "")
    # an address the error doesn't name: no port, nothing looked up
    unnamed = failure(OSError(errno.EADDRINUSE, "in use"))
    assert "port" not in problems([unnamed])[0]["data"]
    # logged where it happened and again where it was caught: one problem, the one that names the port
    assert [p["data"].get("port") for p in problems([unnamed, failure(error), unnamed])] == [port]
