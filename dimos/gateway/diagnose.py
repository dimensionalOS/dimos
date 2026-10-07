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

"""How far a launch got and what went wrong, as stable codes with data, read from its structured log records.

dimos, unmodified, logs its startup as `Starting DimOS`, `Building the blueprint`, `Starting the modules`, one
`Deployed module.` per module and `Blueprint started`; those messages are the stages here (test_diagnose checks dimos
still logs each one). An exception is logged with its traceback (`exception`, or `traceback_lines` from the
uncaught-exception hook): its classes and errno are read from that text, by their type names, never the message's
wording. Refusals dimos only prints (bad arguments, an unknown blueprint, an unmet requirement) have no record: they
reach the launch as its output's last line. The words (and the fixes) are Desktop's, per code.
"""

from __future__ import annotations

import errno
import re
import subprocess
import time
from typing import Any, Literal

import psutil

StepCode = Literal["starting", "building", "starting_modules", "running", "stopped"]
StepState = Literal["done", "now", "todo", "failed"]
ProblemCode = Literal[
    "bad_arguments",
    "unknown_blueprint",
    "requirement_unmet",
    "missing_python_package",
    "robot_ip_missing",
    "robot_unreachable",
    "replay_streams_missing",
    "lfs_data_missing",
    "port_in_use",
    "host_unreachable",
    "connection_refused",
    "timed_out",
    "recording_unopenable",
    "out_of_memory",
    "gpu_out_of_memory",
    "error",
]

# the startup stages a step stands for, in order (the last, running, is dimos's run registry entry)
STAGES: tuple[StepCode, ...] = ("starting", "building", "starting_modules")

# the message dimos logs at each stage -> the stage (module_deployed and started aren't steps of their own)
STAGE_EVENTS: dict[str, str] = {
    "Starting DimOS": "starting",
    "Building the blueprint": "building",
    "Starting the modules": "starting_modules",
    "Deployed module.": "module_deployed",
    "Blueprint started": "started",
}

# an exception class in a record's traceback -> its code
EXCEPTION_CODES: dict[str, ProblemCode] = {
    "ModuleNotFoundError": "missing_python_package",
    "unitree_webrtc_connect.unitree_auth.LocalSignalingPortError": "robot_unreachable",
    "ConnectionRefusedError": "connection_refused",
    "TimeoutError": "timed_out",
    "MemoryError": "out_of_memory",
    "torch.OutOfMemoryError": "gpu_out_of_memory",
}

# an errno name an OSError's text carries (`[Errno 48] ...`), or SQLite's "can't open" -> its code
ERROR_CODES: dict[str, ProblemCode] = {
    "EADDRINUSE": "port_in_use",
    "EHOSTUNREACH": "host_unreachable",
    "ENETUNREACH": "host_unreachable",
    "ECONNREFUSED": "connection_refused",
    "ETIMEDOUT": "timed_out",
    "SQLITE_CANTOPEN": "recording_unopenable",
}

# record fields that are about the log call, not the problem
NOT_DATA = {
    "func_name",
    "lineno",
    "exception",
    "traceback_lines",
    "exception_type",
    "exception_message",
}

# a traceback's exception line: `module.Class: message` or `Class` (Python's own format, not dimos's)
EXCEPTION_LINE = re.compile(
    r"^([A-Za-z_][\w.]*(?:Error|Exception|Exit|Interrupt|Warning)\w*)(?::\s?(.*))?$"
)
ERRNO = re.compile(r"\[Errno (\d+)\]")
MISSING_MODULE = re.compile(r"No module named '([^']+)'")


def exception_info(extra: dict[str, Any]) -> dict[str, Any]:
    """From a record's traceback text: `chain` (the exception classes, outermost last, as Python prints them),
    `message` (the last one's), `exception_code` (an errno name, or SQLITE_CANTOPEN) and `missing_module`."""
    text = extra.get("exception")
    if not isinstance(text, str):
        lines = extra.get("traceback_lines")
        text = "".join(lines) if isinstance(lines, list) else ""
    chain: list[str] = []
    message = None
    code = None
    missing = None
    for line in text.splitlines():
        found = EXCEPTION_LINE.match(line)
        if not found:
            continue
        chain.append(found.group(1))
        message = found.group(2) or ""
        number = ERRNO.search(message)
        if number and int(number.group(1)) in errno.errorcode:
            code = errno.errorcode[int(number.group(1))]
        if (
            found.group(1).endswith("OperationalError")
            and "unable to open database file" in message
        ):
            code = "SQLITE_CANTOPEN"
        module = MISSING_MODULE.search(message)
        if found.group(1) == "ModuleNotFoundError" and module:
            missing = module.group(1)
    if extra.get("exception_type") and not chain:
        chain = [str(extra["exception_type"])]
        message = extra.get("exception_message")
    return {"chain": chain, "message": message, "exception_code": code, "missing_module": missing}


ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def stage(record: dict[str, Any]) -> str | None:
    return STAGE_EVENTS.get(str(record["event"]))


def steps(records: list[dict[str, Any]], phase: str) -> list[dict[str, Any]]:
    """starting, building, starting_modules (with how many modules started; dimos doesn't log how many it will
    start), then running or stopped; each done, now, todo or failed."""
    reached: int | None = None
    deployed, total = 0, None
    for record in records:
        found = stage(record)
        if found in STAGES:
            reached = max(reached or 0, STAGES.index(found))
        if found == "module_deployed":
            deployed += 1
    result = []
    for index, code in enumerate(STAGES):
        if phase in ("running", "stopping", "stopped"):
            state: StepState = "done"
        elif reached is None:
            state = "todo" if index else "failed" if phase == "failed" else "now"
        elif index < reached:
            state = "done"
        elif index == reached:
            state = "failed" if phase == "failed" else "now"
        else:
            state = "todo"
        data = {"deployed": deployed, "total": total} if code == "starting_modules" else {}
        result.append({"code": code, "state": state, "data": data})
    last: StepCode = "stopped" if phase == "stopped" else "running"
    result.append(
        {
            "code": last,
            "state": "done" if phase in ("running", "stopping", "stopped") else "todo",
            "data": {},
        }
    )
    return result


def classify(info: dict[str, Any]) -> ProblemCode:
    for name in info["chain"]:
        if name in EXCEPTION_CODES:
            return EXCEPTION_CODES[name]
    return ERROR_CODES.get(str(info["exception_code"]), "error")


# the address an OSError names, `('127.0.0.1', 3030)` (asyncio's and socket's bind errors print the address tuple)
ADDRESS = re.compile(r"\(\s*'[^']*'\s*,\s*(\d{1,5})\s*\)")
HOLDER_TTL_S = 10.0
_holders: dict[int, tuple[float, dict[str, Any]]] = {}


def port_of(*texts: Any) -> int | None:
    """The port in the address an OSError's text names, when it names one."""
    for text in texts:
        found = ADDRESS.search(str(text or ""))
        if found:
            return int(found.group(1))
    return None


def listening_pid(port: int) -> int | None:
    """lsof (macOS and most Linux; no root needed for the user's own processes), else psutil (Linux, no lsof)."""
    try:
        listed = subprocess.run(
            ["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-Fp"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
        return next((int(line[1:]) for line in listed.splitlines() if line[:1] == "p"), None)
    except FileNotFoundError:
        pass
    except (OSError, subprocess.SubprocessError, ValueError):
        return None
    try:
        return next(
            (
                c.pid
                for c in psutil.net_connections(kind="tcp")
                if c.status == psutil.CONN_LISTEN and c.laddr and c.laddr.port == port and c.pid
            ),
            None,
        )
    except psutil.Error:
        return None


def port_holder(port: int) -> dict[str, Any]:
    """Which process listens on this TCP port: `{holder_pid, holder_command}`, or {} when that can't be told (lsof,
    which needs no root for the user's own processes; kept 10 s, the launch is re-read every second)."""
    now = time.monotonic()
    cached = _holders.get(port)
    if cached and now - cached[0] < HOLDER_TTL_S:
        return cached[1]
    holder: dict[str, Any] = {}
    pid = listening_pid(port)
    if pid is not None:
        holder["holder_pid"] = pid
        try:
            holder["holder_command"] = " ".join(psutil.Process(pid).cmdline())[:300] or None
        except psutil.Error:
            holder["holder_command"] = None
    _holders[port] = (now, holder)
    return holder


def problem(record: dict[str, Any]) -> dict[str, Any]:
    extra = record["extra"]
    info = exception_info(extra)
    message = info["message"] or extra.get("error") or record["event"]
    code = classify(info)
    data = {key: value for key, value in extra.items() if key not in NOT_DATA}
    data.update(
        {
            "exception_chain": info["chain"] or None,
            "exception_code": info["exception_code"],
            "missing_module": info["missing_module"],
        }
    )
    data = {key: value for key, value in data.items() if value is not None}
    if code == "port_in_use":
        port = port_of(info["message"], extra.get("error"), record["event"])
        if port is not None:
            data["port"] = port
            data.update(port_holder(port))
    return {
        "code": code,
        "level": "error",
        "message": str(message),
        "data": data,
        "timestamp": record["timestamp"],
        "logger": record["logger"],
    }


def problems(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Every error record as a problem, the ones with a known code first; when none has one, the last three
    distinct errors (the last is usually the one that ended it)."""
    found = [problem(record) for record in records if record["level"] in ("error", "critical")]
    distinct: list[dict[str, Any]] = []
    for item in found:
        if not any((p["code"], p["message"]) == (item["code"], item["message"]) for p in distinct):
            distinct.append(item)
    # one per known code: the same failure is logged where it happened and again where it was caught; keep the
    # first, or a later one that names the port when the first doesn't
    known: dict[str, dict[str, Any]] = {}
    for item in distinct:
        if item["code"] == "error":
            continue
        kept = known.get(item["code"])
        if kept is None or ("port" in item["data"] and "port" not in kept["data"]):
            known[item["code"]] = item
    return list(known.values()) or distinct[-3:]


# a console log line dimos prints (`14:56:51.285 [inf][...] ...`), as opposed to what it says on its own
LOG_LINE = re.compile(r"^\d\d:\d\d:\d\d\.\d+ \[")


def error_text(problems_found: list[dict[str, Any]], output: str) -> str:
    """One line saying why a launch failed: its first problem, else what dimos said on its own after its last log
    line (a refusal it only prints, e.g. an unknown blueprint and its suggestions: the first line of that), else its
    output's last line."""
    if problems_found:
        return str(problems_found[0]["message"])
    lines = [ANSI.sub("", line).rstrip() for line in output.splitlines()[1:]]
    last_log = max((i for i, line in enumerate(lines) if LOG_LINE.match(line)), default=-1)
    said = next((line.strip() for line in lines[last_log + 1 :] if line.strip()), None)
    return said or next(
        (line.strip() for line in reversed(lines) if line.strip()), "dimos exited during startup"
    )
