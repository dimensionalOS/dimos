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

"""Every `dimos run` on this computer, whoever started it, and runs only heard on the bus.

dimos registers a run under its own $XDG_STATE_HOME (dimos/constants.py), so a run from another DIMOS_HOME (a test
Desktop, an agent) or another user is in a registry this gateway doesn't read, yet shares this computer's bus. So this
also finds `dimos ... run <blueprint>` processes and reads each one's own registry (by its environment) for its entry.
"""

from __future__ import annotations

from datetime import datetime, timezone
import getpass
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time
from typing import Any

import psutil

# a registry entry written more than this before its pid's process started belongs to an older process (pid reused)
PID_REUSE_SLACK = 2.0


def run_argv(cmdline: list[str]) -> list[str] | None:
    """`cmdline` from its `dimos` on, when it is a `dimos ... run <blueprint>` (the script, or `python -m dimos`)."""
    for index, arg in enumerate(cmdline):
        script = Path(arg).name == "dimos" and not arg.startswith("-")
        module = (
            arg == "-m"
            and index + 1 < len(cmdline)
            and cmdline[index + 1] in ("dimos", "dimos.cli.entry")
        )
        if script or module:
            rest = cmdline[index + (2 if module else 1) :]
            return [arg if script else "dimos", *rest] if "run" in rest else None
    return None


def blueprint_of(argv: list[str]) -> str:
    """The blueprint(s) named right after `run`."""
    after = argv[argv.index("run") + 1 :]
    names = []
    for arg in after:
        if arg.startswith("-"):
            break
        names.append(arg)
    return " ".join(names) or "?"


def registry_dir(env: dict[str, str]) -> Path:
    """Where a process with environment `env` registers its run (dimos/constants.py's STATE_DIR / runs)."""
    state = env.get("XDG_STATE_HOME") or str(
        Path(env.get("HOME") or Path.home()) / ".local" / "state"
    )
    return Path(state) / "dimos" / "runs"


def iso(seconds: float) -> str:
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def same_process(started_at: str, created: float) -> bool:
    """Whether a registry entry from `started_at` can be the process created at `created` (else its pid was reused)."""
    try:
        registered = datetime.fromisoformat(started_at.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return True
    return created <= registered + PID_REUSE_SLACK


def held_log_dir(process: psutil.Process) -> Path | None:
    """The `<logs>/<run id>` whose main.jsonl the process has open."""
    try:
        files = process.open_files()
    except (psutil.Error, OSError):
        return None
    for file in files:
        path = Path(file.path)
        if path.name.startswith("main.jsonl"):
            return path.parent
    return None


def _entry(directory: Path, pid: int, created: float) -> dict[str, Any] | None:
    from dimos.core.run_registry import RunEntry

    try:
        files = sorted(directory.glob("*.json"))
    except OSError:
        return None
    for file in files:
        try:
            entry = RunEntry.load(file)
        except Exception:
            continue
        if entry.pid == pid and same_process(entry.started_at, created):
            return {
                "run_id": entry.run_id,
                "pid": entry.pid,
                "blueprint": entry.blueprint,
                "started_at": entry.started_at,
                "log_dir": entry.log_dir,
            }
    return None


def _command(argv: list[str]) -> str:
    from dimos.core.run_registry import _without_secret_options

    return shlex.join(_without_secret_options(argv))


def candidates() -> list[int]:
    """Pids whose command line mentions dimos and run: one `ps` (psutil reads every process's arguments one by one,
    seconds on a busy Mac), then psutil only for these."""
    try:
        listing = subprocess.run(
            ["ps", "-axww", "-o", "pid=,args="],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return [p.pid for p in psutil.process_iter() if run_argv(_cmdline(p)) is not None]
    pids = []
    for line in listing.splitlines():
        pid, _, args = line.strip().partition(" ")
        if "dimos" in args and " run" in args and pid.isdigit():
            pids.append(int(pid))
    return pids


def _cmdline(process: psutil.Process) -> list[str]:
    try:
        return list(process.cmdline())
    except (psutil.Error, OSError):
        return []


def scan(own_registry: Path) -> list[dict[str, Any]]:
    """Every live `dimos ... run` main process on this computer, as a RegistryRun with where it came from: `registry`
    (the registry it's in, null: none yet, it's still starting), `owner`, `command` (secrets left out), `ours` (in this
    gateway's own registry), `stoppable` and, when it isn't, `whyNot`."""
    me = getpass.getuser()
    matched: dict[int, tuple[psutil.Process, dict[str, Any], list[str]]] = {}
    for pid in candidates():
        try:
            process = psutil.Process(pid)
            info = process.as_dict(["ppid", "cmdline", "username", "create_time"])
        except (psutil.Error, OSError):
            continue
        argv = run_argv(info.get("cmdline") or [])
        if argv is not None:
            matched[pid] = (process, info, argv)
    found = []
    for pid, (process, info, argv) in matched.items():
        # a run's own children (forked workers) carry its command line: only the topmost is the run
        if info.get("ppid") in matched:
            continue
        created, owner = info.get("create_time") or time.time(), info.get("username")
        try:
            directories = [registry_dir(process.environ())]
        except (psutil.Error, OSError):
            directories = []
        # a process whose environment can't be read (another user's) may still be in our registry
        directories += [own_registry] if own_registry not in directories else []
        entry, registry = None, None
        for directory in directories:
            if (entry := _entry(directory, pid, created)) is not None:
                registry = directory
                break
        if entry is None:
            log_dir = held_log_dir(process)
            entry = {
                "run_id": log_dir.name if log_dir else f"pid-{pid}",
                "pid": pid,
                "blueprint": blueprint_of(argv),
                "started_at": iso(created),
                "log_dir": str(log_dir) if log_dir else "",
            }
        stoppable = owner == me or os.geteuid() == 0
        entry.update(
            {
                "registry": str(registry) if registry else None,
                "owner": owner,
                "command": _command(argv),
                "ours": registry == own_registry,
                "stoppable": stoppable,
                "whyNot": None
                if stoppable
                else f"started by {owner or 'another user'}; only they (or root) can stop it",
            }
        )
        found.append(entry)
    return sorted(found, key=lambda e: str(e["started_at"]), reverse=True)


class BusWatch:
    """Whether a dimos coordinator answers on the bus that no process on this computer accounts for: a run on
    another machine (through the gateway's zenoh connect endpoint) or one this gateway can't see. Probed in the
    background at most every `every` seconds, so polling GET /dimos/runs never waits on zenoh."""

    def __init__(self, every: float = 10.0) -> None:
        self.every = every
        self.checked = 0.0
        self.value: list[dict[str, Any]] = []
        self.lock = threading.Lock()
        self.busy = False

    def get(self, local: list[dict[str, Any]], connect: list[str]) -> list[dict[str, Any]]:
        with self.lock:
            if not self.busy and time.monotonic() - self.checked >= self.every:
                self.busy = True
                threading.Thread(
                    target=self._probe, args=(bool(local), connect), daemon=True
                ).start()
            return list(self.value)

    def _probe(self, have_local: bool, connect: list[str]) -> None:
        try:
            value = probe(have_local, connect)
        except Exception:
            value = []
        with self.lock:
            self.value, self.checked, self.busy = value, time.monotonic(), False


def probe(have_local: bool, connect: list[str]) -> list[dict[str, Any]]:
    from dimos.gateway import runs

    if runs.coordinator_on_bus():
        if have_local:
            return []
        return [
            {
                "where": "local",
                "peer": None,
                "note": "a dimos run answers on this computer's bus, but no process of it is visible here "
                "(another user's, or a container)",
            }
        ]
    if connect and runs.coordinator_reachable(connect):
        return [
            {
                "where": "network",
                "peer": ", ".join(connect),
                "note": "a dimos run answers through the zenoh connection; it runs on that computer, stop it there",
            }
        ]
    return []
