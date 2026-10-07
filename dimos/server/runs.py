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

"""Blueprint runs: `dimos run` started in its own session (so it outlives the server), plus the run registry.

The server remembers only the last launch it started (launch.json); whether it is starting, running or gone is
re-derived from the registry and the pid on every call, so a restarted server picks up where the last one was.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time
from typing import Any

from dimos.core.run_registry import is_pid_alive
from dimos.server import config, diagnose, logs, overrides as overrides_
from dimos.server.overrides import LaunchOverrides, ModuleValues


class RunError(Exception):
    """A launch or stop that can't happen (a 500 with a readable message)."""


class StillRunningError(RunError):
    """A launch while the last one is still starting or running (a 400)."""


def launch_file() -> Path:
    return config.state_file("launch.json")


@dataclass
class LaunchConfig:
    """What a launch runs with: the effective GlobalConfig and module config, and what the request itself set."""

    global_: dict[str, Any] = field(default_factory=dict)
    modules: ModuleValues = field(default_factory=dict)
    one_off: LaunchOverrides = field(default_factory=LaunchOverrides)

    def secrets(self) -> list[str]:
        return overrides_.secret_paths(self.global_, self.modules, self.one_off.secrets)


def secrets_file() -> Path:
    """The real values of the last launch's secrets (owner-only), so a restart can pass them again."""
    return config.server_dir() / "launch_secrets.json"


def launch_log() -> Path:
    """The launch's console output (shown as is; never read for meaning)."""
    return config.logs_dir() / "launch.log"


def launch_records_dir() -> Path:
    """Where the launch's structured log starts (DIMOS_RUN_LOG_DIR): dimos logs there until it knows its run id, then
    moves to the run's own log dir (LOG_DIR/<run id>, found by run_log_dir)."""
    return config.server_dir() / "launch"


# the records a launch's diagnosis needs (a stage's message, an error) carry one of these; the rest of a log, which
# can be megabytes, is skipped without parsing it
_WANTED = (
    *(json.dumps(event).encode() for event in diagnose.STAGE_EVENTS),
    b'"level": "error"',
    b'"level": "critical"',
)
_records_cache: dict[Path, tuple[tuple[int, int], list[dict[str, Any]]]] = {}


def _records(file: Path) -> list[dict[str, Any]]:
    """`file`'s stage and error records (its last 4 MB), cached while the file is unchanged."""
    try:
        stat = file.stat()
    except OSError:
        return []
    key = (stat.st_size, stat.st_mtime_ns)
    cached = _records_cache.get(file)
    if cached and cached[0] == key:
        return cached[1]
    with file.open("rb") as handle:
        handle.seek(max(0, stat.st_size - logs.MAX_READ))
        lines = [line for line in handle if any(wanted in line for wanted in _WANTED)]
    parsed = [logs.parse_line(line.decode("utf-8", "replace")) for line in lines]
    records = [record for record in parsed if record is not None and record["level"] != "raw"]
    _records_cache[file] = (key, records)
    return records


def run_log_dir(blueprint: str, started_at: str, entry: dict[str, Any] | None) -> Path | None:
    """The launch's own log dir: its registry entry's once dimos registered it, before that the newest
    LOG_DIR/<run id> for this blueprint made since the launch (dimos names it `<YYYYmmdd-HHMMSS>-<blueprint>`)."""
    if entry and entry.get("log_dir"):
        return Path(str(entry["log_dir"]))
    from dimos.constants import LOG_DIR

    try:
        since = datetime.fromisoformat(started_at).timestamp() - 1
        candidates = [
            path
            for path in LOG_DIR.glob(f"*-{re.sub(r'[^a-zA-Z0-9_-]', '-', blueprint)}")
            if path.is_dir() and path.stat().st_mtime >= since
        ]
    except (OSError, ValueError):
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime, default=None)


def launch_records(moved: Path | None) -> list[dict[str, Any]]:
    """The current launch's records: from before it had a run id, then from its run's main.jsonl."""
    records = _records(launch_records_dir() / "main.jsonl")
    if moved is not None:
        records = records + _records(moved / "main.jsonl")
    return records


def registry_runs() -> list[dict[str, Any]]:
    """Live runs from dimos's run registry, newest first (runs started from a terminal too), with RegistryRun's fields
    of each RunEntry."""
    from dimos.core.run_registry import list_runs
    from dimos.server.models import RegistryRun

    runs = [
        {key: value for key, value in asdict(entry).items() if key in RegistryRun.model_fields}
        for entry in list_runs(alive_only=True)
    ]
    return sorted(runs, key=lambda run: str(run["run_id"]), reverse=True)


def _tail(path: Path, size: int) -> str:
    try:
        with path.open("rb") as handle:
            end = handle.seek(0, 2)
            handle.seek(max(0, end - size))
            return handle.read().decode("utf-8", "replace")
    except OSError:
        return ""


def current_launch() -> dict[str, Any] | None:
    """The last launch this server started, with its phase: starting, running, stopping (asked to stop, or out of
    dimos's run registry while its process still exits), stopped or failed (gone before it ever ran, unasked)."""
    try:
        record = json.loads(launch_file().read_text())
        pid, blueprint, started_at = (
            int(record["pid"]),
            str(record["blueprint"]),
            str(record["started_at"]),
        )
    except (OSError, ValueError, KeyError, TypeError):
        return None
    entry = next((run for run in registry_runs() if run["pid"] == pid), None)
    if entry and not record.get("ever_ran"):
        # dimos registers a run once every module is built: from then on it "ran"
        record["ever_ran"] = True
        config.write_atomic(launch_file(), json.dumps(record))
    output = _tail(launch_log(), 200_000)
    alive = is_pid_alive(pid)
    stopping = bool(record.get("stopping"))
    if alive and (stopping or (record.get("ever_ran") and not entry)):
        phase = "stopping"
    elif entry:
        phase = "running"
    elif alive:
        phase = "starting"
    elif record.get("ever_ran") or stopping:
        phase = "stopped"
    else:
        phase = "failed"
    records = launch_records(run_log_dir(blueprint, started_at, entry))
    problems = diagnose.problems(records)
    error = diagnose.error_text(problems, output) if phase == "failed" else None
    overrides = record.get("overrides")
    return {
        "blueprint": blueprint,
        "phase": phase,
        "startedAt": started_at,
        "pid": pid,
        "output": output,
        "runId": entry["run_id"] if entry else None,
        "logDir": entry["log_dir"] if entry else None,
        "error": error,
        "overrides": overrides if isinstance(overrides, dict) else {},
        "modules": record.get("modules") if isinstance(record.get("modules"), dict) else {},
        "oneOff": LaunchOverrides.from_json(record.get("one_off")).to_json(),
        "steps": diagnose.steps(records, phase),
        "problems": problems,
    }


def last_launch_args() -> tuple[str, LaunchConfig] | None:
    """The blueprint and config of the last launch (to launch it again the same way), even after it stopped; its
    secrets come back from the secrets file."""
    try:
        record = json.loads(launch_file().read_text())
        blueprint = str(record["blueprint"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    try:
        real = json.loads(secrets_file().read_text()) if record.get("secret") else {}
    except (OSError, ValueError):
        real = {}
    one_off = LaunchOverrides.from_json(record.get("one_off"))

    def restored(values: Any, saved: Any) -> dict[str, Any]:
        values = dict(values) if isinstance(values, dict) else {}
        saved = saved if isinstance(saved, dict) else {}
        return {
            k: saved[k] if v == overrides_.HIDDEN and k in saved else v for k, v in values.items()
        }

    def restored_modules(values: Any, saved: Any) -> ModuleValues:
        values = values if isinstance(values, dict) else {}
        saved = saved if isinstance(saved, dict) else {}
        return {m: restored(f, saved.get(m)) for m, f in values.items()}

    return blueprint, LaunchConfig(
        restored(record.get("overrides"), real.get("global")),
        restored_modules(record.get("modules"), real.get("modules")),
        LaunchOverrides(
            restored(one_off.global_, real.get("one_off_global")),
            restored_modules(one_off.modules, real.get("one_off_modules")),
            one_off.secrets,
        ),
    )


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def run_args(blueprint: str, launch: LaunchConfig) -> list[str]:
    """`[--key=value ...] run <blueprint> [--<module>.<field>=value ...]`, without the secrets (they go in the
    environment, see run_env)."""
    global_, modules = overrides_.without(launch.global_, launch.modules, launch.secrets())
    return [
        *config.global_config_flags(global_),
        "run",
        blueprint,
        *overrides_.module_flags(modules),
    ]


def run_env(launch: LaunchConfig) -> dict[str, str]:
    return overrides_.secret_env(launch.global_, launch.modules, launch.secrets())


def write_secrets(launch: LaunchConfig) -> None:
    """The real secret values, owner-only, created fresh; no file when the launch has none."""
    secrets_file().unlink(missing_ok=True)
    paths = launch.secrets() + overrides_.secret_paths(
        launch.one_off.global_, launch.one_off.modules, launch.one_off.secrets
    )
    if not paths:
        return

    def only(values: dict[str, Any]) -> dict[str, Any]:
        return {k: v for k, v in values.items() if k in paths}

    def only_modules(values: ModuleValues) -> ModuleValues:
        return {m: {k: v for k, v in f.items() if f"{m}.{k}" in paths} for m, f in values.items()}

    secrets_file().parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(secrets_file(), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w") as handle:
        json.dump(
            {
                "global": only(launch.global_),
                "modules": only_modules(launch.modules),
                "one_off_global": only(launch.one_off.global_),
                "one_off_modules": only_modules(launch.one_off.modules),
            },
            handle,
        )


def start(dimos_dir: Path, blueprint: str, launch_config: LaunchConfig) -> dict[str, Any]:
    """`dimos [--key=value ...] run <blueprint> [--<module>.<field>=value ...]`, secrets in its environment, in the
    foreground of its own session (not `--daemon`: on macOS the daemon's post-fork build segfaults inside
    CoreFoundation). The launch keeps its config (secrets shown as •••), so it can be launched again the same way."""
    previous = current_launch()
    if previous and previous["phase"] in ("starting", "running", "stopping"):
        raise StillRunningError(
            f"{previous['blueprint']} is still {previous['phase']}; stop it first"
        )
    program = config.dimos_bin(dimos_dir)
    if not program.exists():
        raise RunError(f"no dimos at {dimos_dir} (no {program})")
    args = run_args(blueprint, launch_config)
    secret_env = run_env(launch_config)
    shown_env = "".join(f"{name}={overrides_.HIDDEN} " for name in secret_env)
    launch_log().parent.mkdir(parents=True, exist_ok=True)
    launch_log().write_text(f"$ {shown_env}dimos {' '.join(args)}\n")
    shutil.rmtree(launch_records_dir(), ignore_errors=True)
    venv = config.venv_dir(dimos_dir)
    env = {
        **os.environ,
        # the checkout's venv first, as an activated venv would: dimos spawns tools from it by name (mjpython)
        "PATH": f"{venv / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}",
        "VIRTUAL_ENV": str(venv),
        "PYTHONUNBUFFERED": "1",
        "NO_COLOR": "1",
        # its structured log starts here, so even what it logs before it has a run id can be read
        "DIMOS_RUN_LOG_DIR": str(launch_records_dir()),
        **secret_env,
    }
    with launch_log().open("a") as log:
        child = subprocess.Popen(
            [str(program), *args],
            cwd=dimos_dir,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            # its own session: stopping the server leaves the robot running
            start_new_session=True,
        )
    # reap it, so a finished run doesn't linger as a zombie that still looks alive
    threading.Thread(target=child.wait, daemon=True).start()
    write_secrets(launch_config)
    paths = launch_config.secrets()
    global_, modules = overrides_.redact(launch_config.global_, launch_config.modules, paths)
    one_off = launch_config.one_off
    one_off_paths = overrides_.secret_paths(one_off.global_, one_off.modules, one_off.secrets)
    one_off_global, one_off_modules = overrides_.redact(
        one_off.global_, one_off.modules, one_off_paths
    )
    record = {
        "blueprint": blueprint,
        "started_at": now_iso(),
        "pid": child.pid,
        "ever_ran": False,
        "overrides": global_,
        "modules": modules,
        "one_off": LaunchOverrides(one_off_global, one_off_modules, one_off.secrets).to_json(),
        "secret": sorted(set(paths) | set(one_off_paths)),
    }
    config.write_atomic(launch_file(), json.dumps(record))
    launch = current_launch()
    assert launch is not None
    return launch


def mark_stopping(pid: int) -> None:
    """Records that the launch with this pid was asked to stop: it's `stopping` while it exits, then `stopped`."""
    try:
        record = json.loads(launch_file().read_text())
    except (OSError, ValueError):
        return
    if isinstance(record, dict) and record.get("pid") == pid and not record.get("stopping"):
        record["stopping"] = True
        config.write_atomic(launch_file(), json.dumps(record))


async def stop(run_id: str | None, marked: Callable[[], None] | None = None) -> str:
    """Stops this server's launch, or any live registry run by id: its process group gets Ctrl-C, then SIGTERM, then
    SIGKILL, as a terminal would (not `dimos stop`, which picks its own target). `marked` is called once the launch
    is marked `stopping`, before the first signal."""
    if run_id:
        run = next((r for r in registry_runs() if r["run_id"] == run_id), None)
        if run is None:
            raise RunError(f"no live run {run_id}")
        pid, name = int(run["pid"]), str(run["blueprint"])
    else:
        launch = current_launch()
        if not launch or launch["phase"] not in ("starting", "running", "stopping"):
            raise RunError("the dimos server hasn't launched anything that's still running")
        pid, name = launch["pid"], launch["blueprint"]
    mark_stopping(pid)
    if marked is not None:
        marked()
    for signum, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        try:
            # a launch is its own session, so its pgid is its pid; a run from a terminal may not be
            os.killpg(pid, signum)
        except (ProcessLookupError, PermissionError):
            try:
                os.kill(pid, signum)
            except ProcessLookupError:
                pass
        for _ in range(wait * 4):
            if not is_pid_alive(pid):
                return f"stopped {name} (pid {pid})"
            await asyncio.sleep(0.25)
    raise RunError(f"{name} (pid {pid}) won't stop")
