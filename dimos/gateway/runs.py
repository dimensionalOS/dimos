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

"""Blueprint runs: `dimos run` started in its own session (so it outlives the gateway), plus the run registry.

The gateway remembers only the last launch it started (launch.json); whether it is starting, running or gone is
re-derived from the registry and the pid on every call, so a restarted gateway picks up where the last one was.
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
import socket
import subprocess
import threading
import time
from typing import Any

import psutil

from dimos.core.run_registry import is_pid_alive
from dimos.gateway import config, diagnose, logs, overrides as overrides_
from dimos.gateway.overrides import LaunchOverrides, ModuleValues


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
    return config.gateway_dir() / "launch_secrets.json"


def launch_log() -> Path:
    """The launch's console output (shown as is; never read for meaning)."""
    return config.logs_dir() / "launch.log"


def launch_records_dir() -> Path:
    """Where the launch's structured log starts (DIMOS_RUN_LOG_DIR): dimos logs there until it knows its run id, then
    moves to the run's own log dir (LOG_DIR/<run id>, found by run_log_dir)."""
    return config.gateway_dir() / "launch"


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


def held_run_log_dir(pid: int) -> Path | None:
    """The LOG_DIR/<run id> whose main.jsonl process `pid` has open: its own run's, whatever else runs."""
    from dimos.constants import LOG_DIR

    try:
        files = psutil.Process(pid).open_files()
        logs_root = LOG_DIR.resolve()
    except (psutil.Error, OSError):
        return None
    for file in files:
        path = Path(file.path)
        if path.name.startswith("main.jsonl") and path.parent.parent == logs_root:
            return LOG_DIR / path.parent.name
    return None


def run_log_dir(blueprint: str, started_at: str, entry: dict[str, Any] | None) -> Path | None:
    """The launch's own log dir: its registry entry's (or the one its process was seen holding open), else, for a run
    that died before either, the one LOG_DIR/<run id> for this blueprint made since the launch that no live run owns
    (dimos names it `<YYYYmmdd-HHMMSS>-<blueprint>`); two such (another Desktop on this checkout) is no answer."""
    if entry and entry.get("log_dir"):
        return Path(str(entry["log_dir"]))
    from dimos.constants import LOG_DIR

    try:
        # Python 3.10's fromisoformat refuses a trailing Z
        since = datetime.fromisoformat(started_at.replace("Z", "+00:00")).timestamp() - 1
        owned = {Path(str(run["log_dir"])).name for run in registry_runs()}
        candidates = [
            path
            for path in LOG_DIR.glob(f"*-{re.sub(r'[^a-zA-Z0-9_-]', '-', blueprint)}")
            if path.is_dir() and path.stat().st_mtime >= since and path.name not in owned
        ]
    except (OSError, ValueError):
        return None
    return candidates[0] if len(candidates) == 1 else None


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
    from dimos.gateway.models import RegistryRun

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
    """The last launch this gateway started, with its phase: starting, running, stopping (asked to stop, or out of
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
    if entry and not (record.get("ever_ran") and record.get("run_id")):
        # dimos registers a run once every module is built: from then on it "ran"; its run id outlives the entry
        record.update(ever_ran=True, run_id=entry["run_id"], log_dir=entry["log_dir"])
        config.write_atomic(launch_file(), json.dumps(record))
    # its process still exits while anything in its process group (workers, MuJoCo) is left
    alive = group_alive(pid)
    if alive and not record.get("run_id") and (held := held_run_log_dir(pid)):
        # before it registers (or if it never does), the log dir its own process writes is its run's
        record.update(run_id=held.name, log_dir=str(held))
        config.write_atomic(launch_file(), json.dumps(record))
    output = _tail(launch_log(), 200_000)
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
    if entry is None and record.get("run_id"):
        entry = {"run_id": record["run_id"], "log_dir": record.get("log_dir")}
    log_dir = run_log_dir(blueprint, started_at, entry)
    records = launch_records(log_dir)
    problems = diagnose.problems(records)
    error = diagnose.error_text(problems, output) if phase == "failed" else None
    overrides = record.get("overrides")
    return {
        "blueprint": blueprint,
        "phase": phase,
        "startedAt": started_at,
        "pid": pid,
        "output": output,
        # after the run exits (a failed one never registered): its log dir's, so its log stays reachable
        "runId": entry["run_id"] if entry else log_dir.name if log_dir else None,
        "logDir": entry["log_dir"] if entry else str(log_dir) if log_dir else None,
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
    # two runs on one machine share module RPC names: the second's start and stop calls reach the first's modules
    if other := next(iter(registry_runs()), None):
        raise StillRunningError(
            f"{other['blueprint']} (run {other['run_id']}, pid {other['pid']}) is running; stop it first"
        )
    if coordinator_on_bus():
        raise StillRunningError("another dimos run is running on this machine; stop it first")
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
            # its own session: stopping the gateway leaves the robot running
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


def coordinator_on_bus() -> bool:
    """Whether a dimos run (from any checkout, terminal or Desktop) answers on this machine's RPC bus."""
    from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
    from dimos.core.transport_factory import rpc_backend

    backend = rpc_backend()
    # zenoh gossip reaches other machines' runs through any local peer that scouts the LAN; only this machine's count
    probe = backend(gossip=False, connect=[]) if backend.__name__ == "ZenohRPC" else backend()
    probe.start()
    try:
        probe.call_sync(f"{CoordinatorRPC.NAME}/ping", ([], {}), rpc_timeout=0.5)
        return True
    except TimeoutError:
        return False
    finally:
        probe.stop()


def group_alive(pgid: int) -> bool:
    """Whether any process of process group `pgid` is left (a launch is its own session: its pgid is its pid)."""
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def run_processes(
    pgid: int | None, run_id: str | None, also: int | None = None
) -> list[psutil.Process]:
    """A run's processes: its process group (and its main process `also`), and what it started in a session of its
    own (native modules), which carries its DIMOS_RUN_ID."""
    from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV

    found = []
    for process in psutil.process_iter():
        try:
            if (
                process.pid == also
                or (pgid is not None and os.getpgid(process.pid) == pgid)
                or (run_id and process.environ().get(DIMOS_RUN_ID_ENV) == run_id)
            ):
                found.append(process)
        except (psutil.Error, OSError):
            continue
    return found


def listening(processes: list[psutil.Process]) -> dict[tuple[str, int], psutil.Process]:
    """The addresses `processes` listen on, and which listens."""
    found = {}
    for process in processes:
        try:
            for connection in process.net_connections("inet"):
                if connection.status == psutil.CONN_LISTEN:
                    found[(connection.laddr.ip, connection.laddr.port)] = process
        except (psutil.Error, OSError):
            continue
    return found


def port_free(address: tuple[str, int]) -> bool:
    """Whether a server could bind `address` now (as most do, with SO_REUSEADDR)."""
    host, port = address
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    with socket.socket(family, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((host, port))
            return True
        except OSError:
            return False


# how long stop() waits after Ctrl-C, SIGTERM and SIGKILL, and for the ports the run listened on
STOP_WAITS = ((signal.SIGINT, 20.0), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0))
PORT_WAIT = 10.0


async def _until(done: Callable[[], bool], seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while not done():
        if time.monotonic() >= deadline:
            return False
        await asyncio.sleep(0.1)
    return True


async def stop(run_id: str | None, marked: Callable[[], None] | None = None) -> str:
    """Stops this gateway's launch, or any live registry run by id, fully: its process group gets Ctrl-C, then
    SIGTERM, then SIGKILL, as a terminal would (not `dimos stop`, which picks its own target), until no process of it
    is left; then every port it listened on is free again (a leftover of the run's own that still holds one gets
    SIGTERM, then SIGKILL). So a launch right after never finds the old run's ports or modules. `marked` is called
    once the launch is marked `stopping`, before the first signal."""
    if run_id:
        run = next((r for r in registry_runs() if r["run_id"] == run_id), None)
        if run is None:
            raise RunError(f"no live run {run_id}")
        pid, name = int(run["pid"]), str(run["blueprint"])
    else:
        launch = current_launch()
        if not launch or launch["phase"] not in ("starting", "running", "stopping"):
            raise RunError("the dimos gateway hasn't launched anything that's still running")
        pid, name, run_id = launch["pid"], launch["blueprint"], launch["runId"]
    try:
        # a launch is its own session, so its pgid is its pid; a run from a terminal may share its shell's
        leads = os.getpgid(pid) == pid
    except ProcessLookupError:
        leads = True
    ports = await asyncio.to_thread(
        listening, run_processes(pid if leads else None, run_id, also=pid)
    )
    mark_stopping(pid)
    if marked is not None:
        marked()
    gone = (lambda: not group_alive(pid)) if leads else (lambda: not is_pid_alive(pid))
    for signum, wait in STOP_WAITS:
        try:
            if leads:
                os.killpg(pid, signum)
            else:
                os.kill(pid, signum)
        except (ProcessLookupError, PermissionError):
            pass
        if await _until(gone, wait):
            break
    else:
        raise RunError(f"{name} (pid {pid}) won't stop")
    held = [address for address in ports if not port_free(address)]
    for signum in (signal.SIGTERM, signal.SIGKILL):
        if not held:
            break
        for address in held:
            holder = ports[address]
            # only the old run's own processes, never whatever else took the port since
            if run_id and holder.is_running() and holder in run_processes(None, run_id):
                holder.send_signal(signum)
        await _until(lambda held=held: all(map(port_free, held)), PORT_WAIT / 2)
        held = [address for address in held if not port_free(address)]
    if held:
        taken = ", ".join(f"{host}:{port}" for host, port in held)
        raise RunError(f"{name} (pid {pid}) stopped, but {taken} is still in use")
    return f"stopped {name} (pid {pid})"
