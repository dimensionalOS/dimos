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

import asyncio
from dataclasses import dataclass, field, replace
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import threading
import time
from typing import Any

from dimos.constants import LOG_DIR
from dimos.core.coordination.process_lifecycle import kill_run_processes
from dimos.core.run_registry import list_runs
from experimental.gateway import overrides as ov, store
from experimental.gateway.overrides import LaunchOverrides, ModuleValues

ACTIVE = ("starting", "running", "stopping")
STOP_WAITS = ((signal.SIGINT, 20.0), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0))
OUTPUT_TAIL = 200_000
_lock = threading.RLock()


class RunError(Exception):
    pass


class StillRunningError(RunError):
    pass


@dataclass
class LaunchConfig:
    global_: dict[str, Any] = field(default_factory=dict)
    modules: ModuleValues = field(default_factory=dict)
    one_off: LaunchOverrides = field(default_factory=LaunchOverrides)
    args: list[str] = field(default_factory=list)

    def secrets(self) -> list[str]:
        return ov.secret_paths(self.global_, self.modules, self.one_off.secrets)


def launch_file() -> Path:
    return store.gateway_dir() / "launch.json"


def secrets_file() -> Path:
    return store.gateway_dir() / "launch_secrets.json"


def launch_log() -> Path:
    return store.logs_dir() / "launch.log"


def registry_runs() -> list[dict[str, Any]]:
    keys = ("run_id", "pid", "blueprint", "started_at", "log_dir")
    runs = [{key: getattr(entry, key) for key in keys} for entry in list_runs(alive_only=True)]
    return sorted(runs, key=lambda run: str(run["run_id"]), reverse=True)


def group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _tail(path: Path, size: int) -> str:
    try:
        with path.open("rb") as handle:
            end = handle.seek(0, 2)
            handle.seek(max(0, end - size))
            return handle.read().decode("utf-8", "replace")
    except OSError:
        return ""


def _record() -> dict[str, Any] | None:
    record = store.read_json(launch_file())
    ok = isinstance(record, dict) and all(k in record for k in ("pid", "blueprint", "started_at"))
    return record if ok else None


def _save(record: dict[str, Any]) -> None:
    store.write_atomic(launch_file(), json.dumps(record))


def failed_log_dir(blueprint: str, started_at: str) -> Path | None:
    try:
        since = datetime.fromisoformat(started_at.replace("Z", "+00:00")).timestamp() - 1
        name = re.sub(r"[^a-zA-Z0-9_-]", "-", blueprint)
        found = [p for p in LOG_DIR.glob(f"*-{name}") if p.is_dir() and p.stat().st_mtime >= since]
    except (OSError, ValueError):
        return None
    return max(found, default=None, key=lambda p: p.stat().st_mtime)


def last_error(output: str) -> str:
    lines = [line.strip() for line in output.splitlines()[1:] if line.strip()]
    return lines[-1] if lines else "dimos exited during startup"


def current_launch() -> dict[str, Any] | None:
    record = _record()
    if record is None:
        return None
    pid = int(record["pid"])
    entry = next((run for run in registry_runs() if run["pid"] == pid), None)
    if entry and not record.get("run_id"):
        record.update(ever_ran=True, run_id=entry["run_id"], log_dir=entry["log_dir"])
        _save(record)
    alive = group_alive(pid)
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
    run_id, log_dir = record.get("run_id"), record.get("log_dir")
    if run_id is None and phase == "failed":
        found = failed_log_dir(str(record["blueprint"]), str(record["started_at"]))
        run_id, log_dir = (found.name, str(found)) if found else (None, None)
    output = _tail(launch_log(), OUTPUT_TAIL)
    launch: dict[str, Any] = {
        "blueprint": record["blueprint"],
        "phase": phase,
        "startedAt": record["started_at"],
        "pid": pid,
        "output": output,
        "runId": run_id,
        "logDir": log_dir,
        "error": last_error(output) if phase == "failed" else None,
        "overrides": record.get("overrides") or {},
        "modules": record.get("modules") or {},
        "oneOff": LaunchOverrides.from_json(record.get("one_off")).to_json(),
    }
    if isinstance(record.get("args"), list):
        launch["args"] = record["args"]
    return launch


def last_launch_args() -> tuple[str, LaunchConfig] | None:
    record = _record()
    if record is None:
        return None
    real = store.read_json(secrets_file()) or {}

    def restored(values: Any, saved: Any) -> dict[str, Any]:
        values, saved = values or {}, saved or {}
        return {k: saved[k] if v == ov.HIDDEN and k in saved else v for k, v in values.items()}

    def restored_modules(values: Any, saved: Any) -> ModuleValues:
        saved = saved or {}
        return {m: restored(f, saved.get(m)) for m, f in (values or {}).items()}

    one_off = LaunchOverrides.from_json(record.get("one_off"))
    args = real.get("args") or record.get("args") or []
    return str(record["blueprint"]), LaunchConfig(
        restored(record.get("overrides"), real.get("global")),
        restored_modules(record.get("modules"), real.get("modules")),
        LaunchOverrides(
            restored(one_off.global_, real.get("one_off_global")),
            restored_modules(one_off.modules, real.get("one_off_modules")),
            one_off.secrets,
        ),
        [str(arg) for arg in args],
    )


def run_args(blueprint: str, launch: LaunchConfig) -> list[str]:
    global_, modules = ov.without_secrets(launch.global_, launch.modules, launch.secrets())
    if launch.args:
        flags = ov.global_flags(global_, explicit_bools=True)
        return ["run", blueprint, *flags, *ov.module_flags(modules), *launch.args]
    return [*ov.global_flags(global_), "run", blueprint, *ov.module_flags(modules)]


def write_secrets(launch: LaunchConfig) -> None:
    secrets_file().unlink(missing_ok=True)
    one_off = launch.one_off
    paths = launch.secrets() + ov.secret_paths(one_off.global_, one_off.modules, one_off.secrets)
    secret_args = ov.shown_args(launch.args) != launch.args
    if not paths and not secret_args:
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
                "one_off_global": only(one_off.global_),
                "one_off_modules": only_modules(one_off.modules),
                **({"args": launch.args} if secret_args else {}),
            },
            handle,
        )


def coordinator_on_bus() -> bool:
    from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
    from dimos.core.transport_factory import rpc_backend
    from dimos.protocol.rpc.spec import RPCSpec
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

    backend = rpc_backend()
    probe: RPCSpec = ZenohRPC(gossip=False, connect=[]) if backend is ZenohRPC else backend()
    probe.start()
    try:
        probe.call_sync(f"{CoordinatorRPC.NAME}/ping", ([], {}), rpc_timeout=0.5)
        return True
    except TimeoutError:
        return False
    finally:
        probe.stop()


def start(dimos_dir: Path, blueprint: str, launch: LaunchConfig) -> dict[str, Any]:
    with _lock:
        previous = current_launch()
        if previous and previous["phase"] in ACTIVE:
            raise StillRunningError(
                f"{previous['blueprint']} is still {previous['phase']}; stop it first"
            )
        if other := next(iter(registry_runs()), None):
            raise StillRunningError(
                f"{other['blueprint']} (run {other['run_id']}, pid {other['pid']}) is running; stop it first"
            )
        if coordinator_on_bus():
            raise StillRunningError("another dimos run is running on this machine; stop it first")
        program = store.dimos_bin(dimos_dir)
        if not program.exists():
            raise RunError(f"no dimos at {dimos_dir} (no {program})")
        secret_env = ov.secret_env(launch.global_, launch.modules, launch.secrets())
        shown_env = "".join(f"{name}={ov.HIDDEN} " for name in secret_env)
        shown = run_args(blueprint, replace(launch, args=ov.shown_args(launch.args)))
        launch_log().parent.mkdir(parents=True, exist_ok=True)
        launch_log().write_text(f"$ {shown_env}dimos {shlex.join(shown)}\n")
        venv = store.venv_dir(dimos_dir)
        env = {
            **os.environ,
            "PATH": f"{venv / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}",
            "VIRTUAL_ENV": str(venv),
            "PYTHONUNBUFFERED": "1",
            "NO_COLOR": "1",
            **secret_env,
        }
        with launch_log().open("a") as log:
            child = subprocess.Popen(
                [str(program), *run_args(blueprint, launch)],
                cwd=dimos_dir,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
        threading.Thread(target=child.wait, daemon=True).start()
        write_secrets(launch)
        global_, modules = ov.redact(launch.global_, launch.modules, launch.secrets())
        one_off = launch.one_off
        one_off_paths = ov.secret_paths(one_off.global_, one_off.modules, one_off.secrets)
        one_off_global, one_off_modules = ov.redact(one_off.global_, one_off.modules, one_off_paths)
        record = {
            "blueprint": blueprint,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "pid": child.pid,
            "ever_ran": False,
            "overrides": global_,
            "modules": modules,
            "one_off": LaunchOverrides(one_off_global, one_off_modules, one_off.secrets).to_json(),
            **({"args": ov.shown_args(launch.args)} if launch.args else {}),
        }
        _save(record)
        started = current_launch()
        assert started is not None
        return started


def mark_stopping(pid: int) -> None:
    record = _record()
    if record is not None and record.get("pid") == pid:
        record["stopping"] = True
        _save(record)


async def _until_gone(alive: Any, seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while alive():
        if time.monotonic() >= deadline:
            return False
        await asyncio.sleep(0.1)
    return True


async def stop(run_id: str | None) -> str:
    launch = current_launch()
    if run_id and not (launch and launch["runId"] == run_id and launch["phase"] in ACTIVE):
        run = next((r for r in registry_runs() if r["run_id"] == run_id), None)
        if run is None:
            raise RunError(f"no live run {run_id}")
        pid, name = int(run["pid"]), str(run["blueprint"])
    elif launch and launch["phase"] in ACTIVE:
        pid, name, run_id = launch["pid"], launch["blueprint"], launch["runId"]
    else:
        raise RunError("the dimos gateway hasn't launched anything that's still running")
    try:
        leads = os.getpgid(pid) == pid
    except ProcessLookupError:
        leads = True
    mark_stopping(pid)

    def alive() -> bool:
        if leads:
            return group_alive(pid)
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False

    for signum, wait in STOP_WAITS:
        try:
            if leads:
                os.killpg(pid, signum)
            else:
                os.kill(pid, signum)
        except (ProcessLookupError, PermissionError):
            pass
        if await _until_gone(alive, wait):
            break
    else:
        raise RunError(f"{name} (pid {pid}) won't stop")
    if run_id:
        await asyncio.to_thread(kill_run_processes, run_id)
    return f"stopped {name} (pid {pid})"
