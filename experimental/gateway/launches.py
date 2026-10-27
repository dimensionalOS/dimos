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

from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass, field, replace
import inspect
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading
import time
from typing import Any

import psutil
from pydantic import ValidationError

from dimos.constants import LOG_DIR
from dimos.core.coordination.process_lifecycle import is_pid_alive
from dimos.core.run_registry import get_most_recent, list_runs, stop_entry
from experimental.gateway import constants, store
from experimental.gateway.errors import ApiError, StatusError

ModuleValues = dict[str, dict[str, Any]]

HIDDEN = "•••"
STRUCTURED = ("global", "modules", "secrets")

# GlobalConfig.dtop's env name (pydantic-settings: the field name, no prefix)
DTOP_ENV = "DTOP"
# held across every change to what runs (start, stop, relaunch), so none sees another half done
_lock = threading.RLock()
# the gateway's own children, by pid: set once reaped, which is when the gateway stops counting it as running
_reaped: dict[int, threading.Event] = {}
# called from a thread when a launched child exits (the launch watcher wakes on it)
exit_listeners: list[Callable[[], None]] = []


class RunError(StatusError):
    pass


class StillRunningError(RunError):
    status = 409


class LaunchConfigError(StatusError, ValueError):
    status = 400


@dataclass
class LaunchConfig:
    """One launch's config, all of it: the gateway saves none, the caller sends it every time."""

    global_: dict[str, Any] = field(default_factory=dict)
    modules: ModuleValues = field(default_factory=dict)
    secrets: list[str] = field(default_factory=list)
    args: list[str] = field(default_factory=list)
    # POST /active/exec: exactly these args (a relaunch keeps it that way)
    exact: bool = False

    def secret_paths(self) -> list[str]:
        return secret_paths(self.global_, self.modules, self.secrets)

    def to_json(self) -> dict[str, Any]:
        return {
            "global": self.global_,
            "modules": self.modules,
            "secrets": self.secrets,
            "args": self.args,
            "exact": self.exact,
        }

    @staticmethod
    def from_json(value: dict[str, Any]) -> LaunchConfig:
        launch = parse({key: value[key] for key in STRUCTURED if key in value})
        launch.args = [str(arg) for arg in value.get("args") or []]
        launch.exact = bool(value.get("exact"))
        return launch


def launch_env_defaults() -> dict[str, str]:
    """No rerun window pops up from a launch, the web viewer instead: env, not flags (a flag for a module
    the blueprint lacks is refused), so a saved setting still wins."""
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    return module_env({RerunBridgeModule.name: {"rerun_open": "none", "rerun_web": True}})


def parse(raw: Any) -> LaunchConfig:
    if raw is None:
        return LaunchConfig()
    if not isinstance(raw, dict):
        raise LaunchConfigError("overrides must be an object")
    if not any(key in raw for key in STRUCTURED):
        return LaunchConfig(dict(raw))
    other = next((key for key in raw if key not in STRUCTURED), None)
    if other is not None:
        raise LaunchConfigError(f"overrides has `{other}` next to `global`/`modules`")
    global_, modules, secrets = (
        raw.get("global") or {},
        raw.get("modules") or {},
        raw.get("secrets") or [],
    )
    if not isinstance(global_, dict):
        raise LaunchConfigError("overrides.global must be an object")
    if not isinstance(modules, dict) or not all(
        isinstance(fields, dict) for fields in modules.values()
    ):
        raise LaunchConfigError("overrides.modules must be an object of objects")
    if not isinstance(secrets, list) or not all(isinstance(path, str) for path in secrets):
        raise LaunchConfigError("overrides.secrets must be a list of paths")
    return LaunchConfig(
        dict(global_), {module: dict(fields) for module, fields in modules.items()}, secrets
    )


def is_secret_name(path: str) -> bool:
    name = path.rsplit(".", 1)[-1].lower()
    return (
        name == "key"
        or name.endswith("_key")
        or any(word in name for word in ("secret", "token", "password", "passwd"))
    )


def secret_paths(
    global_: dict[str, Any], modules: ModuleValues, listed: list[str] | tuple[str, ...] = ()
) -> list[str]:
    paths = [key for key in sorted(global_) if is_secret_name(key) or key in listed]
    for module, fields in sorted(modules.items()):
        paths += [
            f"{module}.{name}"
            for name in sorted(fields)
            if is_secret_name(name) or f"{module}.{name}" in listed
        ]
    return paths


def as_text(value: Any) -> str:
    return value if isinstance(value, str) else json.dumps(value, separators=(",", ":"))


def module_env(modules: ModuleValues) -> dict[str, str]:
    """dimos's env names for module settings: MODULE__FIELD."""
    return {
        f"{module.replace('/', '_')}__{name}".upper(): as_text(value)
        for module, fields in modules.items()
        for name, value in fields.items()
        if value is not None
    }


def secret_env(global_: dict[str, Any], modules: ModuleValues, paths: list[str]) -> dict[str, str]:
    env = {
        key.upper(): as_text(value)
        for key, value in global_.items()
        if key in paths and value is not None
    }
    secret = {
        module: {name: value for name, value in fields.items() if f"{module}.{name}" in paths}
        for module, fields in modules.items()
    }
    return {**env, **module_env(secret)}


def without_secrets(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    flat = {key: value for key, value in global_.items() if key not in paths}
    nested = {
        module: {name: value for name, value in fields.items() if f"{module}.{name}" not in paths}
        for module, fields in modules.items()
    }
    return flat, {module: fields for module, fields in nested.items() if fields}


def redact(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    flat = {
        key: HIDDEN if key in paths and value is not None else value
        for key, value in global_.items()
    }
    nested = {
        module: {
            name: HIDDEN if f"{module}.{name}" in paths and value is not None else value
            for name, value in fields.items()
        }
        for module, fields in modules.items()
    }
    return flat, nested


def flag_fields() -> set[str]:
    from dimos.cli.commands.global_options import create_dynamic_callback

    callback = create_dynamic_callback()  # type: ignore[no-untyped-call]
    return set(inspect.signature(callback).parameters) - {"ctx"}


def check_global(values: dict[str, Any]) -> None:
    from dimos.core.global_config import GlobalConfig

    unknown = sorted(set(values) - flag_fields())
    if unknown:
        raise LaunchConfigError(f"not a dimos GlobalConfig flag: {', '.join(unknown)}")
    try:
        GlobalConfig.model_validate(
            {key: value for key, value in values.items() if value is not None}
        )
    except ValidationError as error:
        problems = "; ".join(
            f"{'.'.join(map(str, problem['loc']))}: {problem['msg']}" for problem in error.errors()
        )
        raise LaunchConfigError(f"bad GlobalConfig value: {problems}") from error


def global_flags(values: dict[str, Any], explicit_bools: bool = False) -> list[str]:
    flags: list[str] = []
    for key, value in sorted(values.items()):
        flag = "--" + key.replace("_", "-")
        if value is True:
            flags.append(f"{flag}=true" if explicit_bools else flag)
        elif value is False:
            flags.append("--no-" + key.replace("_", "-"))
        elif value is not None:
            flags.append(f"{flag}={as_text(value)}")
    return flags


def nested(fields: dict[str, Any]) -> dict[str, Any]:
    """`addresses.arm` and `addresses.base` become one `addresses` object."""
    out: dict[str, Any] = {}
    for name, value in sorted(fields.items()):
        if value is None:
            continue
        head, dot, rest = name.partition(".")
        if dot:
            out.setdefault(head, {})[rest] = value
        else:
            out[name] = value
    return out


def module_flags(modules: ModuleValues) -> list[str]:
    flags = []
    for module, fields in sorted(modules.items()):
        root = module.replace("/", "_").replace("_", "-")
        for name, value in nested(fields).items():
            flags.append(f"--{root}.{name.replace('_', '-')}={as_text(value)}")
    return flags


def shown_args(args: list[str]) -> list[str]:
    shown: list[str] = []
    hide_next = False
    for arg in args:
        if hide_next and not arg.startswith("-"):
            shown.append(HIDDEN)
            hide_next = False
            continue
        option, separator, _ = arg.partition("=")
        secret = arg.startswith("--") and is_secret_name(option[2:].replace("-", "_"))
        shown.append(f"{option}={HIDDEN}" if secret and separator else arg)
        hide_next = secret and not separator
    return shown


def launch_file() -> Path:
    return store.gateway_dir() / "launch.json"


def request_file() -> Path:
    """What the last launch was started with, secrets included (0600), for POST /active/relaunch."""
    return store.gateway_dir() / "launch_request.json"


def launch_log() -> Path:
    return store.logs_dir() / "launch.log"


def registry_runs() -> list[dict[str, Any]]:
    keys = ("run_id", "pid", "blueprint", "started_at", "log_dir")
    runs = [{key: getattr(entry, key) for key in keys} for entry in list_runs(alive_only=True)]
    return sorted(runs, key=lambda run: str(run["run_id"]), reverse=True)


def run_of(blueprint: str) -> dict[str, Any] | None:
    """The newest live run of this blueprint, however it was started (Desktop, a terminal, tmux)."""
    return next((run for run in registry_runs() if run["blueprint"] == blueprint), None)


@dataclass(frozen=True)
class RunSource:
    """Where a run's dimos came from: its checkout, and the python it runs on."""

    checkout: Path
    python: str
    run_id: str


def run_source(run_id: str) -> RunSource | None:
    """Where a run's dimos came from (None when it isn't a checkout, as for a pip install)."""
    entry = next((run for run in list_runs(alive_only=True) if run.run_id == run_id), None)
    if entry is None:
        return None
    try:
        command = psutil.Process(entry.pid).cmdline()
    except psutil.Error:
        command = []
    paths = [Path(arg) for arg in [*entry.original_argv[:1], *command[:2]] if arg.startswith("/")]
    for path in [*paths, Path(entry.log_dir)]:
        root = next((parent for parent in path.parents if store.checkout_version(parent)[0]), None)
        if root is not None:
            ran_on = command[0] if command else ""
            is_python = ran_on.startswith("/") and Path(ran_on).name.startswith("python")
            return RunSource(root, ran_on if is_python else store.python_for(root), run_id)
    return None


def started_time(pid: int) -> float | None:
    """When `pid` started: with the pid, names one process even after the OS reuses the number."""
    try:
        return float(psutil.Process(pid).create_time())
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return None


def is_launched_process(record: dict[str, Any]) -> bool:
    """The recorded launch is still the process at its pid (not a later one the pid was reused for)."""
    pid = int(record["pid"])
    if (reaped := _reaped.get(pid)) is not None:
        return not reaped.is_set()
    # an earlier gateway's launch, not this one's child: an exited one is a zombie only until its parent reaps it
    return is_pid_alive(pid) and started_time(pid) == record.get("pid_started")


def alive(pid: int, group: bool = True) -> bool:
    try:
        (os.killpg if group else os.kill)(pid, 0)
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
    ok = isinstance(record, dict) and all(
        key in record for key in ("pid", "blueprint", "started_at")
    )
    return record if ok else None


def _save(record: dict[str, Any]) -> None:
    store.write_atomic(launch_file(), json.dumps(record))


def last_error(output: str) -> str:
    lines = [line.strip() for line in output.splitlines()[1:] if line.strip()]
    return lines[-1] if lines else "dimos exited during startup"


def current_launch() -> dict[str, Any] | None:
    """The gateway's last launch; its runId is the run registry's, known once the run registers."""
    record = _record()
    if record is None:
        return None
    pid = int(record["pid"])
    entry = next((run for run in registry_runs() if int(run["pid"]) == pid), None)
    if entry and not record.get("run_id"):
        record.update(run_id=str(entry["run_id"]), ever_ran=True)
        _save(record)
    running = is_launched_process(record)
    stopping = bool(record.get("stopping"))
    if running and (stopping or (record.get("ever_ran") and not entry)):
        phase = "stopping"
    elif entry:
        phase = "running"
    elif running:
        phase = "starting"
    elif record.get("ever_ran") or stopping:
        phase = "stopped"
    else:
        phase = "failed"
    run_id = record.get("run_id")
    log_dir = LOG_DIR / run_id if run_id else None
    output = _tail(launch_log(), constants.LAUNCH_OUTPUT_TAIL_BYTES)
    launch: dict[str, Any] = {
        "blueprint": record["blueprint"],
        "phase": phase,
        "startedAt": record["started_at"],
        "pid": pid,
        "output": output,
        "runId": run_id,
        "logDir": str(log_dir) if log_dir and log_dir.is_dir() else None,
        "error": last_error(output) if phase == "failed" else None,
        "config": record.get("config") or {"global": {}, "modules": {}},
    }
    if record.get("args"):
        launch["args"] = record["args"]
    return launch


def active() -> dict[str, Any]:
    """The running blueprint, however it was started, and the gateway's last launch."""
    entry = get_most_recent(alive_only=True)
    launch = current_launch()
    starting = launch if launch is not None and launch["phase"] == "starting" else {}
    return {
        "running": entry is not None,
        "runId": entry.run_id if entry else None,
        "pid": entry.pid if entry else starting.get("pid"),
        "blueprint": entry.blueprint if entry else starting.get("blueprint"),
        "startedAt": entry.started_at if entry else None,
        "logDir": entry.log_dir if entry else None,
        "launch": launch,
    }


def gossip_env() -> dict[str, str]:
    gossip = store.setting("zenoh", "gossip")
    return {} if gossip is None else {"ZENOH_GOSSIP": "true" if gossip else "false"}


def run_args(blueprint: str, launch: LaunchConfig) -> list[str]:
    global_, modules = without_secrets(launch.global_, launch.modules, launch.secret_paths())
    if launch.args:
        flags = global_flags(global_, explicit_bools=True)
        return ["run", blueprint, *flags, *module_flags(modules), *launch.args]
    return [*global_flags(global_), "run", blueprint, *module_flags(modules)]


def _spawn(command: list[str], shown: list[str], cwd: str, env: dict[str, str]) -> int:
    launch_log().parent.mkdir(parents=True, exist_ok=True)
    launch_log().write_text(f"$ {shlex.join(shown)}\n")
    with launch_log().open("a") as log:
        child = subprocess.Popen(
            command,
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    _reaped[child.pid] = threading.Event()
    threading.Thread(target=_wait_then_tell, args=(child,), daemon=True).start()
    return child.pid


def _wait_then_tell(child: subprocess.Popen[bytes]) -> None:
    try:
        child.wait()
    finally:
        _reaped[child.pid].set()
    for listener in list(exit_listeners):
        listener()


def _remember(blueprint: str, pid: int, request: dict[str, Any], shown: dict[str, Any]) -> None:
    request_file().unlink(missing_ok=True)
    request_file().parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(request_file(), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w") as handle:
        json.dump({"blueprint": blueprint, **request}, handle)
    _save(
        {
            "blueprint": blueprint,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "pid": pid,
            "pid_started": started_time(pid),
            "ever_ran": False,
            **shown,
        }
    )


def check_idle() -> None:
    previous = current_launch()
    if previous and previous["phase"] in ("starting", "running", "stopping"):
        raise StillRunningError(
            f"{previous['blueprint']} is still {previous['phase']}; stop it first"
        )
    if other := next(iter(registry_runs()), None):
        raise StillRunningError(
            f"{other['blueprint']} (run {other['run_id']}, pid {other['pid']}) is running; stop it first"
        )


def start(dimos_dir: Path, blueprint: str, launch: LaunchConfig) -> dict[str, Any]:
    with _lock:
        check_idle()
        program = store.dimos_bin(dimos_dir)
        if not program.exists():
            raise RunError(f"no dimos at {dimos_dir} (no {program})")
        paths = launch.secret_paths()
        secret_vars = secret_env(launch.global_, launch.modules, paths)
        shown_env = [f"{name}={HIDDEN}" for name in secret_vars]
        shown_launch_args = shown_args(launch.args)
        shown_run = run_args(blueprint, replace(launch, args=shown_launch_args))
        venv = store.venv_dir(dimos_dir)
        env = {
            **os.environ,
            "PATH": f"{venv / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}",
            "VIRTUAL_ENV": str(venv),
            "PYTHONUNBUFFERED": "1",
            "NO_COLOR": "1",
            **launch_env_defaults(),
            # dtop's stats for GET /active/stats; a --no-dtop flag still wins
            DTOP_ENV: os.environ.get(DTOP_ENV, "1"),
            # Desktop's zenoh gossip setting, which dimos's GlobalConfig.zenoh_gossip reads from the env
            **gossip_env(),
            **secret_vars,
        }
        pid = _spawn(
            [str(program), *run_args(blueprint, launch)],
            [*shown_env, "dimos", *shown_run],
            str(dimos_dir),
            env,
        )
        global_, modules = redact(launch.global_, launch.modules, paths)
        shown = {"config": {"global": global_, "modules": modules}}
        _remember(
            blueprint,
            pid,
            launch.to_json(),
            {**shown, **({"args": shown_launch_args} if launch.args else {})},
        )
        started = current_launch()
        assert started is not None
        return started


def copy_of(entry: Any) -> dict[str, Any]:
    """The command line, working directory and environment that started a run the gateway didn't launch."""
    # from psutil, since the run registry doesn't record the working directory or environment
    try:
        process = psutil.Process(entry.pid)
        with process.oneshot():
            return {"command": process.cmdline(), "cwd": process.cwd(), "env": process.environ()}
    except psutil.Error as error:
        raise RunError(f"can't read how pid {entry.pid} was started: {error}")


def start_copy(blueprint: str, copy: dict[str, Any]) -> dict[str, Any]:
    with _lock:
        check_idle()
        command = [str(arg) for arg in copy["command"]]
        pid = _spawn(command, shown_args(command), str(copy["cwd"]), dict(copy["env"]))
        _remember(blueprint, pid, {"copy": copy}, {"relaunched": True})
        started = current_launch()
        assert started is not None
        return started


def kill_starting(launch: dict[str, Any]) -> str:
    """SIGKILL a launch that hasn't registered yet: its whole process group, as it may have spawned workers."""
    with suppress(ProcessLookupError, PermissionError):
        os.killpg(launch["pid"], signal.SIGKILL)
    return f"Killed {launch['blueprint']} (PID {launch['pid']}) while it was starting"


def _wait_reaped(pid: int) -> None:
    """Until this gateway's child `pid` is reaped (a stop or kill has made it exit): from then on it isn't running.
    Bounded only for a SIGKILLed process stuck in the kernel (a driver), so the lock isn't held forever."""
    reaped = _reaped.get(pid)
    if reaped is not None and not reaped.wait(constants.KILLED_LAUNCH_EXIT_WAIT_S):
        raise StillRunningError(f"pid {pid} was killed but hasn't exited yet")


def stop() -> str:
    """Exactly `dimos stop`: SIGTERM the newest run, SIGKILL after 5 s. A launch that hasn't registered yet is killed."""
    with _lock:
        entry = get_most_recent(alive_only=True)
        record = _record()
        if entry is None:
            launch = current_launch()
            if launch is None or launch["phase"] != "starting":
                raise ApiError(409, "nothing running")
            message = kill_starting(launch)
            _wait_reaped(int(launch["pid"]))
            return message
        if record is not None and int(record["pid"]) == entry.pid:
            _save({**record, "stopping": True})
        message, _ = stop_entry(entry)
        # stop_entry returns once it has exited; it still counts as running until reaped
        _wait_reaped(entry.pid)
        return f"Stopping {entry.run_id} (PID {entry.pid}) with SIGTERM...\n  {message}"


def relaunch(dimos_dir: Path) -> dict[str, Any]:
    """Stop the running blueprint and start it again: the gateway's own launch as it was POSTed, another one
    (`dimos run` in a terminal) as a copy of its process."""
    # one change: nothing else starts in between the stop and the start
    with _lock:
        entry = get_most_recent(alive_only=True)
        record = _record()
        ours = entry is not None and record is not None and int(record["pid"]) == entry.pid
        request = store.read_json(request_file()) if entry is None or ours else None
        if entry is not None and request is None:
            request = {"blueprint": entry.blueprint, "copy": copy_of(entry)}
        if request is None:
            raise ApiError(
                409, "nothing running, and the gateway hasn't launched anything to relaunch"
            )
        if entry is not None or (
            (launch := current_launch()) is not None and launch["phase"] == "starting"
        ):
            # a stuck or slow startup too: start()'s idle check would refuse while it lives
            stop()
        blueprint = str(request["blueprint"])
        if "copy" in request:
            return start_copy(blueprint, request["copy"])
        return start(dimos_dir, blueprint, LaunchConfig.from_json(request))
