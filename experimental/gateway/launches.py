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
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass, field, replace
import inspect
from itertools import pairwise
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

import psutil
from pydantic import ValidationError

from dimos.constants import LOG_DIR
from dimos.core.coordination.process_lifecycle import is_pid_alive
from dimos.core.run_registry import get_most_recent, list_runs, stop_entry
from experimental.gateway import constants, introspect, shared_config, store
from experimental.gateway.errors import ApiError, StatusError
from experimental.gateway.introspect import is_secret_name
from experimental.gateway.names import check_name

ModuleValues = dict[str, dict[str, Any]]
ConfigTargets = dict[str, list[str]]

HIDDEN = "•••"
STRUCTURED = ("global", "modules", "secrets")

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

    def normalized(self) -> LaunchConfig:
        """Normalize global overrides without changing raw exec requests or adding defaults."""
        return self if self.exact else replace(self, global_=normalize_global(self.global_))

    def secret_paths(self, targets: ConfigTargets | None = None) -> list[str]:
        return secret_paths(self.global_, self.modules, self.secrets, targets)

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

    defaults = module_env({RerunBridgeModule.name: {"rerun_open": "none", "rerun_web": True}})
    if shared_config.shared_config().topics_enabled is True:
        defaults["TOPIC_METADATA"] = "true"
    if zenoh_connect := shared_config.shared_config().zenoh_connect:
        defaults["ZENOH_CONNECT"] = str(zenoh_connect)
    return defaults


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
        raw.get("global", {}),
        raw.get("modules", {}),
        raw.get("secrets", []),
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


def secret_paths(
    global_: dict[str, Any],
    modules: ModuleValues,
    listed: list[str] | tuple[str, ...] = (),
    targets: ConfigTargets | None = None,
) -> list[str]:
    paths: list[str] = []
    explicit = [_secret_path(path, targets) for path in listed]

    def visit(value: Any, path: str, name: str) -> None:
        comparable = _secret_path(path, targets)
        if is_secret_name(name) or any(_within(comparable, parent) for parent in explicit):
            paths.append(path)
        elif isinstance(value, dict):
            for key, child in sorted(value.items()):
                visit(child, f"{path}.{key}" if path else key, key)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                visit(child, f"{path}.{index}", "")
        elif any(_within(secret, comparable) for secret in explicit):
            # An encoded collection cannot be redacted by leaf; protect the whole value.
            paths.append(path)

    visit(global_, "", "")
    for module, fields in sorted(modules.items()):
        visit(fields, module, "")
    return paths


def _within(path: str, parent: str) -> bool:
    return path == parent or path.startswith(parent + ".")


def _contains_secret(path: str, paths: list[str]) -> bool:
    return any(_within(path, secret) or _within(secret, path) for secret in paths)


def _option_name(name: str) -> str:
    return name.lower().replace("-", "_")


def _module_targets(module: str, targets: ConfigTargets | None) -> list[str] | None:
    if targets is None:
        return None
    return next(
        (
            paths
            for name, paths in targets.items()
            if _option_name(name.replace("/", "_")) == _option_name(module.replace("/", "_"))
        ),
        [],
    )


def _secret_path(path: str, targets: ConfigTargets | None) -> str:
    """Match schema aliases without normalizing case or punctuation in dictionary keys."""
    if targets is None:
        return path
    root, dot, field = path.partition(".")
    leaves = _module_targets(root, targets)
    prefix = ""
    if leaves:
        prefix = _option_name(root.replace("/", "_")) + dot
    else:
        field, leaves = path, targets.get("g", [])
    parts = field.split(".")
    boundary = min(
        (
            len(leaf.split("."))
            for leaf in leaves
            if _within(_option_name(field), _option_name(leaf))
        ),
        default=len(parts),
    )
    return prefix + ".".join([*map(_option_name, parts[:boundary]), *parts[boundary:]])


def as_text(value: Any) -> str:
    return value if isinstance(value, str) else json.dumps(value, separators=(",", ":"))


def module_env(modules: ModuleValues) -> dict[str, str]:
    """dimos's env names for module settings: MODULE__FIELD (a dotted field: MODULE__A__B)."""
    return {
        f"{module.replace('/', '_')}__{name.replace('.', '__')}".upper(): as_text(value)
        for module, fields in modules.items()
        for name, value in fields.items()
        if value is not None
    }


def secret_env(
    global_: dict[str, Any],
    modules: ModuleValues,
    paths: list[str],
    targets: ConfigTargets | None = None,
) -> dict[str, str]:
    env = {
        (key.upper() if "." not in key else "G__" + key.replace(".", "__").upper()): as_text(value)
        for key, value in global_.items()
        if _contains_secret(key, paths) and value is not None
    }
    secret = {
        module: {
            name: value
            for name, value in configured_fields(fields, _module_targets(module, targets)).items()
            if _contains_secret(f"{module}.{name}", paths)
        }
        for module, fields in modules.items()
    }
    return {**env, **module_env(secret)}


def without_secrets(
    global_: dict[str, Any],
    modules: ModuleValues,
    paths: list[str],
    targets: ConfigTargets | None = None,
) -> tuple[dict[str, Any], ModuleValues]:
    flat = {key: value for key, value in global_.items() if not _contains_secret(key, paths)}
    nested = {
        module: {
            name: value
            for name, value in configured_fields(fields, _module_targets(module, targets)).items()
            if not _contains_secret(f"{module}.{name}", paths)
        }
        for module, fields in modules.items()
    }
    return flat, {module: fields for module, fields in nested.items() if fields}


def redact(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    def visit(value: Any, path: str) -> Any:
        if value is not None and any(_within(path, secret) for secret in paths):
            return HIDDEN
        if isinstance(value, dict):
            return {
                key: visit(child, f"{path}.{key}" if path else key) for key, child in value.items()
            }
        if isinstance(value, list):
            return [visit(child, f"{path}.{index}") for index, child in enumerate(value)]
        return value

    return visit(global_, ""), {module: visit(fields, module) for module, fields in modules.items()}


def flag_fields() -> set[str]:
    from dimos.cli.commands.global_options import create_dynamic_callback

    callback = create_dynamic_callback()  # type: ignore[no-untyped-call]
    return set(inspect.signature(callback).parameters) - {"ctx"}


def normalize_global(values: dict[str, Any]) -> dict[str, Any]:
    """Return typed overrides, preserving the request's keys and omitted (null) values."""
    from dimos.core.global_config import GlobalConfig

    unknown = sorted(set(values) - flag_fields())
    if unknown:
        raise LaunchConfigError(f"not a dimos GlobalConfig flag: {', '.join(unknown)}")
    try:
        validated = GlobalConfig.model_validate(
            {key: value for key, value in values.items() if value is not None}
        )
    except ValidationError as error:
        problems = "; ".join(
            f"{'.'.join(map(str, problem['loc']))}: {problem['msg']}" for problem in error.errors()
        )
        raise LaunchConfigError(f"bad GlobalConfig value: {problems}") from error
    # as GlobalConfig types them (a "true" is True, 3.0 is 3): the flags are spelled from these
    return {
        key: getattr(validated, key) if value is not None else None for key, value in values.items()
    }


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


def configured_fields(fields: dict[str, Any], targets: list[str] | None = None) -> dict[str, Any]:
    """Normalize dotted/object input at parser leaves, preserving collection data keys.

    Model fields become leaf options; dictionaries and lists stay atomic. Without a
    schema, conservatively treat each top-level field as one atomic value.
    """
    out: dict[str, Any] = {}
    leaves = sorted(
        set(targets) if targets is not None else {name.split(".")[0] for name in fields},
        key=lambda path: (path.count("."), path),
    )

    def merge(destination: dict[str, Any], name: str, value: Any) -> None:
        if isinstance(value, dict):
            current = destination.setdefault(name, {})
            if not isinstance(current, dict):
                raise LaunchConfigError(f"conflicting object and scalar for {name}")
            for key, child in value.items():
                merge(current, key, child)
        elif name in destination and destination[name] != value:
            raise LaunchConfigError(f"conflicting values for {name}")
        else:
            destination[name] = value

    def visit(values: dict[str, Any], prefix: str = "") -> None:
        for name, value in sorted(values.items()):
            if value is None:
                continue
            path = f"{prefix}.{name}" if prefix else name
            target = next(
                (leaf for leaf in leaves if _within(_option_name(path), _option_name(leaf))),
                None,
            )
            if target is not None:
                # Retain the submitted spelling for secret matching; only schema
                # lookup uses CLI aliases, never keys inside collection values.
                leaf = ".".join(path.split(".")[: len(target.split("."))])
                if any(_option_name(key) == _option_name(leaf) and key != leaf for key in out):
                    raise LaunchConfigError(f"conflicting spellings for {leaf}")
                suffix = path[len(leaf) :].removeprefix(".")
                if suffix:
                    for part in reversed(suffix.split(".")):
                        value = {part: value}
                merge(out, leaf, value)
            elif any(
                _within(_option_name(leaf), _option_name(path)) for leaf in leaves
            ) and isinstance(value, dict):
                visit(value, path)
            else:
                raise LaunchConfigError(f"not a settable module config field: {path}")

    visit(fields)
    return out


def module_flags(modules: ModuleValues, targets: ConfigTargets | None = None) -> list[str]:
    flags = []
    for module, fields in sorted(modules.items()):
        root = module.replace("/", "_").replace("_", "-")
        for name, value in configured_fields(fields, _module_targets(module, targets)).items():
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


# dimos' console format: `HH:MM:SS.mmm [lvl][file]`
FATAL_LOG_LINE = re.compile(r"^\d\d:\d\d:\d\d\.\d+ \[(?:err|cri)\]")
# a record of dimos' console, or of a native library (zenoh's start with an ISO date)
LOG_LINE = re.compile(r"^(?:\d\d:\d\d:\d\d\.\d+ \[|\d{4}-\d\d-\d\dT)")
EXCEPTION_LINE = re.compile(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*(?:: .*)?")


def last_error(output: str) -> str:
    """The cause of a failed launch: a native abort, else the last fatal line or the exception
    ending the last traceback.

    Shutdown chatter (`Stopping module...`, `All workers shut down`) follows a crash, so the
    last line alone names the wrong thing. The last recognized cause wins, including shutdown
    errors. A native abort wins over all that follows it: dimos reports the worker it killed
    as a failed health check. Without a recognized cause, the last line still stands, as before.

    Rich draws tracebacks and CLI errors in panels. A panel's text is read without its frame,
    a wrapped message as one line, and a border is never the answer."""
    lines: list[str] = []
    wraps = False  # the line before is text Rich may have wrapped onto this one
    for line in output.splitlines()[1:]:
        framed = line.startswith("│")
        text = line.strip("│ ") if framed else line.rstrip()
        is_text = text[:1] not in ("", "╭", "╰") and not LOG_LINE.match(text)
        if wraps and is_text:
            lines[-1] += f" {text}"
            continue
        # Rich wraps the text inside a panel, and the exception it prints right below one.
        wraps = is_text and (framed or (bool(lines) and lines[-1].startswith("╰")))
        if text:
            lines.append(text)
    cause = None
    in_traceback = False
    for above, line in pairwise(["", *lines]):
        if line.startswith("Traceback (most recent call last):"):
            in_traceback = True
        elif line.startswith("*** Terminating app"):
            return line
        elif FATAL_LOG_LINE.match(line):
            cause = line.strip()
        # Rich prints the exception right below the panel of frames.
        elif (in_traceback or above.startswith("╰")) and EXCEPTION_LINE.fullmatch(line):
            # Other workers' output can land between the frames and the exception.
            if re.match(r"(?:DEBUG|INFO|WARNING|WARN|ERROR|CRITICAL|FATAL|NOTSET):", line):
                continue
            cause = line.strip()
            in_traceback = False
    if cause is None:
        lines = [line for line in lines if not line.startswith(("╭", "╰"))]
        return lines[-1].strip() if lines else "dimos exited during startup"
    return cause


def current_launch() -> dict[str, Any] | None:
    """The gateway's last launch; its runId is the run registry's, known once the run registers."""
    record = _record()
    if record is None:
        return None
    pid = int(record["pid"])
    entry = next((run for run in registry_runs() if int(run["pid"]) == pid), None)
    if entry and not record.get("run_id"):
        # where the run logs is its own dimos's to say: the served checkout's logs/, not this gateway's
        record.update(run_id=str(entry["run_id"]), log_dir=entry.get("log_dir"), ever_ran=True)
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
    log_dir = (
        Path(record["log_dir"]) if record.get("log_dir") else LOG_DIR / run_id if run_id else None
    )
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


def run_args(
    blueprint: str, launch: LaunchConfig, targets: ConfigTargets | None = None
) -> list[str]:
    global_, modules = without_secrets(
        launch.global_, launch.modules, launch.secret_paths(targets), targets
    )
    if launch.args:
        flags = global_flags(global_, explicit_bools=True)
        return ["run", blueprint, *flags, *module_flags(modules, targets), *launch.args]
    return [*global_flags(global_), "run", blueprint, *module_flags(modules, targets)]


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
    check_name(blueprint)
    return _start(dimos_dir, blueprint, launch.normalized())


def _start(dimos_dir: Path, blueprint: str, launch: LaunchConfig) -> dict[str, Any]:
    """Spawn a prepared launch; relaunch also prepares before stopping the current process."""
    with _lock:
        check_idle()
        program = store.dimos_bin(dimos_dir)
        if not program.exists():
            raise RunError(f"no dimos at {dimos_dir} (no {program})")
        targets = None
        if launch.modules:
            try:
                targets = asyncio.run(
                    introspect.run_child(dimos_dir, ["targets", blueprint], stdin=launch.global_)
                )
            except introspect.IntrospectError as error:
                raise LaunchConfigError(f"could not read launch config fields: {error}") from error
        paths = launch.secret_paths(targets)
        secret_vars = secret_env(launch.global_, launch.modules, paths, targets)
        shown_env = [f"{name}={HIDDEN}" for name in secret_vars]
        shown_launch_args = shown_args(launch.args)
        shown_run = run_args(blueprint, replace(launch, args=shown_launch_args), targets)
        venv = store.venv_dir(dimos_dir)
        env = {
            **os.environ,
            "PATH": f"{venv / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}",
            "VIRTUAL_ENV": str(venv),
            "DIMOS_RECORDINGS_DIR": str(store.recordings_dir(dimos_dir).absolute()),
            "PYTHONUNBUFFERED": "1",
            "NO_COLOR": "1",
            **launch_env_defaults(),
            **secret_vars,
        }
        pid = _spawn(
            [str(program), *run_args(blueprint, launch, targets)],
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
    check_name(blueprint)
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
            assert record is not None
            # asked for, so its exit is a stop: without this the dead launch reads as failed
            _save({**record, "stopping": True})
            message = kill_starting(launch)
            _wait_reaped(int(launch["pid"]))
            return message
        if record is not None and int(record["pid"]) == entry.pid:
            _save({**record, "stopping": True})
        message, _ = stop_entry(entry)
        # stop_entry returns once it has exited; it still counts as running until reaped
        _wait_reaped(entry.pid)
        return f"Stopping {entry.run_id} (PID {entry.pid}) with SIGTERM...\n  {message}"


def relaunch(dimos_dir: Path, check_known: Callable[[str], None]) -> dict[str, Any]:
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
        blueprint = check_name(request.get("blueprint"))
        # A copied process may belong to another checkout, outside this catalog.
        if "copy" not in request:
            check_known(blueprint)
        config = None if "copy" in request else LaunchConfig.from_json(request).normalized()
        if entry is not None or (
            (launch := current_launch()) is not None and launch["phase"] == "starting"
        ):
            # a stuck or slow startup too: start()'s idle check would refuse while it lives
            stop()
        if "copy" in request:
            return start_copy(blueprint, request["copy"])
        assert config is not None
        return _start(dimos_dir, blueprint, config)
