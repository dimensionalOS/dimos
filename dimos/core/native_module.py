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

"""NativeModule: blueprint-integrated wrapper for native executables.

A NativeModule is a thin Python Module subclass that declares In/Out/IO ports
for blueprint wiring but delegates all real work to a managed subprocess.
The native process receives its topic names via CLI args, or as a JSON line on
stdin when ``stdin_config`` is set, and does pub/sub on them directly.

Example usage::

    @dataclass(kw_only=True)
    class MyConfig(NativeModuleConfig):
        executable: str = "./build/my_module"
        some_param: float = 1.0

    class MyCppModule(NativeModule):
        config: MyConfig
        pointcloud: Out[PointCloud2]
        cmd_vel: In[Twist]

    # Works with autoconnect, remappings, etc.
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        MyCppModule.blueprint(),
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import enum
import functools
import inspect
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import IO, Any

from pydantic import Field, model_validator

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.transport_factory import channel_for, session_config
from dimos.protocol.service.spec import SessionConfig
from dimos.utils.logging_config import setup_logger

if sys.platform.startswith("linux"):
    import ctypes
    from ctypes.util import find_library

    _LIBC = ctypes.CDLL(find_library("c"), use_errno=True)

    def _set_process_to_die_when_parent_dies() -> None:
        _PR_SET_PDEATHSIG = 1
        if _LIBC.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM) != 0:
            err = ctypes.get_errno()
            raise OSError(err, f"_set_process_to_die_when_parent_dies failed: {os.strerror(err)}")
else:
    _set_process_to_die_when_parent_dies = None  # type: ignore[assignment]

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

logger = setup_logger()


class LogFormat(enum.Enum):
    TEXT = "text"
    JSON = "json"


# convert to Python levels
_NATIVE_TO_PYTHON_LEVELS = {
    "trace": "debug",
    "debug": "debug",
    "info": "info",
    "warn": "warning",
    "warning": "warning",
    "err": "error",
    "error": "error",
    "fatal": "critical",
    "critical": "critical",
}

# convert to Rust levels
_PYTHON_TO_RUST_LEVELS = {
    "DEBUG": "debug",
    "INFO": "info",
    "WARNING": "warn",
    "ERROR": "error",
    "CRITICAL": "error",
}


class NativeModuleConfig(ModuleConfig):
    """Configuration for a native subprocess module."""

    executable: str
    build_command: str | None = None
    cwd: str | None = None
    extra_args: list[str] = Field(default_factory=list)
    extra_env: dict[str, str] = Field(default_factory=dict)
    # Session settings for this module alone, e.g. opening it as the zenoh router
    # the rest of the graph connects to. None follows the global config.
    session: SessionConfig | None = None
    shutdown_timeout: float = DEFAULT_THREAD_JOIN_TIMEOUT
    log_format: LogFormat = LogFormat.JSON
    auto_build: bool = False

    stdin_config: bool = False

    cli_exclude: frozenset[str] = frozenset()
    cli_name_override: dict[str, str] = Field(default_factory=dict)

    # Native config structs reject unknown fields, so a base field only crosses
    # the boundary if that module's native struct declares it.
    base_fields: frozenset[str] = frozenset()

    @model_validator(mode="after")
    def _session_needs_the_stdin_line(self) -> NativeModuleConfig:
        if self.session is not None and not self.stdin_config:
            raise ValueError(
                f"{self.executable} pins a session config but has stdin_config off, "
                "so the module would open its own defaults instead"
            )
        return self

    def _ignore_fields(self) -> set[str]:
        return set(NativeModuleConfig.model_fields) - self.base_fields

    def to_config_dict(self) -> dict[str, Any]:
        """Module-specific config fields as a plain dict, for the stdin JSON."""
        ignore_fields = self._ignore_fields()
        # An opted-in base field is sent even when None, so the native struct
        # reports a null it can name rather than a field that looks unset.
        return {
            k: v
            for k, v in self.model_dump().items()
            if k not in ignore_fields and (v is not None or k in self.base_fields)
        }

    def to_cli_args(self) -> list[str]:
        """Convert subclass config fields to CLI args (--name value)."""
        ignore_fields = self._ignore_fields()
        args: list[str] = []
        for f in self.__class__.model_fields:
            if f in ignore_fields:
                continue
            if f in self.cli_exclude:
                continue
            val = getattr(self, f)
            if val is None:
                continue
            cli_name = self.cli_name_override.get(f, f)
            if isinstance(val, bool):
                args.extend([f"--{cli_name}", str(val).lower()])
            elif isinstance(val, list):
                args.extend([f"--{cli_name}", ",".join(str(v) for v in val)])
            else:
                args.extend([f"--{cli_name}", str(val)])
        return args


_NativeConfig = TypeVar("_NativeConfig", bound=NativeModuleConfig, default=NativeModuleConfig)


class NativeModule(Module):
    """
    Module that wraps a native executable as a managed subprocess.

    Subclass this, declare In/Out/IO ports, and annotate ``config`` with a
    :class:`NativeModuleConfig` subclass pointing at the executable.

    On ``start()``, the binary is launched with CLI args::

        <executable> --<port_name> <topic> ... --<config_field> <value> ... <extra_args>

    Each topic is the wire channel for that port on the transport named by the
    ``DIMOS_TRANSPORT`` env var. With ``stdin_config``, the topics, config,
    publisher QoS and session settings also arrive as one JSON line on stdin.

    A port named in ``topic_funnels`` gets a list of channels instead of one, so
    several same-typed streams reach a single native handler.

    The native process should parse whichever it uses and pub/sub on the given
    topics directly. On ``stop()``, the process receives SIGTERM.
    """

    config: NativeModuleConfig

    _process: subprocess.Popen[bytes] | None = None
    _watchdog: threading.Thread | None = None
    _stopping: bool = False
    _stop_lock: threading.Lock

    @functools.cached_property
    def _module_label(self) -> str:
        exe = Path(self.config.executable).name if self.config.executable else "?"
        return f"{type(self).__name__}({exe})"

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_lock = threading.Lock()

        if self.config.cwd is not None and not Path(self.config.cwd).is_absolute():
            base_dir = Path(inspect.getfile(type(self))).resolve().parent
            self.config.cwd = str(base_dir / self.config.cwd)
        if not Path(self.config.executable).is_absolute():
            # The spawn runs from the executable's own directory, so a relative
            # path has to be resolved before then or it resolves against itself.
            base = Path(self.config.cwd) if self.config.cwd is not None else Path.cwd()
            self.config.executable = str(base / self.config.executable)

    @rpc
    def build(self) -> None:
        super().build()
        self._maybe_build()

    def _spawn_env(self) -> dict[str, str]:
        env = {**os.environ, **self.config.extra_env}

        env["DIMOS_TRANSPORT"] = global_config.transport

        env["RUST_LOG"] = _PYTHON_TO_RUST_LEVELS.get(
            os.environ.get("DIMOS_LOG_LEVEL", "").upper(), "info"
        )
        return env

    def _session(self) -> SessionConfig:
        """The session this module's process opens on the active transport."""
        pinned = self.config.session
        if pinned is None:
            return session_config()
        if pinned.transport != global_config.transport:
            raise ValueError(
                f"[{self._module_label}] pins a {pinned.transport} session config, "
                f"but the transport is {global_config.transport}"
            )
        # A blueprint builds its config before global config is settled.
        return pinned.rebased()

    def _argv(self, topics: dict[str, str | list[str]]) -> list[str]:
        """The command line the native process is spawned with."""
        cmd = [self.config.executable]
        for name, topic_str in topics.items():
            joined = ",".join(topic_str) if isinstance(topic_str, list) else topic_str
            cmd.extend([f"--{name}", joined])
        cmd.extend(self.config.to_cli_args())
        cmd.extend(self.config.extra_args)
        return cmd

    def _stdin_blob(self, topics: dict[str, str | list[str]]) -> bytes:
        """The JSON line the native process reads its launch from."""
        config_dict = self.config.to_config_dict()
        blob: dict[str, Any] = {
            "topics": topics,
            "config": config_dict or None,
            "session": self._session().to_wire(),
        }
        qos = self._collect_output_qos()
        if qos:
            blob["qos"] = qos
        return json.dumps(blob).encode() + b"\n"

    @rpc
    def start(self) -> None:
        super().start()
        if self._process is not None and self._process.poll() is None:
            logger.warning(
                "Native process already running",
                module=self._module_label,
                pid=self._process.pid,
            )
            return

        topics = self._collect_topics()
        cmd = self._argv(topics)

        # Built before the spawn: a config that cannot be serialized must fail
        # without leaving a child blocked on a stdin line it will never get.
        stdin_blob = self._stdin_blob(topics) if self.config.stdin_config else None

        env = self._spawn_env()
        cwd = self.config.cwd or str(Path(self.config.executable).resolve().parent)

        logger.info(
            "Starting native process",
            module=self._module_label,
            cmd=" ".join(cmd),
            cwd=cwd,
        )

        self._process = subprocess.Popen(
            cmd,
            env=env,
            cwd=cwd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            preexec_fn=_set_process_to_die_when_parent_dies,
        )
        assert self._process.stdin is not None
        if stdin_blob is not None:
            self._process.stdin.write(stdin_blob)
        self._process.stdin.close()
        logger.info(
            "Native process started",
            module=self._module_label,
            pid=self._process.pid,
        )

        watchdog = threading.Thread(
            target=self._watch_process,
            daemon=True,
            name=f"native-watchdog-{self._module_label}",
        )
        with self._stop_lock:
            self._stopping = False
            self._watchdog = watchdog
        watchdog.start()

    @rpc
    def stop(self) -> None:
        # Capture refs under lock, but signal/wait/join outside it to avoid
        # deadlocking with the watchdog's own stop() call.
        with self._stop_lock:
            if self._stopping:
                return
            self._stopping = True
            proc = self._process
            watchdog = self._watchdog

        if proc is not None and proc.poll() is None:
            logger.info(
                "Stopping native process",
                module=self._module_label,
                pid=proc.pid,
            )
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=self.config.shutdown_timeout)
            except subprocess.TimeoutExpired:
                logger.warning(
                    "Native process did not exit, sending SIGKILL",
                    module=self._module_label,
                    pid=proc.pid,
                )
                proc.kill()
                try:
                    proc.wait(timeout=self.config.shutdown_timeout)
                except subprocess.TimeoutExpired:
                    logger.error(
                        "Native process not reapable after SIGKILL",
                        module=self._module_label,
                        pid=proc.pid,
                    )

        if watchdog is not None and watchdog is not threading.current_thread():
            watchdog.join(timeout=self.config.shutdown_timeout)

        with self._stop_lock:
            self._watchdog = None
            self._process = None

        super().stop()

    def _watch_process(self) -> None:
        proc = self._process
        if proc is None:
            return
        pid = proc.pid

        stdout_t = self._start_reader(proc.stdout, "info", pid)
        stderr_t = self._start_reader(proc.stderr, "warning", pid)
        rc = proc.wait()
        stdout_t.join(timeout=self.config.shutdown_timeout)
        stderr_t.join(timeout=self.config.shutdown_timeout)

        if self._stopping:
            logger.info(
                "Native process exited (expected)",
                module=self._module_label,
                pid=pid,
                returncode=rc,
            )
            return

        logger.error(
            "Native process died unexpectedly",
            module=self._module_label,
            pid=pid,
            returncode=rc,
        )
        self.stop()

    def _start_reader(
        self,
        stream: IO[bytes] | None,
        level: str,
        pid: int,
    ) -> threading.Thread:
        t = threading.Thread(
            target=self._read_log_stream,
            args=(stream, level, pid),
            daemon=True,
            name=f"native-reader-{level}-{self._module_label}",
        )
        t.start()
        return t

    def _read_log_stream(
        self,
        stream: IO[bytes] | None,
        level: str,
        pid: int,
    ) -> None:
        if stream is None:
            return
        default_log_fn = getattr(logger, level)
        for raw in stream:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            if self.config.log_format == LogFormat.JSON:
                try:
                    data = json.loads(line)
                    fields = data.pop("fields", None)
                    if fields:
                        data.update(fields)
                    message = data.pop("message", None) or line
                    msg_level = data.pop("level", None)
                    method = (
                        _NATIVE_TO_PYTHON_LEVELS.get(msg_level.lower(), level)
                        if msg_level
                        else level
                    )
                    getattr(logger, method)(message, module=self._module_label, pid=pid, **data)
                    continue
                except (json.JSONDecodeError, TypeError, AttributeError):
                    pass
            default_log_fn(line, module=self._module_label, pid=pid)
        stream.close()

    def _maybe_build(self) -> None:
        exe = Path(self.config.executable)

        if self.config.build_command is None:
            if not exe.exists():
                raise FileNotFoundError(
                    f"[{self._module_label}] Executable not found: {exe}. "
                    "Set build_command in config to auto-build, or build it manually."
                )
            return

        if exe.exists() and not self.config.auto_build and not self.config.g.build_native:
            return

        logger.info(
            "Building native module",
            executable=str(exe),
            build_command=self.config.build_command,
        )
        build_start = time.perf_counter()
        proc = subprocess.Popen(
            self.config.build_command,
            shell=True,
            cwd=self.config.cwd,
            env={**os.environ, **self.config.extra_env},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        assert proc.stdout is not None
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", errors="replace").rstrip()
            if line:
                logger.info(line, module=self._module_label)
        proc.wait()
        build_elapsed = time.perf_counter() - build_start

        if proc.returncode != 0:
            raise RuntimeError(
                f"[{self._module_label}] Build command failed after {build_elapsed:.2f}s "
                f"(exit {proc.returncode}): {self.config.build_command}"
            )
        if not exe.exists():
            raise FileNotFoundError(
                f"[{self._module_label}] Build command succeeded but executable still not found: {exe}"
            )

        logger.info(
            "Build command completed",
            module=self._module_label,
            executable=str(exe),
            duration_sec=round(build_elapsed, 3),
        )

    def _collect_topics(self) -> dict[str, str | list[str]]:
        topics: dict[str, str | list[str]] = {}
        for name in list(self.inputs) + list(self.outputs) + list(self.ios):
            stream = getattr(self, name, None)
            if stream is None:
                continue
            transport = getattr(stream, "_transport", None)
            if transport is None:
                continue
            channel = getattr(transport, "channel", None)
            if channel is not None:
                topics[name] = channel
        for port, group in self.config.topic_funnels.items():
            if port in topics:
                raise ValueError(
                    f"[{self._module_label}] topic funnel {port!r} collides with the "
                    "port of the same name declared as a stream"
                )
            channels = []
            for name in group.names:
                transport = getattr(self._funnel_streams[name], "_transport", None)
                channel = getattr(transport, "channel", None)
                # Unwired (standalone use): the channel the default transport lands on.
                channels.append(
                    channel if channel is not None else channel_for(name, group.msg_type)
                )
            topics[port] = channels
        return topics

    def _collect_output_qos(self) -> dict[str, dict[str, str]]:
        """Publisher QoS per published channel, keyed by channel."""
        qos_map: dict[str, dict[str, str]] = {}
        for name in list(self.outputs) + list(self.ios):
            stream = getattr(self, name, None)
            if stream is None:
                continue
            transport = getattr(stream, "_transport", None)
            if transport is None:
                continue
            channel = getattr(transport, "channel", None)
            qos = getattr(transport, "publish_qos", None)
            if channel is not None and qos:
                qos_map[channel] = qos
        return qos_map
