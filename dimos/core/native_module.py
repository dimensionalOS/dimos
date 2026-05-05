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

"""NativeModule: blueprint-integrated wrapper for native (C/C++) executables.

A NativeModule is a thin Python Module subclass that declares In/Out ports
for blueprint wiring but delegates all real work to a managed subprocess.
The native process receives its LCM topic names via CLI args and does
pub/sub directly on the LCM multicast bus.

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

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
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


class NativeModuleConfig(ModuleConfig):
    executable: str
    build_command: str | None = None
    cwd: str | None = None
    extra_args: list[str] = Field(default_factory=list)
    extra_env: dict[str, str] = Field(default_factory=dict)
    shutdown_timeout: float = DEFAULT_THREAD_JOIN_TIMEOUT
    log_format: LogFormat = LogFormat.TEXT
    auto_build: bool = False

    # New version of Native Modules read json configs from stdin
    # Enable this to read from stdin instead of cli args
    stdin_config: bool = False

    cli_exclude: frozenset[str] = frozenset()
    cli_name_override: dict[str, str] = Field(default_factory=dict)

    def to_config_dict(self) -> dict[str, Any]:
        """
        Return module-specific config fields as a plain dict (for stdin JSON).
        """
        ignore_fields = set(NativeModuleConfig.model_fields)
        return {
            k: v for k, v in self.model_dump().items() if k not in ignore_fields and v is not None
        }

    def to_cli_args(self) -> list[str]:
        """Convert subclass config fields to CLI args (--name value)."""
        ignore_fields = {f for f in NativeModuleConfig.model_fields}
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
    """Module that wraps a native executable as a managed subprocess."""

    config: NativeModuleConfig

    _process: subprocess.Popen[bytes] | None = None
    _watchdog: threading.Thread | None = None
    _stopping: bool = False
    _stop_lock: threading.Lock

    @functools.cached_property
    def _mod_label(self) -> str:
        exe = Path(self.config.executable).name if self.config.executable else "?"
        return f"{type(self).__name__}({exe})"

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_lock = threading.Lock()

        if self.config.cwd is not None and not Path(self.config.cwd).is_absolute():
            base_dir = Path(inspect.getfile(type(self))).resolve().parent
            self.config.cwd = str(base_dir / self.config.cwd)
        if not Path(self.config.executable).is_absolute() and self.config.cwd is not None:
            self.config.executable = str(Path(self.config.cwd) / self.config.executable)

    @rpc
    def start(self) -> None:
        super().start()
        if self._process is not None and self._process.poll() is None:
            logger.warning(
                "Native process already running",
                module=self._mod_label,
                pid=self._process.pid,
            )
            return

        self._maybe_build()

        topics = self._collect_topics()

        cmd = [self.config.executable]
        for name, topic_str in topics.items():
            cmd.extend([f"--{name}", topic_str])
        cmd.extend(self.config.to_cli_args())
        cmd.extend(self.config.extra_args)

        env = {**os.environ, **self.config.extra_env}
        cwd = self.config.cwd or str(Path(self.config.executable).resolve().parent)

        logger.info(
            "Starting native process",
            module=self._mod_label,
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
        if self.config.stdin_config:
            config_dict = self.config.to_config_dict()
            stdin_blob = (
                json.dumps({"topics": topics, "config": config_dict or None}).encode() + b"\n"
            )
            self._process.stdin.write(stdin_blob)
        self._process.stdin.close()
        logger.info(
            "Native process started",
            module=self._mod_label,
            pid=self._process.pid,
        )

        watchdog = threading.Thread(
            target=self._watch_process,
            daemon=True,
            name=f"native-watchdog-{self._mod_label}",
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
                module=self._mod_label,
                pid=proc.pid,
            )
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=self.config.shutdown_timeout)
            except subprocess.TimeoutExpired:
                logger.warning(
                    "Native process did not exit, sending SIGKILL",
                    module=self._mod_label,
                    pid=proc.pid,
                )
                proc.kill()
                try:
                    proc.wait(timeout=self.config.shutdown_timeout)
                except subprocess.TimeoutExpired:
                    logger.error(
                        "Native process not reapable after SIGKILL",
                        module=self._mod_label,
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
                module=self._mod_label,
                pid=pid,
                returncode=rc,
            )
            return

        logger.error(
            "Native process died unexpectedly",
            module=self._mod_label,
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
            name=f"native-reader-{level}-{self._mod_label}",
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
        log_fn = getattr(logger, level)
        for raw in stream:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            if self.config.log_format == LogFormat.JSON:
                try:
                    data = json.loads(line)
                    event = data.pop("event", line)
                    log_fn(event, module=self._mod_label, pid=pid, **data)
                    continue
                except (json.JSONDecodeError, TypeError):
                    pass
            log_fn(line, module=self._mod_label, pid=pid)
        stream.close()

    def _maybe_build(self) -> None:
        exe = Path(self.config.executable)

        if self.config.build_command is None:
            if not exe.exists():
                raise FileNotFoundError(
                    f"[{self._mod_label}] Executable not found: {exe}. "
                    "Set build_command in config to auto-build, or build it manually."
                )
            return

        if exe.exists() and not self.config.auto_build and not global_config.build_native:
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
            stderr=subprocess.PIPE,
        )
        stdout, stderr = proc.communicate()
        build_elapsed = time.perf_counter() - build_start

        stdout_lines = stdout.decode("utf-8", errors="replace").splitlines()
        stderr_lines = stderr.decode("utf-8", errors="replace").splitlines()

        for line in stdout_lines:
            if line.strip():
                logger.info(line, module=self._mod_label)
        for line in stderr_lines:
            if line.strip():
                logger.warning(line, module=self._mod_label)

        if proc.returncode != 0:
            raise RuntimeError(
                f"[{self._mod_label}] Build command failed after {build_elapsed:.2f}s "
                f"(exit {proc.returncode}): {self.config.build_command}"
            )
        if not exe.exists():
            raise FileNotFoundError(
                f"[{self._mod_label}] Build command succeeded but executable still not found: {exe}"
            )

        logger.info(
            "Build command completed",
            module=self._mod_label,
            executable=str(exe),
            duration_sec=round(build_elapsed, 3),
        )

    def _collect_topics(self) -> dict[str, str]:
        topics: dict[str, str] = {}
        for name in list(self.inputs) + list(self.outputs):
            stream = getattr(self, name, None)
            if stream is None:
                continue
            transport = getattr(stream, "_transport", None)
            if transport is None:
                continue
            topic = getattr(transport, "topic", None)
            if topic is not None:
                topics[name] = str(topic)
        return topics


__all__ = [
    "LogFormat",
    "NativeModule",
    "NativeModuleConfig",
]
