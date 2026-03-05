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
        default_config = MyConfig
        pointcloud: Out[PointCloud2]
        cmd_vel: In[Twist]

    # Works with autoconnect, remappings, etc.
    autoconnect(
        MyCppModule.blueprint(),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import enum
import inspect
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
from typing import IO, Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class LogFormat(enum.Enum):
    TEXT = "text"
    JSON = "json"


@dataclass(kw_only=True)
class NativeModuleConfig(ModuleConfig):
    """Configuration for a native (C/C++) subprocess module."""

    executable: str
    build_command: str | None = None
    cwd: str | None = None
    extra_args: list[str] = field(default_factory=list)
    extra_env: dict[str, str] = field(default_factory=dict)
    shutdown_timeout: float = 10.0
    log_format: LogFormat = LogFormat.TEXT

    # Override in subclasses to exclude fields from CLI arg generation
    cli_exclude: frozenset[str] = frozenset()

    def to_cli_args(self) -> list[str]:
        """Auto-convert subclass config fields to CLI args.

        Iterates fields defined on the concrete subclass (not NativeModuleConfig
        or its parents) and converts them to ``["--name", str(value)]`` pairs.
        Skips fields whose values are ``None`` and fields in ``cli_exclude``.
        """
        ignore_fields = {f.name for f in fields(NativeModuleConfig)}
        args: list[str] = []
        for f in fields(self):
            if f.name in ignore_fields:
                continue
            if f.name in self.cli_exclude:
                continue
            val = getattr(self, f.name)
            if val is None:
                continue
            if isinstance(val, bool):
                args.extend([f"--{f.name}", str(val).lower()])
            elif isinstance(val, list):
                args.extend([f"--{f.name}", ",".join(str(v) for v in val)])
            else:
                args.extend([f"--{f.name}", str(val)])
        return args


class NativeModule(Module[NativeModuleConfig]):
    """Module that wraps a native executable as a managed subprocess.

    Subclass this, declare In/Out ports, and set ``default_config`` to a
    :class:`NativeModuleConfig` subclass pointing at the executable.

    On ``start()``, the binary is launched with CLI args::

        <executable> --<port_name> <lcm_topic_string> ... <extra_args>

    The native process should parse these args and pub/sub on the given
    LCM topics directly.  On ``stop()``, the process receives SIGTERM.
    """

    default_config: type[NativeModuleConfig] = NativeModuleConfig
    _process: subprocess.Popen[bytes] | None = None
    _watchdog: threading.Thread | None = None
    _stopping: bool = False

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._resolve_paths()

    @rpc
    def start(self) -> None:
        if self._process is not None and self._process.poll() is None:
            logger.warning("Native process already running", pid=self._process.pid)
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

        logger.info("Starting native process", cmd=" ".join(cmd), cwd=cwd)
        self._process = subprocess.Popen(
            cmd,
            env=env,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        logger.info("Native process started", pid=self._process.pid)

        self._stopping = False
        self._watchdog = threading.Thread(target=self._watch_process, daemon=True)
        self._watchdog.start()

    @rpc
    def stop(self) -> None:
        self._stopping = True
        if self._process is not None and self._process.poll() is None:
            logger.info("Stopping native process", pid=self._process.pid)
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout=self.config.shutdown_timeout)
            except subprocess.TimeoutExpired:
                logger.warning(
                    "Native process did not exit, sending SIGKILL", pid=self._process.pid
                )
                self._process.kill()
                self._process.wait(timeout=5)
        if self._watchdog is not None and self._watchdog is not threading.current_thread():
            self._watchdog.join(timeout=2)
        self._watchdog = None
        self._process = None
        super().stop()

    def _watch_process(self) -> None:
        """Block until the native process exits; trigger stop() if it crashed."""
        if self._process is None:
            return

        stdout_t = self._start_reader(self._process.stdout, "info")
        stderr_t = self._start_reader(self._process.stderr, "warning")
        rc = self._process.wait()
        stdout_t.join(timeout=2)
        stderr_t.join(timeout=2)

        if self._stopping:
            return
        logger.error(
            "Native process died unexpectedly",
            pid=self._process.pid,
            returncode=rc,
        )
        self.stop()

    def _start_reader(self, stream: IO[bytes] | None, level: str) -> threading.Thread:
        """Spawn a daemon thread that pipes a subprocess stream through the logger."""
        t = threading.Thread(target=self._read_log_stream, args=(stream, level), daemon=True)
        t.start()
        return t

    def _read_log_stream(self, stream: IO[bytes] | None, level: str) -> None:
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
                    log_fn(event, **data)
                    continue
                except (json.JSONDecodeError, TypeError):
                    logger.warning("malformed JSON from native module", raw=line)
            log_fn(line, pid=self._process.pid if self._process else None)
        stream.close()

    def _resolve_paths(self) -> None:
        """Resolve relative ``cwd`` and ``executable`` against the subclass's source file."""
        if self.config.cwd is not None and not Path(self.config.cwd).is_absolute():
            source_file = inspect.getfile(type(self))
            base_dir = Path(source_file).resolve().parent
            self.config.cwd = str(base_dir / self.config.cwd)
        if not Path(self.config.executable).is_absolute() and self.config.cwd is not None:
            self.config.executable = str(Path(self.config.cwd) / self.config.executable)

    def _maybe_build(self) -> None:
        """Run ``build_command`` if the executable does not exist."""
        exe = Path(self.config.executable)
        if exe.exists():
            return
        if self.config.build_command is None:
            raise FileNotFoundError(
                f"Executable not found: {exe}. "
                "Set build_command in config to auto-build, or build it manually."
            )
        logger.info(
            "Executable not found, running build",
            executable=str(exe),
            build_command=self.config.build_command,
        )
        proc = subprocess.Popen(
            self.config.build_command,
            shell=True,
            cwd=self.config.cwd,
            env={**os.environ, **self.config.extra_env},
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        stdout, stderr = proc.communicate()
        for line in stdout.decode("utf-8", errors="replace").splitlines():
            if line.strip():
                logger.info(line)
        for line in stderr.decode("utf-8", errors="replace").splitlines():
            if line.strip():
                logger.warning(line)
        if proc.returncode != 0:
            raise RuntimeError(
                f"Build command failed (exit {proc.returncode}): {self.config.build_command}"
            )
        if not exe.exists():
            raise FileNotFoundError(
                f"Build command succeeded but executable still not found: {exe}"
            )

    def _collect_topics(self) -> dict[str, str]:
        """Extract LCM topic strings from blueprint-assigned stream transports."""
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
