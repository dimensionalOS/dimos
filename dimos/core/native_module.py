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
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        MyCppModule.blueprint(),
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import collections
import enum
import inspect
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
from typing import IO, Any

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.change_detect import PathEntry, did_change
from dimos.utils.logging_config import setup_logger

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

logger = setup_logger()


class LogFormat(enum.Enum):
    TEXT = "text"
    JSON = "json"


class NativeModuleConfig(ModuleConfig):
    """Configuration for a native (C/C++) subprocess module."""

    executable: str
    build_command: str | None = None
    cwd: str | None = None
    extra_args: list[str] = Field(default_factory=list)
    extra_env: dict[str, str] = Field(default_factory=dict)
    shutdown_timeout: float = 10.0
    log_format: LogFormat = LogFormat.TEXT
    rebuild_on_change: list[PathEntry] | None = None

    # Override in subclasses to exclude fields from CLI arg generation
    cli_exclude: frozenset[str] = frozenset({"rebuild_on_change"})
    # Override in subclasses to map field names to custom CLI arg names
    # (bypasses the automatic snake_case → camelCase conversion).
    cli_name_override: dict[str, str] = Field(default_factory=dict)

    def to_cli_args(self) -> list[str]:
        """Convert subclass config fields to CLI args.

        Iterates fields defined on the concrete subclass (not NativeModuleConfig
        or its parents) and converts them to ``["--name", str(value)]`` pairs.
        Field names are passed as-is (snake_case) unless overridden via
        ``cli_name_override``.
        Skips fields whose values are ``None`` and fields in ``cli_exclude``.
        """
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


class NativeModule(Module[_NativeConfig]):
    """Module that wraps a native executable as a managed subprocess.

    Subclass this, declare In/Out ports, and set ``default_config`` to a
    :class:`NativeModuleConfig` subclass pointing at the executable.

    On ``start()``, the binary is launched with CLI args::

        <executable> --<port_name> <lcm_topic_string> ... <extra_args>

    The native process should parse these args and pub/sub on the given
    LCM topics directly.  On ``stop()``, the process receives SIGTERM.
    """

    default_config: type[_NativeConfig] = NativeModuleConfig  # type: ignore[assignment]
    _process: subprocess.Popen[bytes] | None = None
    _watchdog: threading.Thread | None = None
    _stopping: bool = False
    _stderr_tail: list[str]
    _stdout_tail: list[str]
    _tail_lock: threading.Lock

    @property
    def _mod_label(self) -> str:
        """Short human-readable label: ClassName(executable_basename)."""
        exe = Path(self.config.executable).name if self.config.executable else "?"
        return f"{type(self).__name__}({exe})"

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stderr_tail: collections.deque[str] = collections.deque(maxlen=50)
        self._stdout_tail: collections.deque[str] = collections.deque(maxlen=50)
        self._tail_lock = threading.Lock()
        self._resolve_paths()

    @rpc
    def start(self) -> None:
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

        # Reset tail buffers for this run.
        with self._tail_lock:
            self._stderr_tail.clear()
            self._stdout_tail.clear()

        logger.info(
            "Starting native process",
            module=self._mod_label,
            cmd=" ".join(cmd),
            cwd=cwd,
        )
        def _child_preexec() -> None:
            """Ensure child is killed when parent dies (Linux only)."""
            import os as _os

            try:
                import ctypes

                PR_SET_PDEATHSIG = 1
                libc = ctypes.CDLL("libc.so.6", use_errno=True)
                libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM)
            except Exception:
                pass
            # Also start a new session so terminal SIGINT doesn't reach child.
            _os.setsid()

        self._process = subprocess.Popen(
            cmd,
            env=env,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=_child_preexec,
        )
        logger.info(
            "Native process started",
            module=self._mod_label,
            pid=self._process.pid,
        )

        self._stopping = False
        self._watchdog = threading.Thread(
            target=self._watch_process,
            daemon=True,
            name=f"native-watchdog-{self._mod_label}",
        )
        self._watchdog.start()

    @rpc
    def stop(self) -> None:
        self._stopping = True
        if self._process is not None and self._process.poll() is None:
            logger.info(
                "Stopping native process",
                module=self._mod_label,
                pid=self._process.pid,
            )
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout=self.config.shutdown_timeout)
            except subprocess.TimeoutExpired:
                logger.warning(
                    "Native process did not exit, sending SIGKILL",
                    module=self._mod_label,
                    pid=self._process.pid,
                )
                self._process.kill()
                self._process.wait(timeout=5)
        if self._watchdog is not None and self._watchdog is not threading.current_thread():
            self._watchdog.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._watchdog = None
        self._process = None
        super().stop()

    def _watch_process(self) -> None:
        """Block until the native process exits; trigger stop() if it crashed."""
        if self._process is None:
            return

        stdout_t = self._start_reader(self._process.stdout, "info", self._stdout_tail)
        stderr_t = self._start_reader(self._process.stderr, "warning", self._stderr_tail)
        rc = self._process.wait()
        stdout_t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        stderr_t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if self._stopping:
            logger.info(
                "Native process exited (expected)",
                module=self._mod_label,
                pid=self._process.pid,
                returncode=rc,
            )
            return

        # Grab the tail for diagnostics.
        with self._tail_lock:
            stderr_snapshot = list(self._stderr_tail)
            stdout_snapshot = list(self._stdout_tail)

        logger.error(
            "Native process died unexpectedly",
            module=self._mod_label,
            pid=self._process.pid,
            returncode=rc,
        )

        # Log the last stderr/stdout lines so the cause is visible.
        if stderr_snapshot:
            logger.error(
                f"Last {len(stderr_snapshot)} stderr lines from {self._mod_label}:",
                module=self._mod_label,
                pid=self._process.pid,
            )
            for line in stderr_snapshot:
                logger.error(f"  stderr| {line}", module=self._mod_label)

        if stdout_snapshot and not stderr_snapshot:
            # Only dump stdout if stderr was empty (avoid double-noise).
            logger.error(
                f"Last {len(stdout_snapshot)} stdout lines from {self._mod_label}:",
                module=self._mod_label,
                pid=self._process.pid,
            )
            for line in stdout_snapshot:
                logger.error(f"  stdout| {line}", module=self._mod_label)

        if not stderr_snapshot and not stdout_snapshot:
            logger.error(
                "No output captured from native process — "
                "binary may have crashed before producing any output",
                module=self._mod_label,
                pid=self._process.pid,
            )

        self.stop()

    def _start_reader(
        self,
        stream: IO[bytes] | None,
        level: str,
        tail_buf: list[str],
    ) -> threading.Thread:
        """Spawn a daemon thread that pipes a subprocess stream through the logger."""
        t = threading.Thread(
            target=self._read_log_stream,
            args=(stream, level, tail_buf),
            daemon=True,
            name=f"native-reader-{level}-{self._mod_label}",
        )
        t.start()
        return t

    def _read_log_stream(
        self,
        stream: IO[bytes] | None,
        level: str,
        tail_buf: list[str],
    ) -> None:
        if stream is None:
            return
        log_fn = getattr(logger, level)
        for raw in stream:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue

            # Keep a rolling tail buffer for crash diagnostics.
            with self._tail_lock:
                tail_buf.append(line)

            if self.config.log_format == LogFormat.JSON:
                try:
                    data = json.loads(line)
                    event = data.pop("event", line)
                    log_fn(event, module=self._mod_label, **data)
                    continue
                except (json.JSONDecodeError, TypeError):
                    logger.warning(
                        "malformed JSON from native module",
                        module=self._mod_label,
                        raw=line,
                    )
            log_fn(line, module=self._mod_label, pid=self._process.pid if self._process else None)
        stream.close()

    def _resolve_paths(self) -> None:
        """Resolve relative ``cwd`` and ``executable`` against the subclass's source file."""
        if self.config.cwd is not None and not Path(self.config.cwd).is_absolute():
            source_file = inspect.getfile(type(self))
            base_dir = Path(source_file).resolve().parent
            self.config.cwd = str(base_dir / self.config.cwd)
        if not Path(self.config.executable).is_absolute() and self.config.cwd is not None:
            self.config.executable = str(Path(self.config.cwd) / self.config.executable)

    def _build_cache_name(self) -> str:
        """Return a stable, unique cache name for this module's build state."""
        source_file = Path(inspect.getfile(type(self))).resolve()
        return f"native_{source_file}"

    def _maybe_build(self) -> None:
        """Run ``build_command`` if the executable does not exist or sources changed."""
        exe = Path(self.config.executable)

        # Check if rebuild needed due to source changes
        needs_rebuild = False
        if self.config.rebuild_on_change and exe.exists():
            if did_change(
                self._build_cache_name(),
                self.config.rebuild_on_change,
                cwd=self.config.cwd,
                extra_hash=self.config.build_command,
            ):
                logger.info("Source files changed, triggering rebuild", executable=str(exe))
                needs_rebuild = True

        if exe.exists() and not needs_rebuild:
            return

        if self.config.build_command is None:
            raise FileNotFoundError(
                f"[{self._mod_label}] Executable not found: {exe}. "
                "Set build_command in config to auto-build, or build it manually."
            )

        # Don't unlink the exe before rebuilding — the build command is
        # responsible for replacing it.  For nix builds the exe lives inside
        # a read-only store; `nix build -o` atomically swaps the output
        # symlink without touching store contents.
        logger.info(
            "Rebuilding" if needs_rebuild else "Executable not found, building",
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

        stdout_lines = stdout.decode("utf-8", errors="replace").splitlines()
        stderr_lines = stderr.decode("utf-8", errors="replace").splitlines()

        for line in stdout_lines:
            if line.strip():
                logger.info(line, module=self._mod_label)
        for line in stderr_lines:
            if line.strip():
                logger.warning(line, module=self._mod_label)

        if proc.returncode != 0:
            # Include the last stderr lines in the exception for RPC callers.
            tail = [l for l in stderr_lines if l.strip()][-20:]
            tail_str = "\n".join(tail) if tail else "(no stderr output)"
            raise RuntimeError(
                f"[{self._mod_label}] Build command failed "
                f"(exit {proc.returncode}): {self.config.build_command}\n"
                f"--- last stderr ---\n{tail_str}"
            )
        if not exe.exists():
            raise FileNotFoundError(
                f"[{self._mod_label}] Build command succeeded but executable still not found: {exe}"
            )

        # Seed the cache after a successful build so the next check has a baseline
        # (needed for the initial build when the pre-build change check was skipped)
        if self.config.rebuild_on_change:
            did_change(
                self._build_cache_name(),
                self.config.rebuild_on_change,
                cwd=self.config.cwd,
                extra_hash=self.config.build_command,
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
