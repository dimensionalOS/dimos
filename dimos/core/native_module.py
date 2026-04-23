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

import functools
import inspect
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
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

# ctypes is only needed for the Linux child-preexec helper below.  Hoisting
# the import out of the inner function avoids re-importing on every start().
if sys.platform.startswith("linux"):
    import ctypes

    from ctypes.util import find_library
    _LIBC = ctypes.CDLL(find_library("c"), use_errno=True)
    _PR_SET_PDEATHSIG = 1

    def _child_preexec_linux() -> None:
        """Kill child when parent dies. Linux only.

        Runs in the child between fork() and exec().  Async-signal-safe
        operations only — the call into libc.prctl is fine, but anything
        that touches the threading runtime (allocating, importing) is not.
        """
        if _LIBC.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM) != 0:
            err = ctypes.get_errno()
            raise OSError(err, f"prctl(PR_SET_PDEATHSIG) failed: {os.strerror(err)}")
else:
    _child_preexec_linux = None  # type: ignore[assignment]

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

logger = setup_logger()


class NativeModuleConfig(ModuleConfig):
    """Configuration for a native (C/C++) subprocess module."""

    executable: str
    build_command: str | None = None
    cwd: str | None = None
    extra_args: list[str] = Field(default_factory=list)
    extra_env: dict[str, str] = Field(default_factory=dict)
    shutdown_timeout: float = DEFAULT_THREAD_JOIN_TIMEOUT

    # Override in subclasses to exclude fields from CLI arg generation
    cli_exclude: frozenset[str] = frozenset()
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


class NativeModule(Module):
    """Module that wraps a native executable as a managed subprocess.

    Subclass this, declare In/Out ports, and annotate ``config`` with a
    :class:`NativeModuleConfig` subclass pointing at the executable.

    On ``start()``, the binary is launched with CLI args::

        <executable> --<port_name> <lcm_topic_string> ... <extra_args>

    The native process should parse these args and pub/sub on the given
    LCM topics directly.  On ``stop()``, the process receives SIGTERM.
    """

    config: NativeModuleConfig

    _process: subprocess.Popen[bytes] | None = None
    _watchdog: threading.Thread | None = None
    _stopping: bool = False
    _stop_lock: threading.Lock

    @functools.cached_property
    def _mod_label(self) -> str:
        """Short human-readable label: ClassName(executable_basename)."""
        exe = Path(self.config.executable).name if self.config.executable else "?"
        return f"{type(self).__name__}({exe})"

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_lock = threading.Lock()

        # Resolve relative cwd and executable against the subclass's source file.
        if self.config.cwd is not None and not Path(self.config.cwd).is_absolute():
            base_dir = Path(inspect.getfile(type(self))).resolve().parent
            self.config.cwd = str(base_dir / self.config.cwd)
        if not Path(self.config.executable).is_absolute() and self.config.cwd is not None:
            self.config.executable = str(Path(self.config.cwd) / self.config.executable)

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

        logger.info(
            "Starting native process",
            module=self._mod_label,
            cmd=" ".join(cmd),
            cwd=cwd,
        )

        # start_new_session=True is the thread-safe way to isolate the child
        # from terminal signals (SIGINT from the tty).  preexec_fn is unsafe
        # in the presence of threads (subprocess docs), so we only use it on
        # Linux where prctl(PR_SET_PDEATHSIG) has no alternative — see
        # _child_preexec_linux defined at module scope.
        self._process = subprocess.Popen(
            cmd,
            env=env,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            preexec_fn=_child_preexec_linux,
        )
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
        # Two callers can race here: the RPC stop() and the watchdog calling
        # self.stop() after it detects an unexpected exit.  Serialize on a
        # per-instance lock and let the second caller no-op via the
        # _stopping flag.  We capture the proc/watchdog refs under the lock
        # but do the actual signal/wait/join *outside* it — joining the
        # watchdog while holding the lock would deadlock with the watchdog's
        # own stop() call waiting on the same lock.
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
                proc.wait(timeout=self.config.shutdown_timeout)

        if watchdog is not None and watchdog is not threading.current_thread():
            watchdog.join(timeout=self.config.shutdown_timeout)

        with self._stop_lock:
            self._watchdog = None
            self._process = None

        super().stop()

    def _watch_process(self) -> None:
        """Block until the native process exits; trigger stop() if it crashed."""
        # Cache the Popen reference and pid locally so a concurrent stop()
        # setting self._process = None can't race us into an AttributeError.
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
        """Spawn a daemon thread that pipes a subprocess stream through the logger."""
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
            # Use the captured pid rather than self._process.pid — stop() can
            # null self._process out from under us between the check and the
            # attribute read.
            log_fn(line, module=self._mod_label, pid=pid)
        stream.close()

    def _maybe_build(self) -> None:
        """Run ``build_command`` when not in PROD mode, or if the executable is missing.

        When ``PROD`` env var is set, skip rebuilding entirely — the executable
        must already exist.  Otherwise, always invoke ``build_command`` and let
        nix handle caching/cache-busting.
        """
        exe = Path(self.config.executable)
        is_prod = os.environ.get("PROD")

        if is_prod:
            if not exe.exists():
                raise FileNotFoundError(
                    f"[{self._mod_label}] PROD is set but executable not found: {exe}. "
                    "Build it before deploying."
                )
            return

        if self.config.build_command is None:
            if not exe.exists():
                raise FileNotFoundError(
                    f"[{self._mod_label}] Executable not found: {exe}. "
                    "Set build_command in config to auto-build, or build it manually."
                )
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
    "NativeModule",
    "NativeModuleConfig",
]
