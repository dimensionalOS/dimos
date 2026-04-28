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
import inspect
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.thread_utils import ModuleProcess

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

    # New version of Native Modules read json configs from stdin
    # Enable this to read from stdin instead of cli args
    stdin_config: bool = False

    # Override in subclasses to exclude fields from CLI arg generation
    cli_exclude: frozenset[str] = frozenset()

    def to_config_dict(self) -> dict[str, Any]:
        """
        Return module-specific config fields as a plain dict (for stdin JSON).
        """
        ignore_fields = set(NativeModuleConfig.model_fields)
        return {
            k: v for k, v in self.model_dump().items() if k not in ignore_fields and v is not None
        }

    def to_cli_args(self) -> list[str]:
        """
        Auto-convert subclass config fields to CLI args.
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
            if isinstance(val, bool):
                args.extend([f"--{f}", str(val).lower()])
            elif isinstance(val, list):
                args.extend([f"--{f}", ",".join(str(v) for v in val)])
            else:
                args.extend([f"--{f}", str(val)])
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

    default_config: type[_NativeConfig] = NativeModuleConfig  # type: ignore[assignment]
    _proc: ModuleProcess | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._resolve_paths()

    @rpc
    def start(self) -> None:
        super().start()
        self._maybe_build()
        self._proc = ModuleProcess(
            module=self,
            args=[
                self.config.executable,
                *[
                    arg
                    for name, topic in self._collect_topics().items()
                    for arg in (f"--{name}", topic)
                ],
                *self.config.to_cli_args(),
                *self.config.extra_args,
            ],
            env={**os.environ, **self.config.extra_env},
            cwd=self.config.cwd or str(Path(self.config.executable).resolve().parent),
            on_exit=self.stop,
            shutdown_timeout=self.config.shutdown_timeout,
            log_json=self.config.log_format == LogFormat.JSON,
        )
        self._proc.start()

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
