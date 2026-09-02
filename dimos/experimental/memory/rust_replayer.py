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

"""Experimental native publisher for time-ordered Memory2 recordings."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RUST_WORKSPACE = DIMOS_PROJECT_ROOT
_EXECUTABLE = _RUST_WORKSPACE / "target" / "release" / "dimos-memory-replayer"


class RustReplayStreamSpec(BaseModel):
    """One declared output and its exact artifact stream."""

    port: str
    name: str
    payload_type: str


class RustSqliteReplayStoreConfig(BaseModel):
    """Read-only SQLite artifact selected for native replay."""

    model_config = ConfigDict(validate_default=True, extra="forbid")

    path: str | None = None

    @field_validator("path", mode="before")
    @classmethod
    def _path_string(cls, value: str | os.PathLike[str] | None) -> str | None:
        return None if value is None else os.fspath(value)


class RustReplayerConfig(NativeModuleConfig):
    """Launch and clock settings for :class:`RustReplayer`."""

    executable: str = str(_EXECUTABLE)
    # ponytail: Cargo sees tracked and untracked workspace sources. The Nix flake
    # remains the reproducible packaging path, but Git flakes omit new untracked
    # crates during local development.
    build_command: str | None = "cargo build --release -p dimos-memory-replayer"
    cwd: str = str(_RUST_WORKSPACE)
    stdin_config: bool = True
    persistent_stdin: bool = True

    store: RustSqliteReplayStoreConfig = Field(default_factory=RustSqliteReplayStoreConfig)
    speed: float = Field(default=1.0, gt=0)
    seek: float | None = Field(default=None, ge=0)
    duration: float | None = Field(default=None, ge=0)
    from_timestamp: float | None = None
    loop: bool = False
    stream_remapping: dict[str, str] = Field(default_factory=dict, exclude=True)
    streams: list[RustReplayStreamSpec] = Field(default_factory=list, init=False)

    @field_validator("seek", "duration", "from_timestamp")
    @classmethod
    def _finite_timestamp(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("replay timestamps must be finite")
        return value

    @model_validator(mode="after")
    def _valid_window(self) -> RustReplayerConfig:
        if self.seek is not None and self.from_timestamp is not None:
            raise ValueError("seek and from_timestamp are mutually exclusive")
        if not math.isfinite(self.speed):
            raise ValueError("speed must be finite")
        if self.extra_args:
            raise ValueError("RustReplayer is stdin-only and does not accept extra_args")
        return self

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        config.update(
            seek=self.seek,
            duration=self.duration,
            from_timestamp=self.from_timestamp,
        )
        return config


class RustReplayer(NativeModule):
    """Replay declared ``Out`` ports directly from SQLite.

    Subclass this module and declare LCM-backed output types. Artifact names
    default to port names and can be changed explicitly with
    ``stream_remapping``. This is a graph replay source, not a query API.
    """

    config: RustReplayerConfig

    @rpc
    def start(self) -> None:
        specs = self._stream_specs()
        if not specs:
            Module.start(self)
            return
        self.config.store.path = str(self._resolve_store_path())
        self.config.streams = specs
        super().start()

    @rpc
    def pause(self) -> None:
        """Pause replay at its current recording position."""
        self._native_command("pause")

    @rpc
    def resume(self) -> None:
        """Resume replay from its paused recording position."""
        self._native_command("resume")

    @rpc
    def set_speed(self, speed: float) -> None:
        """Change playback speed without jumping in recording time."""
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("speed must be finite and greater than zero")
        self._native_command("set_speed", speed=speed)
        self.config.speed = speed

    def _stream_specs(self) -> list[RustReplayStreamSpec]:
        specs: list[RustReplayStreamSpec] = []
        for port_name, port in self.outputs.items():
            if getattr(port, "_transport", None) is None:
                continue
            payload_type = port.type
            if not hasattr(payload_type, "lcm_encode") or not hasattr(payload_type, "lcm_decode"):
                raise TypeError(
                    "RustReplayer only supports LCM-backed messages, got "
                    f"{payload_type.__qualname__}"
                )
            specs.append(
                RustReplayStreamSpec(
                    port=port_name,
                    name=self.config.stream_remapping.get(port_name, port_name),
                    payload_type=f"{payload_type.__module__}.{payload_type.__qualname__}",
                )
            )

        names = [spec.name for spec in specs]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            raise ValueError(f"Duplicate replay stream names: {duplicates}")
        if not specs:
            logger.warning("Native replayer has no connected outputs")
        return specs

    def _resolve_store_path(self) -> Path:
        configured = self.config.store.path or self.config.g.replay_db
        path = resolve_named_path(configured, ".db")
        if not path.is_file():
            raise FileNotFoundError(f"Replay artifact not found: {path}")
        return path.resolve()

    def _argv(self, _topics: dict[str, str]) -> list[str]:
        return [self.config.executable]
