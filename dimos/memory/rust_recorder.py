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

"""Native recorder for high-throughput LCM-backed memory streams."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Literal, TypeAlias, cast

from pydantic import BaseModel, Field, field_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In
from dimos.memory.module import OnExisting
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import backup_file
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RUST_WORKSPACE = DIMOS_PROJECT_ROOT / "native" / "rust"
_EXECUTABLE = _RUST_WORKSPACE / "target" / "release" / "dimos-memory-recorder"
_SUPPORTED_CODECS = {"lcm", "jpeg", "lz4+lcm"}
CodecId: TypeAlias = Literal["lcm", "jpeg", "lz4+lcm"]


class RustStreamSpec(BaseModel):
    """Fully resolved stream settings sent to the native process."""

    port: str
    name: str
    payload_type: str
    codec: CodecId
    payload_kind: Literal["raw", "image", "tf"]


class RustRecorderConfig(NativeModuleConfig):
    """Configuration for :class:`RustRecorder`."""

    executable: str = str(_EXECUTABLE)
    build_command: str = "cargo build --release -p dimos-memory-recorder"
    cwd: str = str(_RUST_WORKSPACE)
    stdin_config: bool = True
    cli_exclude: frozenset[str] = frozenset(
        {
            "backup_keep_last",
            "db_path",
            "encoding_threads",
            "flush_interval_ms",
            "jpeg_quality",
            "on_existing",
            "queue_capacity",
            "record_tf",
            "stream_codecs",
            "stream_remapping",
            "streams",
            "write_batch_size",
        }
    )

    db_path: str = "recording.db"
    on_existing: OnExisting = OnExisting.BACKUP
    backup_keep_last: int = Field(default=10, ge=0)
    record_tf: bool = True
    stream_remapping: dict[str, str] = Field(default_factory=dict)
    stream_codecs: dict[str, str] = Field(default_factory=dict)

    encoding_threads: int = Field(default=4, ge=1)
    queue_capacity: int = Field(default=256, ge=1)
    write_batch_size: int = Field(default=128, ge=1)
    flush_interval_ms: int = Field(default=100, ge=1)
    jpeg_quality: int = Field(default=50, ge=0, le=100)

    streams: list[RustStreamSpec] = Field(default_factory=list)

    @field_validator("db_path", mode="before")
    @classmethod
    def _resolve_path(cls, value: str | os.PathLike[str]) -> str:
        path = Path(os.fspath(value))
        if not path.is_absolute():
            path = DIMOS_PROJECT_ROOT / path
        return str(path)

    def to_config_dict(self) -> dict[str, Any]:
        """Return only fields consumed by the strict native config."""
        return {
            "db_path": self.db_path,
            "encoding_threads": self.encoding_threads,
            "queue_capacity": self.queue_capacity,
            "write_batch_size": self.write_batch_size,
            "flush_interval_ms": self.flush_interval_ms,
            "jpeg_quality": self.jpeg_quality,
            "streams": [stream.model_dump() for stream in self.streams],
        }


class RustRecorder(NativeModule):
    """Record connected ``In`` ports with native encoding and SQLite writes.

    Subclass this recorder and declare the streams to capture, just like the
    Python :class:`dimos.memory.module.Recorder`::

        class CameraRecorder(RustRecorder):
            color_image: In[Image]

    LCM-backed message types use their wire bytes directly. Images default to
    JPEG; ``stream_codecs`` may select ``lcm`` or ``lz4+lcm`` instead. The
    The native fast path preserves source timestamps for common stamped message
    types. Other LCM messages use their reception timestamp. Spatial pose
    attachment is not supported yet.
    """

    config: RustRecorderConfig
    tf: In[TFMessage]

    @rpc
    def start(self) -> None:
        if self.config.g.replay:
            Module.start(self)
            logger.info(
                "Replay mode active; native recorder disabled",
                db_path=self.config.db_path,
            )
            return

        specs = self._stream_specs()
        if not specs:
            Module.start(self)
            return

        self._prepare_store(specs)
        self.config.streams = specs
        super().start()

    def _stream_specs(self) -> list[RustStreamSpec]:
        specs: list[RustStreamSpec] = []
        for port_name, port in self.inputs.items():
            if getattr(port, "_transport", None) is None:
                continue
            if port is self.tf:
                if self.config.record_tf:
                    specs.append(
                        RustStreamSpec(
                            port=port_name,
                            name="tf",
                            payload_type=f"{TFMessage.__module__}.{TFMessage.__qualname__}",
                            codec="lcm",
                            payload_kind="tf",
                        )
                    )
                continue

            stream_name = self.config.stream_remapping.get(port_name, port_name)
            codec = self.config.stream_codecs.get(stream_name, self._default_codec(port.type))
            self._validate_codec(stream_name, port.type, codec)
            specs.append(
                RustStreamSpec(
                    port=port_name,
                    name=stream_name,
                    payload_type=f"{port.type.__module__}.{port.type.__qualname__}",
                    codec=cast("CodecId", codec),
                    payload_kind="image" if issubclass(port.type, Image) else "raw",
                )
            )

        if not specs:
            logger.warning("Native recorder has no connected streams")
        return specs

    @staticmethod
    def _default_codec(payload_type: type[Any]) -> CodecId:
        if issubclass(payload_type, Image):
            return "jpeg"
        if hasattr(payload_type, "lcm_encode") and hasattr(payload_type, "lcm_decode"):
            return "lcm"
        raise TypeError(
            f"RustRecorder only supports LCM-backed messages, got {payload_type.__qualname__}"
        )

    @staticmethod
    def _validate_codec(stream_name: str, payload_type: type[Any], codec: str) -> None:
        if codec not in _SUPPORTED_CODECS:
            raise ValueError(
                f"Unsupported native codec {codec!r} for stream {stream_name!r}; "
                f"choose one of {sorted(_SUPPORTED_CODECS)}"
            )
        if codec == "jpeg" and not issubclass(payload_type, Image):
            raise TypeError(f"JPEG codec requires Image, got {payload_type.__qualname__}")
        if not hasattr(payload_type, "lcm_encode") or not hasattr(payload_type, "lcm_decode"):
            raise TypeError(
                f"Native codec {codec!r} requires an LCM-backed type, "
                f"got {payload_type.__qualname__}"
            )

    def _prepare_store(self, specs: list[RustStreamSpec]) -> None:
        path = Path(self.config.db_path)
        if path.exists():
            if self.config.on_existing is OnExisting.OVERWRITE:
                path.unlink()
                logger.info("Deleted existing recording", db_path=str(path))
            elif self.config.on_existing is OnExisting.BACKUP:
                backup = backup_file(path, keep_last=self.config.backup_keep_last)
                logger.info(
                    "Rotated existing recording",
                    db_path=str(path),
                    backup_path=str(backup) if backup is not None else None,
                )
            elif self.config.on_existing is OnExisting.ERROR:
                raise FileExistsError(f"Recording already exists: {path}")

        path.parent.mkdir(parents=True, exist_ok=True)
        with SqliteStore(path=str(path)) as store:
            if self.config.on_existing is OnExisting.APPEND:
                existing = set(store.list_streams())
                for name in {spec.name for spec in specs}.intersection(existing):
                    store.delete_stream(name)

            ports = self.inputs
            for spec in specs:
                payload_type = TFMessage if spec.payload_kind == "tf" else ports[spec.port].type
                store.stream(spec.name, payload_type, codec=spec.codec)
