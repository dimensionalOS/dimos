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

"""Experimental native recorder for high-throughput registered generated memory streams."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Annotated, Any, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In
from dimos.experimental.memory.cdr_schema import recording_schema
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.recording import OnExisting
from dimos.messages.sensor_msgs.msg import CompressedImage, Image
from dimos.messages.std_msgs.msg import String
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.utils.data import backup_file
from dimos.utils.logging_config import setup_logger


def is_registered_message(kind: type) -> bool:
    from dimos_cdrgen.registry import message_types

    return any(item is kind for item in message_types().values())


logger = setup_logger()

_SUPPORTED_NATIVE_CODECS = {"cdr", "cdrjpeg", "lz4+cdr", "json"}


class RustStreamSpec(BaseModel):
    """Fully resolved stream settings sent to the native process."""

    port: str
    name: str
    payload_type: str
    codec: str
    timestamp_field: str | None = None
    json_schema: dict[str, Any] | None = None
    cdr_schema: dict[str, Any] | None = None
    # Old raw CDR configs retain reception timestamps and complete TF batches.
    source_timestamp: bool = False

    @model_validator(mode="after")
    def _json_options(self) -> RustStreamSpec:
        if self.codec != "json" and (
            self.timestamp_field is not None or self.json_schema is not None
        ):
            raise ValueError("JSON options require the json codec")
        if self.cdr_schema is not None and self.codec not in {"cdr", "lz4+cdr", "cdrjpeg"}:
            raise ValueError("CDR schema requires a CDR storage codec")
        return self


class RustStoreConfig(BaseModel):
    """Artifact path shared by native recording-store configurations."""

    model_config = ConfigDict(validate_default=True)

    kind: str
    path: str = Field(description="Absolute path of the recording artifact.")

    @field_validator("path", mode="before")
    @classmethod
    def _resolve_path(cls, value: str | os.PathLike[str]) -> str:
        path = Path(os.fspath(value))
        if not path.is_absolute():
            path = DIMOS_PROJECT_ROOT / path
        return str(path)


class RustSqliteStoreConfig(RustStoreConfig):
    """Write a Python-compatible Mem2 SQLite artifact."""

    kind: Literal["sqlite"] = "sqlite"
    path: str = "recording.db"


class RustMcapStoreConfig(RustStoreConfig):
    """Write storage-encoded observations to an indexed, compressed MCAP artifact."""

    kind: Literal["mcap"] = "mcap"
    path: str = "recording.mcap"


RustRecordingStoreConfig: TypeAlias = Annotated[
    RustSqliteStoreConfig | RustMcapStoreConfig,
    Field(discriminator="kind"),
]


class RustRecorderConfig(NativeModuleConfig):
    """Configuration for :class:`RustRecorder`.

    Python owns artifact lifecycle and stream registration. The native process
    receives only ``store``, ``encoding_threads``, and the internally resolved
    ``streams`` list over stdin.
    """

    executable: str = "result/bin/dimos-memory-recorder"
    build_command: str | None = "nix build -L path:."
    source_dir: str | None = "dimos/experimental/memory/rust"
    stdin_config: bool = True

    store: RustRecordingStoreConfig = Field(
        default_factory=RustSqliteStoreConfig,
        description="Record-only native storage backend and artifact path.",
    )
    on_existing: OnExisting = Field(
        default=OnExisting.BACKUP,
        exclude=True,
        description="How Python prepares an artifact path that already exists.",
    )
    backup_keep_last: int = Field(
        default=10,
        ge=0,
        exclude=True,
        description="Maximum rotated artifacts retained when on_existing is backup.",
    )
    record_tf: bool = Field(
        default=True,
        exclude=True,
        description="Record the connected TF input as the 'tf' stream.",
    )
    stream_remapping: dict[str, str] = Field(
        default_factory=dict,
        exclude=True,
        description="Map input port names to artifact stream names.",
    )
    stream_timestamp_fields: dict[str, str] = Field(default_factory=dict, exclude=True)
    stream_json_schemas: dict[str, dict[str, Any]] = Field(default_factory=dict, exclude=True)

    stream_codecs: dict[str, str] = Field(
        default_factory=dict,
        exclude=True,
        description="Per-stream Python Memory codec IDs.",
    )
    encoding_threads: int = Field(
        default=4,
        ge=1,
        description="CPU workers for transport decoding and storage encoding.",
    )
    streams: list[RustStreamSpec] = Field(
        default_factory=list,
        init=False,
        description="Resolved stream plan populated internally before native launch.",
    )

    @model_validator(mode="after")
    def _stdin_only(self) -> RustRecorderConfig:
        if self.extra_args:
            raise ValueError("RustRecorder is stdin-only and does not accept extra_args")
        return self

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # Older recorder binaries reject unknown stream fields, even null ones.
        config["streams"] = [
            stream.model_dump(exclude_none=True, exclude_defaults=True) for stream in self.streams
        ]
        return config


class RustRecorder(NativeModule):
    """Experimentally record connected ``In`` ports to native SQLite or MCAP.

    This API is under active evaluation and may change without compatibility
    aliases until it is promoted out of :mod:`dimos.experimental`.

    Subclass this recorder and declare the streams to capture, just like the
    Python :class:`dimos.memory.module.Recorder`::

        class CameraRecorder(RustRecorder):
            color_image: In[Image]

    Both stores use the Python Mem2 codec configuration: images default to
    JPEG, and ``stream_codecs`` may select ``cdr`` or ``lz4+cdr``. MCAP uses
    indexed Zstd chunks around those storage-encoded observations. The native
    path preserves source timestamps for common stamped message types. Other
    Messages without a supported source Header use their reception timestamp. Spatial pose attachment is not
    supported yet.
    """

    config: RustRecorderConfig
    tf: In[TFMessage]

    @rpc
    def build(self) -> None:
        if self.config.g.replay:
            Module.build(self)
            return
        super().build()

    @rpc
    def start(self) -> None:
        if self.config.g.replay:
            Module.start(self)
            logger.info(
                "Replay mode active; native recorder disabled",
                artifact_path=self.config.store.path,
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
                            payload_type=TFMessage.__msgtype__,
                            codec="cdr",
                            cdr_schema=recording_schema(TFMessage),
                            source_timestamp=True,
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
                    payload_type=port.type.__msgtype__,
                    codec=codec,
                    timestamp_field=self.config.stream_timestamp_fields.get(stream_name),
                    json_schema=self.config.stream_json_schemas.get(stream_name),
                    cdr_schema=recording_schema(
                        CompressedImage if codec == "cdrjpeg" else port.type
                    )
                    if codec in {"cdr", "lz4+cdr", "cdrjpeg"}
                    else None,
                    source_timestamp=True,
                )
            )

        names = [spec.name for spec in specs]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            raise ValueError(f"Duplicate recorded stream names: {duplicates}")
        if not specs:
            logger.warning("Native recorder has no connected streams")
        return specs

    def _collect_topics(self) -> dict[str, str]:
        topics = super()._collect_topics()
        enabled_ports = {spec.port for spec in self.config.streams}
        return {port: topic for port, topic in topics.items() if port in enabled_ports}

    @staticmethod
    def _default_codec(payload_type: type[Any]) -> str:
        if issubclass(payload_type, Image):
            return "cdrjpeg"
        if is_registered_message(payload_type):
            return "cdr"
        raise TypeError(
            f"RustRecorder only supports registered generated messages, got {payload_type.__qualname__}"
        )

    @staticmethod
    def _validate_codec(stream_name: str, payload_type: type[Any], codec: str) -> None:
        if codec not in _SUPPORTED_NATIVE_CODECS:
            raise ValueError(
                f"Unsupported native codec {codec!r} for stream {stream_name!r}; "
                f"choose one of {sorted(_SUPPORTED_NATIVE_CODECS)}"
            )
        if codec == "json" and payload_type is not String:
            raise TypeError("JSON codec requires std_msgs.String")
        if codec == "cdrjpeg" and not issubclass(payload_type, Image):
            raise TypeError(f"JPEG codec requires Image, got {payload_type.__qualname__}")
        if not is_registered_message(payload_type):
            raise TypeError(
                f"Native codec {codec!r} requires an registered generated type, "
                f"got {payload_type.__qualname__}"
            )

    def _prepare_store(self, specs: list[RustStreamSpec]) -> None:
        path = Path(self.config.store.path)
        if self.config.store.kind == "mcap" and self.config.on_existing is OnExisting.APPEND:
            raise ValueError("MCAP append is unsupported; choose overwrite, backup, or error")
        if path.exists():
            if self.config.on_existing is OnExisting.OVERWRITE:
                path.unlink()
                logger.info("Deleted existing recording", artifact_path=str(path))
            elif self.config.on_existing is OnExisting.BACKUP:
                backup = backup_file(path, keep_last=self.config.backup_keep_last)
                logger.info(
                    "Rotated existing recording",
                    artifact_path=str(path),
                    backup_path=str(backup) if backup is not None else None,
                )
            elif self.config.on_existing is OnExisting.ERROR:
                raise FileExistsError(f"Recording already exists: {path}")

        path.parent.mkdir(parents=True, exist_ok=True)
        if self.config.store.kind == "mcap":
            return

        with SqliteStore(path=str(path)) as store:
            if self.config.on_existing is OnExisting.APPEND:
                existing = set(store.list_streams())
                for name in {spec.name for spec in specs}.intersection(existing):
                    store.delete_stream(name)

            ports = self.inputs
            for spec in specs:
                tf_payload_type = TFMessage.__msgtype__
                payload_type = (
                    TFMessage if spec.payload_type == tf_payload_type else ports[spec.port].type
                )
                store.stream(spec.name, payload_type, codec=spec.codec)

    def _argv(self, _topics: dict[str, str]) -> list[str]:
        """Launch the stdin-only recorder without topic or configuration arguments."""
        return [self._executable]
