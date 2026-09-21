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

"""Blueprint factory for recorders with profile-defined typed inputs."""

from __future__ import annotations

from abc import ABCMeta
from collections.abc import Callable
import copyreg
import hashlib
import keyword
from pathlib import Path
import sys
from threading import RLock
from typing import Any, ClassVar, Literal, cast, get_type_hints

from pydantic import Field, field_validator, model_validator

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import (
    RustMcapStoreConfig,
    RustRecorder,
    RustRecorderConfig,
    RustRecordingStoreConfig,
    RustSqliteStoreConfig,
    RustStreamSpec,
)
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recording import RecordingSchema
from dimos.memory.module import OnExisting
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus

PortTypes = tuple[tuple[str, type[Any]], ...]


class CollectionRecorderConfig(RustRecorderConfig):
    recording: Path
    format: Literal["mcap", "sqlite"] = "mcap"
    record_tf: bool = False
    on_existing: Literal[OnExisting.ERROR] = Field(
        default=OnExisting.ERROR,
        description="Collection requires a new recording directory; existing files are errors.",
    )
    backup_keep_last: Literal[0] = Field(
        default=0, description="Collection does not rotate backups."
    )

    @field_validator("recording", mode="before")
    @classmethod
    def _resolve_recording(cls, value: str | Path) -> Path:
        return Path(value).expanduser().resolve()

    @model_validator(mode="after")
    def _collection_store(self) -> CollectionRecorderConfig:
        store: RustRecordingStoreConfig
        if self.format == "sqlite":
            store = RustSqliteStoreConfig(kind="sqlite", path=str(self.recording / "recording.db"))
        else:
            store = RustMcapStoreConfig(kind="mcap", path=str(self.recording / "recording.mcap"))
        if "store" in self.model_fields_set and self.store != store:
            raise ValueError("Collection store is derived from recording and format")
        self.store = store
        return self

    def to_config_dict(self) -> dict[str, Any]:
        # Collection settings stay in Python; Rust accepts only its recorder config.
        return {
            "store": self.store.model_dump(),
            "encoding_threads": self.encoding_threads,
            "streams": [stream.model_dump() for stream in self.streams],
        }


class CollectionRecorder(RustRecorder):
    config: CollectionRecorderConfig

    def __init__(self, *, recording_schema: RecordingSchema, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._recording_schema = recording_schema
        self._prepared = False

    @rpc
    def build(self) -> None:
        if self.config.g.replay:
            super().build()
            return
        specs = self._stream_specs()
        super().build()
        self._prepare_store(specs)

    def _prepare_store(self, specs: list[RustStreamSpec]) -> None:
        if self._prepared:
            return
        schema = self._recording_schema.model_copy(deep=True)
        schema.payload = "recording.db" if self.config.format == "sqlite" else "recording.mcap"
        streams = {spec.port: spec.name for spec in specs}
        for feature in (*schema.observation.values(), *schema.action.values()):
            feature.stream = streams[feature.stream]
        schema.episodes.status_stream = streams["status"]
        encoded = schema.model_dump_json(indent=2)
        self.config.recording.mkdir(parents=True, exist_ok=False)
        with (self.config.recording / "schema.json").open("x") as file:
            file.write(encoded + "\n")
        super()._prepare_store(specs)
        self._prepared = True

    recording_inputs: ClassVar[PortTypes] = ()

    def _stream_specs(self) -> list[RustStreamSpec]:
        specs = super()._stream_specs()
        connected = {spec.port for spec in specs}
        missing = sorted(name for name, _ in self.recording_inputs if name not in connected)
        if missing:
            raise ValueError(f"Missing required collection inputs: {missing}")
        return specs


class _RecorderMeta(ABCMeta):
    recording_inputs: PortTypes


# Class identity must survive standard pickle, fresh workers, and source reload.
# This follows the dynamic relay bridge's metaclass/copyreg protocol.
_class_lock = RLock()
_reserved = (
    frozenset(dir(CollectionRecorder))
    | frozenset(get_type_hints(CollectionRecorder))
    | {
        "ref",
        "rpc",
        "encoded",
        "status",
    }
)


def _recorder_class(ports: PortTypes) -> type[CollectionRecorder]:
    with _class_lock:
        identity = "\n".join(
            f"{name}:{kind.__module__}.{kind.__qualname__}" for name, kind in ports
        )
        name = "CollectionRecorder_" + hashlib.sha256(identity.encode()).hexdigest()[:16]
        existing = globals().get(name)
        if existing is not None:
            if existing.recording_inputs != ports:
                raise ValueError(f"Recorder class name collision: {name}")
            result = cast("type[CollectionRecorder]", existing)
        else:
            # Runtime types are intentional: each profile supplies these annotations.
            input_type: Any = In
            result = cast(
                "type[CollectionRecorder]",
                _RecorderMeta(
                    name,
                    (CollectionRecorder,),
                    {
                        "__module__": __name__,
                        "__annotations__": {port: input_type[kind] for port, kind in ports},
                        "recording_inputs": ports,
                    },
                ),
            )
            globals()[name] = result
        return result


def _reduce_recorder(cls: _RecorderMeta) -> tuple[Callable[..., Any], tuple[PortTypes]]:
    return _recorder_class, (cls.recording_inputs,)


copyreg.pickle(_RecorderMeta, _reduce_recorder)


def collection_recorder(
    *,
    profile: CollectionProfile,
    recording: Path | None = None,
    format: Literal["mcap", "sqlite"] = "mcap",
    instance_name: str = "recorder",
) -> Blueprint:
    """Declare profile inputs and record a portable SQLite or MCAP session directory."""
    inputs = profile.input_types()
    for name, kind in inputs.items():
        if (
            not name.isidentifier()
            or keyword.iskeyword(name)
            or name.startswith("_")
            or name in _reserved
        ):
            raise ValueError(f"Invalid or reserved collection input name: {name!r}")
        if (
            kind.__module__ == "__main__"
            or kind.__qualname__ != kind.__name__
            or getattr(sys.modules.get(kind.__module__), kind.__name__, None) is not kind
        ):
            raise ValueError(f"Message type {kind!r} must be importable at module level")
        if not callable(getattr(kind, "lcm_encode", None)) or not callable(
            getattr(kind, "lcm_decode", None)
        ):
            raise TypeError(f"Message type {kind!r} does not support native recording")
    inputs["status"] = EpisodeStatus
    recorder = _recorder_class(tuple(sorted(inputs.items())))
    return recorder.blueprint(
        recording_schema=profile.to_schema(),
        format=format,
        instance_name=instance_name,
        **({"recording": recording} if recording is not None else {}),
    )
