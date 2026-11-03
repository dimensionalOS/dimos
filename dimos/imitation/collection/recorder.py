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

from collections.abc import Mapping
from dataclasses import replace
import keyword
from pathlib import Path
import sys
from typing import Any, Literal

from pydantic import Field, field_validator, model_validator

from dimos.core.coordination.blueprints import Blueprint, StreamRef
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
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.dataprep.schema import RecordingSchema
from dimos.memory.type.recording import OnExisting
from dimos.msgs.std_msgs.String import String

PortTypes = tuple[tuple[str, type[Any]], ...]


def _validate_input_types(
    module: type[CollectionModule],
    input_types: Mapping[str, type[Any]],
) -> dict[str, type[Any]]:
    annotations = {
        name for base in module.__mro__ for name in vars(base).get("__annotations__", {})
    }
    reserved = set(dir(module)) | annotations | {"ref", "rpc", "input_types"}
    for name, kind in input_types.items():
        if (
            not isinstance(name, str)
            or not name.isidentifier()
            or keyword.iskeyword(name)
            or name.startswith("_")
            or name in reserved
        ):
            raise ValueError(f"Invalid or reserved input name: {name!r}")
        if not isinstance(kind, type):
            raise TypeError(f"Input {name!r} requires a message class")
    return dict(input_types)


class CollectionModuleConfig(RustRecorderConfig):
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
    def _collection_store(self) -> CollectionModuleConfig:
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


class CollectionModule(RustRecorder):
    config: CollectionModuleConfig

    status: In[String]

    def __init__(
        self,
        *,
        recording_schema: RecordingSchema,
        input_types: Mapping[str, type[Any]],
        **kwargs: Any,
    ) -> None:
        inputs = _validate_input_types(type(self), input_types)
        for name, kind in inputs.items():
            setattr(self, name, In(kind, name, self))
        super().__init__(**kwargs)
        self.recording_inputs = (*input_types.items(), ("status", String))
        self._recording_schema = recording_schema
        self._store_prepared = False

    @classmethod
    def blueprint(cls, **kwargs: Any) -> Blueprint:
        """Declare this recorder's runtime inputs without extending core Modules."""
        inputs = _validate_input_types(cls, kwargs["input_types"])
        blueprint = Blueprint.create(cls, **kwargs)
        [atom] = blueprint.blueprints
        streams = (
            *atom.streams,
            *(StreamRef(name=name, type=kind, direction="in") for name, kind in inputs.items()),
        )
        return replace(blueprint, blueprints=(replace(atom, streams=streams),))

    @rpc
    def build(self) -> None:
        if self.config.g.replay:
            super().build()
            return
        specs = self._stream_specs()
        super().build()
        self._prepare_store(specs)

    def _prepare_store(self, specs: list[RustStreamSpec]) -> None:
        if self._store_prepared:
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
        self._store_prepared = True

    recording_inputs: PortTypes

    def _stream_specs(self) -> list[RustStreamSpec]:
        specs = super()._stream_specs()
        connected = {spec.port for spec in specs}
        missing = sorted(name for name, _ in self.recording_inputs if name not in connected)
        if missing:
            raise ValueError(f"Missing required collection inputs: {missing}")
        for spec in specs:
            if spec.port == "status":
                spec.codec = "json"
                spec.timestamp_field = "ts"
                spec.json_schema = EpisodeStatus.json_schema()
        return specs


def collect(
    *,
    profile: CollectionProfile,
    recording: Path | None = None,
    format: Literal["mcap", "sqlite"] = "mcap",
    instance_name: str = "recorder",
) -> Blueprint:
    """Declare profile inputs and record a portable SQLite or MCAP session directory."""
    inputs = profile.input_types()
    for kind in inputs.values():
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
    return CollectionModule.blueprint(
        input_types=inputs,
        recording_schema=profile.to_schema(),
        format=format,
        instance_name=instance_name,
        **({"recording": recording} if recording is not None else {}),
    )
