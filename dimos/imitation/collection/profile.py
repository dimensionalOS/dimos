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

"""Typed raw inputs and dataset projections, independent of robot hardware."""

from __future__ import annotations

from typing import Any

from pydantic import Field, field_validator, model_validator

from dimos.imitation.dataprep.schema import (
    DEFAULT_FPS,
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    RecordingSchema,
    SyncConfig,
    validate_source_kinds,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig


class CollectionProfile(BaseConfig):
    """Declare recorded features. Alignment uses a fixed 30 Hz grid and 20 ms tolerance."""

    observations: dict[str, FeatureSpec] = Field(min_length=1)
    actions: dict[str, FeatureSpec] = Field(min_length=1)
    metadata: dict[str, Any] = Field(default_factory=dict)

    @field_validator("observations", "actions")
    @classmethod
    def _feature_names(cls, value: dict[str, FeatureSpec]) -> dict[str, FeatureSpec]:
        if any(not key.strip() for key in value):
            raise ValueError("feature names must not be blank")
        return value

    @model_validator(mode="after")
    def validate_features(self) -> CollectionProfile:
        if set(self.observations) & set(self.actions):
            raise ValueError("observation and action feature names must be distinct")
        self.input_types()
        return self

    def input_types(self) -> dict[str, type[Any]]:
        """Deduplicate raw inputs without merging their dataset projections."""
        validate_source_kinds((*self.observations.values(), *self.actions.values()))
        inputs: dict[str, type[Any]] = {}
        for feature in (*self.observations.values(), *self.actions.values()):
            kind = feature.message_type
            if kind is None:
                raise ValueError(f"Collection stream {feature.stream!r} requires message_type")
            if feature.source_kind == "joint_position_updates" and not issubclass(kind, JointState):
                raise ValueError("joint_position_updates requires JointState messages")
            previous = inputs.setdefault(feature.stream, kind)
            if previous is not kind:
                raise ValueError(f"stream {feature.stream!r} has conflicting message types")
        return inputs

    def to_schema(self) -> RecordingSchema:
        """Persist defaults and projections so preparation needs no live robot config."""
        anchor = next(
            (key for key, feature in self.observations.items() if feature.dtype == "video"),
            next(iter(self.observations)),
        )
        return RecordingSchema(
            name=self.metadata.get("name", "collection"),
            robot_type=self.metadata.get("robot_type", ""),
            metadata=self.metadata.copy(),
            observation={
                key: FeatureSpec(**feature.model_dump())
                for key, feature in self.observations.items()
            },
            action={
                key: FeatureSpec(**feature.model_dump()) for key, feature in self.actions.items()
            },
            sync=SyncConfig(anchor=anchor, rate_hz=DEFAULT_FPS, tolerance_ms=20),
        )

    def dataprep_config(self, *, source: str = "", output: OutputConfig) -> DataPrepConfig:
        """Prepare an explicit raw artifact with the same persisted interpretation."""
        schema = self.to_schema()
        return DataPrepConfig(
            recording_schema=schema,
            source=source,
            output=output.model_copy(update={"metadata": {**self.metadata, **output.metadata}}),
        )
