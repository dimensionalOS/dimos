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

"""Typed recording inputs and dataset features declared together."""

from __future__ import annotations

from typing import Any

from pydantic import Field, field_validator, model_validator

from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
    SyncConfig,
)
from dimos.protocol.service.spec import BaseConfig


class CollectionFeature(FeatureSpec):
    """A dataset feature with the raw message type needed to record its stream."""

    message_type: type[Any]


class CollectionProfile(BaseConfig):
    """One recording contract, independent of camera hardware and policy backends."""

    name: str = Field(min_length=1)
    robot_type: str = Field(min_length=1)
    observations: dict[str, CollectionFeature] = Field(min_length=1)
    actions: dict[str, CollectionFeature] = Field(min_length=1)
    sync: SyncConfig
    quality: QualityConfig = QualityConfig()

    @field_validator("observations", "actions")
    @classmethod
    def _feature_names(cls, value: dict[str, CollectionFeature]) -> dict[str, CollectionFeature]:
        if any(not key.strip() for key in value):
            raise ValueError("feature names must not be blank")
        return value

    @model_validator(mode="after")
    def validate_features(self) -> CollectionProfile:
        if set(self.observations) & set(self.actions):
            raise ValueError("observation and action feature names must be distinct")
        if self.sync.anchor not in self.observations:
            raise ValueError("sync anchor must name an observation feature")
        self.input_types()
        return self

    def input_types(self) -> dict[str, type[Any]]:
        """Deduplicate raw inputs without merging their dataset projections."""
        inputs: dict[str, type[Any]] = {}
        for feature in (*self.observations.values(), *self.actions.values()):
            previous = inputs.setdefault(feature.stream, feature.message_type)
            if previous is not feature.message_type:
                raise ValueError(f"stream {feature.stream!r} has conflicting message types")
        return inputs

    def to_schema(self) -> RecordingSchema:
        """Snapshot dataset interpretation without runtime message classes."""
        return RecordingSchema(
            name=self.name,
            robot_type=self.robot_type,
            observation={
                key: FeatureSpec(**feature.model_dump(exclude={"message_type"}))
                for key, feature in self.observations.items()
            },
            action={
                key: FeatureSpec(**feature.model_dump(exclude={"message_type"}))
                for key, feature in self.actions.items()
            },
            sync=self.sync.model_copy(deep=True),
            quality=self.quality.model_copy(deep=True),
        )

    def dataprep_config(self, *, source: str = "", output: OutputConfig) -> DataPrepConfig:
        """Build a low-level preparation request for an explicit raw artifact."""
        return DataPrepConfig(
            **self.to_schema().model_dump(exclude={"name", "robot_type", "payload"}),
            source=source,
            output=output,
        )
