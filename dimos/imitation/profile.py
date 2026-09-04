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

"""Typed observation and action contracts shared by collection and rollout."""

from __future__ import annotations

import keyword

from pydantic import Field, model_validator

from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
    SyncConfig,
)
from dimos.protocol.service.spec import BaseConfig


class ImageSource(BaseConfig):
    """One RGB image stream consumed under a backend feature key."""

    stream: str
    shape: tuple[int, int, int]

    @model_validator(mode="after")
    def validate_source(self) -> ImageSource:
        _validate_stream_name(self.stream)
        if any(dimension <= 0 for dimension in self.shape):
            raise ValueError("image shape must contain positive dimensions")
        if self.shape[2] != 3:
            raise ValueError("image source must be RGB HWC with three channels")
        return self


class JointPositionSource(BaseConfig):
    """One named JointState position projection."""

    stream: str
    joints: tuple[str, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_source(self) -> JointPositionSource:
        _validate_stream_name(self.stream)
        if any(not joint.strip() for joint in self.joints):
            raise ValueError("joint names must not be blank")
        if len(set(self.joints)) != len(self.joints):
            raise ValueError("joint names must be unique")
        return self


PolicySource = ImageSource | JointPositionSource


class JointPositionAction(BaseConfig):
    """Backend action key and the demonstration stream that teaches it."""

    key: str = Field(min_length=1)
    demonstration: JointPositionSource


class PolicyIOProfile(BaseConfig):
    """Complete feature-key to DimOS-stream contract for one robot setup."""

    name: str = Field(min_length=1)
    robot_type: str = Field(min_length=1)
    observations: dict[str, PolicySource] = Field(min_length=1)
    action: JointPositionAction
    sync: SyncConfig
    quality: QualityConfig = QualityConfig()

    @model_validator(mode="after")
    def validate_contract(self) -> PolicyIOProfile:
        if any(not key.strip() for key in self.observations):
            raise ValueError("observation feature keys must not be blank")
        if self.action.key in self.observations:
            raise ValueError("action key must not also be an observation key")
        if self.sync.anchor not in self.observations:
            raise ValueError("sync anchor must name an observation feature")

        types_by_stream: dict[str, type[PolicySource]] = {}
        sources = [*self.observations.values(), self.action.demonstration]
        for source in sources:
            existing = types_by_stream.setdefault(source.stream, type(source))
            if existing is not type(source):
                raise ValueError(
                    f"stream {source.stream!r} is declared with conflicting source types"
                )
        matching_states = [
            key
            for key, source in self.observations.items()
            if isinstance(source, JointPositionSource)
            and source.joints == self.action.demonstration.joints
        ]
        if len(matching_states) != 1:
            raise ValueError(
                "profile must have exactly one joint observation matching the action joints"
            )
        return self

    @property
    def action_state_key(self) -> str:
        """Return the observation feature used as a trajectory's current state."""
        return next(
            key
            for key, source in self.observations.items()
            if isinstance(source, JointPositionSource)
            and source.joints == self.action.demonstration.joints
        )

    def dataprep_config(self, *, source: str = "", output: OutputConfig) -> DataPrepConfig:
        """Project this live contract into the native-recording dataset schema."""
        observations = {
            key: _feature_spec(source_spec) for key, source_spec in self.observations.items()
        }
        return DataPrepConfig(
            source=source,
            observation=observations,
            action={self.action.key: _feature_spec(self.action.demonstration)},
            sync=self.sync,
            quality=self.quality,
            output=output,
        )


def _feature_spec(source: PolicySource) -> FeatureSpec:
    if isinstance(source, ImageSource):
        return FeatureSpec(
            stream=source.stream,
            field="data",
            dtype="video",
            shape=source.shape,
            names=["height", "width", "channels"],
        )
    return FeatureSpec(
        stream=source.stream,
        field="position",
        dtype="float32",
        shape=(len(source.joints),),
        names=list(source.joints),
    )


def _validate_stream_name(name: str) -> None:
    if not name.isidentifier() or keyword.iskeyword(name):
        raise ValueError(f"stream {name!r} must be a Python identifier")
