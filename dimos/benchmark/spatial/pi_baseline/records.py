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

# Copyright 2026 Dimensional Inc.
"""Strict versioned records owned by the Python PI baseline."""

from __future__ import annotations

from datetime import datetime
from typing import Annotated, Literal

from pydantic import AfterValidator, Field, model_validator

from dimos.benchmark.spatial.models import AnswerType, SpatialModel
from dimos.benchmark.spatial.utilities import validate_relative_path

SafePath = Annotated[str, AfterValidator(validate_relative_path)]


class VersionedRecord(SpatialModel):
    schema_version: Literal["1.0"] = "1.0"


class RunRecord(VersionedRecord):
    record_type: Literal["pi-run"] = "pi-run"
    run_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    started_at: datetime
    release_id: str = Field(min_length=1)
    release_version: str = Field(pattern=r"^v[0-9]+\.[0-9]+\.[0-9]+$")


class BaselineConfig(VersionedRecord):
    record_type: Literal["pi-config"] = "pi-config"
    config_version: Literal["1.0"] = "1.0"
    resolver_version: Literal["public-exact-v1"] = "public-exact-v1"
    answer_policy_version: Literal["typed-first-valid-v1"] = "typed-first-valid-v1"


class StagingRecord(VersionedRecord):
    record_type: Literal["pi-staging"] = "pi-staging"
    case_path: SafePath
    map_path: SafePath
    map_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    schema_path: SafePath = "schema.v1.json"
    schema_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class Provenance(VersionedRecord):
    record_type: Literal["pi-provenance"] = "pi-provenance"
    release_id: str = Field(min_length=1)
    release_version: str = Field(pattern=r"^v[0-9]+\.[0-9]+\.[0-9]+$")
    case_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    map_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class Prediction(VersionedRecord):
    record_type: Literal["pi-prediction"] = "pi-prediction"
    instance_id: str = Field(min_length=1)
    answer_type: AnswerType
    value: bool | int

    @model_validator(mode="after")
    def validate_value_type(self) -> Prediction:
        if self.answer_type is AnswerType.BOOLEAN and type(self.value) is not bool:
            raise ValueError("boolean predictions require a bool")
        if self.answer_type is AnswerType.INTEGER and (
            type(self.value) is not int or self.value < 0
        ):
            raise ValueError("integer predictions require a non-negative int")
        return self

    @classmethod
    def typed(cls, instance_id: str, answer_type: AnswerType, value: bool | int) -> Prediction:
        if answer_type is AnswerType.BOOLEAN and type(value) is not bool:
            raise ValueError("boolean predictions require a bool")
        if answer_type is AnswerType.INTEGER and (type(value) is not int or value < 0):
            raise ValueError("integer predictions require a non-negative int")
        return cls(instance_id=instance_id, answer_type=answer_type, value=value)
