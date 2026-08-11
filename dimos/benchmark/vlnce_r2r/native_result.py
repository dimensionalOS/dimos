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

"""Strict host-side validation for the untouched official runtime artifact."""

import hashlib
import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, model_validator

from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest

RESULT_SCHEMA_PATH = Path(__file__).with_name("result-schema.v1.json")


class NativeResultModel(BaseModel):
    """Strict base which does not inject DimOS-owned fields into native JSON."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class OfficialMetrics(NativeResultModel):
    DISTANCE_TO_GOAL: float = Field(allow_inf_nan=False)
    SUCCESS: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)
    SPL: float = Field(ge=0.0, allow_inf_nan=False)
    NDTW: float = Field(ge=0.0, allow_inf_nan=False)
    PATH_LENGTH: float = Field(ge=0.0, allow_inf_nan=False)
    ORACLE_SUCCESS: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)
    STEPS_TAKEN: float = Field(ge=0.0, allow_inf_nan=False)

    @model_validator(mode="after")
    def binary_metrics_are_binary(self) -> "OfficialMetrics":
        if self.SUCCESS not in (0.0, 1.0) or self.ORACLE_SUCCESS not in (0.0, 1.0):
            raise ValueError("official success metrics must be binary")
        return self


class TrajectoryIdentity(NativeResultModel):
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    points: int = Field(ge=1)


class VlnceNativeResult(NativeResultModel):
    schema_version: Literal["vlnce-result.v1"]
    attempt_id: str = Field(min_length=1)
    case_id: str = Field(min_length=1)
    case_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    benchmark: Literal["vlnce_r2r"]
    dataset_revision: str = Field(min_length=1)
    split: str = Field(min_length=1)
    episode_id: str = Field(min_length=1)
    scene_id: str = Field(min_length=1)
    upstream_revision: str = Field(pattern=r"^[0-9a-f]{40}$")
    runtime_image_digest: str = Field(pattern=r"^[0-9a-f]{64}$")
    protocol_revision: str = Field(min_length=1)
    result_schema_revision: str = Field(min_length=1)
    condition_label: str = Field(min_length=1)
    terminal_reason: Literal["submitted", "timeout"]
    duration_seconds: float = Field(ge=0.0, allow_inf_nan=False)
    trajectory: TrajectoryIdentity
    metrics: OfficialMetrics
    runtime: dict[str, JsonValue]


def validate_native_result(
    payload: bytes,
    *,
    case: VlnceTaskManifest,
    attempt_id: str,
) -> VlnceNativeResult:
    """Validate schema pin, exact identities, and official values without rescoring."""

    source = case.source
    validator = case.validator
    schema_digest = hashlib.sha256(RESULT_SCHEMA_PATH.read_bytes()).hexdigest()
    if schema_digest != validator.result_schema_sha256:
        raise ValueError("native result validator does not match the pinned schema")
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("native result is not valid JSON") from error
    result = VlnceNativeResult.model_validate(document)
    expected = {
        "attempt_id": attempt_id,
        "case_id": case.case_id,
        "case_fingerprint": case.fingerprint,
        "benchmark": source.benchmark,
        "dataset_revision": source.dataset_revision,
        "split": source.split,
        "episode_id": source.episode_id,
        "scene_id": source.scene_id,
        "upstream_revision": source.upstream_revision,
        "runtime_image_digest": source.preparation.image.image_digest,
        "protocol_revision": source.protocol_revision,
        "result_schema_revision": source.result_schema_revision,
        "condition_label": source.condition_label,
    }
    for field, value in expected.items():
        if getattr(result, field) != value:
            raise ValueError(f"native result identity {field!r} does not match the attempt")
    for field in validator.identity_fields:
        if field not in expected:
            raise ValueError(f"unsupported native result identity field {field!r}")
    return result
