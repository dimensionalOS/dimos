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

"""Immutable records describing a prepared frozen-memory bundle."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class FrozenQaModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class MapperSettings(FrozenQaModel):
    voxel_size_m: float = Field(default=0.05, gt=0)
    block_count: int = Field(default=2_000_000, gt=0)
    device: str = "CUDA:0"
    carve_columns: bool = True
    frame_id: str = Field(default="world", min_length=1)
    emit_every: int = Field(default=5, gt=0)


class StreamBoundary(FrozenQaModel):
    name: str = Field(min_length=1)
    count: int = Field(ge=0)
    last_observation_id: int | None
    last_timestamp: float | None


class CutoffRecord(FrozenQaModel):
    cutoff_seconds: float = Field(ge=0)
    cutoff_timestamp: float
    normalized_progress: float | None = Field(default=None, ge=0, le=1, allow_inf_nan=False)
    stream_boundaries: tuple[StreamBoundary, ...]
    map_observation_id: int
    map_timestamp: float
    map_frame_count: int = Field(gt=0)

    @model_validator(mode="after")
    def progress_is_finite(self) -> CutoffRecord:
        if self.normalized_progress is not None and not math.isfinite(self.normalized_progress):
            raise ValueError("normalized progress must be finite")
        return self


class FrozenMemoryManifest(FrozenQaModel):
    record_type: Literal["frozen-memory-bundle"] = "frozen-memory-bundle"
    schema_version: Literal["1.0"] = "1.0"
    source_identity: str = Field(min_length=1)
    source_path: str = Field(min_length=1)
    source_size_bytes: int = Field(gt=0)
    source_mtime_ns: int = Field(gt=0)
    recording_start_timestamp: float
    recording_end_timestamp: float
    derived_path: Literal["derived.db"] = "derived.db"
    mapper: MapperSettings
    cutoffs: tuple[CutoffRecord, ...] = Field(min_length=1)
