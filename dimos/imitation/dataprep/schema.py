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

"""Dataset declarations shared by alignment, inspection, writers, and profiles.

Field annotations validate individual declarations; model validators handle only
relationships between fields or projections. Importing this module does not load
recording backends or the alignment implementation.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator
from pathlib import Path
from typing import Annotated, Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import AfterValidator, BaseModel, ConfigDict, Field, model_validator
from pydantic.json_schema import SkipJsonSchema

from dimos.constants import STATE_DIR
from dimos.protocol.service.spec import BaseConfig

# Each host-supported format package exposes a writer through ``get_writer``.
Writer = Callable[[Iterator["Sample"], "OutputConfig"], Path]
Inspector = Callable[[Path], dict[str, Any]]

SourceKind = Literal["snapshot", "joint_position_updates"]

DEFAULT_FPS = 30.0  # resample rate == written video/timestamp rate


# ─────────────────────────────────────────────────────────────────────────────
# Sub-configs
# ─────────────────────────────────────────────────────────────────────────────


class EpisodeExtractor(BaseConfig):
    extractor: Literal["episode_status", "ranges"] = "episode_status"
    # Recorded stream name for EpisodeStatus events. Must match the recorder's
    # `status` In port (CollectionRecorder records it as "status").
    status_stream: str = "status"
    ranges: list[tuple[float, float]] | None = None


def _validate_dtype(value: str) -> str:
    """Accept the video marker or a dtype understood by NumPy."""
    if value != "video":
        try:
            np.dtype(value)
        except TypeError as error:
            raise ValueError(f"unsupported feature dtype {value!r}") from error
    return value


class FeatureSpec(BaseConfig):
    """A dataset projection, optionally bound to a live input message class.

    Offline declarations need no Python class. Live collection supplies
    ``message_type`` for capture; serialization always omits that runtime type.
    """

    stream: str
    field: str | None = None
    dtype: Annotated[str, AfterValidator(_validate_dtype)]
    shape: Annotated[tuple[Annotated[int, Field(gt=0)], ...], Field(min_length=1)]
    names: list[Annotated[str, Field(pattern=r"\S")]]
    message_type: SkipJsonSchema[type[Any] | None] = Field(default=None, exclude=True)
    source_kind: SourceKind = Field(
        default="snapshot",
        description=(
            "Recorded source semantics, not an observation/action role: snapshot selects "
            "the nearest complete sample; joint_position_updates reconstructs sparse "
            "accepted targets using only commands at or before the dataset timestamp."
        ),
    )

    @model_validator(mode="after")
    def validate_schema(self) -> FeatureSpec:
        if self.dtype == "video":
            if len(self.names) != len(self.shape):
                raise ValueError("video feature names must name every axis")
        elif len(self.shape) == 1 and len(self.names) != self.shape[0]:
            raise ValueError("vector feature names must match its length")
        if self.source_kind == "joint_position_updates" and (
            self.field != "position"
            or self.dtype == "video"
            or len(self.shape) != 1
            or len(self.names) != len(set(self.names))
        ):
            raise ValueError(
                "joint_position_updates requires a position vector with unique joint names"
            )
        return self


class SyncConfig(BaseConfig):
    anchor: str
    rate_hz: float = Field(gt=0)
    tolerance_ms: float = Field(ge=0)


class QualityConfig(BaseConfig):
    mode: Literal["strict", "fill"] = "strict"
    min_source_rate_ratio: float = 0.95
    max_camera_gap_ms: float = 100.0
    max_alignment_error_ms: float = 20.0


class OutputConfig(BaseConfig):
    format: Literal["lerobot", "hdf5"] = "lerobot"
    path: Path
    metadata: dict[str, Any] = Field(default_factory=dict)


class DatasetSchema(BaseConfig):
    """Dataset features, episode extraction, alignment, and quality rules."""

    episodes: EpisodeExtractor = EpisodeExtractor()
    observation: dict[str, FeatureSpec] = Field(default_factory=dict)
    action: dict[str, FeatureSpec] = Field(default_factory=dict)
    sync: SyncConfig = SyncConfig(anchor="image", rate_hz=DEFAULT_FPS, tolerance_ms=50.0)
    quality: QualityConfig = QualityConfig()

    @model_validator(mode="after")
    def validate_sources(self) -> DatasetSchema:
        validate_source_kinds((*self.observation.values(), *self.action.values()))
        return self


def validate_source_kinds(features: Iterable[FeatureSpec]) -> None:
    """Every projection of a recorded stream must agree on its source meaning."""
    kinds: dict[str, SourceKind] = {}
    for feature in features:
        previous = kinds.setdefault(feature.stream, feature.source_kind)
        if previous != feature.source_kind:
            raise ValueError(f"stream {feature.stream!r} has conflicting source kinds")


class DataPrepConfig(DatasetSchema):
    """Dataset interpretation plus this preparation's input and output paths."""

    source: str = ""
    output: OutputConfig = OutputConfig(format="lerobot", path=STATE_DIR / "datasets" / "default")


# ─────────────────────────────────────────────────────────────────────────────
# Data records
# ─────────────────────────────────────────────────────────────────────────────


class Episode(BaseModel):
    id: str
    start_ts: float
    end_ts: float
    task_label: str | None = None
    success: bool = True
    metadata: dict[str, Any] = Field(default_factory=dict)


class IncompleteEpisode(BaseModel):
    start_ts: float
    task_label: str | None = None


class EpisodeReport(BaseModel):
    episodes: list[Episode] = Field(default_factory=list)
    incomplete: list[IncompleteEpisode] = Field(default_factory=list)


class Sample(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    ts: float
    episode_id: str
    observation: dict[str, NDArray[Any]]
    action: dict[str, NDArray[Any]]
    task_label: str | None = None  # carried from the episode for multi-task datasets
    complementary_info: dict[str, NDArray[Any]] = Field(default_factory=dict)


class EpisodeQualityReport(BaseModel):
    episode_id: str
    valid: bool
    mode: Literal["strict", "fill"]
    expected_frames: int = 0
    emitted_frames: int = 0
    filled_frames: int = 0
    source_rates_hz: dict[str, float] = Field(default_factory=dict)
    max_gaps_ms: dict[str, float] = Field(default_factory=dict)
    max_alignment_error_ms: float = 0.0
    rejection_reasons: list[str] = Field(default_factory=list)
