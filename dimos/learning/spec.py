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

"""Data definitions for the DimOS Learning Framework.

Contains the YAML/JSON-backed `DatasetSpec` schema and the runtime data
classes (`Episode`, `Sample`) shared between collection, training, and
inference. No logic — just typed records and constants. Safe to import
from anywhere (no circular dependencies).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

# ─────────────────────────────────────────────────────────────────────────────
# DatasetSpec — the YAML/JSON schema
# ─────────────────────────────────────────────────────────────────────────────


class EpisodeConfig(BaseModel):
    """How to slice the continuous recording into episodes."""

    extractor: Literal["buttons", "ranges", "whole_session"] = "buttons"

    # BUTTONS extractor: friendly names map to Quest Buttons attrs via BUTTON_ALIASES.
    # The state machine always discards an in-progress episode if the recording ends
    # without an explicit save/discard press.
    button_stream: str = "buttons"
    start: str = "A"  # rising edge -> begin episode
    save: str = "B"  # rising edge -> end + save
    discard: str = "X"  # rising edge -> end + drop

    # RANGES extractor: explicit absolute timestamps
    ranges: list[tuple[float, float]] | None = None

    # Optional default label applied to every extracted episode
    default_task_label: str | None = None


class FieldRef(BaseModel):
    """Pointer to a field in a recorded stream."""

    stream: str  # LCM stream / topic name as recorded in session.db
    type: str | None = None  # optional dotted type (e.g. "sensor_msgs.Image"); for codec dispatch
    field: str | None = None  # attribute on the message; None = whole message
    preprocess: str | None = None  # named preprocess hook (e.g. "jpeg_decode", "normalize_image")


class SyncConfig(BaseModel):
    """How to build per-timestep samples by aligning multiple streams."""

    anchor: str  # key in `observation` that drives the timeline
    rate_hz: float = 30.0  # downsample anchor to this rate; 0 = use anchor's native rate
    tolerance_ms: float = 50.0  # max allowed time delta when picking nearest sample
    strategy: Literal["nearest", "interp"] = "nearest"


class FilterConfig(BaseModel):
    """Per-episode filters applied after extraction."""

    success_only: bool = True
    min_duration_s: float = 0.0
    max_duration_s: float | None = None
    task_labels: list[str] | None = None  # whitelist; None = all


class OutputConfig(BaseModel):
    """Where and how to write the built dataset."""

    format: Literal["lerobot", "hdf5", "rlds"]
    path: Path
    metadata: dict[str, Any] = Field(default_factory=dict)


class DatasetSpec(BaseModel):
    """Top-level spec. Same instance used at build, load, and inference time."""

    source: Path  # path to session.db produced by RecordReplay
    episodes: EpisodeConfig
    observation: dict[str, FieldRef]  # obs key -> stream field
    action: dict[str, FieldRef]  # action key -> stream field
    sync: SyncConfig
    filters: FilterConfig | None = None
    output: OutputConfig | None = None  # only required by build_dataset()


# ─────────────────────────────────────────────────────────────────────────────
# Runtime data
# ─────────────────────────────────────────────────────────────────────────────


# Friendly Quest controller names -> Buttons attribute names.
# Override by supplying an attribute name directly in the spec.
BUTTON_ALIASES: dict[str, str] = {
    "A": "right_primary",
    "B": "right_secondary",
    "X": "left_primary",
    "Y": "left_secondary",
    "LT": "left_trigger",
    "RT": "right_trigger",
    "LG": "left_grip",
    "RG": "right_grip",
    "MENU_L": "left_menu",
    "MENU_R": "right_menu",
}


class Episode(BaseModel):
    """A single demonstration carved from a session."""

    id: str
    start_ts: float
    end_ts: float
    task_label: str | None = None
    success: bool = True
    metadata: dict[str, Any] = Field(default_factory=dict)

    @property
    def duration(self) -> float:
        return self.end_ts - self.start_ts


class Sample(BaseModel):
    """One synchronized timestep: aligned obs + action at ts."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    ts: float
    episode_id: str
    observation: dict[str, np.ndarray]
    action: dict[str, np.ndarray]
