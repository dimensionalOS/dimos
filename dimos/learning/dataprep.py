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

"""Dataset builder/loader for the DimOS Learning Framework.

Reads a `DatasetSpec` (see `dimos.learning.spec`) and either:
  - builds a training-ready dataset on disk in HDF5/RLDS/LeRobot, or
  - returns a PyTorch Dataset for training.

The same spec also drives inference observation construction.

Workflow:
    # 1. Record a teleop session (Sam's PR #1708)
    dimos --blueprint quest_teleop_xarm7 --record-path session.db

    # 2. Build a training-ready dataset
    python -m dimos.learning.dataprep build dataset.yaml

    # 3. Train using the same spec
    from dimos.learning.dataprep import load_dataset, load_spec
    spec = load_spec("dataset.yaml")
    ds = load_dataset(spec)
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.learning.spec import (
    DatasetSpec,
    Episode,
    EpisodeConfig,
    FieldRef,
    FilterConfig,
    OutputConfig,
    Sample,
)

Writer = Callable[[Iterator[Sample], OutputConfig], Path]

if TYPE_CHECKING:
    import torch

    from dimos.memory2.store.sqlite import SqliteStore


# ─────────────────────────────────────────────────────────────────────────────
# Spec I/O
# ─────────────────────────────────────────────────────────────────────────────


def load_spec(path: str | Path) -> DatasetSpec:
    """Load a DatasetSpec from .yaml/.yml/.json (dispatch by extension)."""
    raise NotImplementedError


def save_spec(spec: DatasetSpec, path: str | Path) -> None:
    """Write a DatasetSpec back to .yaml/.yml/.json (round-trip safe)."""
    raise NotImplementedError


# ─────────────────────────────────────────────────────────────────────────────
# Episode extraction
# ─────────────────────────────────────────────────────────────────────────────


def extract_episodes(store: SqliteStore, cfg: EpisodeConfig) -> list[Episode]:
    """Extract episode boundaries from the recording per the configured strategy.

    BUTTONS: scan cfg.button_stream for rising edges on cfg.start/save/discard.
        State machine:
            IDLE   --start press-->          RECORDING  (begin episode)
            RECORDING --save press-->        IDLE       (commit, success=True)
            RECORDING --discard press-->     IDLE       (drop)
            RECORDING --start press-->       RECORDING  (auto-commit, begin new)
            session ends mid-episode:        always discard

    RANGES: emit one Episode per (start_ts, end_ts) tuple in cfg.ranges.

    WHOLE: emit a single Episode covering the entire recording's time range.
    """
    raise NotImplementedError


def filter_episodes(eps: list[Episode], cfg: FilterConfig | None) -> list[Episode]:
    """Apply success/duration/label whitelist filters. None = pass-through."""
    raise NotImplementedError


# ─────────────────────────────────────────────────────────────────────────────
# Stream synchronization (build per-timestep samples)
# ─────────────────────────────────────────────────────────────────────────────


def iter_samples(
    store: SqliteStore,
    episode: Episode,
    spec: DatasetSpec,
) -> Iterator[Sample]:
    """Yield synced (obs, action) Samples for one episode.

    Walks the anchor stream at sync.rate_hz between episode.start_ts and
    episode.end_ts. For each anchor timestamp, pulls the nearest observation/
    action from each configured stream within sync.tolerance_ms. Applies any
    declared preprocess (e.g. jpeg_decode for Image, field projection for
    JointState). Skips frames where any required stream lacks a sample within
    tolerance.
    """
    raise NotImplementedError


def _resolve_field(msg: Any, ref: FieldRef) -> np.ndarray:
    """Pull a single field from a stream message and convert to np.ndarray.

    Applies ref.field projection (attribute access) and ref.preprocess hook
    (named transform like jpeg_decode). Returns a numpy array suitable for
    inclusion in a Sample.
    """
    raise NotImplementedError


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────


def _get_writer(format_name: str) -> Writer:
    """Lazy-import the `write` function for a given format. Avoids loading
    heavy deps (h5py, tfds, lerobot) for unused formats."""
    if format_name == "lerobot":
        from dimos.learning.formats.lerobot import write
    elif format_name == "hdf5":
        from dimos.learning.formats.hdf5 import write
    elif format_name == "rlds":
        from dimos.learning.formats.rlds import write
    else:
        raise ValueError(
            f"Unknown dataset format: {format_name!r}. Supported: lerobot, hdf5, rlds."
        )
    return write


def build_dataset(spec: DatasetSpec) -> Path:
    """End-to-end: raw session.db -> on-disk dataset in spec.output.format.

    Returns the path written. Requires spec.output to be set. Dispatches to
    the appropriate writer in `dimos.learning.formats` via `_get_writer`.
    """
    raise NotImplementedError


def load_dataset(spec: DatasetSpec) -> torch.utils.data.Dataset[Sample]:
    """Training-time loader: returns a PyTorch Dataset over the source recording.

    Materializes Samples on the fly (lazy). Does not require spec.output.
    Pre-extracts episodes once and indexes anchor timestamps for O(1) __getitem__.
    """
    raise NotImplementedError


def inspect(spec: DatasetSpec) -> dict[str, Any]:
    """Stats for a session: episode count, duration distribution, per-stream counts.

    Used by `python -m dimos.learning.dataset inspect`.
    """
    raise NotImplementedError


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """CLI entrypoint: build / inspect a dataset spec."""
    raise NotImplementedError


if __name__ == "__main__":
    main()
