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

"""Round-trip tests for the HDF5 dataset writer/reader.

Builds a tiny in-memory Sample stream (no SqliteStore), writes it, then reads
it back through `inspect` and asserts the episode/frame counts, per-feature
shapes, and that stats landed. h5py is a test dependency (`learning` extra), so
these always run.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import h5py
import numpy as np

from dimos.learning.dataprep.core import OutputConfig, Sample
from dimos.learning.dataprep.formats.hdf5.reader import inspect
from dimos.learning.dataprep.formats.hdf5.writer import write


def _samples(n_episodes: int = 2, n_frames: int = 3) -> Iterator[Sample]:
    """obs `state` (4-vec) + `action` (2-vec), `n_frames` per episode."""
    for ep in range(n_episodes):
        for i in range(n_frames):
            yield Sample(
                ts=float(i),
                episode_id=f"ep_{ep:06d}",
                observation={"state": (np.arange(4, dtype=np.float32) + i)},
                action={"action": np.full(2, float(i), dtype=np.float32)},
            )


def test_hdf5_roundtrip_counts_and_shapes(tmp_path: Path) -> None:
    out = OutputConfig(
        format="hdf5",
        path=tmp_path / "session",
        metadata={"fps": 20.0, "robot": "xarm7"},
    )
    path = write(_samples(), out)
    assert path.suffix == ".hdf5"
    assert path.exists()

    info = inspect(path)
    assert info["format"] == "hdf5"
    assert info["episodes"] == 2
    assert info["frames"] == 6
    assert info["fps"] == 20.0
    assert info["robot"] == "xarm7"
    assert info["observation"]["state"]["shape"] == [4]
    assert info["action"]["action"]["shape"] == [2]
    assert info["shapes_uniform"] is True
    assert info["has_stats"] is True
    assert info["episode_lengths"] == {"min": 3, "max": 3, "mean": 3.0, "uniform": True}


def test_hdf5_extension_appended_when_missing(tmp_path: Path) -> None:
    # path with no suffix → writer appends .hdf5
    out = OutputConfig(format="hdf5", path=tmp_path / "noext")
    path = write(_samples(n_episodes=1, n_frames=2), out)
    assert path.name == "noext.hdf5"


def test_hdf5_stats_values_match(tmp_path: Path) -> None:
    out = OutputConfig(format="hdf5", path=tmp_path / "s.hdf5")
    path = write(_samples(n_episodes=1, n_frames=3), out)
    with h5py.File(path, "r") as f:
        assert "observation.state" in f["stats"]
        # state = [0..3]+i for i in 0,1,2 → per-dim mean = base + mean(0,1,2)=base+1
        mean = f["stats"]["observation.state"]["mean"][:]
        np.testing.assert_allclose(mean, np.arange(4) + 1.0)
