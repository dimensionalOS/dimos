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

"""HDF5 dataset reader — read-back / summary of a dataset written by `writer.py`.

Mirror of ``_Hdf5Writer``: opens the file read-only and exposes summary helpers.
New read-side features (e.g. logging episodes to Rerun) belong here as methods.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.learning.dataprep.core import summarize_lengths


class _Hdf5Reader:
    """Read-only view over a built ``.hdf5`` dataset.

    One instance per file, owning an open read-only ``h5py.File``. Use as a
    context manager (``with _Hdf5Reader(path) as r: r.summary()``) so the handle
    is always released.
    """

    def __init__(self, path: Path) -> None:
        try:
            import h5py
        except ImportError as e:
            raise RuntimeError(
                "HDF5 inspect requires h5py — install with `pip install h5py`"
            ) from e

        self.path = Path(path)
        self._h5 = h5py.File(self.path, "r")
        self._episodes_g = self._h5["episodes"]
        self._ep_names = sorted(self._episodes_g.keys())

    def __enter__(self) -> _Hdf5Reader:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        if self._h5:
            self._h5.close()

    def _episode_lengths(self) -> list[int]:
        return [int(self._episodes_g[e].attrs.get("length", 0)) for e in self._ep_names]

    def _features(self) -> tuple[dict[str, Any], dict[str, Any]]:
        """Per-frame (shape, dtype) of each obs/action feature, from the first
        episode (per-frame shape = ``dataset.shape[1:]``)."""
        observation: dict[str, Any] = {}
        action: dict[str, Any] = {}
        if self._ep_names:
            first = self._episodes_g[self._ep_names[0]]
            for grp, ref in (("observation", observation), ("action", action)):
                if grp in first:
                    for k, d in first[grp].items():
                        ref[k] = {"shape": list(d.shape[1:]), "dtype": str(d.dtype)}
        return observation, action

    def _shapes_uniform(self, observation: dict[str, Any], action: dict[str, Any]) -> bool:
        """Whether per-frame shapes are consistent across every episode."""
        for ep_name in self._ep_names[1:]:
            g = self._episodes_g[ep_name]
            for grp, ref in (("observation", observation), ("action", action)):
                if grp in g:
                    for k, d in g[grp].items():
                        if k in ref and list(d.shape[1:]) != ref[k]["shape"]:
                            return False
        return True

    def summary(self) -> dict[str, Any]:
        """Features (per-frame shape/dtype), episode/frame counts, and whether
        feature shapes are uniform across episodes."""
        h5 = self._h5
        lengths = self._episode_lengths()
        observation, action = self._features()
        return {
            "format": "hdf5",
            "path": str(self.path),
            "episodes": int(h5.attrs.get("num_episodes", len(self._ep_names))),
            "frames": int(h5.attrs.get("num_frames", sum(lengths))),
            "fps": float(h5.attrs.get("fps", 0.0)),
            "robot": str(h5.attrs.get("robot", "unknown")),
            "observation": observation,
            "action": action,
            "episode_lengths": summarize_lengths(lengths),
            "shapes_uniform": self._shapes_uniform(observation, action),
            "has_stats": "stats" in h5,
        }


def inspect(path: Path) -> dict[str, Any]:
    """Summarize an .hdf5 dataset. Thin driver over ``_Hdf5Reader``."""
    with _Hdf5Reader(path) as reader:
        return reader.summary()
