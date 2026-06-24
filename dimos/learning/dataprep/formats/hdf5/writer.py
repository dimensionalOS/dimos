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

"""HDF5 dataset writer.

Single ``.hdf5`` file with one group per episode plus a stats group.
Layout::

    <output.path>.hdf5            (or output.path.<format> if a dir was given)
        /                         attrs: codebase_version, robot, fps,
                                         num_episodes, num_frames, num_tasks
        /tasks                    attrs: task_<i> = "<description>"
        /stats/<feature>          attrs: count + datasets mean/std/min/max[/q01/q99]
        /episodes/episode_NNNNNN
            timestamp             (T,)            float32
            <obs_key>             (T, ...)        as recorded
            <action_key>          (T, ...)        as recorded
                                  attrs: length, start_ts, task_index

This is the ACT-original style adapted to one file with multiple episodes.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.learning.dataprep.core import DEFAULT_FPS, OutputConfig, Sample
from dimos.learning.dataprep.formats._stats import stats_from_metadata


class _Hdf5Writer:
    """Streaming writer for the single-file multi-episode HDF5 layout.

    One instance per dataset, owning the open ``h5py.File``. Drive it as
    ``append`` per sample, ``flush_episode`` at each episode boundary (and once
    at the end), then ``finalize`` to write the meta groups and close the file.
    Per-episode buffers that the old single-function version kept as ``nonlocal``
    closure state live here as instance fields.
    """

    def __init__(self, output: OutputConfig) -> None:
        try:
            import h5py
        except ImportError as e:
            raise RuntimeError("HDF5 writer requires h5py — install with `pip install h5py`") from e

        self.output = output
        self.out = Path(output.path)
        if self.out.suffix not in (".h5", ".hdf5"):
            self.out = self.out.with_suffix(".hdf5")
        self.out.parent.mkdir(parents=True, exist_ok=True)

        self.stats = stats_from_metadata(output.metadata)
        self.default_task_label: str = output.metadata.get("default_task_label", "task")
        self.fps = float(output.metadata.get("fps", DEFAULT_FPS))

        self.tasks_index: dict[str, int] = {}
        self.total_frames = 0

        # Per-episode buffers — flushed at episode boundary.
        self.cur_id: str | None = None
        self.cur_idx = 0
        self.cur_task = self.default_task_label  # actual label for the in-progress episode
        self.cur_start_ts: float | None = None
        self.buf_ts: list[float] = []
        self.buf_obs: dict[str, list[NDArray[Any]]] = {}
        self.buf_act: dict[str, list[NDArray[Any]]] = {}

        self._h5 = h5py.File(self.out, "w")
        self._episodes_g = self._h5.create_group("episodes")

    def append(self, sample: Sample) -> None:
        """Ingest one sample: roll over the episode if needed, then buffer the
        timestamp + obs/action frames and update stats."""
        if sample.episode_id != self.cur_id:
            if self.flush_episode():
                self.cur_idx += 1
            self.cur_id = sample.episode_id
            self.cur_start_ts = float(sample.ts)
            self.cur_task = sample.task_label or self.default_task_label
            if self.cur_task not in self.tasks_index:
                self.tasks_index[self.cur_task] = len(self.tasks_index)

        self.buf_ts.append(float(sample.ts) - (self.cur_start_ts or 0.0))
        for k, v in sample.observation.items():
            a = np.asarray(v)
            self.buf_obs.setdefault(k, []).append(a)
            self.stats.update(f"observation.{k}", a)
        for k, v in sample.action.items():
            a = np.asarray(v)
            self.buf_act.setdefault(k, []).append(a)
            self.stats.update(f"action.{k}", a)
        self.total_frames += 1

    def flush_episode(self) -> bool:
        """Write the buffered episode as an ``episodes/episode_NNNNNN`` group.
        Returns False (and does nothing) when the buffer is empty."""
        if not self.buf_ts:
            return False
        ep = self._episodes_g.create_group(f"episode_{self.cur_idx:06d}")
        ep.attrs["length"] = len(self.buf_ts)
        ep.attrs["start_ts"] = float(self.cur_start_ts or 0.0)
        ep.attrs["task_index"] = self.tasks_index[self.cur_task]
        ep.create_dataset("timestamp", data=np.asarray(self.buf_ts, dtype=np.float32))
        for k, frames in self.buf_obs.items():
            arr = np.stack(frames, axis=0)
            # arr is time-stacked (T, …); an image is therefore ndim >= 3
            # here (2D grayscale → (T, H, W)), low-dim → (T, D).
            is_image = arr.ndim >= 3
            ep.create_dataset(
                f"observation/{k}",
                data=arr,
                compression="gzip" if is_image else None,
                compression_opts=4 if is_image else None,
            )
        for k, frames in self.buf_act.items():
            ep.create_dataset(f"action/{k}", data=np.stack(frames, axis=0))
        self.buf_ts.clear()
        self.buf_obs.clear()
        self.buf_act.clear()
        return True

    def finalize(self) -> None:
        """Write the root attrs + tasks/stats groups, then close the file."""
        h5 = self._h5
        h5.attrs["codebase_version"] = "dimos-v1"
        h5.attrs["robot"] = self.output.metadata.get("robot", "unknown")
        h5.attrs["fps"] = self.fps
        h5.attrs["num_episodes"] = len(self._episodes_g)
        h5.attrs["num_frames"] = self.total_frames
        h5.attrs["num_tasks"] = len(self.tasks_index)

        tasks_g = h5.create_group("tasks")
        for task, idx in self.tasks_index.items():
            tasks_g.attrs[f"task_{idx}"] = task

        stats_g = h5.create_group("stats")
        for name, entry in self.stats.finalize().items():
            g = stats_g.create_group(name)
            g.attrs["count"] = entry["count"]
            for k in ("mean", "std", "min", "max", "q01", "q99"):
                if k in entry and entry[k] is not None:
                    g.create_dataset(k, data=np.asarray(entry[k], dtype=np.float64))
        h5.close()

    def close(self) -> None:
        """Release the file without writing meta — for teardown on a partial write."""
        if self._h5:
            self._h5.close()


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Drain `samples` into a single .hdf5 file. Returns the file path."""
    writer = _Hdf5Writer(output)
    try:
        for sample in samples:
            writer.append(sample)
        writer.flush_episode()
        writer.finalize()
    except BaseException:
        writer.close()  # release the file handle on a partial/failed write
        raise
    return writer.out
