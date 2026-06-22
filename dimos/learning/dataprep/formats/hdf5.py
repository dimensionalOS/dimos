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

from dimos.learning.dataprep.core import OutputConfig, Sample
from dimos.learning.dataprep.formats._stats import StreamingStats


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Drain `samples` into a single .hdf5 file. Returns the file path."""
    try:
        import h5py
    except ImportError as e:
        raise RuntimeError("HDF5 writer requires h5py — install with `pip install h5py`") from e

    out = Path(output.path)
    if out.suffix not in (".h5", ".hdf5"):
        out = out.with_suffix(".hdf5")
    out.parent.mkdir(parents=True, exist_ok=True)

    stats = StreamingStats(
        image_subsample=int(output.metadata.get("image_subsample", 10)),
        quantile_reservoir=int(output.metadata.get("quantile_reservoir", 10_000)),
        seed=int(output.metadata.get("stats_seed", 0)),
    )

    default_task_label: str = output.metadata.get("default_task_label", "task")
    fps = float(output.metadata.get("fps", 30.0))

    tasks_index: dict[str, int] = {}

    # Per-episode buffers — flushed at episode boundary.
    cur_id: str | None = None
    cur_idx = 0
    cur_task = default_task_label  # actual label for the in-progress episode
    cur_start_ts: float | None = None
    buf_ts: list[float] = []
    buf_obs: dict[str, list[np.ndarray]] = {}
    buf_act: dict[str, list[np.ndarray]] = {}

    total_frames = 0

    with h5py.File(out, "w") as h5:
        episodes_g = h5.create_group("episodes")

        def _flush() -> bool:
            if not buf_ts:
                return False
            ep = episodes_g.create_group(f"episode_{cur_idx:06d}")
            ep.attrs["length"] = len(buf_ts)
            ep.attrs["start_ts"] = float(cur_start_ts or 0.0)
            ep.attrs["task_index"] = tasks_index[cur_task]
            ep.create_dataset("timestamp", data=np.asarray(buf_ts, dtype=np.float32))
            for k, frames in buf_obs.items():
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
            for k, frames in buf_act.items():
                ep.create_dataset(f"action/{k}", data=np.stack(frames, axis=0))
            buf_ts.clear()
            buf_obs.clear()
            buf_act.clear()
            return True

        for sample in samples:
            if sample.episode_id != cur_id:
                if _flush():
                    cur_idx += 1
                cur_id = sample.episode_id
                cur_start_ts = float(sample.ts)
                cur_task = sample.task_label or default_task_label
                if cur_task not in tasks_index:
                    tasks_index[cur_task] = len(tasks_index)

            buf_ts.append(float(sample.ts) - (cur_start_ts or 0.0))
            for k, v in sample.observation.items():
                a = np.asarray(v)
                buf_obs.setdefault(k, []).append(a)
                stats.update(f"observation.{k}", a)
            for k, v in sample.action.items():
                a = np.asarray(v)
                buf_act.setdefault(k, []).append(a)
                stats.update(f"action.{k}", a)
            total_frames += 1

        _flush()

        # ── meta ────────────────────────────────────────────────────────────
        h5.attrs["codebase_version"] = "dimos-v1"
        h5.attrs["robot"] = output.metadata.get("robot", "unknown")
        h5.attrs["fps"] = fps
        h5.attrs["num_episodes"] = len(episodes_g)
        h5.attrs["num_frames"] = total_frames
        h5.attrs["num_tasks"] = len(tasks_index)

        tasks_g = h5.create_group("tasks")
        for task, idx in tasks_index.items():
            tasks_g.attrs[f"task_{idx}"] = task

        stats_g = h5.create_group("stats")
        for name, entry in stats.finalize().items():
            g = stats_g.create_group(name)
            g.attrs["count"] = entry["count"]
            for k in ("mean", "std", "min", "max", "q01", "q99"):
                if k in entry and entry[k] is not None:
                    g.create_dataset(k, data=np.asarray(entry[k], dtype=np.float64))

    return out


def inspect(path: Path) -> dict[str, Any]:
    """Summarize an .hdf5 dataset: features (per-frame shape/dtype), episode
    counts, and whether feature shapes are uniform across episodes."""
    try:
        import h5py
    except ImportError as e:
        raise RuntimeError("HDF5 inspect requires h5py — install with `pip install h5py`") from e

    from dimos.learning.dataprep.core import summarize_lengths

    out = Path(path)
    with h5py.File(out, "r") as h5:
        eps_g = h5["episodes"]
        ep_names = sorted(eps_g.keys())
        lengths = [int(eps_g[e].attrs.get("length", 0)) for e in ep_names]

        observation: dict[str, Any] = {}
        action: dict[str, Any] = {}
        # Feature schema from the first episode (per-frame shape = dataset.shape[1:]).
        if ep_names:
            first = eps_g[ep_names[0]]
            for grp, ref in (("observation", observation), ("action", action)):
                if grp in first:
                    for k, d in first[grp].items():
                        ref[k] = {"shape": list(d.shape[1:]), "dtype": str(d.dtype)}

        # Are per-frame shapes consistent across every episode?
        shapes_uniform = True
        for ep_name in ep_names[1:]:
            g = eps_g[ep_name]
            for grp, ref in (("observation", observation), ("action", action)):
                if grp in g:
                    for k, d in g[grp].items():
                        if k in ref and list(d.shape[1:]) != ref[k]["shape"]:
                            shapes_uniform = False

        return {
            "format": "hdf5",
            "path": str(out),
            "episodes": int(h5.attrs.get("num_episodes", len(ep_names))),
            "frames": int(h5.attrs.get("num_frames", sum(lengths))),
            "fps": float(h5.attrs.get("fps", 0.0)),
            "robot": str(h5.attrs.get("robot", "unknown")),
            "observation": observation,
            "action": action,
            "episode_lengths": summarize_lengths(lengths),
            "shapes_uniform": shapes_uniform,
            "has_stats": "stats" in h5,
        }
