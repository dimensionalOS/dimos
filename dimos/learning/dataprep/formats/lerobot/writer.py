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

"""LeRobot v3.0 dataset writer.

v3.0 differs structurally from v2.x: instead of one parquet + one MP4 *per
episode*, episodes are **concatenated** into shared chunked files, and all
per-episode bookkeeping (frame/byte ranges, video time offsets, per-episode
stats) moves into an episodes *parquet*.

Layout::

    <output.path>/
        meta/info.json                              schema, fps, totals, features
        meta/tasks.parquet                          task strings (indexed by `task`)
        meta/stats.json                             aggregated per-feature stats
        meta/episodes/chunk-000/file-000.parquet    one row per episode (+ stats)
        data/chunk-000/file-000.parquet             ALL episodes' frames concatenated
        videos/<key>/chunk-000/file-000.mp4         ALL episodes for a camera, concatenated

This writer emits a **single** data file and a single MP4 per camera (chunk
000 / file 000); LeRobot supports multi-file rolling at size limits, which we
don't need yet (logged if a soft limit is exceeded). A frame's `timestamp` is
relative to its episode; the episode's `videos/<key>/from_timestamp` gives its
offset inside the shared MP4, so `from_timestamp + timestamp` locates the frame.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.learning.dataprep.core import DEFAULT_FPS, OutputConfig, Sample, is_image_array
from dimos.learning.dataprep.formats._stats import StreamingStats, stats_from_metadata
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

CHUNK = "chunk-000"
FILE = "file-000"
DATA_DIR = "data"
VIDEO_DIR = "videos"
META_DIR = "meta"
EPISODES_DIR = "episodes"

# LeRobot defaults; we write a single file but warn past these soft limits.
DATA_FILE_SIZE_MB = 100
VIDEO_FILE_SIZE_MB = 200
CHUNKS_SIZE = 1000


def _feature_name(
    prefix: str, key: str, is_image: bool, single_action: bool, single_state: bool = False
) -> str:
    """Translate (prefix, key) into the LeRobot feature name.

    Canonical names lerobot policies (ACT, Diffusion, π₀) expect:
        observation.state         single proprio vector
        action                    single action vector
        observation.images.<k>    per-camera RGB
    Multi-key fallbacks: ``observation.<key>`` / ``action.<key>``.
    """
    if prefix == "action" and single_action:
        return "action"
    if is_image:
        return f"observation.images.{key}"
    if prefix == "observation" and single_state:
        return "observation.state"
    if prefix == "observation":
        return f"observation.{key}"
    return f"action.{key}"


def _nest_image_stat(vals: list[float]) -> list[list[list[float]]]:
    """Per-channel [c0,c1,c2] → shape (C,1,1) [[[c0]],[[c1]],[[c2]]] (lerobot image stats)."""
    return [[[float(c)]] for c in vals]


def _flatten_episode_stats(
    final: dict[str, dict[str, Any]], feature_dtypes: dict[str, str]
) -> dict[str, Any]:
    """Flatten a per-episode StreamingStats result into ``stats/<feature>/<k>`` columns.

    Image features get the (C,1,1) nesting lerobot expects; low-dim stay flat.
    """
    out: dict[str, Any] = {}
    for feat, entry in final.items():
        is_video = feature_dtypes.get(feat) == "video"
        for k in ("mean", "std", "min", "max"):
            v = entry.get(k)
            if v is None:
                continue
            out[f"stats/{feat}/{k}"] = _nest_image_stat(v) if is_video else v
        out[f"stats/{feat}/count"] = int(entry["count"])
        for q in ("q01", "q99"):
            if q in entry:
                out[f"stats/{feat}/{q}"] = _nest_image_stat(entry[q]) if is_video else entry[q]
    return out


class _LeRobotV3Writer:
    """Streaming writer for the LeRobot v3.0 on-disk layout.

    One instance per dataset. Drive it as ``append`` per sample, ``flush_episode``
    at each episode boundary (and once at the end), ``close`` to release the
    parquet footer + MP4 handles, then ``finalize`` to emit the meta files. State
    that the old single-function version threaded through ``nonlocal`` closures
    lives here as instance fields, and the writer holds the lazily-imported
    pyarrow/pandas/cv2 handles so the meta step needs no module-passing params.
    """

    def __init__(self, output: OutputConfig) -> None:
        try:
            import cv2
        except ImportError as e:
            raise RuntimeError(
                "LeRobot writer requires opencv-python (cv2) for MP4 encoding"
            ) from e
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError as e:
            raise RuntimeError("LeRobot writer requires pyarrow for parquet writes") from e
        try:
            import pandas as pd
        except ImportError as e:
            raise RuntimeError("LeRobot writer requires pandas for tasks.parquet") from e

        self._cv2 = cv2
        self._pa = pa
        self._pq = pq
        self._pd = pd

        self.output = output
        self.root = Path(output.path)
        (self.root / META_DIR / EPISODES_DIR / CHUNK).mkdir(parents=True, exist_ok=True)
        (self.root / DATA_DIR / CHUNK).mkdir(parents=True, exist_ok=True)

        self.fps = float(output.metadata.get("fps", DEFAULT_FPS))
        self._fourcc = cv2.VideoWriter.fourcc(*"mp4v")
        self.default_task_label = output.metadata.get("default_task_label", "task")

        self.global_stats = self._new_stats()  # aggregated across all frames → meta/stats.json

        # Schema discovery (filled as samples flow).
        self.image_keys: list[str] = []
        self.state_keys: list[str] = []
        self.action_keys: list[str] = []
        self.feature_shapes: dict[str, tuple[int, ...]] = {}
        self.feature_dtypes: dict[str, str] = {}

        self.tasks_index: dict[str, int] = {}
        self.episode_rows: list[dict[str, Any]] = []

        # Single concatenated data file (opened on first flush).
        self.data_path = self.root / DATA_DIR / CHUNK / f"{FILE}.parquet"
        self.data_writer: Any = None

        # One MP4 per camera, persisting across episodes; from/to timestamps per episode.
        self.video_writers: dict[str, Any] = {}
        self.video_cum_frames: dict[str, int] = {}  # frames written per camera so far

        self.global_index = 0
        self.episode_index = -1

        # Per-episode buffers.
        self.cur_id: str | None = None
        self.cur_rows: list[dict[str, Any]] = []
        self.cur_ep_stats = self._new_stats()
        self.cur_task = self.default_task_label  # actual label for the in-progress episode

    def _new_stats(self) -> StreamingStats:
        return stats_from_metadata(self.output.metadata)

    def _video_path(self, image_key: str) -> Path:
        feat = _feature_name("observation", image_key, is_image=True, single_action=False)
        d = self.root / VIDEO_DIR / feat / CHUNK
        d.mkdir(parents=True, exist_ok=True)
        return d / f"{FILE}.mp4"

    def _open_video(self, image_key: str, frame: NDArray[Any]) -> Any:
        h, w = frame.shape[:2]
        path = self._video_path(image_key)
        vw = self._cv2.VideoWriter(str(path), self._fourcc, self.fps, (w, h))
        if not vw.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter for {path}")
        return vw

    def append(self, sample: Sample) -> None:
        """Ingest one sample: roll over the episode if needed, update schema +
        stats, append image frames to the per-camera MP4, and buffer the row."""
        cv2 = self._cv2
        if sample.episode_id != self.cur_id:
            self.flush_episode()
            self.cur_id = sample.episode_id
            self.episode_index += 1
            self.cur_ep_stats = self._new_stats()
            # Per-episode task label (falls back to the config default).
            self.cur_task = sample.task_label or self.default_task_label
            if self.cur_task not in self.tasks_index:
                self.tasks_index[self.cur_task] = len(self.tasks_index)

        # Schema discovery + stats (global + per-episode).
        n_low_dim_obs = sum(
            1 for v in sample.observation.values() if not is_image_array(np.asarray(v))
        )
        single_state = n_low_dim_obs == 1
        for k, arr in sample.observation.items():
            a = np.asarray(arr)
            is_image = is_image_array(a)
            name = _feature_name("observation", k, is_image, False, single_state=single_state)
            if name not in self.feature_shapes:
                self.feature_shapes[name] = tuple(a.shape)
                self.feature_dtypes[name] = "video" if is_image else str(a.dtype)
            if is_image:
                if k not in self.image_keys:
                    self.image_keys.append(k)
            elif k not in self.state_keys:
                self.state_keys.append(k)
            self.global_stats.update(name, a)
            self.cur_ep_stats.update(name, a)
        single_action = len(sample.action) == 1
        for k, arr in sample.action.items():
            a = np.asarray(arr)
            name = _feature_name("action", k, is_image=False, single_action=single_action)
            if name not in self.feature_shapes:
                self.feature_shapes[name] = tuple(a.shape)
                self.feature_dtypes[name] = str(a.dtype)
            if k not in self.action_keys:
                self.action_keys.append(k)
            self.global_stats.update(name, a)
            self.cur_ep_stats.update(name, a)

        # Append image frames to the per-camera MP4 (RGB→BGR; cv2 is BGR-native).
        for k, arr in sample.observation.items():
            a = np.asarray(arr)
            if is_image_array(a):
                if k not in self.video_writers:
                    self.video_writers[k] = self._open_video(k, a)
                if a.ndim == 2:  # grayscale → 3-channel BGR for the MP4
                    bgr = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
                elif a.shape[-1] == 3:  # RGB → BGR (cv2 is BGR-native)
                    bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
                else:
                    bgr = a
                self.video_writers[k].write(bgr)
                self.video_cum_frames[k] = self.video_cum_frames.get(k, 0) + 1

        frame_index = len(self.cur_rows)
        self.cur_rows.append(
            {
                "timestamp": frame_index / self.fps,  # relative to this episode
                "frame_index": frame_index,
                "episode_index": self.episode_index,
                "index": self.global_index,
                "task_index": self.tasks_index[self.cur_task],
                "obs": {
                    k: np.asarray(v)
                    for k, v in sample.observation.items()
                    if not is_image_array(np.asarray(v))
                },
                "act": {k: np.asarray(v) for k, v in sample.action.items()},
            }
        )
        self.global_index += 1

    def flush_episode(self) -> None:
        """Write the buffered episode's rows to the concatenated data parquet and
        append its metadata row. No-op when the buffer is empty."""
        if not self.cur_rows:
            return
        pa = self._pa
        cur_rows = self.cur_rows
        length = len(cur_rows)
        single_state = len(self.state_keys) == 1
        single_action = len(self.action_keys) == 1

        cols: dict[str, Any] = {
            "timestamp": pa.array([r["timestamp"] for r in cur_rows], pa.float32()),
            "frame_index": pa.array([r["frame_index"] for r in cur_rows], pa.int64()),
            "episode_index": pa.array([r["episode_index"] for r in cur_rows], pa.int64()),
            "index": pa.array([r["index"] for r in cur_rows], pa.int64()),
            "task_index": pa.array([r["task_index"] for r in cur_rows], pa.int64()),
        }
        f32_list = pa.list_(pa.float32())
        for k in self.state_keys:
            name = _feature_name("observation", k, False, False, single_state=single_state)
            cols[name] = pa.array([r["obs"][k].tolist() for r in cur_rows], type=f32_list)
        for k in self.action_keys:
            name = _feature_name("action", k, False, single_action=single_action)
            cols[name] = pa.array([r["act"][k].tolist() for r in cur_rows], type=f32_list)
        table = pa.Table.from_pydict(cols)
        if self.data_writer is None:
            self.data_writer = self._pq.ParquetWriter(
                self.data_path, table.schema, compression="snappy"
            )
        self.data_writer.write_table(table)

        # Episode metadata row.
        row: dict[str, Any] = {
            "episode_index": self.episode_index,
            "tasks": [list(self.tasks_index.keys())[cur_rows[0]["task_index"]]],
            "length": length,
            "data/chunk_index": 0,
            "data/file_index": 0,
            "dataset_from_index": self.global_index - length,
            "dataset_to_index": self.global_index,
            "meta/episodes/chunk_index": 0,
            "meta/episodes/file_index": 0,
        }
        for k in self.image_keys:
            feat = _feature_name("observation", k, is_image=True, single_action=False)
            cum = self.video_cum_frames.get(k, 0)
            row[f"videos/{feat}/chunk_index"] = 0
            row[f"videos/{feat}/file_index"] = 0
            row[f"videos/{feat}/from_timestamp"] = (cum - length) / self.fps
            row[f"videos/{feat}/to_timestamp"] = cum / self.fps
        row.update(_flatten_episode_stats(self.cur_ep_stats.finalize(), self.feature_dtypes))
        self.episode_rows.append(row)
        cur_rows.clear()

    def close(self) -> None:
        """Release the parquet footer and MP4 handles. Safe to call on partial
        writes — without this the data file has no footer and is unreadable."""
        if self.data_writer is not None:
            self.data_writer.close()
            self.data_writer = None
        for vw in self.video_writers.values():
            vw.release()
        self.video_writers.clear()

    def finalize(self) -> None:
        """Write info.json, tasks.parquet, episodes parquet, and aggregated stats.json."""
        pa, pq, pd = self._pa, self._pq, self._pd
        total_episodes = len(self.episode_rows)
        total_frames = self.global_index
        if self.data_path.exists() and self.data_path.stat().st_size > DATA_FILE_SIZE_MB * 1e6:
            logger.warning(
                "[dataprep] data file exceeds %d MB (single-file writer, no rolling): %s",
                DATA_FILE_SIZE_MB,
                self.data_path,
            )

        features: dict[str, Any] = {}
        for name, shape in self.feature_shapes.items():
            if self.feature_dtypes[name] == "video":
                features[name] = {
                    "dtype": "video",
                    "shape": list(shape),
                    "names": ["height", "width", "channel"],
                    "info": {
                        "video.fps": self.fps,
                        "video.height": int(shape[0]),
                        "video.width": int(shape[1]),
                        "video.channels": int(shape[2]) if len(shape) > 2 else 3,
                        "video.codec": "mp4v",
                        "video.pix_fmt": "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio": False,
                    },
                }
            else:
                n = int(shape[0]) if shape else 0
                base = name.split(".")[-1]
                features[name] = {
                    "dtype": self.feature_dtypes[name],
                    "shape": list(shape),
                    "names": [f"{base}_{i}" for i in range(n)],
                }
        for col, dt in [
            ("timestamp", "float32"),
            ("frame_index", "int64"),
            ("episode_index", "int64"),
            ("index", "int64"),
            ("task_index", "int64"),
        ]:
            features[col] = {"dtype": dt, "shape": [1], "names": None}

        info = {
            "codebase_version": "v3.0",
            "robot_type": self.output.metadata.get("robot", "unknown"),
            "total_episodes": total_episodes,
            "total_frames": total_frames,
            "total_tasks": len(self.tasks_index),
            "chunks_size": CHUNKS_SIZE,
            "data_files_size_in_mb": DATA_FILE_SIZE_MB,
            "video_files_size_in_mb": VIDEO_FILE_SIZE_MB,
            "fps": self.fps,
            "splits": {"train": f"0:{total_episodes}"},
            "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
            "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
            "features": features,
        }
        with open(self.root / META_DIR / "info.json", "w") as f:
            json.dump(info, f, indent=2)

        # tasks.parquet — task strings as the (named) index + a task_index column.
        tasks_df = pd.DataFrame(
            {"task_index": list(self.tasks_index.values())},
            index=pd.Index(list(self.tasks_index.keys()), name="task"),
        )
        tasks_df.to_parquet(self.root / META_DIR / "tasks.parquet")

        # episodes parquet — one row per episode (+ flattened per-episode stats).
        ep_table = pa.Table.from_pylist(self.episode_rows)
        pq.write_table(
            ep_table,
            self.root / META_DIR / EPISODES_DIR / CHUNK / f"{FILE}.parquet",
            compression="snappy",
        )

        # Aggregated stats.json (image features nested to (C,1,1)).
        final_stats = self.global_stats.finalize()
        for name, entry in final_stats.items():
            if self.feature_dtypes.get(name) == "video":
                for k in ("mean", "std", "min", "max"):
                    if entry.get(k) is not None:
                        entry[k] = _nest_image_stat(entry[k])
        with open(self.root / META_DIR / "stats.json", "w") as f:
            json.dump(final_stats, f, indent=2)


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Drain `samples`, write a LeRobot v3.0 dataset. Returns the dataset root path."""
    writer = _LeRobotV3Writer(output)
    # try/finally so the parquet footer is written and MP4s are released even if
    # the drain raises mid-stream — otherwise the data file is unreadable (no
    # footer) and the videos lose their index.
    try:
        for sample in samples:
            writer.append(sample)
        writer.flush_episode()
    finally:
        writer.close()

    writer.finalize()
    return writer.root
