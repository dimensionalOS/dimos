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

"""LeRobot v2 dataset writer.

Layout::

    <output.path>/
        meta/info.json          schema, fps, total episodes/frames, features
        meta/episodes.jsonl     per-episode metadata
        meta/tasks.jsonl        task descriptions for language conditioning
        meta/stats.json         per-feature mean/std/min/max/q01/q99
        data/chunk-000/episode_NNNNNN.parquet
        videos/chunk-000/observation.images.<key>/episode_NNNNNN.mp4

Single pass: streams samples to disk per-episode and accumulates stats in
parallel. Image frames go to MP4 (one per camera, per episode); their
columns are excluded from the parquet — lerobot loads them from MP4 at
``__getitem__`` time using the ``video_path`` template + episode_index +
timestamp.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.learning.dataprep.core import OutputConfig, Sample
from dimos.learning.dataprep.formats._stats import StreamingStats

CHUNK = "chunk-000"
DATA_DIR = "data"
VIDEO_DIR = "videos"
META_DIR = "meta"


def _feature_name(
    prefix: str, key: str, is_image: bool, single_action: bool, single_state: bool = False
) -> str:
    """Translate (prefix, key) into the LeRobot v2 feature name.

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


def write(samples: Iterator[Sample], output: OutputConfig) -> Path:
    """Drain `samples`, write parquet+MP4+meta in LeRobot v2 layout.
    Returns the dataset root path.
    """
    try:
        import cv2
    except ImportError as e:
        raise RuntimeError("LeRobot writer requires opencv-python (cv2) for MP4 encoding") from e
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as e:
        raise RuntimeError("LeRobot writer requires pyarrow for parquet writes") from e

    root = Path(output.path)
    (root / META_DIR).mkdir(parents=True, exist_ok=True)
    (root / DATA_DIR / CHUNK).mkdir(parents=True, exist_ok=True)
    (root / VIDEO_DIR / CHUNK).mkdir(parents=True, exist_ok=True)

    fps = float(output.metadata.get("fps", 30.0))
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    stats = StreamingStats(
        image_subsample=int(output.metadata.get("image_subsample", 10)),
        quantile_reservoir=int(output.metadata.get("quantile_reservoir", 10_000)),
        seed=int(output.metadata.get("stats_seed", 0)),
    )

    image_keys: set[str] = set()
    state_keys: list[str] = []
    action_keys: list[str] = []
    feature_shapes: dict[str, tuple[int, ...]] = {}
    feature_dtypes: dict[str, str] = {}

    episodes_meta: list[dict[str, Any]] = []
    tasks_index: dict[str, int] = {}
    default_task_label = output.metadata.get("default_task_label", "task")

    current_episode_id: str | None = None
    current_episode_index = -1
    current_frames: list[dict[str, Any]] = []
    current_video_writers: dict[str, Any] = {}
    global_index = 0

    def _episode_path_parquet(ep_idx: int) -> Path:
        return root / DATA_DIR / CHUNK / f"episode_{ep_idx:06d}.parquet"

    def _episode_path_video(image_key: str, ep_idx: int) -> Path:
        feat_name = _feature_name("observation", image_key, is_image=True, single_action=False)
        d = root / VIDEO_DIR / CHUNK / feat_name
        d.mkdir(parents=True, exist_ok=True)
        return d / f"episode_{ep_idx:06d}.mp4"

    def _open_video(image_key: str, ep_idx: int, frame: np.ndarray) -> Any:
        # Frames are RGB→BGR converted at write time (see the write loop below).
        h, w = frame.shape[:2]
        path = _episode_path_video(image_key, ep_idx)
        writer = cv2.VideoWriter(str(path), fourcc, fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter for {path}")
        return writer

    def _flush_episode() -> None:
        nonlocal current_frames, current_video_writers, current_episode_index
        if not current_frames:
            return
        for vw in current_video_writers.values():
            vw.release()
        current_video_writers = {}

        cols: dict[str, list[Any]] = {
            "timestamp": [f["timestamp"] for f in current_frames],
            "frame_index": [f["frame_index"] for f in current_frames],
            "episode_index": [f["episode_index"] for f in current_frames],
            "index": [f["index"] for f in current_frames],
            "task_index": [f["task_index"] for f in current_frames],
        }
        f32_list = pa.list_(pa.float32())
        single_state = len(state_keys) == 1
        for k in state_keys:
            name = _feature_name(
                "observation", k, is_image=False, single_action=False, single_state=single_state
            )
            cols[name] = pa.array([f["obs"][k].tolist() for f in current_frames], type=f32_list)
        single_action = len(action_keys) == 1
        for k in action_keys:
            name = _feature_name("action", k, is_image=False, single_action=single_action)
            cols[name] = pa.array([f["act"][k].tolist() for f in current_frames], type=f32_list)
        # Video columns intentionally omitted: lerobot's hf_features schema
        # skips dtype="video" and reads frames from MP4 at __getitem__ time.

        table = pa.Table.from_pydict(cols)
        pq.write_table(table, _episode_path_parquet(current_episode_index))

        episodes_meta.append(
            {
                "episode_index": current_episode_index,
                "tasks": [list(tasks_index.keys())[current_frames[0]["task_index"]]],
                "length": len(current_frames),
            }
        )
        current_frames = []

    for sample in samples:
        if sample.episode_id != current_episode_id:
            _flush_episode()
            current_episode_id = sample.episode_id
            current_episode_index += 1
            label = default_task_label
            if label not in tasks_index:
                tasks_index[label] = len(tasks_index)

        # Schema discovery + stats accumulation.
        n_low_dim_obs = sum(1 for _, v in sample.observation.items() if np.asarray(v).ndim < 3)
        single_state = n_low_dim_obs == 1
        for k, arr in sample.observation.items():
            a = np.asarray(arr)
            is_image = a.ndim >= 3
            name = _feature_name(
                "observation", k, is_image=is_image, single_action=False, single_state=single_state
            )
            if name not in feature_shapes:
                feature_shapes[name] = tuple(a.shape)
                feature_dtypes[name] = "video" if is_image else str(a.dtype)
            if is_image:
                image_keys.add(k)
            elif k not in state_keys:
                state_keys.append(k)
            stats.update(name, a)

        for k, arr in sample.action.items():
            a = np.asarray(arr)
            single_action = len(sample.action) == 1
            name = _feature_name("action", k, is_image=False, single_action=single_action)
            if name not in feature_shapes:
                feature_shapes[name] = tuple(a.shape)
                feature_dtypes[name] = str(a.dtype)
            if k not in action_keys:
                action_keys.append(k)
            stats.update(name, a)

        # Video frame write + parquet row buffer.
        frame_index = len(current_frames)
        for k, arr in sample.observation.items():
            a = np.asarray(arr)
            if a.ndim >= 3:
                if k not in current_video_writers:
                    current_video_writers[k] = _open_video(k, current_episode_index, a)
                # Frames are RGB; cv2.VideoWriter is BGR-native — convert or
                # the MP4 decodes color-swapped.
                bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if a.shape[-1] == 3 else a
                current_video_writers[k].write(bgr)

        rel_ts = frame_index / fps
        current_frames.append(
            {
                "timestamp": rel_ts,
                "frame_index": frame_index,
                "episode_index": current_episode_index,
                "index": global_index,
                "task_index": tasks_index[default_task_label],
                "obs": {
                    k: np.asarray(v)
                    for k, v in sample.observation.items()
                    if np.asarray(v).ndim < 3
                },
                "act": {k: np.asarray(v) for k, v in sample.action.items()},
            }
        )
        global_index += 1

    _flush_episode()

    # ── meta files ───────────────────────────────────────────────────────────
    total_episodes = len(episodes_meta)
    total_frames = global_index

    features: dict[str, Any] = {}
    for name, shape in feature_shapes.items():
        if feature_dtypes[name] == "video":
            features[name] = {
                "dtype": "video",
                "shape": list(shape),
                "names": ["height", "width", "channel"],
                "info": {
                    "video.fps": fps,
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
            # Per-dim names; downstream loaders only require len(names) == shape[0].
            n = int(shape[0]) if shape else 0
            base = name.split(".")[-1]
            features[name] = {
                "dtype": feature_dtypes[name],
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
        "codebase_version": "v2.0",
        "robot_type": output.metadata.get("robot", "unknown"),
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": len(tasks_index),
        "total_videos": total_episodes * len(image_keys),
        "total_chunks": 1,
        "chunks_size": max(1, total_episodes),
        "fps": fps,
        "splits": {"train": f"0:{total_episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": features,
    }
    with open(root / META_DIR / "info.json", "w") as f:
        json.dump(info, f, indent=2)
    with open(root / META_DIR / "episodes.jsonl", "w") as f:
        for ep in episodes_meta:
            f.write(json.dumps(ep) + "\n")
    with open(root / META_DIR / "tasks.jsonl", "w") as f:
        for task, idx in tasks_index.items():
            f.write(json.dumps({"task_index": idx, "task": task}) + "\n")
    final_stats = stats.finalize()
    for name, entry in final_stats.items():
        if feature_dtypes.get(name) == "video":
            for k in ("mean", "std", "min", "max"):
                if entry.get(k) is not None:
                    entry[k] = [[[c]] for c in entry[k]]
    with open(root / META_DIR / "stats.json", "w") as f:
        json.dump(final_stats, f, indent=2)

    return root


_META_COLS = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


def inspect(path: Path) -> dict[str, Any]:
    """Summarize a LeRobot v2 dataset from its meta/ files (no parquet load)."""
    from dimos.learning.dataprep.core import summarize_lengths

    root = Path(path)
    info = json.loads((root / META_DIR / "info.json").read_text())
    features = info.get("features", {})

    observation: dict[str, Any] = {}
    action: dict[str, Any] = {}
    for name, feat in features.items():
        if name in _META_COLS:
            continue
        entry = {"shape": feat.get("shape"), "dtype": feat.get("dtype")}
        if name.startswith("observation"):
            observation[name] = entry
        elif name.startswith("action"):
            action[name] = entry

    lengths: list[int] = []
    ep_path = root / META_DIR / "episodes.jsonl"
    if ep_path.exists():
        for line in ep_path.read_text().splitlines():
            if line.strip():
                lengths.append(int(json.loads(line).get("length", 0)))

    return {
        "format": "lerobot",
        "path": str(root),
        "episodes": info.get("total_episodes"),
        "frames": info.get("total_frames"),
        "fps": info.get("fps"),
        "robot": info.get("robot_type"),
        "observation": observation,
        "action": action,
        "episode_lengths": summarize_lengths(lengths),
        "shapes_uniform": True,  # LeRobot declares one global feature schema
        "has_stats": (root / META_DIR / "stats.json").exists(),
    }
