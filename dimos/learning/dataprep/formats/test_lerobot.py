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

"""Smoke tests for the LeRobot v3.0 writer/reader.

Asserts the v3.0 layout: a single concatenated data parquet, parquet meta
(tasks + episodes, no jsonl), and one MP4 per camera under
`videos/<key>/chunk-000/`. pyarrow/pandas (the `learning` extra) and cv2 are
test dependencies, so these always run.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow.parquet as pq
import pytest

from dimos.learning.dataprep.core import OutputConfig, Sample
from dimos.learning.dataprep.formats.lerobot.reader import inspect
from dimos.learning.dataprep.formats.lerobot.writer import write


def _state_samples(n: int = 4) -> Iterator[Sample]:
    for i in range(n):
        yield Sample(
            ts=float(i),
            episode_id="ep_000000",
            observation={"state": np.arange(6, dtype=np.float32)},
            action={"action": np.full(6, float(i), dtype=np.float32)},
        )


def _two_episode_samples() -> Iterator[Sample]:
    for ep in range(2):
        for i in range(3):
            yield Sample(
                ts=float(ep * 3 + i),
                episode_id=f"ep_{ep:06d}",
                observation={"state": np.arange(6, dtype=np.float32) + ep},
                action={"action": np.full(6, float(i), dtype=np.float32)},
            )


def _image_samples(n: int = 4) -> Iterator[Sample]:
    for i in range(n):
        yield Sample(
            ts=float(i),
            episode_id="ep_000000",
            observation={
                "state": np.arange(6, dtype=np.float32),
                "cam": np.full((16, 16, 3), i, dtype=np.uint8),
            },
            action={"action": np.zeros(6, dtype=np.float32)},
        )


def test_lerobot_v3_state_only_layout_and_naming(tmp_path: Path) -> None:
    out = OutputConfig(
        format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0, "robot": "xarm7"}
    )
    root = write(_state_samples(), out)

    # v3.0: concatenated single data file + parquet meta (no jsonl, no per-episode parquet)
    assert (root / "meta" / "info.json").exists()
    assert (root / "meta" / "tasks.parquet").exists()
    assert (root / "meta" / "stats.json").exists()
    assert (root / "meta" / "episodes" / "chunk-000" / "file-000.parquet").exists()
    assert (root / "data" / "chunk-000" / "file-000.parquet").exists()
    assert not (root / "meta" / "episodes.jsonl").exists()
    assert not (root / "meta" / "tasks.jsonl").exists()

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["codebase_version"] == "v3.0"
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 4
    assert info["fps"] == 10.0
    assert info["data_path"] == "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
    # single low-dim state + single action → canonical names
    assert "observation.state" in info["features"]
    assert "action" in info["features"]


def test_lerobot_v3_episode_metadata_columns(tmp_path: Path) -> None:
    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    # two episodes so dataset_from/to_index advance
    root = write(_two_episode_samples(), out)
    ep = pq.read_table(root / "meta" / "episodes" / "chunk-000" / "file-000.parquet")
    cols = set(ep.column_names)
    for required in (
        "episode_index",
        "tasks",
        "length",
        "dataset_from_index",
        "dataset_to_index",
        "data/chunk_index",
        "data/file_index",
        "meta/episodes/chunk_index",
        "meta/episodes/file_index",
    ):
        assert required in cols, f"missing episode column {required}"
    # per-episode stats are embedded (flattened)
    assert any(c.startswith("stats/observation.state/") for c in cols)
    rows = ep.to_pylist()
    assert [r["episode_index"] for r in rows] == [0, 1]
    assert rows[0]["dataset_from_index"] == 0 and rows[0]["dataset_to_index"] == 3
    assert rows[1]["dataset_from_index"] == 3 and rows[1]["dataset_to_index"] == 6


def test_lerobot_v3_writer_closed_on_midstream_error(tmp_path: Path) -> None:
    """If the drain raises after an episode was flushed, the data parquet must
    still be readable (footer written by the finally), not a headerless stub."""

    def bad_samples() -> Iterator[Sample]:
        for i in range(3):  # episode 0
            yield Sample(
                ts=float(i),
                episode_id="ep_000000",
                observation={"state": np.arange(6, dtype=np.float32)},
                action={"action": np.full(6, float(i), dtype=np.float32)},
            )
        # first frame of episode 1 flushes episode 0 (opens + writes the parquet)…
        yield Sample(
            ts=3.0,
            episode_id="ep_000001",
            observation={"state": np.arange(6, dtype=np.float32)},
            action={"action": np.zeros(6, dtype=np.float32)},
        )
        raise RuntimeError("boom mid-stream")  # …then blow up before the final flush

    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    with pytest.raises(RuntimeError, match="boom"):
        write(bad_samples(), out)

    # episode 0's 3 frames were flushed; the file must have a valid footer.
    data = tmp_path / "ds" / "data" / "chunk-000" / "file-000.parquet"
    assert data.exists()
    assert pq.read_table(data).num_rows == 3  # raises ArrowInvalid if footer missing


def test_lerobot_v3_per_episode_task_labels(tmp_path: Path) -> None:
    """Episodes with distinct task_labels must produce distinct tasks + task_index
    (multi-task recordings must not collapse to one task)."""

    def samples() -> Iterator[Sample]:
        for ep, task in ((0, "pick"), (1, "place")):
            for i in range(3):
                yield Sample(
                    ts=float(ep * 3 + i),
                    episode_id=f"ep_{ep:06d}",
                    observation={"state": np.arange(6, dtype=np.float32)},
                    action={"action": np.zeros(6, dtype=np.float32)},
                    task_label=task,
                )

    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    root = write(samples(), out)

    tasks = pd.read_parquet(root / "meta" / "tasks.parquet")
    assert set(tasks.index) == {"pick", "place"}

    ep = pq.read_table(root / "meta" / "episodes" / "chunk-000" / "file-000.parquet").to_pylist()
    assert ep[0]["tasks"] == ["pick"]
    assert ep[1]["tasks"] == ["place"]

    data = pq.read_table(root / "data" / "chunk-000" / "file-000.parquet")
    ti = data.column("task_index").to_pylist()
    assert ti[:3] == [0, 0, 0]  # episode 0 → task 0 (pick)
    assert ti[3:] == [1, 1, 1]  # episode 1 → task 1 (place)


def test_lerobot_v3_inspect_state_only(tmp_path: Path) -> None:
    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    root = write(_state_samples(), out)
    info = inspect(root)
    assert info["format"] == "lerobot"
    assert info["version"] == "v3.0"
    assert info["episodes"] == 1
    assert info["frames"] == 4
    assert "observation.state" in info["observation"]
    assert "action" in info["action"]
    assert info["has_stats"] is True


def test_lerobot_v3_with_images_writes_concatenated_mp4(tmp_path: Path) -> None:
    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    try:
        root = write(_image_samples(), out)
    except RuntimeError as e:
        if "VideoWriter" in str(e):
            pytest.skip(f"no mp4v encoder available in this environment: {e}")
        raise

    # v3.0 video path: videos/<key>/chunk-000/file-000.mp4 (key before chunk, one per camera)
    mp4 = root / "videos" / "observation.images.cam" / "chunk-000" / "file-000.mp4"
    assert mp4.exists() and mp4.stat().st_size > 0

    info = json.loads((root / "meta" / "info.json").read_text())
    assert (
        info["video_path"] == "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
    )
    assert info["features"]["observation.images.cam"]["dtype"] == "video"
    # image column is excluded from parquet; state/action remain
    assert "observation.state" in info["features"]
    assert info["total_frames"] == 4
