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

from __future__ import annotations

import tarfile
from pathlib import Path

import cv2
import numpy as np
import pytest

from dimos.evals.media.frames_to_store import (
    dir_to_store,
    frame_index,
    sorted_frame_paths,
    tar_to_store,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image


def _write_png(path: Path, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = np.full((8, 12, 3), value, dtype=np.uint8)
    assert cv2.imwrite(str(path), bgr)


def test_frame_index_parses_openeqa_names() -> None:
    assert frame_index("000-hm3d-x/00042-rgb.png") == 42
    assert frame_index(Path("7-rgb.JPG")) == 7
    assert frame_index("readme.txt") is None


def test_dir_to_store_writes_color_image_in_index_order(tmp_path: Path) -> None:
    root = tmp_path / "ep"
    _write_png(root / "00002-rgb.png", 20)
    _write_png(root / "00000-rgb.png", 0)
    _write_png(root / "00001-rgb.png", 10)
    out = tmp_path / "recording.db"

    assert dir_to_store(root, out, hz=2.0) == 3
    with SqliteStore(path=str(out), must_exist=True) as store:
        rows = list(store.stream("color_image", Image))
        assert [obs.ts for obs in rows] == pytest.approx([0.0, 0.5, 1.0])
        # Blobs load while the store is open; JPEG keeps brightness order only.
        means = [float(obs.data.data.mean()) for obs in rows]
    assert means[0] < means[1] < means[2]


def test_tar_to_store_reads_members_without_extracting(tmp_path: Path) -> None:
    stage = tmp_path / "stage"
    _write_png(stage / "ep" / "00000-rgb.png", 5)
    _write_png(stage / "ep" / "00001-rgb.png", 9)
    tar_path = tmp_path / "ep.tar"
    with tarfile.open(tar_path, "w") as archive:
        archive.add(stage / "ep" / "00000-rgb.png", arcname="ep/00000-rgb.png")
        archive.add(stage / "ep" / "00001-rgb.png", arcname="ep/00001-rgb.png")
    out = tmp_path / "from_tar.db"

    assert tar_to_store(tar_path, out, hz=1.0) == 2
    with SqliteStore(path=str(out), must_exist=True) as store:
        assert len(list(store.stream("color_image", Image))) == 2


def test_sorted_frame_paths_ignores_non_rgb(tmp_path: Path) -> None:
    _write_png(tmp_path / "00001-rgb.png", 1)
    (tmp_path / "notes.txt").write_text("x")
    paths = sorted_frame_paths(tmp_path.iterdir())
    assert [p.name for p in paths] == ["00001-rgb.png"]
