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

from collections.abc import Iterator
from contextlib import closing, contextmanager
from pathlib import Path
import sqlite3
import threading
from typing import Any

import numpy as np
import pytest

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.experimental.world_belief.recall import index_stream_name
from dimos.experimental.world_belief.scene_scan import ScanIncompleteError, SceneScanner
from dimos.experimental.world_belief.world_belief import WorldBelief
from dimos.experimental.world_belief.worldbelief_module import WorldBeliefModule
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.test_embedding import _MockEmbeddingModel
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class _EmptyDetector(Detector):
    def process_image(self, image: Image) -> ImageDetections2D[Any]:
        return ImageDetections2D(image)


class _FixedRecorder:
    def __init__(self, path: Path) -> None:
        self._path = str(path)

    @rpc
    def recording_path(self) -> str:
        return self._path


def _delete_mode_bytes(path: Path) -> bytes:
    with closing(sqlite3.connect(path)) as conn:
        assert conn.execute("PRAGMA journal_mode=DELETE").fetchone() == ("delete",)
    return path.read_bytes()


@contextmanager
def _world_belief_module(**kwargs: Any) -> Iterator[WorldBeliefModule]:
    module = WorldBeliefModule(scan_prompts=["chair"], **kwargs)
    try:
        module._engine_lock = threading.Lock()
        module._belief = WorldBelief()
        module._live_scanner = SceneScanner(detector=_EmptyDetector(), embed=False)
        yield module
    finally:
        module.stop()


def test_scan_preserves_incomplete_recording(tmp_path: Path) -> None:
    path = tmp_path / "recording.db"
    with SqliteStore(path=path) as store:
        store.stream("sample", int).append(17, ts=1.0)
    original = _delete_mode_bytes(path)

    with (
        _world_belief_module(db_path=path, g=GlobalConfig(replay=True)) as module,
        pytest.raises(ScanIncompleteError, match="no color frames"),
    ):
        module.scan()

    assert list(tmp_path.iterdir()) == [path]
    assert path.read_bytes() == original


def test_recall_skips_invalid_session_and_preserves_inputs(tmp_path: Path) -> None:
    current = tmp_path / "memory_20261124_120000_000001.db"
    invalid = tmp_path / "memory_20261125_120000_000001.db"
    later = tmp_path / "memory_20261126_120000_000001.db"
    history = tmp_path / "history.db"

    with SqliteStore(path=current) as store:
        rgb = Image(data=np.full((12, 16, 3), 140, dtype=np.uint8), format=ImageFormat.RGB, ts=1.0)
        info = CameraInfo(
            width=16, height=12, K=[10.0, 0.0, 8.0, 0.0, 10.0, 6.0, 0.0, 0.0, 1.0], ts=1.0
        )
        store.stream("color_image", Image).append(rgb, ts=1.0)
        store.stream("camera_info", CameraInfo).append(info, ts=1.0)
    invalid.touch()
    with SqliteStore(path=later) as store:
        rgb2 = Image(data=np.full((12, 16, 3), 150, dtype=np.uint8), format=ImageFormat.RGB, ts=2.0)
        store.stream("color_image", Image).append(rgb2, ts=2.0)

    before = {path: _delete_mode_bytes(path) for path in (current, later)}
    before[invalid] = invalid.read_bytes()

    with _world_belief_module(
        db_path=tmp_path / "memory.db",
        history_path=history,
        clip_model_name="local-fixture",
    ) as module:
        module.recorder = _FixedRecorder(current)
        module._recall_clip = _MockEmbeddingModel()
        hit = module.recall("chair")
        stream_name = index_stream_name(module.config.clip_model_name)

        assert hit is not None
        with SqliteStore(path=history, read_only=True) as hist_store:
            indexed = {Path(obs.tags["rec"]).name for obs in hist_store.stream(stream_name, Image)}
        assert indexed == {current.name, later.name}
        assert {path: path.read_bytes() for path in (current, invalid, later)} == before

        with closing(sqlite3.connect(later)) as conn:
            conn.execute("DROP TABLE _streams")
        before[later] = _delete_mode_bytes(later)
        second_hit = module.recall("chair")

    assert second_hit is not None
    assert {path: path.read_bytes() for path in (current, invalid, later)} == before
