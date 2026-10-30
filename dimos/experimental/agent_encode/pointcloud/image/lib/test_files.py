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

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud.image.camera import CameraView
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_explicit_directory_wins_over_the_run_directory(room, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", "run")

    view = CameraView(pose=(0, 0, 1, 0, 0), size=(64, 48))
    explicit = replace(view, out_dir=Path("explicit")).run(room)
    run = view.run(room)

    assert explicit.path.parent == tmp_path / "explicit"
    assert run.path.parent == tmp_path / "run" / "agent_encode"


@pytest.mark.parametrize("frame_id", ["../escaped", "sensor/child", "/absolute", "sensor a:$"])
def test_file_names_are_confined_and_deterministic(frame_id, tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[0.5, 0.5, 0], [0.5, 0.5, 1]], dtype=np.float32), frame_id=frame_id, timestamp=7
    )
    view = CameraView(pose=(0, 0, 0.5, 0, 0), size=(64, 48), out_dir=tmp_path)
    turned = replace(view, pose=(0, 0, 0.5, 90, 0))

    first, second, other = view.run(cloud), view.run(cloud), turned.run(cloud)

    assert first.path == second.path != other.path
    assert all(path.parent == tmp_path and path.is_file() for path in (first.path, other.path))
    assert ".." not in first.path.name and "/" not in first.path.name
