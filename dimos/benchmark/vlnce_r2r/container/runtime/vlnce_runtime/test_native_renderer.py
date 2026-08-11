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

import json
from types import SimpleNamespace

import numpy as np

from dimos.benchmark.vlnce_r2r.container.runtime.vlnce_runtime import native_renderer
from dimos.benchmark.vlnce_r2r.container.runtime.vlnce_runtime.native_renderer import (
    NativeEpisodeRenderer,
    compose_native_frame,
)


def _pose(x=0.0, z=0.0):
    return SimpleNamespace(
        position=np.array([x, 0.0, z]),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _occupancy():
    traversability = np.ones((100, 200), dtype=np.uint8)
    traversability[:5] = 0
    return {
        "width": 200,
        "height": 100,
        "origin_x": -10.0,
        "origin_z": -5.0,
        "upper_x": 10.0,
        "upper_z": 5.0,
        "traversability": traversability,
    }


def test_frame_uses_exact_rgb_and_draws_only_pose_and_accepted_trajectory() -> None:
    rgb = np.zeros((224, 224, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255

    frame = compose_native_frame(
        rgb,
        _occupancy(),
        [[-5.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        _pose(),
        control_count=1,
    )

    assert frame.shape == (448, 896, 3)
    assert frame.dtype == np.uint8
    assert frame[200, 200].tolist() == [0, 0, 255]
    map_panel = frame[:, 448:]
    assert np.any(np.all(map_panel == [255, 170, 0], axis=2))
    assert np.any(np.all(map_panel == [0, 230, 255], axis=2))


def test_renderer_records_action_time_holds_and_metadata(tmp_path, monkeypatch) -> None:
    writers = []

    class FakeWriter:
        def __init__(self, *args):
            self.frames = []
            self.released = False
            writers.append(self)

        def isOpened(self):
            return True

        def write(self, frame):
            self.frames.append(frame.copy())

        def release(self):
            self.released = True

    monkeypatch.setattr(native_renderer.cv2, "VideoWriter", FakeWriter)
    environment = SimpleNamespace(
        observations={"rgb": np.zeros((224, 224, 3), dtype=np.uint8)},
        trajectory=[[0.0, 0.0, 0.0]],
        pose=_pose(),
        static_occupancy=_occupancy,
    )
    metadata_path = tmp_path / "native-render.v1.json"
    renderer = NativeEpisodeRenderer(tmp_path / "native-render.mp4", metadata_path, environment)

    renderer.capture_initial()
    environment.trajectory.append([1.0, 0.0, 0.0])
    renderer.capture_control()
    renderer.capture_terminal("submitted")
    renderer.close()

    assert len(writers) == 1
    assert len(writers[0].frames) == 21
    assert writers[0].released is True
    metadata = json.loads(metadata_path.read_text())
    assert metadata == {
        "composition": "rgb_and_initial_floor_navmesh",
        "control_frame_count": 1,
        "fps": 10.0,
        "frame_count": 21,
        "height": 448,
        "schema_version": "native-render.v1",
        "status": "completed",
        "terminal_reason": "submitted",
        "timing": "simulated_action_time",
        "width": 896,
    }


def test_renderer_failure_is_metadata_not_an_environment_exception(tmp_path, monkeypatch) -> None:
    class ClosedWriter:
        def __init__(self, *args):
            pass

        def isOpened(self):
            return False

        def release(self):
            pass

    monkeypatch.setattr(native_renderer.cv2, "VideoWriter", ClosedWriter)
    environment = SimpleNamespace(
        observations={"rgb": np.zeros((224, 224, 3), dtype=np.uint8)},
        trajectory=[[0.0, 0.0, 0.0]],
        pose=_pose(),
        static_occupancy=_occupancy,
    )
    metadata_path = tmp_path / "native-render.v1.json"
    renderer = NativeEpisodeRenderer(tmp_path / "native-render.mp4", metadata_path, environment)

    renderer.capture_initial()
    renderer.close()

    metadata = json.loads(metadata_path.read_text())
    assert metadata["status"] == "failed"
    assert metadata["frame_count"] == 0
    assert metadata["diagnostic"].startswith("RuntimeError:")
