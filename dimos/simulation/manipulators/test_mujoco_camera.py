# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for MujocoCamera and engine registry."""

from __future__ import annotations

import math
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.simulation.engines.mujoco_engine import (
    CameraConfig,
    CameraFrame,
    MujocoEngine,
    _clear_registry,
    _engine_registry,
    get_or_create_engine,
    unregister_engine,
)
from dimos.simulation.manipulators.camera import MujocoCamera
from dimos.simulation.manipulators.test_sim_adapter import _patch_mujoco_engine

ARM_DOF = 7


@pytest.fixture(autouse=True)
def clean_registry():
    _clear_registry()
    yield
    _clear_registry()


def _make_camera_frame(
    cam_pos: list[float] | None = None,
    cam_mat: list[float] | None = None,
) -> CameraFrame:
    """Create a CameraFrame with sensible defaults."""
    return CameraFrame(
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth=np.ones((480, 640), dtype=np.float32),
        cam_pos=np.array(cam_pos or [1.0, 2.0, 3.0]),
        cam_mat=np.array(cam_mat or np.eye(3).flatten()),
        fovy=45.0,
        timestamp=1.0,
    )


def _make_mock_engine(fovy: float = 45.0) -> MagicMock:
    mock_engine = MagicMock(spec=MujocoEngine)
    mock_engine.get_camera_fovy.return_value = fovy
    mock_engine.connected = True
    mock_engine._camera_configs = []
    mock_engine.read_camera.return_value = _make_camera_frame()
    return mock_engine


@pytest.fixture
def mock_engine() -> MagicMock:
    return _make_mock_engine()


@pytest.fixture
def camera_with_mock_engine(mock_engine: MagicMock):
    cam = MujocoCamera(camera_name="wrist_camera")
    cam.set_engine(mock_engine)
    yield cam
    cam.stop()


# ---------------------------------------------------------------------------
# 1. Engine Registry
# ---------------------------------------------------------------------------


@pytest.mark.mujoco
class TestEngineRegistry:
    def test_creates_new(self):
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            engine = get_or_create_engine(config_path=Path("/fake/scene.xml"), headless=True)
            assert engine is not None
            assert len(_engine_registry) == 1
        finally:
            for p in patches:
                p.stop()

    def test_returns_same_instance(self):
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            e1 = get_or_create_engine(config_path=Path("/fake/scene.xml"), headless=True)
            e2 = get_or_create_engine(config_path=Path("/fake/scene.xml"), headless=True)
            assert e1 is e2
            assert len(_engine_registry) == 1
        finally:
            for p in patches:
                p.stop()

    def test_merges_new_cameras(self):
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            e1 = get_or_create_engine(
                config_path=Path("/fake/scene.xml"),
                cameras=[CameraConfig(name="cam_a")],
            )
            get_or_create_engine(
                config_path=Path("/fake/scene.xml"),
                cameras=[CameraConfig(name="cam_b")],
            )
            names = {c.name for c in e1._camera_configs}
            assert names == {"cam_a", "cam_b"}
        finally:
            for p in patches:
                p.stop()

    def test_deduplicates_cameras(self):
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            get_or_create_engine(
                config_path=Path("/fake/scene.xml"),
                cameras=[CameraConfig(name="cam_a")],
            )
            get_or_create_engine(
                config_path=Path("/fake/scene.xml"),
                cameras=[CameraConfig(name="cam_a")],
            )
            engine = get_or_create_engine(config_path=Path("/fake/scene.xml"))
            cam_names = [c.name for c in engine._camera_configs]
            assert cam_names.count("cam_a") == 1
        finally:
            for p in patches:
                p.stop()

    def test_unregister_removes(self):
        patches = _patch_mujoco_engine(ARM_DOF)
        for p in patches:
            p.start()
        try:
            engine = get_or_create_engine(config_path=Path("/fake/scene.xml"))
            assert len(_engine_registry) == 1
            unregister_engine(engine)
            assert len(_engine_registry) == 0
        finally:
            for p in patches:
                p.stop()



# ---------------------------------------------------------------------------
# 2. Camera Intrinsics
# ---------------------------------------------------------------------------


@pytest.mark.mujoco
class TestCameraIntrinsics:
    def test_fovy_45(self, camera_with_mock_engine: MujocoCamera):
        cam = camera_with_mock_engine
        cam._build_camera_info()
        info = cam._camera_info_base
        assert info is not None

        expected_fy = 480.0 / (2.0 * math.tan(math.radians(45.0) / 2.0))
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        assert info.K[0] == pytest.approx(expected_fy, abs=0.01)  # fx
        assert info.K[4] == pytest.approx(expected_fy, abs=0.01)  # fy
        assert info.K[2] == pytest.approx(320.0)  # cx
        assert info.K[5] == pytest.approx(240.0)  # cy

    def test_fovy_90(self, mock_engine: MagicMock):
        mock_engine.get_camera_fovy.return_value = 90.0
        cam = MujocoCamera(camera_name="wrist_camera")
        cam.set_engine(mock_engine)
        cam._build_camera_info()
        info = cam._camera_info_base
        assert info is not None
        # tan(45°) = 1.0, so fy = 480 / 2 = 240
        assert info.K[0] == pytest.approx(240.0, abs=0.01)
        assert info.K[4] == pytest.approx(240.0, abs=0.01)
        cam.stop()

    def test_unknown_camera(self, mock_engine: MagicMock):
        mock_engine.get_camera_fovy.return_value = None
        cam = MujocoCamera(camera_name="nonexistent")
        cam.set_engine(mock_engine)
        cam._build_camera_info()
        assert cam._camera_info_base is None
        cam.stop()

    def test_distortion_and_frame_id(self, camera_with_mock_engine: MujocoCamera):
        cam = camera_with_mock_engine
        cam._build_camera_info()
        info = cam._camera_info_base
        assert info is not None
        assert info.D == [0.0, 0.0, 0.0, 0.0, 0.0]
        assert info.distortion_model == "plumb_bob"
        assert info.frame_id == "wrist_camera_color_optical_frame"


# ---------------------------------------------------------------------------
# 3. Camera Lifecycle
# ---------------------------------------------------------------------------


@pytest.mark.mujoco
class TestMujocoCameraLifecycle:
    def test_start_no_engine_raises(self):
        cam = MujocoCamera(camera_name="cam", address="")
        try:
            with pytest.raises(RuntimeError, match="set address"):
                cam.start()
        finally:
            cam.stop()

    def test_start_creates_thread(self, camera_with_mock_engine: MujocoCamera):
        cam = camera_with_mock_engine
        cam._engine.connected = False  # thread will wait, won't spin
        # Patch rx.interval to avoid spawning scheduler threads that leak
        with patch("dimos.simulation.manipulators.camera.rx.interval", return_value=MagicMock()):
            cam.start()
            assert cam._thread is not None
            assert cam._thread.is_alive()
            cam.stop()
            assert cam._thread is None

    def test_stop_sets_event(self, camera_with_mock_engine: MujocoCamera):
        cam = camera_with_mock_engine
        cam._engine.connected = False
        with patch("dimos.simulation.manipulators.camera.rx.interval", return_value=MagicMock()):
            cam.start()
            cam.stop()
        assert cam._stop_event.is_set()
        assert cam._thread is None

    def test_frame_name_properties(self):
        cam = MujocoCamera(camera_name="wrist_camera")
        assert cam._camera_link == "wrist_camera_link"
        assert cam._color_frame == "wrist_camera_color_frame"
        assert cam._color_optical_frame == "wrist_camera_color_optical_frame"
        assert cam._depth_frame == "wrist_camera_depth_frame"
        assert cam._depth_optical_frame == "wrist_camera_depth_optical_frame"
        cam.stop()



# ---------------------------------------------------------------------------
# 4. TF Publishing
# ---------------------------------------------------------------------------


@pytest.mark.mujoco
class TestTFPublishing:
    def test_publish_tf_correct_frames(self, camera_with_mock_engine: MujocoCamera):
        cam = camera_with_mock_engine
        mock_tf = MagicMock()
        frame = _make_camera_frame(cam_pos=[1.0, 2.0, 3.0])

        with patch.object(type(cam), "tf", new_callable=lambda: property(lambda self: mock_tf)):
            cam._publish_tf(ts=0.0, frame=frame)

        mock_tf.publish.assert_called_once()
        transforms = mock_tf.publish.call_args.args
        assert len(transforms) == 3

        child_ids = {t.child_frame_id for t in transforms}
        assert child_ids == {
            "wrist_camera_color_optical_frame",
            "wrist_camera_depth_optical_frame",
            "wrist_camera_link",
        }
        for t in transforms:
            assert t.frame_id == "world"
            assert t.translation.x == pytest.approx(1.0)
            assert t.translation.y == pytest.approx(2.0)
            assert t.translation.z == pytest.approx(3.0)

    def test_publish_tf_none_noop(self, camera_with_mock_engine: MujocoCamera):
        cam = camera_with_mock_engine
        mock_tf = MagicMock()

        with patch.object(type(cam), "tf", new_callable=lambda: property(lambda self: mock_tf)):
            cam._publish_tf(ts=0.0, frame=None)

        mock_tf.publish.assert_not_called()
