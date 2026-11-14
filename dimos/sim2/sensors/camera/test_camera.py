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

from contextlib import ExitStack

import numpy as np
import pytest

from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer

# Requires an offscreen OpenGL context on the graphics-capable runner.
pytestmark = pytest.mark.self_hosted


@pytest.fixture
def make_renderer(world):
    with ExitStack() as cleanup:

        def load(cameras):
            renderer = MujocoRenderer()
            cleanup.callback(renderer.close)
            renderer.load(world, cameras)
            return renderer

        yield load


@pytest.fixture
def renderer(make_renderer):
    return make_renderer((CameraConfig("left/camera"), CameraConfig("right/camera")))


def test_camera_renders_recorded_body_poses_without_physics_state(
    backend, poses, renderer, motor_commands
):
    sample = poses()
    rgb, depth = renderer.render(sample, ("left/camera",))["left/camera"]
    assert rgb.std() > 5
    assert depth.shape == (240, 320) and np.isfinite(depth).all()
    assert renderer.geometry.model.nq == renderer.geometry.model.nv == 0
    backend.apply_action(motor_commands(), [])
    for _ in range(300):
        backend.step()
    moved = poses()
    after, _ = renderer.render(moved, ("left/camera",))["left/camera"]
    # Require visible motion without coupling the check to scene brightness.
    changed_pixels = np.count_nonzero(np.any(after != rgb, axis=-1))
    assert changed_pixels > 0.01 * rgb.shape[0] * rgb.shape[1]
    replay, _ = renderer.render(sample, ("left/camera",))["left/camera"]
    np.testing.assert_array_equal(replay, rgb)


def test_usd_authored_lights_illuminate_camera_image(poses, renderer):
    sample = poses()
    lit, _ = renderer.render(sample, ("left/camera",))["left/camera"]
    assert renderer.geometry.model.nlight == 1
    renderer.geometry.model.light_active[:] = False
    unlit, _ = renderer.render(sample, ("left/camera",))["left/camera"]
    assert lit.mean() > unlit.mean() + 5


def test_two_views_share_scene_and_context_and_retain_their_pixels(poses, renderer, mocker):
    update = mocker.spy(renderer.geometry, "set_poses")
    sample = poses(time_ns=200_000_000)
    images = renderer.render(sample, ("left/camera", "right/camera"))
    update.assert_called_once_with(sample)
    assert len(renderer._renderers) == 1
    left, right = images["left/camera"][0], images["right/camera"][0]
    assert np.count_nonzero(np.any(left != right, axis=-1)) > 0.01 * 320 * 240
    saved = left.copy()
    renderer.render(sample, ("right/camera",))
    np.testing.assert_array_equal(left, saved)


def test_different_resolutions_keep_intrinsics_and_share_the_model(poses, make_renderer):
    renderer = make_renderer(
        (
            CameraConfig("left/camera", width=160, height=120, fx=94.18, fy=94.18, cx=80, cy=60),
            CameraConfig("right/camera"),
        )
    )
    images = renderer.render(poses(), ("left/camera", "right/camera"))
    assert images["left/camera"][0].shape == (120, 160, 3)
    assert images["left/camera"][1].shape == (120, 160)
    assert images["right/camera"][0].shape == (240, 320, 3)
    assert all(r.model is renderer.geometry.model for r in renderer._renderers.values())


def test_any_pinhole_calibration_renders_where_its_camera_info_says(poses, make_renderer):
    # The same view through an ordinary camera and through one with unequal focal
    # lengths and an off-center principal point: every surface point one sees must
    # be where the other's calibration puts it.
    centered = CameraConfig("left/camera")
    skewed = CameraConfig("left/camera", width=300, height=200, fx=150, fy=230, cx=170, cy=80)
    sample = poses()
    _, reference = make_renderer((centered,)).render(sample, ("left/camera",))["left/camera"]
    _, depth = make_renderer((skewed,)).render(sample, ("left/camera",))["left/camera"]
    assert depth.shape == (200, 300)
    v, u = np.mgrid[5:195:7, 5:295:7]
    z = depth[v, u]
    x = (u + 0.5 - skewed.cx) / skewed.fx * z
    y = (v + 0.5 - skewed.cy) / skewed.fy * z
    ur = np.floor(centered.fx * x / z + centered.cx).astype(int)
    vr = np.floor(centered.fy * y / z + centered.cy).astype(int)
    seen = (z > 0) & (ur >= 0) & (ur < centered.width) & (vr >= 0) & (vr < centered.height)
    assert seen.sum() > 200
    error = np.abs(reference[vr[seen], ur[seen]] - z[seen])
    # Object edges land on the neighbouring surface; everything else agrees to a centimetre.
    assert np.mean(error < 0.01) > 0.95
    assert np.unique(np.round(z[seen], 2)).size > 20  # A real scene, not one flat depth.
