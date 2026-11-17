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

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack

import numpy as np
import pytest

from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.filament import FilamentColorRenderer
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def renderers(world):
    with ExitStack() as cleanup:

        def load(kind, cameras):
            renderer = kind()
            cleanup.callback(renderer.close)
            renderer.load(world, cameras)
            return renderer

        yield load


def test_color_projection_matches_calibration_and_classic_depth_path(renderers, poses):
    # Unequal focal lengths and an off-center principal point must survive the
    # conversion to Filament, or RGB no longer lines up with the metric depth.
    camera = CameraConfig(
        "left/camera", width=300, height=200, fx=150, fy=230, cx=170, cy=80, depth=False
    )
    classic = renderers(MujocoRenderer, (camera,))
    filament = renderers(FilamentColorRenderer, (camera,))
    sample = poses()
    centers = []
    for renderer in (classic, filament):
        rgb, depth = renderer.render(sample, (camera.frame_id,))[camera.frame_id]
        assert depth.size == 0
        r, g, b = np.moveaxis(rgb.astype(float), -1, 0)
        box = (r > g * 1.35) & (g > b * 1.8) & (r > 30)
        assert box.sum() > 20
        centers.append(np.argwhere(box).mean(axis=0))
    np.testing.assert_allclose(centers[0], centers[1], atol=2.0)


def test_batched_views_keep_distinct_pixels_and_publications_survive_next_render(renderers, poses):
    cameras = (
        CameraConfig("left/camera", depth=False),
        CameraConfig("right/camera", width=300, height=200, depth=False),
    )
    renderer = renderers(FilamentColorRenderer, cameras)
    names = tuple(c.frame_id for c in cameras)
    result = renderer.render(poses(), names)
    left = result[names[0]][0]
    assert left.shape == (240, 320, 3) and left.std() > 5
    assert result[names[1]][0].shape == (200, 300, 3)
    difference = np.abs(left[:200, :300].astype(float) - result[names[1]][0])
    assert np.count_nonzero(np.max(difference, axis=-1) > 10) > 0.01 * 200 * 300
    kept = left.copy()
    renderer.render(poses(), (names[1],))
    np.testing.assert_array_equal(left, kept)
    with pytest.raises(ValueError, match="distinct"):
        renderer.render(poses(), (names[0], names[0]))


def test_metric_depth_is_not_silently_replaced_with_display_depth(renderers):
    with pytest.raises(ValueError, match="color-only"):
        renderers(FilamentColorRenderer, (CameraConfig("left/camera"),))


def test_sensor_thread_can_capture_and_close_without_a_main_thread_event_loop(world, poses):
    sample = poses()

    def capture():
        renderer = FilamentColorRenderer()
        try:
            renderer.load(world, (CameraConfig("left/camera", depth=False),))
            process = renderer._process
            rgb, _ = renderer.render(sample, ("left/camera",))["left/camera"]
            assert rgb.std() > 5
        finally:
            renderer.close()
        assert process.poll() == 0
        # Returned publications own their memory after the process and SHM close.
        assert rgb.std() > 5

    with ThreadPoolExecutor(max_workers=1) as executor:
        executor.submit(capture).result(timeout=30)
