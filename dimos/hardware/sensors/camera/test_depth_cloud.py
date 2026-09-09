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

import time

import numpy as np
import pytest

from dimos.hardware.sensors.camera.depth_cloud import DepthCloud
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

WIDTH = 8
HEIGHT = 6
FX = 100.0
FY = 120.0
CX = 3.5
CY = 2.5


class Bus:
    """In-process stand-in for a Transport, so `In` ports can be fed directly."""

    def __init__(self) -> None:
        self.subscribers: list = []

    def subscribe(self, callback, stream=None):
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback)

    def publish(self, msg) -> None:
        for callback in list(self.subscribers):
            callback(msg)

    def stop(self) -> None:
        pass


@pytest.fixture
def module():
    built = []

    def build(**config):
        instance = DepthCloud(**config)
        instance.depth.transport = Bus()
        instance.camera_info.transport = Bus()
        built.append(instance)
        return instance

    yield build
    for instance in built:
        instance.dispose()


def camera_info(width: int = WIDTH, height: int = HEIGHT) -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=FX, fy=FY, cx=CX, cy=CY, width=width, height=height, frame_id="calibration_frame"
    )


def depth_image(metres, frame_id: str = "head_optical") -> Image:
    return Image(data=np.asarray(metres), format=ImageFormat.DEPTH, frame_id=frame_id, ts=7.0)


def settle(clouds: list, expected: int) -> list:
    """`In.observable()` is backpressured onto a worker thread, so delivery is async."""
    deadline = time.monotonic() + 5.0
    while len(clouds) < expected and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.05)
    return clouds


def run(instance: DepthCloud, depth: Image, info: CameraInfo | None = None):
    clouds: list = []
    instance.cloud.subscribe(clouds.append)
    instance.start()
    instance.camera_info.transport.publish(info if info is not None else camera_info())
    # Each In is backpressured onto its own thread, so without a gap the depth
    # frame can be handled before the intrinsics land — and get dropped, exactly
    # as the first frames off a real camera are.
    time.sleep(0.05)
    instance.depth.transport.publish(depth)
    return settle(clouds, 1)


def test_unprojects_with_the_pinhole_model(module):
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    clouds = run(module(decimation=1), depth_image(depth))

    assert len(clouds) == 1
    points = clouds[0].points_f32()
    assert points.shape == (HEIGHT * WIDTH, 3)

    columns, rows = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    expected = np.stack(
        [
            (columns.ravel() - CX) * 2.0 / FX,
            (rows.ravel() - CY) * 2.0 / FY,
            np.full(HEIGHT * WIDTH, 2.0),
        ],
        axis=1,
    )
    np.testing.assert_allclose(points, expected, atol=1e-6)


def test_float_depth_is_already_metres(module):
    """The R1 Pro head publishes 32FC1 metres, so depth_scale must not be applied."""
    depth = np.full((HEIGHT, WIDTH), 3.0, dtype=np.float32)
    clouds = run(module(decimation=1, depth_scale=0.001), depth_image(depth))

    np.testing.assert_allclose(clouds[0].points_f32()[:, 2], 3.0, atol=1e-6)


def test_integer_depth_is_scaled(module):
    depth = np.full((HEIGHT, WIDTH), 3000, dtype=np.uint16)
    clouds = run(module(decimation=1, depth_scale=0.001), depth_image(depth))

    np.testing.assert_allclose(clouds[0].points_f32()[:, 2], 3.0, atol=1e-6)


def test_range_gate_drops_out_of_range_pixels(module):
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    depth[0, 0] = 0.05
    depth[0, 1] = 50.0
    depth[0, 2] = 0.0
    clouds = run(module(decimation=1, min_range_m=0.2, max_range_m=6.0), depth_image(depth))

    assert clouds[0].points_f32().shape == (HEIGHT * WIDTH - 3, 3)


def test_decimation_keeps_full_resolution_pixel_coordinates(module):
    """Decimated points must be a strict subset of the undecimated ones."""
    depth = depth_image(np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32))
    every_pixel = run(module(decimation=1), depth)[0].points_f32()
    every_other = run(module(decimation=2), depth)[0].points_f32()

    assert every_other.shape == (HEIGHT // 2 * WIDTH // 2, 3)
    for point in every_other:
        assert np.isclose(every_pixel, point, atol=1e-6).all(axis=1).any()


def test_rescales_intrinsics_to_the_depth_resolution(module):
    """Registered depth at half the calibrated resolution must not skew the geometry."""
    depth = depth_image(np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32))
    clouds = run(module(decimation=1), depth, camera_info(width=WIDTH * 2, height=HEIGHT * 2))

    columns, rows = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    expected = np.stack(
        [
            (columns.ravel() - CX / 2) * 2.0 / (FX / 2),
            (rows.ravel() - CY / 2) * 2.0 / (FY / 2),
            np.full(HEIGHT * WIDTH, 2.0),
        ],
        axis=1,
    )
    np.testing.assert_allclose(clouds[0].points_f32(), expected, atol=1e-6)


def test_frame_id_falls_back_to_the_depth_image(module):
    depth = depth_image(np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32))
    clouds = run(module(decimation=1), depth)

    assert clouds[0].frame_id == "head_optical"


def test_frame_id_override_wins(module):
    """The vendor driver's optical frame need not be the frame tf publishes."""
    depth = depth_image(np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32))
    clouds = run(module(decimation=1, frame_id="camera_head_left_link"), depth)

    assert clouds[0].frame_id == "camera_head_left_link"


def test_republished_camera_info_does_not_re_emit(module):
    """CameraInfo is effectively static; only a new depth frame is a new cloud."""
    instance = module(decimation=1)
    depth = depth_image(np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32))
    clouds = run(instance, depth)

    instance.camera_info.transport.publish(camera_info())
    instance.camera_info.transport.publish(camera_info())
    time.sleep(0.3)

    assert len(clouds) == 1


def test_waits_for_intrinsics(module):
    """Without a CameraInfo there is no valid unprojection, so nothing is published."""
    instance = module(decimation=1)
    clouds: list = []
    instance.cloud.subscribe(clouds.append)
    instance.start()
    instance.depth.transport.publish(depth_image(np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)))
    time.sleep(0.3)

    assert clouds == []
