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

"""Native Filament worker: GPU resources and Cocoa calls stay on its main thread."""

import math
import mmap
from multiprocessing.connection import Connection
import os
import sys
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import mujoco
from mujoco import mjrf
from mujoco.studio import window as _window  # noqa: F401 -- registers Filament material resources
import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import UsdModel
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.mujoco import _NO_DEPTH, MujocoRenderer


def calibrated_camera(camera: mujoco.MjvGLCamera) -> Any:
    """Preserve asymmetric intrinsics omitted by MuJoCo 4's View convenience conversion."""
    return mjrf.Camera(
        pos=camera.pos,
        forward=camera.forward,
        up=camera.up,
        frustum_bottom=camera.frustum_bottom,
        frustum_top=camera.frustum_top,
        frustum_near=camera.frustum_near,
        frustum_far=camera.frustum_far,
        frustum_width=camera.frustum_width,
        frustum_center=camera.frustum_center,
        orthographic=camera.orthographic,
    )


class _NativeFilamentColorRenderer(MujocoRenderer):
    """Retain scene resources, draw camera viewports together and read back once.

    MuJoCo 4.0 currently accepts one pixel read per batch on this platform. Packing
    viewports into an atlas keeps the batch within that contract. Returned images
    own their pixels, so rendering the next sample cannot overwrite a publication.
    """

    def __init__(self) -> None:
        super().__init__()
        self._context: Any = None
        self._scene: Any = None
        self._target: Any = None
        self._read: Any = None
        self._resources: Any = None
        self._lights: Any = None
        self._renderables: Any = None
        self._rects: dict[str, tuple[int, int, int, int]] = {}
        self._atlas = np.empty((0, 0, 3), dtype=np.uint8)

    def load(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None:
        if any(not c.color or c.depth for c in cameras):
            raise ValueError("FilamentColorRenderer requires color-only cameras")
        self._load_geometry(scene, cameras)
        try:
            self._context = mjrf.Context(mjrf.ContextConfig())
            self._context.set_clear_color([0.0, 0.0, 0.0])
            native_scene = self._scene = self._context.create_scene(mjrf.SceneParams())
            native_scene.configure_from_model(self.geometry.model)
            self._resources = mjrf.ModelResources(self._context, self.geometry.model)
            self._lights = mjrf.ModelLights(native_scene, self._resources)
            self._renderables = mjrf.ModelRenderables(native_scene, self._resources)
            self._renderables.set_options(mujoco.MjvOption())
            columns = math.ceil(math.sqrt(len(cameras)))
            width = height = 0
            for start in range(0, len(cameras), columns):
                row = cameras[start : start + columns]
                x = 0
                for c in row:
                    self._rects[c.frame_id] = (x, height, c.width, c.height)
                    x += c.width
                width = max(width, x)
                height += max(c.height for c in row)
            self._atlas = np.zeros((height, width, 3), dtype=np.uint8)
            config = mjrf.RenderTargetConfig(
                width=width, height=height, color_format=mjrf.PixelFormat.PIXEL_FORMAT_RGB8
            )
            self._target = self._context.create_render_target(config)
            self._read = mjrf.ReadRequest(target=self._target)
            self._read.set_buffer(self._atlas)
        except Exception:
            self.close()
            raise

    def render(
        self, sample: WorldState, camera_ids: tuple[str, ...]
    ) -> dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]]:
        if len(set(camera_ids)) != len(camera_ids) or not set(camera_ids) <= self._cameras.keys():
            raise ValueError("render requests require distinct configured camera IDs")
        if not camera_ids:
            return {}
        self._set_cameras(sample, camera_ids)
        model, data = self.geometry.model, self.geometry.data
        self._lights.update(data)
        self._renderables.update(data)
        height = self._atlas.shape[0]
        views = []
        for name in camera_ids:
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            camera.fixedcamid = self._camera_ids[name]
            x, y, w, h = self._rects[name]
            request = mjrf.RenderRequest()
            request.scene = self._scene
            request.target = self._target
            request.viewport.left = x
            request.viewport.bottom = height - y - h
            request.viewport.width = w
            request.viewport.height = h
            request.camera = calibrated_camera(mujoco.mjv_camera2GLCamera(model, data, camera))
            request.post_processing = True
            request.shadows = request.reflections = request.skybox = request.haze = False
            request.headlight = bool(model.vis.headlight.active)
            request.headlight_intensity = 40000
            views.append(request)
        frame = self._context.render(views, [self._read])
        self._context.wait_for_frame(frame)
        return {
            name: (self._atlas[y : y + h, x : x + w].copy(), _NO_DEPTH)
            for name in camera_ids
            for x, y, w, h in [self._rects[name]]
        }

    def close(self) -> None:
        self._read = self._target = self._renderables = self._lights = self._resources = (
            self._scene
        ) = None
        self._context = None
        self._rects.clear()
        self._atlas = np.empty((0, 0, 3), dtype=np.uint8)
        super().close()


def _serve(connection_fd: int, pixels_fd: int) -> None:
    connection = Connection(connection_fd)
    renderer = _NativeFilamentColorRenderer()
    memory = None
    images: dict[str, NDArray[np.uint8]] = {}
    try:
        scene, cameras = connection.recv()
        renderer.load(scene, cameras)
        memory = mmap.mmap(pixels_fd, os.fstat(pixels_fd).st_size)
        os.close(pixels_fd)
        offset = 0
        for camera in cameras:
            images[camera.frame_id] = np.ndarray(
                (camera.height, camera.width, 3),
                dtype=np.uint8,
                buffer=memory,
                offset=offset,
            )
            offset += camera.width * camera.height * 3
        connection.send((True, None))
        while True:
            encoded, names = connection.recv()
            result = renderer.render(WorldState.lcm_decode(encoded), names)
            for name in names:
                images[name][:] = result[name][0]
            connection.send((True, None))
    except EOFError:
        pass
    except Exception as error:
        connection.send((False, repr(error)))
    finally:
        renderer.close()
        images.clear()
        if memory is not None:
            memory.close()
        connection.close()


if __name__ == "__main__":
    _serve(int(sys.argv[1]), int(sys.argv[2]))
