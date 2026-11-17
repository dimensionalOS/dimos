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
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.query import MujocoGeometry
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.mujoco import _NO_COLOR, _NO_DEPTH, _shared_views

# The sensor's pixel pitch, in metres. Only its ratio to the focal length matters.
_PIXEL_M = 1e-5
# Headlight strength (RGB, 0 to 1) for a scene with no authored light.
_UNLIT_AMBIENT = (0.45, 0.45, 0.45)
_UNLIT_DIFFUSE = (0.7, 0.7, 0.7)


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


class _NativeMujocoRenderer:
    """Retain scene resources, draw camera viewports together and read back once.

    MuJoCo 4.0 currently accepts one pixel read per batch on this platform. Packing
    viewports into one atlas per channel keeps the batch within that contract.
    The public adapter copies shared pixels before returning a publication.
    """

    def __init__(self) -> None:
        self._cameras: dict[str, CameraConfig] = {}
        self._camera_ids: dict[str, int] = {}
        self._context: Any = None
        self._scene: Any = None
        self._targets: dict[str, Any] = {}
        self._reads: dict[str, Any] = {}
        self._resources: Any = None
        self._lights: Any = None
        self._renderables: Any = None
        self._rects: dict[str, tuple[int, int, int, int]] = {}
        self._atlases: dict[str, Any] = {}

    def _load_geometry(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None:
        if self._cameras:
            raise RuntimeError("close the existing camera scene before loading another")
        if not cameras or len({c.frame_id for c in cameras}) != len(cameras):
            raise ValueError("cameras require distinct, nonempty frame IDs")
        if not all(camera.frame_id for camera in cameras):
            raise ValueError("cameras require distinct, nonempty frame IDs")
        spec = build_scene(scene, kinematic=True)
        _load_lights(spec, scene)
        if not spec.lights:
            # A scene that authors no light is lit by the camera's own headlight
            # alone. MuJoCo's default leaves such a view too dark to work in.
            spec.visual.headlight.ambient = _UNLIT_AMBIENT
            spec.visual.headlight.diffuse = _UNLIT_DIFFUSE
        for i, camera in enumerate(cameras):
            # The full pinhole model: MuJoCo builds the frustum from the sensor size,
            # both focal lengths and the principal point, so fx != fy and an
            # off-center principal point render as CameraInfo states them.
            spec.worldbody.add_camera(
                name=f"sim2_view_{i}",
                resolution=[camera.width, camera.height],
                sensor_size=[camera.width * _PIXEL_M, camera.height * _PIXEL_M],
                focal_pixel=[camera.fx, camera.fy],
                # MuJoCo measures the principal point from the image center, toward
                # decreasing pixel coordinates on both axes.
                principal_pixel=[camera.width / 2 - camera.cx, camera.height / 2 - camera.cy],
            )
        self.geometry = MujocoGeometry(spec.compile(), scene.revision)
        self.geometry.select_surfaces()
        if not {c.frame_id for c in cameras} <= set(self.geometry.body_ids):
            raise ValueError("camera frames must exist in the composed USD scene")
        # MuJoCo scales clipping by the scene extent; keep a 1 cm near plane.
        self.geometry.model.vis.map.znear = 0.01 / self.geometry.model.stat.extent
        self._cameras = {c.frame_id: c for c in cameras}
        self._camera_ids = {
            c.frame_id: self.geometry.model.camera(f"sim2_view_{i}").id
            for i, c in enumerate(cameras)
        }

    def _set_cameras(self, sample: WorldState, camera_ids: tuple[str, ...]) -> None:
        self.geometry.set_poses(sample)
        for name in camera_ids:
            body = self.geometry.model.body(name).id
            camera = self._camera_ids[name]
            self.geometry.model.cam_pos[camera] = self.geometry.data.xpos[body]
            self.geometry.model.cam_quat[camera] = Rotation.from_matrix(
                self.geometry.data.xmat[body].reshape(3, 3) @ np.diag([1, -1, -1])
            ).as_quat(scalar_first=True)
        mujoco.mj_camlight(self.geometry.model, self.geometry.data)

    def load(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None:
        self._load_geometry(scene, cameras)
        try:
            self._context = mjrf.Context(mjrf.ContextConfig())
            self._context.set_clear_color([0.0, 0.0, 0.0])
            native_scene = self._scene = self._context.create_scene(mjrf.SceneParams())
            native_scene.configure_from_model(self.geometry.model)
            # Depth edges must contain a single surface, never averaged distances.
            native_scene.set_msaa(1)
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
            # MuJoCo 4 recreates its offscreen swapchain when the submitted
            # viewport extents change. Share one atlas size across RGB and depth.
            for kind, dtype, pixel_format in (
                ("color", np.uint8, mjrf.PixelFormat.PIXEL_FORMAT_RGB8),
                ("depth", np.float32, mjrf.PixelFormat.PIXEL_FORMAT_R32F),
            ):
                if not any(getattr(c, kind) for c in cameras):
                    continue
                shape = (height, width, 3) if kind == "color" else (height, width)
                atlas = self._atlases[kind] = np.zeros(shape, dtype=dtype)
                config = mjrf.RenderTargetConfig(
                    width=width, height=height, color_format=pixel_format
                )
                target = self._targets[kind] = self._context.create_render_target(config)
                read = self._reads[kind] = mjrf.ReadRequest(target=target)
                read.set_buffer(atlas)
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
        result = {name: (_NO_COLOR, _NO_DEPTH) for name in camera_ids}
        for kind, atlas in self._atlases.items():
            rects = {
                name: self._rects[name] for name in camera_ids if getattr(self._cameras[name], kind)
            }
            if not rects:
                continue
            # Anchor each due subset at the atlas's upper-right corner. In
            # OpenGL coordinates this keeps the maximum viewport X/Y constant,
            # even when cameras with different rates or image types are absent.
            dx = atlas.shape[1] - max(x + w for x, y, w, h in rects.values())
            dy = -min(y for x, y, w, h in rects.values())
            rects = {name: (x + dx, y + dy, w, h) for name, (x, y, w, h) in rects.items()}
            views = []
            cameras = {}
            for name, (x, y, w, h) in rects.items():
                camera = mujoco.MjvCamera()
                camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
                camera.fixedcamid = self._camera_ids[name]
                cameras[name] = gl_camera = mujoco.mjv_camera2GLCamera(model, data, camera)
                request = mjrf.RenderRequest()
                request.scene = self._scene
                request.target = self._targets[kind]
                request.viewport.left = x
                request.viewport.bottom = atlas.shape[0] - y - h
                request.viewport.width = w
                request.viewport.height = h
                request.camera = calibrated_camera(gl_camera)
                request.post_processing = kind == "color"
                request.shadows = request.reflections = request.skybox = request.haze = False
                request.headlight = kind == "color" and bool(model.vis.headlight.active)
                request.headlight_intensity = 40000
                if kind == "depth":
                    request.draw_mode = mjrf.DrawMode.DRAW_MODE_DEPTH
                views.append(request)
            # The MuJoCo 4.0 OpenGL backend permits one readback per submission.
            # Batch all views of a channel into one atlas; share geometry across both.
            frame = self._context.render(views, [self._reads[kind]])
            self._context.wait_for_frame(frame)
            for name, gl_camera in cameras.items():
                x, y, w, h = rects[name]
                pixels = atlas[y : y + h, x : x + w]
                rgb, depth = result[name]
                if kind == "color":
                    rgb = pixels
                else:
                    # Filament's depth shader writes reversed projected Z (near=1,
                    # far=0) into R32F. Invert the projection into optical-axis metres.
                    # Postprocessing/MSAA are disabled so values remain geometric.
                    near, far = gl_camera.frustum_near, gl_camera.frustum_far
                    depth = np.where(
                        pixels > 0, near * far / (near + (far - near) * pixels), 0
                    ).astype(np.float32)
                result[name] = rgb, depth
        return result

    def close(self) -> None:
        self._reads.clear()
        self._targets.clear()
        self._renderables = self._lights = self._resources = self._scene = None
        self._context = None
        self._rects.clear()
        self._atlases.clear()
        self._cameras.clear()


def _load_lights(spec: mujoco.MjSpec, scene: UsdModel) -> None:
    """Import USD point/directional lights omitted by MuJoCo 4.0's USD decoder.

    This adapter uses the classic renderer's RGB light strengths; it is not a
    photometric renderer. Light transforms remain attached to their rigid body.
    """
    pxr = load_pxr()
    # USD is optional during blueprint discovery; load it only for a camera scene.
    # load_pxr() must register MuJoCo's schemas before the first pxr import.
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    root = scene.source_path.parent if scene.source_path else None
    stage = pxr.Usd.Stage.Open(layer_from(pxr, scene.usd, "camera lights", root))
    transforms = UsdGeom.XformCache()
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdLux.LightAPI):
            continue
        if not (prim.IsA(UsdLux.DistantLight) or prim.IsA(UsdLux.SphereLight)):
            raise ValueError(
                f"MuJoCo camera cannot render USD light {prim.GetPath()}: {prim.GetTypeName()}"
            )
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        light = UsdLux.LightAPI(prim)
        transform = transforms.GetLocalToWorldTransform(prim)
        parent = prim.GetParent()
        while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
            parent = parent.GetParent()
        body = spec.worldbody
        if parent:
            name = str(parent.GetDisplayName() or parent.GetName())
            body = spec.body(name)
            transform = transform * transforms.GetLocalToWorldTransform(parent).GetInverse()
        color = np.asarray(light.GetColorAttr().Get(), dtype=float)
        if light.GetEnableColorTemperatureAttr().Get():
            color *= UsdLux.BlackbodyTemperatureAsRgb(light.GetColorTemperatureAttr().Get())
        strength = float(light.GetIntensityAttr().Get()) * 2 ** float(light.GetExposureAttr().Get())
        directional = prim.IsA(UsdLux.DistantLight)
        shaping = UsdLux.ShapingAPI(prim)
        cone = shaping.GetShapingConeAngleAttr().Get() if shaping else None
        shadow = UsdLux.ShadowAPI(prim)
        casts_shadow = shadow.GetShadowEnableAttr().Get() if shadow else True
        kind = (
            mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
            if directional
            else mujoco.mjtLightType.mjLIGHT_SPOT
            if cone is not None
            else mujoco.mjtLightType.mjLIGHT_POINT  # type: ignore[attr-defined]  # MuJoCo 4.0
        )
        body.add_light(
            name=str(prim.GetPath()),
            type=kind,
            pos=tuple(transform.ExtractTranslation()),
            dir=tuple(transform.TransformDir(Gf.Vec3d(0, 0, -1))),
            diffuse=color * strength * float(light.GetDiffuseAttr().Get()),
            specular=color * strength * float(light.GetSpecularAttr().Get()),
            cutoff=float(cone) if cone is not None else 90,
            attenuation=(1, 0, 1) if not directional else (1, 0, 0),
            castshadow=bool(casts_shadow),
        )


def _serve(connection_fd: int, pixels_fd: int) -> None:
    connection = Connection(connection_fd)
    renderer = _NativeMujocoRenderer()
    memory = None
    images: dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]] = {}
    try:
        scene, cameras = connection.recv()
        renderer.load(scene, cameras)
        memory = mmap.mmap(pixels_fd, os.fstat(pixels_fd).st_size)
        os.close(pixels_fd)
        images = _shared_views(memory, cameras)
        connection.send((True, None))
        while True:
            encoded, names = connection.recv()
            result = renderer.render(WorldState.lcm_decode(encoded), names)
            for name in names:
                images[name][0][:] = result[name][0]
                images[name][1][:] = result[name][1]
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
