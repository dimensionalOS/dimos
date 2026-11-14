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

"""Native camera objects are private to this renderer, never physics config."""

from dimos_lcm.sim_msgs import WorldState
import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.query import MujocoGeometry
from dimos.sim2.sensors.camera.config import CameraConfig

# The sensor's pixel pitch, in metres. Only its ratio to the focal length matters.
_PIXEL_M = 1e-5
# Headlight strength (RGB, 0 to 1) for a scene with no authored light.
_UNLIT_AMBIENT = (0.45, 0.45, 0.45)
_UNLIT_DIFFUSE = (0.7, 0.7, 0.7)


class MujocoRenderer:
    def __init__(self) -> None:
        self._cameras: dict[str, CameraConfig] = {}
        self._camera_ids: dict[str, int] = {}
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}

    def load(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None:
        if self._renderers:
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
        spec.visual.global_.offwidth = max(c.width for c in cameras)
        spec.visual.global_.offheight = max(c.height for c in cameras)
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
        try:
            for c in cameras:
                size = c.width, c.height
                if size not in self._renderers:
                    # Classic Renderer fixes its viewport at construction. Different
                    # resolutions share the model/data, equal resolutions also share GL.
                    self._renderers[size] = mujoco.Renderer(
                        self.geometry.model, height=c.height, width=c.width
                    )
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
        self.geometry.set_poses(sample)
        for name in camera_ids:
            body = self.geometry.model.body(name).id
            camera = self._camera_ids[name]
            self.geometry.model.cam_pos[camera] = self.geometry.data.xpos[body]
            self.geometry.model.cam_quat[camera] = Rotation.from_matrix(
                self.geometry.data.xmat[body].reshape(3, 3) @ np.diag([1, -1, -1])
            ).as_quat(scalar_first=True)
        mujoco.mj_camlight(self.geometry.model, self.geometry.data)
        images = {}
        for name in camera_ids:
            c = self._cameras[name]
            renderer = self._renderers[c.width, c.height]
            renderer.update_scene(self.geometry.data, camera=self._camera_ids[name])
            # Classic OpenGL only guarantees eight fixed-function lights; more
            # triggers software rendering on macOS despite MuJoCo 4's higher limit.
            renderer.scene.nlight = min(renderer.scene.nlight, 8)  # type: ignore[attr-defined]
            renderer.disable_depth_rendering()
            rgb = renderer.render()
            renderer.enable_depth_rendering()
            depth = renderer.render()
            renderer.disable_depth_rendering()
            images[name] = rgb, depth
        return images

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()


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
