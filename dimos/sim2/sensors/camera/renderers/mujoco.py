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

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.sim_msgs import WorldState
import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.poses import pose_array
from dimos.sim2.geometry.query import MujocoGeometry
from dimos.sim2.sensors.camera.config import CameraConfig


class MujocoRenderer:
    def load(self, scene: SceneConfig, camera: CameraConfig) -> None:
        self.camera = camera
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        spec = build_scene(scene, kinematic=True)
        spec.worldbody.add_camera(name="render_camera", ipd=0)
        spec.visual.global_.offwidth = camera.width
        spec.visual.global_.offheight = camera.height
        model = spec.compile()
        # MuJoCo clips in multiples of model extent; keep close wrist geometry
        # visible even when the same scene includes an entire office building.
        model.vis.map.znear = 0.01 / model.stat.extent
        model.vis.map.zfar = 100.0 / model.stat.extent
        self.geometry = MujocoGeometry(model, scene.revision)

    def set_poses(self, sample: WorldState) -> None:
        self.geometry.set_poses(sample)

    def render(self, pose_world: Pose) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        return self.render_camera(self.camera, pose_world)

    def render_camera(
        self, camera: CameraConfig, pose_world: Pose
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        if camera.frame_id not in self.geometry.body_ids:
            raise ValueError(f"unknown camera frame {camera.frame_id}")
        model = self.geometry.model
        size = (camera.width, camera.height)
        if size not in self._renderers:
            if len(self._renderers) == 3:
                self._renderers.pop(next(iter(self._renderers))).close()
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, camera.width)
            model.vis.global_.offheight = max(model.vis.global_.offheight, camera.height)
            self._renderers[size] = mujoco.Renderer(model, width=camera.width, height=camera.height)
        renderer = self._renderers[size]
        # Sensor lengths in pixel units retain independent fx/fy. MuJoCo's
        # principal point shifts the projection plane relative to the image center.
        model.cam_sensorsize[0] = size
        model.cam_resolution[0] = size
        model.cam_intrinsic[0] = [
            camera.fx,
            camera.fy,
            camera.width / 2 - camera.cx,
            camera.height / 2 - camera.cy,
        ]
        pose = pose_array(pose_world)
        model.cam_pos[0] = pose[:3]
        model.cam_quat[0] = Rotation.from_matrix(
            Rotation.from_quat(pose[3:]).as_matrix() @ np.diag([1, -1, -1])
        ).as_quat(scalar_first=True)
        mujoco.mj_camlight(model, self.geometry.data)
        renderer.update_scene(self.geometry.data, camera="render_camera")
        renderer.disable_depth_rendering()
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        depth[depth >= 99.9] = np.inf
        return rgb, depth

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    def cancel(self) -> None:
        # No external I/O to interrupt. OpenGL cleanup must stay on its owner thread.
        pass
