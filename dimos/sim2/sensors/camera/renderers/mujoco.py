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

import math

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.sim_msgs import BodyPoses
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
        if not (
            math.isclose(camera.fx, camera.fy)
            and camera.cx == camera.width / 2
            and camera.cy == camera.height / 2
        ):
            raise ValueError(
                "this renderer currently supports centered square-pixel pinhole cameras"
            )
        spec = build_scene(scene, kinematic=True)
        spec.worldbody.add_camera(
            name="render_camera", fovy=math.degrees(2 * math.atan(camera.height / (2 * camera.fy)))
        )
        spec.visual.global_.offwidth = camera.width
        spec.visual.global_.offheight = camera.height
        self.geometry = MujocoGeometry(spec.compile(), scene.revision)
        self.renderer = mujoco.Renderer(
            self.geometry.model, height=camera.height, width=camera.width
        )

    def set_poses(self, sample: BodyPoses) -> None:
        self.geometry.set_poses(sample)

    def render(self, pose_world: Pose) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        pose = pose_array(pose_world)
        self.geometry.model.cam_pos[0] = pose[:3]
        self.geometry.model.cam_quat[0] = Rotation.from_matrix(
            Rotation.from_quat(pose[3:]).as_matrix() @ np.diag([1, -1, -1])
        ).as_quat(scalar_first=True)
        mujoco.mj_camlight(self.geometry.model, self.geometry.data)
        self.renderer.update_scene(self.geometry.data, camera="render_camera")
        self.renderer.disable_depth_rendering()
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        return rgb, depth

    def close(self) -> None:
        self.renderer.close()
