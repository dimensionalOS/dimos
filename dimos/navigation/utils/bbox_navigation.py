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

import logging

from dimos_lcm.sensor_msgs import CameraInfo
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.DEBUG)


class Config(ModuleConfig):
    goal_distance: float = 1.0


class BBoxNavigationModule(Module):
    """Minimal module that converts 2D bbox center to navigation goals."""

    config: Config

    detection2d: In[Detection2DArray]
    camera_info: In[CameraInfo]
    goal_request: Out[PoseStamped]
    camera_intrinsics = None

    @rpc
    def start(self) -> None:
        unsub = self.camera_info.subscribe(
            lambda msg: setattr(self, "camera_intrinsics", [msg.K[0], msg.K[4], msg.K[2], msg.K[5]])
        )
        self.register_disposable(Disposable(unsub))

        unsub = self.detection2d.subscribe(self._on_detection)
        self.register_disposable(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_detection(self, det: Detection2DArray) -> None:
        if det.detections_length == 0 or not self.camera_intrinsics:
            return
        fx, fy, cx, cy = self.camera_intrinsics
        center_x, center_y = (
            det.detections[0].bbox.center.position.x,
            det.detections[0].bbox.center.position.y,
        )
        x, y, z = (
            (center_x - cx) / fx * self.config.goal_distance,
            (center_y - cy) / fy * self.config.goal_distance,
            self.config.goal_distance,
        )
        goal = PoseStamped(
            position=Vector3(z, -x, -y),
            orientation=Quaternion(0, 0, 0, 1),
            frame_id=det.header.frame_id,
        )
        logger.debug(
            f"BBox center: ({center_x:.1f}, {center_y:.1f}) → "
            f"Goal pose: ({z:.2f}, {-x:.2f}, {-y:.2f}) in frame '{det.header.frame_id}'"
        )
        self.goal_request.publish(goal)
