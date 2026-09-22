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

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.vision_msgs.msg import Detection2DArray
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
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
            lambda msg: setattr(self, "camera_intrinsics", [msg.k[0], msg.k[4], msg.k[2], msg.k[5]])
        )
        self.register_disposable(Disposable(unsub))

        unsub = self.detection2d.subscribe(self._on_detection)
        self.register_disposable(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_detection(self, det: Detection2DArray) -> None:
        if not det.detections or not self.camera_intrinsics:
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
            header=det.header,
            pose=Pose(position=Point(x=z, y=-x, z=-y), orientation=Quaternion(w=1)),
        )
        logger.debug(
            f"BBox center: ({center_x:.1f}, {center_y:.1f}) → "
            f"Goal pose: ({z:.2f}, {-x:.2f}, {-y:.2f}) in frame '{det.header.frame_id}'"
        )
        self.goal_request.publish(goal)
